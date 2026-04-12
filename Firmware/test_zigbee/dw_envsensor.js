// Zigbee2MQTT external converter for DW Environmental Sensor
// Firmware: test_zigbee (STM32WBA55, STM32CubeWBA 1.9.0)
//
// Deploy to: /config/zigbee2mqtt/external_converters/dw_envsensor.js
//
// Key design notes:
// - cluster-level name: property on deviceAddCustomCluster is required for herdsman to
//   resolve incoming clusterID → cluster name string. Without it msg.cluster = 'undefined'.
//   (Proven by ChatGPT test — the only test where cluster names resolved correctly.)
// - ep.addInputCluster() does NOT exist in this herdsman version. deviceAddCustomCluster
//   registers custom clusters sufficiently for ep.bind() to succeed without it.
// - ep.configureReporting() uses string cluster names (not numeric IDs) to match what herdsman
//   emits after name resolution.
// - fromZigbee converters use string cluster names. A wildcard (cluster: undefined) fallback
//   catches messages if name resolution still fails.
// - commands/commandsResponse omitted from deviceAddCustomCluster — empty objects cause a
//   localeCompare crash in the Z2M Dev Console.
// - meta.logger is not always available in fromZigbee (e.g. during initial interview reads).
//   Use console.error() for logging inside fromZigbee converters.

console.error('[DW] External converter loaded.');

const {deviceAddCustomCluster, temperature, humidity, pressure, illuminance} =
    require('zigbee-herdsman-converters/lib/modernExtend');
const {numeric: numericExpose} =
    require('zigbee-herdsman-converters/lib/exposes');
const ea = require('zigbee-herdsman-converters/lib/exposes').access;

const manufacturerCode = 0x1002;

// Intercept null measuredValue before standard temperature/pressure converters crash
// on it with "Value is not a number, got number (NaN)".
function nullGuardExtend() {
    return {
        isModernExtend: true,
        fromZigbee: [
            {
                cluster: 'msTemperatureMeasurement',
                type: ['attributeReport', 'readResponse'],
                convert(model, msg, publish, options, meta) {
                    if ('measuredValue' in msg.data &&
                        (msg.data.measuredValue === null || isNaN(msg.data.measuredValue))) {
                        console.error('[DW] Dropped null temperature measuredValue');
                        delete msg.data.measuredValue;
                    }
                    return undefined;
                },
            },
            {
                cluster: 'msPressureMeasurement',
                type: ['attributeReport', 'readResponse'],
                convert(model, msg, publish, options, meta) {
                    if ('measuredValue' in msg.data &&
                        (msg.data.measuredValue === null || isNaN(msg.data.measuredValue))) {
                        console.error('[DW] Dropped null pressure measuredValue');
                        delete msg.data.measuredValue;
                    }
                    return undefined;
                },
            },
        ],
        toZigbee: [],
        exposes: [],
    };
}

// Custom sensor data extend for the 4 manufacturer-specific clusters.
// Primary converters use string cluster names (which resolve after deviceAddCustomCluster
// registers the cluster-level name: property).
// Fallback converter uses cluster: undefined wildcard and routes by msg.data keys,
// covering the case where name resolution still fails.
function customSensorsExtend() {
    // Helper: check both friendly name and raw numeric-string fallback
    function get(data, name, id) {
        if (name in data) return data[name];
        const k = String(id);
        if (k in data) return data[k];
        return undefined;
    }

    function extractCO2(data) {
        const v = get(data, 'co2', 0x0000);
        return v !== undefined ? {co2: v} : undefined;
    }
    function extractCO(data) {
        const v = get(data, 'co', 0x0001);
        return v !== undefined ? {co: v} : undefined;
    }
    function extractAQI(data) {
        const r = {};
        const voc = get(data, 'voc_index', 0x0002);
        const nox = get(data, 'nox_index', 0x0003);
        if (voc !== undefined) r.voc_index = voc;
        if (nox !== undefined) r.nox_index = nox;
        return Object.keys(r).length > 0 ? r : undefined;
    }
    function extractPM(data) {
        const r = {};
        const pm1  = get(data, 'pm1',  0x0004);
        const pm25 = get(data, 'pm25', 0x0005);
        const pm4  = get(data, 'pm4',  0x0006);
        const pm10 = get(data, 'pm10', 0x0007);
        if (pm1  !== undefined) r.pm1  = pm1;
        if (pm25 !== undefined) r.pm25 = pm25;
        if (pm4  !== undefined) r.pm4  = pm4;
        if (pm10 !== undefined) r.pm10 = pm10;
        return Object.keys(r).length > 0 ? r : undefined;
    }

    return {
        isModernExtend: true,
        fromZigbee: [
            // Primary: string cluster name matching (works when deviceAddCustomCluster
            // name: property causes herdsman to resolve cluster names correctly)
            {
                cluster: 'manuSpecificDW_CO2',
                type: ['attributeReport', 'readResponse'],
                convert(model, msg) { return extractCO2(msg.data); },
            },
            {
                cluster: 'manuSpecificDW_CO',
                type: ['attributeReport', 'readResponse'],
                convert(model, msg) { return extractCO(msg.data); },
            },
            {
                cluster: 'manuSpecificDW_AQI',
                type: ['attributeReport', 'readResponse'],
                convert(model, msg) { return extractAQI(msg.data); },
            },
            {
                cluster: 'manuSpecificDW_PM',
                type: ['attributeReport', 'readResponse'],
                convert(model, msg) { return extractPM(msg.data); },
            },
            // Fallback: wildcard — catches messages if name resolution still returns
            // 'undefined'. Routes by checking which attribute names/IDs are present.
            {
                cluster: undefined,
                type: ['attributeReport', 'readResponse'],
                convert(model, msg, publish, options, meta) {
                    if (typeof msg.cluster === 'string' && msg.cluster !== 'undefined') {
                        return undefined; // Named cluster — handled above, skip
                    }
                    const r = Object.assign(
                        {},
                        extractCO2(msg.data),
                        extractCO(msg.data),
                        extractAQI(msg.data),
                        extractPM(msg.data),
                    );
                    if (Object.keys(r).length > 0) {
                        meta.logger.warn(`[DW] Fallback converter matched: ${JSON.stringify(r)}`);
                        return r;
                    }
                    return undefined;
                },
            },
        ],
        toZigbee: [],
        exposes: [
            numericExpose('co2',       ea.STATE).withUnit('ppm').withDescription('CO2 concentration'),
            numericExpose('co',        ea.STATE).withUnit('ppm').withDescription('CO concentration'),
            numericExpose('voc_index', ea.STATE).withUnit('idx').withDescription('VOC index (1-500)'),
            numericExpose('nox_index', ea.STATE).withUnit('idx').withDescription('NOx index (1-500)'),
            numericExpose('pm1',       ea.STATE).withUnit('µg/m³').withDescription('PM1 concentration'),
            numericExpose('pm25',      ea.STATE).withUnit('µg/m³').withDescription('PM2.5 concentration'),
            numericExpose('pm4',       ea.STATE).withUnit('µg/m³').withDescription('PM4 concentration'),
            numericExpose('pm10',      ea.STATE).withUnit('µg/m³').withDescription('PM10 concentration'),
        ],
        configure: [async (device, coordinatorEndpoint) => {
            const ep = device.getEndpoint(1);
            if (!ep) {
                console.error('[DW_CFG] Error: endpoint 1 not found');
                return;
            }

            // Bind custom clusters to coordinator
            for (const [name, id] of [
                ['manuSpecificDW_CO2',  0xFC00],
                ['manuSpecificDW_CO',   0xFC01],
                ['manuSpecificDW_AQI',  0xFC02],
                ['manuSpecificDW_PM',   0xFC03],
            ]) {
                try {
                    await ep.bind(name, coordinatorEndpoint);
                    console.error(`[DW_CFG] bind ${name} OK`);
                } catch (e) {
                    console.error(`[DW_CFG] bind ${name} failed: ${e.message}`);
                }
            }

            // Configure reporting. Attribute IDs match firmware defines:
            // CO2=0x0000, CO=0x0001, VOC=0x0002, NOx=0x0003,
            // PM1=0x0004, PM2.5=0x0005, PM4=0x0006, PM10=0x0007
            const reportCfg = [
                ['manuSpecificDW_CO2',
                    [{attribute: {ID: 0x0000, type: 0x21}, minimumReportInterval: 10, maximumReportInterval: 300, reportableChange: 1}]],
                ['manuSpecificDW_CO',
                    [{attribute: {ID: 0x0001, type: 0x21}, minimumReportInterval: 10, maximumReportInterval: 300, reportableChange: 1}]],
                ['manuSpecificDW_AQI', [
                    {attribute: {ID: 0x0002, type: 0x21}, minimumReportInterval: 10, maximumReportInterval: 300, reportableChange: 1},
                    {attribute: {ID: 0x0003, type: 0x21}, minimumReportInterval: 10, maximumReportInterval: 300, reportableChange: 1},
                ]],
                ['manuSpecificDW_PM', [
                    {attribute: {ID: 0x0004, type: 0x21}, minimumReportInterval: 10, maximumReportInterval: 300, reportableChange: 1},
                    {attribute: {ID: 0x0005, type: 0x21}, minimumReportInterval: 10, maximumReportInterval: 300, reportableChange: 1},
                    {attribute: {ID: 0x0006, type: 0x21}, minimumReportInterval: 10, maximumReportInterval: 300, reportableChange: 1},
                    {attribute: {ID: 0x0007, type: 0x21}, minimumReportInterval: 10, maximumReportInterval: 300, reportableChange: 1},
                ]],
            ];
            for (const [name, attrs] of reportCfg) {
                try {
                    await ep.configureReporting(name, attrs, {manufacturerCode});
                    console.error(`[DW_CFG] configureReporting ${name} OK`);
                } catch (e) {
                    console.error(`[DW_CFG] configureReporting ${name} failed: ${e.message}`);
                }
            }
        }],
    };
}

const definition = {
    zigbeeModel: ['STM32WBA'],
    model: 'STM32WBA',
    vendor: 'STMicroelectronics',
    description: 'DW Environmental Sensor (CO2, CO, VOC, NOx, PM)',
    extend: [
        // 1. Register custom clusters FIRST — must precede converters that reference them.
        //    name: at the cluster level is required for herdsman to resolve clusterID → name.
        //    commands/commandsResponse omitted to avoid localeCompare crash in Dev Console.
        deviceAddCustomCluster('manuSpecificDW_CO2', {
            name: 'manuSpecificDW_CO2',
            ID: 0xFC00,
            manufacturerCode,
            attributes: {
                co2: {name: 'co2', ID: 0x0000, type: 0x21},
            },
        }),
        deviceAddCustomCluster('manuSpecificDW_CO', {
            name: 'manuSpecificDW_CO',
            ID: 0xFC01,
            manufacturerCode,
            attributes: {
                co: {name: 'co', ID: 0x0001, type: 0x21},
            },
        }),
        deviceAddCustomCluster('manuSpecificDW_AQI', {
            name: 'manuSpecificDW_AQI',
            ID: 0xFC02,
            manufacturerCode,
            attributes: {
                voc_index: {name: 'voc_index', ID: 0x0002, type: 0x21},
                nox_index: {name: 'nox_index', ID: 0x0003, type: 0x21},
            },
        }),
        deviceAddCustomCluster('manuSpecificDW_PM', {
            name: 'manuSpecificDW_PM',
            ID: 0xFC03,
            manufacturerCode,
            attributes: {
                pm1:  {name: 'pm1',  ID: 0x0004, type: 0x21},
                pm25: {name: 'pm25', ID: 0x0005, type: 0x21},
                pm4:  {name: 'pm4',  ID: 0x0006, type: 0x21},
                pm10: {name: 'pm10', ID: 0x0007, type: 0x21},
            },
        }),

        // 2. Null guard — intercepts null measuredValue from temp/pressure clusters on startup
        //    before temperature()/pressure() converters crash with assertNumber(NaN).
        //    TODO: revisit when real sensor drivers replace synthetic data — if sensors
        //    initialise attributes before Z2M interviews, this guard may no longer be needed.
        nullGuardExtend(),

        // 3. Standard clusters
        temperature(),
        humidity(),
        pressure(),
        illuminance(),

        // 4. Custom sensor data converters + configure + exposes
        customSensorsExtend(),
    ],
};

module.exports = definition;
