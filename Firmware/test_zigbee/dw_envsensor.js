// Zigbee2MQTT external converter for DW Environmental Sensor
// Firmware: test_zigbee (STM32WBA55, STM32CubeWBA 1.9.0)
//
// Standard clusters (temp, humidity, pressure, illuminance) are handled by
// built-in extends. Custom manufacturer-specific clusters (CO2, CO, AQI, PM)
// are defined here with manufacturer code 0x1002.
//
// The device's simple descriptor may not advertise the custom clusters, so
// we use a custom configure function that directly calls endpoint.bind() and
// endpoint.configureReporting() — these bypass the input cluster check that
// the automatic modernExtend configure uses.
//
// Deploy to: /config/zigbee2mqtt/external_converters/dw_envsensor.js
// Then add to /config/zigbee2mqtt/configuration.yaml:
//   external_converters:
//     - dw_envsensor.js

const {deviceAddCustomCluster, numeric, temperature, humidity, pressure, illuminance} =
    require('zigbee-herdsman-converters/lib/modernExtend');

const manufacturerCode = 0x1002;

// Custom cluster definitions with their attributes for configure
const customClusters = [
    {
        name: 'manuSpecificDW_CO2',
        ID: 0xFC00,
        attributes: [{attribute: {ID: 0x0000, type: 0x21}, minimumReportInterval: 10, maximumReportInterval: 300, reportableChange: 1}],
    },
    {
        name: 'manuSpecificDW_CO',
        ID: 0xFC01,
        attributes: [{attribute: {ID: 0x0000, type: 0x21}, minimumReportInterval: 10, maximumReportInterval: 300, reportableChange: 1}],
    },
    {
        name: 'manuSpecificDW_AQI',
        ID: 0xFC02,
        attributes: [
            {attribute: {ID: 0x0000, type: 0x21}, minimumReportInterval: 10, maximumReportInterval: 300, reportableChange: 1},
            {attribute: {ID: 0x0001, type: 0x21}, minimumReportInterval: 10, maximumReportInterval: 300, reportableChange: 1},
        ],
    },
    {
        name: 'manuSpecificDW_PM',
        ID: 0xFC03,
        attributes: [
            {attribute: {ID: 0x0000, type: 0x21}, minimumReportInterval: 10, maximumReportInterval: 300, reportableChange: 1},
            {attribute: {ID: 0x0001, type: 0x21}, minimumReportInterval: 10, maximumReportInterval: 300, reportableChange: 1},
            {attribute: {ID: 0x0002, type: 0x21}, minimumReportInterval: 10, maximumReportInterval: 300, reportableChange: 1},
            {attribute: {ID: 0x0003, type: 0x21}, minimumReportInterval: 10, maximumReportInterval: 300, reportableChange: 1},
        ],
    },
];

const definition = {
    zigbeeModel: ['STM32WBA'],
    model: 'STM32WBA',
    vendor: 'STMicroelectronics',
    description: 'DW Environmental Sensor (CO2, CO, VOC, NOx, PM)',
    extend: [
        // Standard clusters
        temperature(),
        humidity(),
        pressure(),
        illuminance(),

        // CO2 — cluster 0xFC00, attribute 0x0000, uint16, ppm
        deviceAddCustomCluster('manuSpecificDW_CO2', {
            ID: 0xFC00,
            manufacturerCode,
            attributes: {
                measuredValue: {ID: 0x0000, type: 0x21, name: 'measuredValue'},
            },
            commands: {},
            commandsResponse: {},
        }),
        numeric({
            name: 'co2',
            cluster: 'manuSpecificDW_CO2',
            attribute: 'measuredValue',
            description: 'CO2 concentration',
            unit: 'ppm',
            access: 'STATE',
            zigbeeCommandOptions: {manufacturerCode},
        }),

        // CO — cluster 0xFC01, attribute 0x0000, uint16, ppm
        deviceAddCustomCluster('manuSpecificDW_CO', {
            ID: 0xFC01,
            manufacturerCode,
            attributes: {
                measuredValue: {ID: 0x0000, type: 0x21, name: 'measuredValue'},
            },
            commands: {},
            commandsResponse: {},
        }),
        numeric({
            name: 'co',
            cluster: 'manuSpecificDW_CO',
            attribute: 'measuredValue',
            description: 'CO concentration',
            unit: 'ppm',
            access: 'STATE',
            zigbeeCommandOptions: {manufacturerCode},
        }),

        // AQI (VOC Index + NOx Index) — cluster 0xFC02, attributes 0x0000/0x0001, uint16
        deviceAddCustomCluster('manuSpecificDW_AQI', {
            ID: 0xFC02,
            manufacturerCode,
            attributes: {
                vocIndex: {ID: 0x0000, type: 0x21, name: 'vocIndex'},
                noxIndex: {ID: 0x0001, type: 0x21, name: 'noxIndex'},
            },
            commands: {},
            commandsResponse: {},
        }),
        numeric({
            name: 'voc_index',
            cluster: 'manuSpecificDW_AQI',
            attribute: 'vocIndex',
            description: 'VOC index (1–500)',
            unit: 'idx',
            access: 'STATE',
            zigbeeCommandOptions: {manufacturerCode},
        }),
        numeric({
            name: 'nox_index',
            cluster: 'manuSpecificDW_AQI',
            attribute: 'noxIndex',
            description: 'NOx index (1–500)',
            unit: 'idx',
            access: 'STATE',
            zigbeeCommandOptions: {manufacturerCode},
        }),

        // Particulate Matter — cluster 0xFC03, attributes 0x0000–0x0003, uint16, µg/m³
        deviceAddCustomCluster('manuSpecificDW_PM', {
            ID: 0xFC03,
            manufacturerCode,
            attributes: {
                pm1:  {ID: 0x0000, type: 0x21, name: 'pm1'},
                pm25: {ID: 0x0001, type: 0x21, name: 'pm25'},
                pm4:  {ID: 0x0002, type: 0x21, name: 'pm4'},
                pm10: {ID: 0x0003, type: 0x21, name: 'pm10'},
            },
            commands: {},
            commandsResponse: {},
        }),
        numeric({
            name: 'pm1',
            cluster: 'manuSpecificDW_PM',
            attribute: 'pm1',
            description: 'PM1 concentration',
            unit: 'µg/m³',
            access: 'STATE',
            zigbeeCommandOptions: {manufacturerCode},
        }),
        numeric({
            name: 'pm25',
            cluster: 'manuSpecificDW_PM',
            attribute: 'pm25',
            description: 'PM2.5 concentration',
            unit: 'µg/m³',
            access: 'STATE',
            zigbeeCommandOptions: {manufacturerCode},
        }),
        numeric({
            name: 'pm4',
            cluster: 'manuSpecificDW_PM',
            attribute: 'pm4',
            description: 'PM4 concentration',
            unit: 'µg/m³',
            access: 'STATE',
            zigbeeCommandOptions: {manufacturerCode},
        }),
        numeric({
            name: 'pm10',
            cluster: 'manuSpecificDW_PM',
            attribute: 'pm10',
            description: 'PM10 concentration',
            unit: 'µg/m³',
            access: 'STATE',
            zigbeeCommandOptions: {manufacturerCode},
        }),
    ],

    // Custom configure: bind and set up reporting for manufacturer-specific clusters.
    // We call endpoint.bind() and endpoint.configureReporting() directly because they
    // do NOT check the endpoint's input cluster list (unlike the automatic modernExtend
    // configure which calls getEndpointsWithCluster and fails if the device's simple
    // descriptor doesn't advertise the custom clusters).
    configure: async (device, coordinatorEndpoint) => {
        const endpoint = device.getEndpoint(1);
        const opts = {manufacturerCode};

        for (const cluster of customClusters) {
            await endpoint.bind(cluster.name, coordinatorEndpoint);
            await endpoint.configureReporting(cluster.name, cluster.attributes, opts);
        }
    },
};

module.exports = definition;
