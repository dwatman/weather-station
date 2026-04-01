/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_zigbee_endpoint.c
  * Description        : Zigbee Application to manage endpoints and these clusters.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include <assert.h>
#include <stdint.h>

#include "app_common.h"
#include "app_conf.h"
#include "log_module.h"
#include "app_entry.h"
#include "app_zigbee.h"
#include "dbg_trace.h"
#include "ieee802154_enums.h"
#include "mcp_enums.h"

#include "stm32_lpm.h"
#include "stm32_rtos.h"
#include "stm32_timer.h"
#include "stm32_lpm_if.h"

#include "zigbee.h"
#include "zigbee.nwk.h"
#include "zigbee.security.h"

/* Private includes -----------------------------------------------------------*/
#include "zcl/zcl.h"
#include "zcl/general/zcl.identify.h"
#include "zcl/general/zcl.press.meas.h"
#include "zcl/general/zcl.temp.meas.h"
#include "zcl/general/zcl.wcm.h"
#include "zcl/general/zcl.illum.meas.h"

/* USER CODE BEGIN PI */
#include "sensor_data.h"
/* USER CODE END PI */

/* Private defines -----------------------------------------------------------*/
#define APP_ZIGBEE_CHANNEL                11u
#define APP_ZIGBEE_CHANNEL_MASK           ( 1u << APP_ZIGBEE_CHANNEL )
#define APP_ZIGBEE_TX_POWER               ((int8_t) 10)    /* TX-Power is at +10 dBm. */

#define APP_ZIGBEE_ENDPOINT               1u
#define APP_ZIGBEE_PROFILE_ID             ZCL_PROFILE_HOME_AUTOMATION
#define APP_ZIGBEE_DEVICE_ID              ZCL_DEVICE_ENVIRONMENTAL_SENSOR

#define APP_ZIGBEE_CLUSTER1_ID            ZCL_CLUSTER_IDENTIFY
#define APP_ZIGBEE_CLUSTER1_NAME          "Identify Server"

#define APP_ZIGBEE_CLUSTER2_ID            ZCL_CLUSTER_MEAS_PRESSURE
#define APP_ZIGBEE_CLUSTER2_NAME          "PressMeas Server"

#define APP_ZIGBEE_CLUSTER3_ID            ZCL_CLUSTER_MEAS_TEMPERATURE
#define APP_ZIGBEE_CLUSTER3_NAME          "TempMeas Server"

#define APP_ZIGBEE_CLUSTER4_ID            ZCL_CLUSTER_WATER_CONTENT
#define APP_ZIGBEE_CLUSTER4_NAME          "WaterContent Server"

#define APP_ZIGBEE_CLUSTER5_ID            ZCL_CLUSTER_ILLUMINANCE_MEAS
#define APP_ZIGBEE_CLUSTER5_NAME          "Illuminance_meas Server"

/* MeasPressure specific defines ----------------------------------------------------*/
#define APP_ZIGBEE_PRESS_MAX              1200
#define APP_ZIGBEE_PRESS_MIN              800
/* USER CODE BEGIN MeasPressure defines */
/* USER CODE END MeasPressure defines */

/* MeasTemperature specific defines ----------------------------------------------------*/
#define APP_ZIGBEE_TEMP_MIN               -4000
#define APP_ZIGBEE_TEMP_MAX               10000
#define APP_ZIGBEE_TEMP_TOLERANCE         20
/* USER CODE BEGIN MeasTemperature defines */
/* USER CODE END MeasTemperature defines */

/* WaterContent specific defines ----------------------------------------------------*/
#define APP_ZIGBEE_HUMIDITY_MIN           0
#define APP_ZIGBEE_HUMIDITY_MAX           10000
/* USER CODE BEGIN WaterContent defines */
/* USER CODE END WaterContent defines */

/* IlluminanceMeas specific defines ----------------------------------------------------*/
#define APP_ZIGBEE_ILLUM_MIN              1
#define APP_ZIGBEE_ILLUM_MAX              65534
/* USER CODE BEGIN IlluminanceMeas defines */
/* USER CODE END IlluminanceMeas defines */

/* USER CODE BEGIN PD */

// Manufacturer code for custom clusters
#define APP_ZIGBEE_MFR_CODE               0x1002  // STMicroelectronics

// Custom cluster IDs (manufacturer-specific range)
#define APP_ZIGBEE_CLUSTER_CO2            0xFC00
#define APP_ZIGBEE_CLUSTER_CO             0xFC01
#define APP_ZIGBEE_CLUSTER_AQI            0xFC02
#define APP_ZIGBEE_CLUSTER_PM             0xFC03
#define APP_ZIGBEE_CLUSTER_SOUND          0xFC04

// Attribute IDs for custom clusters
#define CUSTOM_ATTR_MEAS_VAL              0x0000
#define CUSTOM_ATTR_MEAS_VAL_2            0x0001
#define CUSTOM_ATTR_MEAS_VAL_3            0x0002
#define CUSTOM_ATTR_MEAS_VAL_4            0x0003

// Sensor update period
#define SENSOR_UPDATE_PERIOD_MS           30000  // 30 seconds

/* USER CODE END PD */

// -- Redefine Clusters to better code read --
#define IdentifyServer                    pstZbCluster[0]
#define PressMeasServer                   pstZbCluster[1]
#define TempMeasServer                    pstZbCluster[2]
#define WaterContentServer                pstZbCluster[3]
#define Illuminance_measServer            pstZbCluster[4]

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private constants ---------------------------------------------------------*/
/* USER CODE BEGIN PC */

// CO2 cluster: 1 attribute (ppm, uint16)
static const struct ZbZclAttrT co2_attr_list[] = {
  {
    .attributeId = CUSTOM_ATTR_MEAS_VAL,
    .dataType = ZCL_DATATYPE_UNSIGNED_16BIT,
    .flags = ZCL_ATTR_FLAG_REPORTABLE,
    .customValSz = 0,
    .callback = NULL,
    .range = { .min = 0, .max = 10000 },
    .reporting = { .interval_min = 10, .interval_max = 300 },
  },
};

// CO cluster: 1 attribute (ppm, uint16)
static const struct ZbZclAttrT co_attr_list[] = {
  {
    .attributeId = CUSTOM_ATTR_MEAS_VAL,
    .dataType = ZCL_DATATYPE_UNSIGNED_16BIT,
    .flags = ZCL_ATTR_FLAG_REPORTABLE,
    .customValSz = 0,
    .callback = NULL,
    .range = { .min = 0, .max = 1000 },
    .reporting = { .interval_min = 10, .interval_max = 300 },
  },
};

// AQI cluster: 2 attributes (VOC Index + NOx Index, uint16)
static const struct ZbZclAttrT aqi_attr_list[] = {
  {
    .attributeId = CUSTOM_ATTR_MEAS_VAL,
    .dataType = ZCL_DATATYPE_UNSIGNED_16BIT,
    .flags = ZCL_ATTR_FLAG_REPORTABLE,
    .customValSz = 0,
    .callback = NULL,
    .range = { .min = 1, .max = 500 },
    .reporting = { .interval_min = 10, .interval_max = 300 },
  },
  {
    .attributeId = CUSTOM_ATTR_MEAS_VAL_2,
    .dataType = ZCL_DATATYPE_UNSIGNED_16BIT,
    .flags = ZCL_ATTR_FLAG_REPORTABLE,
    .customValSz = 0,
    .callback = NULL,
    .range = { .min = 1, .max = 500 },
    .reporting = { .interval_min = 10, .interval_max = 300 },
  },
};

// PM cluster: 4 attributes (PM1, PM2.5, PM4, PM10 in ug/m3, uint16)
static const struct ZbZclAttrT pm_attr_list[] = {
  {
    .attributeId = CUSTOM_ATTR_MEAS_VAL,
    .dataType = ZCL_DATATYPE_UNSIGNED_16BIT,
    .flags = ZCL_ATTR_FLAG_REPORTABLE,
    .customValSz = 0,
    .callback = NULL,
    .range = { .min = 0, .max = 1000 },
    .reporting = { .interval_min = 10, .interval_max = 300 },
  },
  {
    .attributeId = CUSTOM_ATTR_MEAS_VAL_2,
    .dataType = ZCL_DATATYPE_UNSIGNED_16BIT,
    .flags = ZCL_ATTR_FLAG_REPORTABLE,
    .customValSz = 0,
    .callback = NULL,
    .range = { .min = 0, .max = 1000 },
    .reporting = { .interval_min = 10, .interval_max = 300 },
  },
  {
    .attributeId = CUSTOM_ATTR_MEAS_VAL_3,
    .dataType = ZCL_DATATYPE_UNSIGNED_16BIT,
    .flags = ZCL_ATTR_FLAG_REPORTABLE,
    .customValSz = 0,
    .callback = NULL,
    .range = { .min = 0, .max = 1000 },
    .reporting = { .interval_min = 10, .interval_max = 300 },
  },
  {
    .attributeId = CUSTOM_ATTR_MEAS_VAL_4,
    .dataType = ZCL_DATATYPE_UNSIGNED_16BIT,
    .flags = ZCL_ATTR_FLAG_REPORTABLE,
    .customValSz = 0,
    .callback = NULL,
    .range = { .min = 0, .max = 1000 },
    .reporting = { .interval_min = 10, .interval_max = 300 },
  },
};

// Sound cluster: 1 attribute (0.01 dB, uint16)
static const struct ZbZclAttrT sound_attr_list[] = {
  {
    .attributeId = CUSTOM_ATTR_MEAS_VAL,
    .dataType = ZCL_DATATYPE_UNSIGNED_16BIT,
    .flags = ZCL_ATTR_FLAG_REPORTABLE,
    .customValSz = 0,
    .callback = NULL,
    .range = { .min = 0, .max = 15000 },
    .reporting = { .interval_min = 10, .interval_max = 300 },
  },
};

/* USER CODE END PC */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static struct ZbZclClusterT *pstCo2Cluster;
static struct ZbZclClusterT *pstCoCluster;
static struct ZbZclClusterT *pstAqiCluster;
static struct ZbZclClusterT *pstPmCluster;
static struct ZbZclClusterT *pstSoundCluster;

static UTIL_TIMER_Object_t stSensorUpdateTimer;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */
static void APP_ZIGBEE_SensorUpdateTask(void);
static void APP_ZIGBEE_SensorUpdateTimerCallback(void *arg);
/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/

/**
 * @brief  Zigbee application initialization
 * @param  None
 * @retval None
 */
void APP_ZIGBEE_ApplicationInit(void)
{
  LOG_INFO_APP( "ZIGBEE Application Init" );

  /* Initialization of the Zigbee stack */
  APP_ZIGBEE_Init();

  /* Configure Application Form/Join parameters : Startup, Persistence and Start with/without Form/Join */
  stZigbeeAppInfo.eStartupControl = ZbStartTypeJoin;
  stZigbeeAppInfo.bPersistNotification = false;
  stZigbeeAppInfo.bNwkStartup = true;

  /* USER CODE BEGIN APP_ZIGBEE_ApplicationInit */

  /* USER CODE END APP_ZIGBEE_ApplicationInit */

  /* Initialize Zigbee stack layers */
  APP_ZIGBEE_StackLayersInit();
}

/**
 * @brief  Zigbee application start
 * @param  None
 * @retval None
 */
void APP_ZIGBEE_ApplicationStart( void )
{
  /* USER CODE BEGIN APP_ZIGBEE_ApplicationStart */
  UTIL_SEQ_RegTask(1U << CFG_TASK_ZIGBEE_APP1, UTIL_SEQ_RFU, APP_ZIGBEE_SensorUpdateTask);
  UTIL_TIMER_Create(&stSensorUpdateTimer, SENSOR_UPDATE_PERIOD_MS,
      UTIL_TIMER_PERIODIC, &APP_ZIGBEE_SensorUpdateTimerCallback, NULL);
  UTIL_TIMER_Start(&stSensorUpdateTimer);
  LOG_INFO_APP("Sensor update timer started (%d ms).", SENSOR_UPDATE_PERIOD_MS);
  /* USER CODE END APP_ZIGBEE_ApplicationStart */

#if ( CFG_LPM_LEVEL != 0)
  /* Authorize LowPower now */
  UTIL_LPM_SetStopMode( 1 << CFG_LPM_APP, UTIL_LPM_ENABLE );
#if (CFG_LPM_STDBY_SUPPORTED > 0)
  UTIL_LPM_SetOffMode( 1 << CFG_LPM_APP, UTIL_LPM_ENABLE );
#endif /* CFG_LPM_STDBY_SUPPORTED */
#endif /* CFG_LPM_LEVEL */
}

/**
 * @brief  Configure Zigbee application endpoints
 * @param  None
 * @retval None
 */
void APP_ZIGBEE_ConfigEndpoints(void)
{
  struct ZbApsmeAddEndpointReqT   stRequest;
  struct ZbApsmeAddEndpointConfT  stConfig;
  /* USER CODE BEGIN APP_ZIGBEE_ConfigEndpoints1 */

  /* USER CODE END APP_ZIGBEE_ConfigEndpoints1 */

  /* Add EndPoint */
  memset( &stRequest, 0, sizeof( stRequest ) );
  memset( &stConfig, 0, sizeof( stConfig ) );

  stRequest.profileId = APP_ZIGBEE_PROFILE_ID;
  stRequest.deviceId = APP_ZIGBEE_DEVICE_ID;
  stRequest.endpoint = APP_ZIGBEE_ENDPOINT;
  ZbZclAddEndpoint( stZigbeeAppInfo.pstZigbee, &stRequest, &stConfig );
  assert( stConfig.status == ZB_STATUS_SUCCESS );

  /* Add Identify Server Cluster */
  stZigbeeAppInfo.IdentifyServer = ZbZclIdentifyServerAlloc( stZigbeeAppInfo.pstZigbee, APP_ZIGBEE_ENDPOINT, NULL, NULL );
  assert( stZigbeeAppInfo.IdentifyServer != NULL );
  if ( ZbZclClusterEndpointRegister( stZigbeeAppInfo.IdentifyServer ) == false )
  {
    LOG_ERROR_APP( "Error during Identify Server Endpoint Register." );
  }

  /* Add PressMeas Server Cluster */
  stZigbeeAppInfo.PressMeasServer = ZbZclPressMeasServerAlloc( stZigbeeAppInfo.pstZigbee, APP_ZIGBEE_ENDPOINT, APP_ZIGBEE_PRESS_MIN, APP_ZIGBEE_PRESS_MAX );
  assert( stZigbeeAppInfo.PressMeasServer != NULL );
  if ( ZbZclClusterEndpointRegister( stZigbeeAppInfo.PressMeasServer ) == false )
  {
    LOG_ERROR_APP( "Error during PressMeas Server Endpoint Register." );
  }

  /* Add TempMeas Server Cluster */
  stZigbeeAppInfo.TempMeasServer = ZbZclTempMeasServerAlloc( stZigbeeAppInfo.pstZigbee, APP_ZIGBEE_ENDPOINT, APP_ZIGBEE_TEMP_MIN, APP_ZIGBEE_TEMP_MAX, APP_ZIGBEE_TEMP_TOLERANCE );
  assert( stZigbeeAppInfo.TempMeasServer != NULL );
  if ( ZbZclClusterEndpointRegister( stZigbeeAppInfo.TempMeasServer ) == false )
  {
    LOG_ERROR_APP( "Error during TempMeas Server Endpoint Register." );
  }

  /* Add WaterContent Server Cluster */
  stZigbeeAppInfo.WaterContentServer = ZbZclWaterContentMeasServerAlloc( stZigbeeAppInfo.pstZigbee, APP_ZIGBEE_ENDPOINT, ZCL_CLUSTER_MEAS_HUMIDITY, APP_ZIGBEE_HUMIDITY_MIN, APP_ZIGBEE_HUMIDITY_MAX );
  assert( stZigbeeAppInfo.WaterContentServer != NULL );
  if ( ZbZclClusterEndpointRegister( stZigbeeAppInfo.WaterContentServer ) == false )
  {
    LOG_ERROR_APP( "Error during WaterContent Server Endpoint Register." );
  }

  /* Add Illuminance_meas Server Cluster */
  stZigbeeAppInfo.Illuminance_measServer = ZbZclIllumMeasServerAlloc( stZigbeeAppInfo.pstZigbee, APP_ZIGBEE_ENDPOINT, APP_ZIGBEE_ILLUM_MIN, APP_ZIGBEE_ILLUM_MAX );
  assert( stZigbeeAppInfo.Illuminance_measServer != NULL );
  if ( ZbZclClusterEndpointRegister( stZigbeeAppInfo.Illuminance_measServer ) == false )
  {
    LOG_ERROR_APP( "Error during Illuminance_meas Server Endpoint Register." );
  }

  /* USER CODE BEGIN APP_ZIGBEE_ConfigEndpoints2 */

  // CO2 Measurement cluster (0xFC00)
  pstCo2Cluster = ZbZclClusterAlloc(stZigbeeAppInfo.pstZigbee,
      sizeof(struct ZbZclClusterT), (enum ZbZclClusterIdT)APP_ZIGBEE_CLUSTER_CO2,
      APP_ZIGBEE_ENDPOINT, ZCL_DIRECTION_TO_SERVER);
  assert(pstCo2Cluster != NULL);
  ZbZclClusterSetMfrCode(pstCo2Cluster, APP_ZIGBEE_MFR_CODE);
  ZbZclAttrAppendList(pstCo2Cluster, co2_attr_list, ZCL_ATTR_LIST_LEN(co2_attr_list));
  ZbZclClusterAttach(pstCo2Cluster);
  if (!ZbZclClusterEndpointRegister(pstCo2Cluster)) {
    LOG_ERROR_APP("Error registering CO2 cluster.");
  }

  // CO Measurement cluster (0xFC01)
  pstCoCluster = ZbZclClusterAlloc(stZigbeeAppInfo.pstZigbee,
      sizeof(struct ZbZclClusterT), (enum ZbZclClusterIdT)APP_ZIGBEE_CLUSTER_CO,
      APP_ZIGBEE_ENDPOINT, ZCL_DIRECTION_TO_SERVER);
  assert(pstCoCluster != NULL);
  ZbZclClusterSetMfrCode(pstCoCluster, APP_ZIGBEE_MFR_CODE);
  ZbZclAttrAppendList(pstCoCluster, co_attr_list, ZCL_ATTR_LIST_LEN(co_attr_list));
  ZbZclClusterAttach(pstCoCluster);
  if (!ZbZclClusterEndpointRegister(pstCoCluster)) {
    LOG_ERROR_APP("Error registering CO cluster.");
  }

  // Air Quality Index cluster (0xFC02) — VOC Index + NOx Index
  pstAqiCluster = ZbZclClusterAlloc(stZigbeeAppInfo.pstZigbee,
      sizeof(struct ZbZclClusterT), (enum ZbZclClusterIdT)APP_ZIGBEE_CLUSTER_AQI,
      APP_ZIGBEE_ENDPOINT, ZCL_DIRECTION_TO_SERVER);
  assert(pstAqiCluster != NULL);
  ZbZclClusterSetMfrCode(pstAqiCluster, APP_ZIGBEE_MFR_CODE);
  ZbZclAttrAppendList(pstAqiCluster, aqi_attr_list, ZCL_ATTR_LIST_LEN(aqi_attr_list));
  ZbZclClusterAttach(pstAqiCluster);
  if (!ZbZclClusterEndpointRegister(pstAqiCluster)) {
    LOG_ERROR_APP("Error registering AQI cluster.");
  }

  // Particulate Matter cluster (0xFC03) — PM1, PM2.5, PM4, PM10
  pstPmCluster = ZbZclClusterAlloc(stZigbeeAppInfo.pstZigbee,
      sizeof(struct ZbZclClusterT), (enum ZbZclClusterIdT)APP_ZIGBEE_CLUSTER_PM,
      APP_ZIGBEE_ENDPOINT, ZCL_DIRECTION_TO_SERVER);
  assert(pstPmCluster != NULL);
  ZbZclClusterSetMfrCode(pstPmCluster, APP_ZIGBEE_MFR_CODE);
  ZbZclAttrAppendList(pstPmCluster, pm_attr_list, ZCL_ATTR_LIST_LEN(pm_attr_list));
  ZbZclClusterAttach(pstPmCluster);
  if (!ZbZclClusterEndpointRegister(pstPmCluster)) {
    LOG_ERROR_APP("Error registering PM cluster.");
  }

  // Sound Level cluster (0xFC04)
  pstSoundCluster = ZbZclClusterAlloc(stZigbeeAppInfo.pstZigbee,
      sizeof(struct ZbZclClusterT), (enum ZbZclClusterIdT)APP_ZIGBEE_CLUSTER_SOUND,
      APP_ZIGBEE_ENDPOINT, ZCL_DIRECTION_TO_SERVER);
  assert(pstSoundCluster != NULL);
  ZbZclClusterSetMfrCode(pstSoundCluster, APP_ZIGBEE_MFR_CODE);
  ZbZclAttrAppendList(pstSoundCluster, sound_attr_list, ZCL_ATTR_LIST_LEN(sound_attr_list));
  ZbZclClusterAttach(pstSoundCluster);
  if (!ZbZclClusterEndpointRegister(pstSoundCluster)) {
    LOG_ERROR_APP("Error registering Sound cluster.");
  }

  LOG_INFO_APP("Custom clusters registered (CO2, CO, AQI, PM, Sound).");

  /* USER CODE END APP_ZIGBEE_ConfigEndpoints2 */
}

/**
 * @brief  Set Group Addressing mode (if used)
 * @param  None
 * @retval 'true' if Group Address used else 'false'.
 */
bool APP_ZIGBEE_ConfigGroupAddr( void )
{
  /* Not used */

  return false;
}

/**
 * @brief  Return the Startup Configuration
 * @param  pstConfig  Configuration structure to fill
 * @retval None
 */
void APP_ZIGBEE_GetStartupConfig( struct ZbStartupT * pstConfig )
{
  /* Attempt to join a zigbee network */
  ZbStartupConfigGetProDefaults( pstConfig );

  /* Using the default HA preconfigured Link Key */
  memcpy( pstConfig->security.preconfiguredLinkKey, sec_key_ha, ZB_SEC_KEYSIZE );

  /* Setting up additional startup configuration parameters */
  pstConfig->startupControl = stZigbeeAppInfo.eStartupControl;
  pstConfig->channelList.count = 1;
  pstConfig->channelList.list[0].page = 0;
  pstConfig->channelList.list[0].channelMask = APP_ZIGBEE_CHANNEL_MASK;

  /* Set the TX-Power */
  if ( APP_ZIGBEE_SetTxPower( APP_ZIGBEE_TX_POWER ) == false )
  {
    LOG_ERROR_APP( "Switching to %d dB failed.", APP_ZIGBEE_TX_POWER );
    return;
  }

  /* USER CODE BEGIN APP_ZIGBEE_GetStartupConfig */

  /* USER CODE END APP_ZIGBEE_GetStartupConfig */
}

/**
 * @brief  Manage a New Device on Network (called only if Coord or Router).
 * @param  iShortAddress      Short Address of new Device
 * @param  dlExtendedAddress  Extended Address of new Device
 * @param  cCapability        Capability of new Device
 * @retval Group Address
 */
void APP_ZIGBEE_SetNewDevice( uint16_t iShortAddress, uint64_t dlExtendedAddress, uint8_t cCapability )
{
  LOG_INFO_APP( "New Device (%d) on Network : with Extended ( " LOG_DISPLAY64() " ) and Short ( 0x%04X ) Address.", cCapability, LOG_NUMBER64( dlExtendedAddress ), iShortAddress );

  /* USER CODE BEGIN APP_ZIGBEE_SetNewDevice */

  /* USER CODE END APP_ZIGBEE_SetNewDevice */
}

/**
 * @brief  Print application information to the console
 * @param  None
 * @retval None
 */
void APP_ZIGBEE_PrintApplicationInfo(void)
{
  LOG_INFO_APP( "**********************************************************" );
  LOG_INFO_APP( "Network config : CENTRALIZED END DEVICE" );

  /* USER CODE BEGIN APP_ZIGBEE_PrintApplicationInfo1 */

  /* USER CODE END APP_ZIGBEE_PrintApplicationInfo1 */
  LOG_INFO_APP( "Channel used: %d.", APP_ZIGBEE_CHANNEL );

  APP_ZIGBEE_PrintGenericInfo();

  LOG_INFO_APP( "Clusters allocated are:" );
  LOG_INFO_APP( "  %s on Endpoint %d.", APP_ZIGBEE_CLUSTER1_NAME, APP_ZIGBEE_ENDPOINT );
  LOG_INFO_APP( "  %s on Endpoint %d.", APP_ZIGBEE_CLUSTER2_NAME, APP_ZIGBEE_ENDPOINT );
  LOG_INFO_APP( "  %s on Endpoint %d.", APP_ZIGBEE_CLUSTER3_NAME, APP_ZIGBEE_ENDPOINT );
  LOG_INFO_APP( "  %s on Endpoint %d.", APP_ZIGBEE_CLUSTER4_NAME, APP_ZIGBEE_ENDPOINT );
  LOG_INFO_APP( "  %s on Endpoint %d.", APP_ZIGBEE_CLUSTER5_NAME, APP_ZIGBEE_ENDPOINT );

  /* USER CODE BEGIN APP_ZIGBEE_PrintApplicationInfo2 */
  LOG_INFO_APP( "  CO2 Measurement (0x%04X) on Endpoint %d.", APP_ZIGBEE_CLUSTER_CO2, APP_ZIGBEE_ENDPOINT );
  LOG_INFO_APP( "  CO Measurement (0x%04X) on Endpoint %d.", APP_ZIGBEE_CLUSTER_CO, APP_ZIGBEE_ENDPOINT );
  LOG_INFO_APP( "  Air Quality Index (0x%04X) on Endpoint %d.", APP_ZIGBEE_CLUSTER_AQI, APP_ZIGBEE_ENDPOINT );
  LOG_INFO_APP( "  Particulate Matter (0x%04X) on Endpoint %d.", APP_ZIGBEE_CLUSTER_PM, APP_ZIGBEE_ENDPOINT );
  LOG_INFO_APP( "  Sound Level (0x%04X) on Endpoint %d.", APP_ZIGBEE_CLUSTER_SOUND, APP_ZIGBEE_ENDPOINT );
  /* USER CODE END APP_ZIGBEE_PrintApplicationInfo2 */

  LOG_INFO_APP( "**********************************************************" );
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS */

static void APP_ZIGBEE_SensorUpdateTimerCallback(void *arg) {
  UNUSED(arg);
  UTIL_SEQ_SetTask(1U << CFG_TASK_ZIGBEE_APP1, CFG_SEQ_PRIO_1);
}

static void APP_ZIGBEE_SensorUpdateTask(void) {
  // Standard clusters
  ZbZclAttrIntegerWrite(stZigbeeAppInfo.TempMeasServer,
      ZCL_TEMP_MEAS_ATTR_MEAS_VAL, sensor_get_temperature());
  ZbZclAttrIntegerWrite(stZigbeeAppInfo.PressMeasServer,
      ZCL_PRESS_MEAS_ATTR_MEAS_VAL, sensor_get_pressure());
  ZbZclAttrIntegerWrite(stZigbeeAppInfo.WaterContentServer,
      ZCL_WC_MEAS_ATTR_MEAS_VAL, sensor_get_humidity());
  ZbZclAttrIntegerWrite(stZigbeeAppInfo.Illuminance_measServer,
      ZCL_ILLUM_MEAS_ATTR_MEAS_VAL, sensor_get_illuminance());

  // Custom clusters
  ZbZclAttrIntegerWrite(pstCo2Cluster, CUSTOM_ATTR_MEAS_VAL, sensor_get_co2());
  ZbZclAttrIntegerWrite(pstCoCluster, CUSTOM_ATTR_MEAS_VAL, sensor_get_co());
  ZbZclAttrIntegerWrite(pstAqiCluster, CUSTOM_ATTR_MEAS_VAL, sensor_get_voc_index());
  ZbZclAttrIntegerWrite(pstAqiCluster, CUSTOM_ATTR_MEAS_VAL_2, sensor_get_nox_index());
  ZbZclAttrIntegerWrite(pstPmCluster, CUSTOM_ATTR_MEAS_VAL, sensor_get_pm1());
  ZbZclAttrIntegerWrite(pstPmCluster, CUSTOM_ATTR_MEAS_VAL_2, sensor_get_pm25());
  ZbZclAttrIntegerWrite(pstPmCluster, CUSTOM_ATTR_MEAS_VAL_3, sensor_get_pm4());
  ZbZclAttrIntegerWrite(pstPmCluster, CUSTOM_ATTR_MEAS_VAL_4, sensor_get_pm10());
  ZbZclAttrIntegerWrite(pstSoundCluster, CUSTOM_ATTR_MEAS_VAL, sensor_get_sound());

  LOG_INFO_APP("Sensor update complete.");
}

/* USER CODE END FD_LOCAL_FUNCTIONS */
