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

/* USER CODE END PC */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */

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

  /* USER CODE END APP_ZIGBEE_PrintApplicationInfo2 */

  LOG_INFO_APP( "**********************************************************" );
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS */

/* USER CODE END FD_LOCAL_FUNCTIONS */
