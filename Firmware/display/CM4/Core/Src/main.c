/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "mdma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "circbuf.h"
#include "usart_util.h"
#include "network.h"
#include "wifi_sm.h"
#include "opt4001.h"
#include "lps25hb.h"
#include "shared.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern volatile uint32_t flag_update;
extern volatile uint32_t flag_status;
extern volatile uint32_t flag_mqtt;

extern I2c_Status_t i2c4_status;

extern volatile uint32_t opt4001_data_available;
extern volatile uint32_t lps25hb_data_available;

extern volatile uint32_t opt4001_newdata;
extern volatile uint32_t lps25hb_newdata;

extern CircularBuffer_t tx1Buf;

extern volatile uint32_t RxNewData;
uint32_t newRx1Message = 0;
//extern volatile uint32_t RxBurstEnd;

NinaMessage_t NinaMessage;

SharedData_t *sharedMem = SHARED_DATA_PTR;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
  /*HW semaphore Clock enable*/
  __HAL_RCC_HSEM_CLK_ENABLE();
  /* Activate HSEM notification for Cortex-M4*/
  HAL_HSEM_ActivateNotification(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));
  /*
  Domain D2 goes to STOP mode (Cortex-M4 in deep-sleep) waiting for Cortex-M7 to
  perform system initialization (system clock config, external memory configuration.. )
  */
  HAL_PWREx_ClearPendingEvent();
  HAL_PWREx_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFE, PWR_D2_DOMAIN);
  /* Clear HSEM flag */
  __HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_0));

/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_MDMA_Init();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C4_Init();
  MX_TIM15_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
  // Backlight PWM
  LL_TIM_OC_SetCompareCH1(TIM15, 500);
  LL_TIM_CC_EnableChannel(TIM15, LL_TIM_CHANNEL_CH1);
  LL_TIM_EnableAllOutputs(TIM15);
  LL_TIM_EnableCounter(TIM15);

  // Ensure semaphore flag is clear
  __HAL_HSEM_CLEAR_FLAG(__HAL_HSEM_SEMID_TO_MASK(HSEM_ID_1));

  // Initialise UARTs
  initUsart1();
  initUsart4();
  i2c4_status.state = I2C_STATE_IDLE;

  // Initialise WiFi state machine
  wifi_sm_init();

  // Initialise ambient light sensor
  opt4001_init();

  // Initialise pressure sensor
  lps25hb_init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("\nStart\n");

  // Set initial data values
  sharedMem->time = 0;
  sharedMem->t1 = -10.0f;
  sharedMem->t2 = -20.0f;
  sharedMem->h1 = 0.0f;
  sharedMem->h2 = 0.0f;
  sharedMem->pres = 1000.0f;
  sharedMem->lux = 0.0f;
  sharedMem->rssi = 0;
  sharedMem->soil1 = 0;
  sharedMem->soil2 = 20;
  sharedMem->soil3 = 40;
  sharedMem->soil4 = 60;
  sharedMem->status = 0;

  // Enable ALS interrupt
  LL_C2_EXTI_EnableIT_0_31(LL_EXTI_LINE_4); // _C2 for M4 core
  // Enable pressure interrupt
  LL_C2_EXTI_EnableIT_0_31(LL_EXTI_LINE_11); // _C2 for M4 core


  printfCircBuf(&tx1Buf, "\r\n");
  HAL_Delay(100);

  //printfCircBuf(&tx1Buf, "ATE1\r\n"); // Set echo on
  //HAL_Delay(100);

  printfCircBuf(&tx1Buf, "AT+UDWS=3,1\r\n"); // Enable WiFi watchdog
  HAL_Delay(100);
  emptyRx1Buffer();

//
//
//  printfCircBuf(&tx1Buf, "AT+UWSSTAT=3\r\n"); // Get WiFi connection status
//  HAL_Delay(100);

/*
  // Connect to MQTT
  printfCircBuf(&tx1Buf, "AT+UDCP=at-mqtt://192.168.0.200:1883/?client=NINA-W132&user=mqtt_user&passwd=MQ.jaygram&pt=test&st=display&encr=0&qos=0\r\n");
  HAL_Delay(100);
*/
  lps25hb_data_available = 1; // First trigger to get it started

  uint16_t backlight = 0;
  while (1) {
	// Read ALS data from sensor
	if ((opt4001_data_available) && (i2c4_status.state == I2C_STATE_IDLE)) {
		opt4001_data_available = 0;
		opt4001_StartRead(&i2c4_status);
	}
	// Read pressure data from sensor
	if ((lps25hb_data_available) && (i2c4_status.state == I2C_STATE_IDLE)) {
		lps25hb_data_available = 0;
		lps25hb_StartRead(&i2c4_status);
	}

	// New ambient light data
	if (opt4001_newdata) {
		opt4001_newdata = 0;

		float lux = opt4001_Convert();
		//printf("lux: %.1f\n", lux);

		sharedMem->lux = lux;
	}

	// New pressure data
	if (lps25hb_newdata) {
		lps25hb_newdata = 0;

		float hpa = lps25hb_Convert();
		//printf("press: %.1f\n", hpa);

		sharedMem->pres = hpa;
	}

	// End of RX data burst on USART1
	if (RxNewData) {
		newRx1Message = getNinaMsg(&NinaMessage);

		if (newRx1Message) processNinaMsg(&NinaMessage);

		// No full message in the buffer, don't check again until new data arrives
		if (newRx1Message == 0)
			RxNewData = 0;
	}

	// WiFi state machine
	wifi_state_machine_step();

	// Update data
	if (flag_update) {
		flag_update = 0;

//		if (backlight < 500)
//			backlight++;
//		else
//			backlight = 0;
		backlight = 100;

		sharedMem->bl = backlight;
		LL_TIM_OC_SetCompareCH1(TIM15, backlight);

		if (wifi_ctx.state == SM_OPERATIONAL)
			sharedMem->status |= STATUS_CONNECTED;
		else
			sharedMem->status &= ~STATUS_CONNECTED;

		// move variables for testing
		sharedMem->t1 += 0.1f;
		sharedMem->t2 += 0.2f;
		sharedMem->h1 = (sharedMem->h1 == 100) ? 0 : sharedMem->h1 + 0.2;
		sharedMem->h2 = (sharedMem->h2 == 100) ? 0 : sharedMem->h2 + 0.3;
		sharedMem->soil1 = (sharedMem->soil1 == 100) ? 0 : sharedMem->soil1 + 1;
		sharedMem->soil2 = (sharedMem->soil2 == 100) ? 0 : sharedMem->soil2 + 1;
		sharedMem->soil3 = (sharedMem->soil3 == 100) ? 0 : sharedMem->soil3 + 1;
		sharedMem->soil4 = (sharedMem->soil4 == 100) ? 0 : sharedMem->soil4 + 1;

		// Signal data ready via HSEM1
		HAL_HSEM_FastTake(HSEM_ID_1);   // Take
		HAL_HSEM_Release(HSEM_ID_1, 0); // Release to interrupt to M7
	}

	// Check status
	if (flag_status) {
		flag_status = 0;

		if (wifi_ctx.state == SM_OPERATIONAL) {
			printfCircBuf(&tx1Buf, "AT+UWSSTAT=6\r\n"); // Get WiFi RSSI
		}
	}

	// Send MQTT message
	if (flag_mqtt) {
		flag_mqtt = 0;

		if (wifi_ctx.state == SM_OPERATIONAL) {
			printf("Sent MQTT data\n");
			printfCircBuf(&tx1Buf, "AT+UDATW=%d,0,testdata\r\n", wifi_ctx.peer_handle);
		}
	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  LL_RCC_PLL2R_Enable();
  LL_RCC_PLL2_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_4_8);
  LL_RCC_PLL2_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);
  LL_RCC_PLL2_SetM(4);
  LL_RCC_PLL2_SetN(180);
  LL_RCC_PLL2_SetP(4);
  LL_RCC_PLL2_SetQ(2);
  LL_RCC_PLL2_SetR(3);
  LL_RCC_PLL2_SetFRACN(3072);
  LL_RCC_PLL2FRACN_Enable();
  LL_RCC_PLL2_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL2_IsReady() != 1)
  {
  }

  LL_RCC_PLL3R_Enable();
  LL_RCC_PLL3_SetVCOInputRange(LL_RCC_PLLINPUTRANGE_1_2);
  LL_RCC_PLL3_SetVCOOutputRange(LL_RCC_PLLVCORANGE_WIDE);
  LL_RCC_PLL3_SetM(12);
  LL_RCC_PLL3_SetN(231);
  LL_RCC_PLL3_SetP(6);
  LL_RCC_PLL3_SetQ(6);
  LL_RCC_PLL3_SetR(6);
  LL_RCC_PLL3_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL3_IsReady() != 1)
  {
  }

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
