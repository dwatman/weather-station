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
#include "ir_decode.h"
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
extern CircularBuffer_t rx1Buf;

extern volatile uint32_t RxNewData;
uint32_t newRx1Message = 0;
//extern volatile uint32_t RxBurstEnd;

extern uint8_t wifi_up;
extern uint8_t network_up;
extern uint8_t peer_up;

uint8_t recv_timeout = 0;
uint8_t timeouts = 0;

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
  sharedMem->t1 = 0.0f;
  sharedMem->t2 = 0.0f;
  sharedMem->h1 = 0.0f;
  sharedMem->h2 = 0.0f;
  sharedMem->pres = 1000.0f;
  sharedMem->lux = 0.0f;
  sharedMem->rssi = 0;
  sharedMem->soil[0] = 0;
  sharedMem->soil[1] = 0;
  sharedMem->soil[2] = 0;
  sharedMem->soil[3] = 0;
  sharedMem->soil[4] = 0;
  sharedMem->soil[5] = 0;
  sharedMem->status = 0;

  // Enable ALS interrupt
  LL_C2_EXTI_EnableIT_0_31(LL_EXTI_LINE_4); // _C2 for M4 core
  // Enable pressure interrupt
  LL_C2_EXTI_EnableIT_0_31(LL_EXTI_LINE_11); // _C2 for M4 core

  // Initialize IR decoder (starts DMA capture)
  IR_Decoder_Init(&ir_decode);

  printfCircBuf(&tx1Buf, "\r\n");
  HAL_Delay(100);

  printfCircBuf(&tx1Buf, "AT+UDWS=3,1\r\n"); // Enable WiFi watchdog
  HAL_Delay(100);
  emptyRx1Buffer();

  lps25hb_data_available = 1; // First trigger to get it started
  int restarted = 1;
  uint16_t backlight = 100;
  float backlight_tgt = 100.0f;
  float pressure_filt = 1000.0f;
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
		//printf("lux: %.3f\n", lux);

		// Set backlight target brightness (filtered)
		// Linear brightness up to 1000 lux
		if (restarted)
			backlight_tgt = lux*4;
		else
			backlight_tgt = 0.95f*backlight_tgt + 0.05f*lux*4;

		// Max PWM value is 3999
		if (backlight_tgt > 3999.0f) backlight_tgt = 3999.0f;
		//printf("BL tgt: %.3f\n", backlight_tgt);

		// Round to 3 decimal places
		sharedMem->lux = roundf(1000*lux)*0.001;
	}

	// New pressure data
	if (lps25hb_newdata) {
		lps25hb_newdata = 0;

		float hpa = lps25hb_Convert();
		//printf("press: %.1f\n", hpa);

		// Filter pressure data
		if (restarted)
			pressure_filt = hpa;
		else
			pressure_filt = 0.95f*pressure_filt + 0.05f*hpa;

		// Round to 1 decimal place
		sharedMem->pres = 0.1f*roundf(10*pressure_filt);
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

		// Slowly adjust backlight towards target
		if (backlight < roundf(backlight_tgt)) backlight++;
		if (backlight > roundf(backlight_tgt)) backlight--;

		sharedMem->bl = backlight;
		LL_TIM_OC_SetCompareCH1(TIM15, backlight);

		if (wifi_ctx.state == SM_OPERATIONAL)
			sharedMem->status |= STATUS_CONNECTED;
		else
			sharedMem->status &= ~STATUS_CONNECTED;

		// Signal data ready via HSEM1
		HAL_HSEM_FastTake(HSEM_ID_1);   // Take
		HAL_HSEM_Release(HSEM_ID_1, 0); // Release to interrupt to M7
	}

	// Check status
	if (flag_status) {
		flag_status = 0;

		if (wifi_ctx.state == SM_OPERATIONAL) {
			restarted = 0; // Clear after a few seconds
			printfCircBuf(&tx1Buf, "AT+UWSSTAT=6\r\n"); // Get WiFi RSSI
		}
		else
			restarted = 1;
	}

	// Send MQTT message
	if (flag_mqtt) {
		flag_mqtt = 0;

		printf("wifi %u, network %u, peer %u, inbuf %lu timeouts %u (%u)\n", wifi_up, network_up, peer_up, inCircBuf(&rx1Buf), timeouts, recv_timeout);

		// Detect if we stopped getting MQTT messages
		if (recv_timeout >= 3) {
			printf("***MQTT receive timeout***\n");
			printfCircBuf(&tx1Buf, "AT+UDATR=%i,2,%i\r\n", wifi_ctx.peer_handle, 0); // Request 0 data to get a buffer update
			recv_timeout = 0;
			timeouts++;
		}
		else
			recv_timeout++;

		if ((wifi_ctx.state == SM_OPERATIONAL) && (wifi_ctx.data_send_state == DATA_SEND_IDLE)) {
			//printf("Sending MQTT data\n");
			sendMqttData(sharedMem->pres, sharedMem->lux);
		}
	}

 	// Check for new IR code
    if (ir_decode.new_data_available) {
		ir_decode.new_data_available = false;

		bool frame_ok = IR_Decode_Frame(&ir_decode);
		if (frame_ok) {
			//printf("Decoded code: 0x%08lX\n", ir_decode.decoded_code);
			bool code_ok = IR_CheckAndDecode(&ir_decode);
			if (code_ok) {
				if ((ir_decode.decoded_address == 0x707) && (ir_decode.decoded_command == 0x46)) {
					// Reset wifi
					wifi_sm_init();
				}
			}
		}
		else
			printf("Decode error\n");

		// Restart DMA after processing
		IR_Start_DMA_Capture(&ir_decode);
		ir_decode.dma_active = 0;
		ir_decode.overflow_count = 0;
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
