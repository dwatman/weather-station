/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_crs.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_system.h"
#include "stm32h7xx_ll_exti.h"
#include "stm32h7xx_ll_cortex.h"
#include "stm32h7xx_ll_utils.h"
#include "stm32h7xx_ll_pwr.h"
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_rtc.h"
#include "stm32h7xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define VFP 12
#define VSYNC 10
#define HFP 160
#define VACT 600
#define VBP 23
#define HBP 160
#define HACT 1024
#define HSYNC 70
#define D28_Pin LL_GPIO_PIN_6
#define D28_GPIO_Port GPIOI
#define FMC_NBL3_Pin LL_GPIO_PIN_5
#define FMC_NBL3_GPIO_Port GPIOI
#define FMC_NBL2_Pin LL_GPIO_PIN_4
#define FMC_NBL2_GPIO_Port GPIOI
#define D25_Pin LL_GPIO_PIN_1
#define D25_GPIO_Port GPIOI
#define D24_Pin LL_GPIO_PIN_0
#define D24_GPIO_Port GPIOI
#define D29_Pin LL_GPIO_PIN_7
#define D29_GPIO_Port GPIOI
#define FMC_NBL1_Pin LL_GPIO_PIN_1
#define FMC_NBL1_GPIO_Port GPIOE
#define D26_Pin LL_GPIO_PIN_2
#define D26_GPIO_Port GPIOI
#define D23_Pin LL_GPIO_PIN_15
#define D23_GPIO_Port GPIOH
#define D22_Pin LL_GPIO_PIN_14
#define D22_GPIO_Port GPIOH
#define USB1_OVERCURRENT_Pin LL_GPIO_PIN_15
#define USB1_OVERCURRENT_GPIO_Port GPIOC
#define FMC_NBL0_Pin LL_GPIO_PIN_0
#define FMC_NBL0_GPIO_Port GPIOE
#define D27__IS42S32800G_DQ27_Pin LL_GPIO_PIN_3
#define D27__IS42S32800G_DQ27_GPIO_Port GPIOI
#define SDNCAS_Pin LL_GPIO_PIN_15
#define SDNCAS_GPIO_Port GPIOG
#define D2_Pin LL_GPIO_PIN_0
#define D2_GPIO_Port GPIOD
#define D21_Pin LL_GPIO_PIN_13
#define D21_GPIO_Port GPIOH
#define D30_Pin LL_GPIO_PIN_9
#define D30_GPIO_Port GPIOI
#define D3_Pin LL_GPIO_PIN_1
#define D3_GPIO_Port GPIOD
#define MCO1_Pin LL_GPIO_PIN_8
#define MCO1_GPIO_Port GPIOA
#define D31_Pin LL_GPIO_PIN_10
#define D31_GPIO_Port GPIOI
#define USR_BTN_1_Pin LL_GPIO_PIN_6
#define USR_BTN_1_GPIO_Port GPIOC
#define SDCLK_Pin LL_GPIO_PIN_8
#define SDCLK_GPIO_Port GPIOG
#define A2_Pin LL_GPIO_PIN_2
#define A2_GPIO_Port GPIOF
#define A1_Pin LL_GPIO_PIN_1
#define A1_GPIO_Port GPIOF
#define A0_Pin LL_GPIO_PIN_0
#define A0_GPIO_Port GPIOF
#define A3_Pin LL_GPIO_PIN_3
#define A3_GPIO_Port GPIOF
#define OSC_IN_Pin LL_GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOH
#define A5_Pin LL_GPIO_PIN_5
#define A5_GPIO_Port GPIOF
#define A4_Pin LL_GPIO_PIN_4
#define A4_GPIO_Port GPIOF
#define USB1_EN_Pin LL_GPIO_PIN_10
#define USB1_EN_GPIO_Port GPIOF
#define USR_LED_Pin LL_GPIO_PIN_10
#define USR_LED_GPIO_Port GPIOJ
#define D7_Pin LL_GPIO_PIN_10
#define D7_GPIO_Port GPIOE
#define SDNWE_Pin LL_GPIO_PIN_5
#define SDNWE_GPIO_Port GPIOH
#define A7_Pin LL_GPIO_PIN_13
#define A7_GPIO_Port GPIOF
#define A8_Pin LL_GPIO_PIN_14
#define A8_GPIO_Port GPIOF
#define D6_Pin LL_GPIO_PIN_9
#define D6_GPIO_Port GPIOE
#define D8_Pin LL_GPIO_PIN_11
#define D8_GPIO_Port GPIOE
#define D18_Pin LL_GPIO_PIN_10
#define D18_GPIO_Port GPIOH
#define D19_Pin LL_GPIO_PIN_11
#define D19_GPIO_Port GPIOH
#define D1_Pin LL_GPIO_PIN_15
#define D1_GPIO_Port GPIOD
#define D0_Pin LL_GPIO_PIN_14
#define D0_GPIO_Port GPIOD
#define LCD_DISP_Pin LL_GPIO_PIN_6
#define LCD_DISP_GPIO_Port GPIOA
#define A6_Pin LL_GPIO_PIN_12
#define A6_GPIO_Port GPIOF
#define A9_Pin LL_GPIO_PIN_15
#define A9_GPIO_Port GPIOF
#define D9_Pin LL_GPIO_PIN_12
#define D9_GPIO_Port GPIOE
#define D12_Pin LL_GPIO_PIN_15
#define D12_GPIO_Port GPIOE
#define D17_Pin LL_GPIO_PIN_9
#define D17_GPIO_Port GPIOH
#define D20_Pin LL_GPIO_PIN_12
#define D20_GPIO_Port GPIOH
#define SNDRAS_Pin LL_GPIO_PIN_11
#define SNDRAS_GPIO_Port GPIOF
#define A10_Pin LL_GPIO_PIN_0
#define A10_GPIO_Port GPIOG
#define D5_Pin LL_GPIO_PIN_8
#define D5_GPIO_Port GPIOE
#define D10_Pin LL_GPIO_PIN_13
#define D10_GPIO_Port GPIOE
#define SDNE1_Pin LL_GPIO_PIN_6
#define SDNE1_GPIO_Port GPIOH
#define D16_Pin LL_GPIO_PIN_8
#define D16_GPIO_Port GPIOH
#define D15_Pin LL_GPIO_PIN_10
#define D15_GPIO_Port GPIOD
#define D14_Pin LL_GPIO_PIN_9
#define D14_GPIO_Port GPIOD
#define A11_Pin LL_GPIO_PIN_1
#define A11_GPIO_Port GPIOG
#define D4_Pin LL_GPIO_PIN_7
#define D4_GPIO_Port GPIOE
#define D11_Pin LL_GPIO_PIN_14
#define D11_GPIO_Port GPIOE
#define SDCKE1_Pin LL_GPIO_PIN_7
#define SDCKE1_GPIO_Port GPIOH
#define D13_Pin LL_GPIO_PIN_8
#define D13_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
