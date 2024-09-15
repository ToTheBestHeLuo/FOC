/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

#include "stm32g4xx_ll_adc.h"
#include "stm32g4xx_ll_comp.h"
#include "stm32g4xx_ll_exti.h"
#include "stm32g4xx_ll_cordic.h"
#include "stm32g4xx_ll_dac.h"
#include "stm32g4xx_ll_dma.h"
#include "stm32g4xx_ll_i2c.h"
#include "stm32g4xx_ll_rcc.h"
#include "stm32g4xx_ll_bus.h"
#include "stm32g4xx_ll_crs.h"
#include "stm32g4xx_ll_system.h"
#include "stm32g4xx_ll_cortex.h"
#include "stm32g4xx_ll_utils.h"
#include "stm32g4xx_ll_pwr.h"
#include "stm32g4xx_ll_rng.h"
#include "stm32g4xx_ll_spi.h"
#include "stm32g4xx_ll_tim.h"
#include "stm32g4xx_ll_usart.h"
#include "stm32g4xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "SEGGER_RTT.h"
#define __FPU_PRESENT 1U
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
extern volatile uint16_t tempDat;
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GPIO_LED_GREEN_Pin LL_GPIO_PIN_13
#define GPIO_LED_GREEN_GPIO_Port GPIOC
#define GPIO_LED_RED_Pin LL_GPIO_PIN_0
#define GPIO_LED_RED_GPIO_Port GPIOC
#define ADC1_VoltageA_Pin LL_GPIO_PIN_2
#define ADC1_VoltageA_GPIO_Port GPIOC
#define ADC2_VoltageB_Pin LL_GPIO_PIN_3
#define ADC2_VoltageB_GPIO_Port GPIOC
#define ADC2_VoltageC_Pin LL_GPIO_PIN_1
#define ADC2_VoltageC_GPIO_Port GPIOA
#define TIM3_Zindex_Pin LL_GPIO_PIN_2
#define TIM3_Zindex_GPIO_Port GPIOD
#define TIM3_Zindex_EXTI_IRQn EXTI2_IRQn
#define GPIO_MC_SS_Pin LL_GPIO_PIN_6
#define GPIO_MC_SS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
