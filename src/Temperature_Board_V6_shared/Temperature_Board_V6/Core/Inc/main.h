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
#include "stm32l4xx_hal.h"

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
#define MUX2_EN_Pin GPIO_PIN_4
#define MUX2_EN_GPIO_Port GPIOA
#define MUX2_SEL_Pin GPIO_PIN_5
#define MUX2_SEL_GPIO_Port GPIOA
#define ADC2_NDRDY_Pin GPIO_PIN_6
#define ADC2_NDRDY_GPIO_Port GPIOA
#define ADC2_NDRDY_EXTI_IRQn EXTI9_5_IRQn
#define ADC2_NRST_Pin GPIO_PIN_7
#define ADC2_NRST_GPIO_Port GPIOA
#define PCB_TEMP_Pin GPIO_PIN_0
#define PCB_TEMP_GPIO_Port GPIOB
#define RGB_LED_G_Pin GPIO_PIN_1
#define RGB_LED_G_GPIO_Port GPIOB
#define RGB_LED_R_Pin GPIO_PIN_8
#define RGB_LED_R_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define RGB_LED_B_Pin GPIO_PIN_15
#define RGB_LED_B_GPIO_Port GPIOA
#define TRACE_SWO_Pin GPIO_PIN_3
#define TRACE_SWO_GPIO_Port GPIOB
#define MUX1_EN_Pin GPIO_PIN_4
#define MUX1_EN_GPIO_Port GPIOB
#define MUX1_SEL_Pin GPIO_PIN_5
#define MUX1_SEL_GPIO_Port GPIOB
#define ADC1_NRST_Pin GPIO_PIN_6
#define ADC1_NRST_GPIO_Port GPIOB
#define ADC1_NDRDY_Pin GPIO_PIN_7
#define ADC1_NDRDY_GPIO_Port GPIOB
#define ADC1_NDRDY_EXTI_IRQn EXTI9_5_IRQn
#define BOOT0_Pin GPIO_PIN_3
#define BOOT0_GPIO_Port GPIOH

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
