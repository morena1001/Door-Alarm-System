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
#include "stm32f3xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define NP1_Pin GPIO_PIN_1
#define NP1_GPIO_Port GPIOC
#define NP2_Pin GPIO_PIN_2
#define NP2_GPIO_Port GPIOC
#define NP3_Pin GPIO_PIN_3
#define NP3_GPIO_Port GPIOC
#define RM_Pin GPIO_PIN_5
#define RM_GPIO_Port GPIOA
#define SM_Pin GPIO_PIN_6
#define SM_GPIO_Port GPIOA
#define NP4_Pin GPIO_PIN_4
#define NP4_GPIO_Port GPIOC
#define NP5_Pin GPIO_PIN_5
#define NP5_GPIO_Port GPIOC
#define LUB_Pin GPIO_PIN_0
#define LUB_GPIO_Port GPIOB
#define LUB_EXTI_IRQn EXTI0_IRQn
#define RPB_Pin GPIO_PIN_1
#define RPB_GPIO_Port GPIOB
#define RPB_EXTI_IRQn EXTI1_IRQn
#define NP6_Pin GPIO_PIN_6
#define NP6_GPIO_Port GPIOC
#define NP7_Pin GPIO_PIN_7
#define NP7_GPIO_Port GPIOC
#define NP8_Pin GPIO_PIN_8
#define NP8_GPIO_Port GPIOC
#define NP9_Pin GPIO_PIN_9
#define NP9_GPIO_Port GPIOC
#define NP0_Pin GPIO_PIN_10
#define NP0_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
