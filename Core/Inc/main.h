/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

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
#define J1_LIM_Pin GPIO_PIN_7
#define J1_LIM_GPIO_Port GPIOE
#define J1_LIM_EXTI_IRQn EXTI9_5_IRQn
#define J2_LIM_Pin GPIO_PIN_8
#define J2_LIM_GPIO_Port GPIOE
#define J2_LIM_EXTI_IRQn EXTI9_5_IRQn
#define J3_Lim_Pin GPIO_PIN_9
#define J3_Lim_GPIO_Port GPIOE
#define J3_Lim_EXTI_IRQn EXTI9_5_IRQn
#define J4_Lim_Pin GPIO_PIN_10
#define J4_Lim_GPIO_Port GPIOE
#define J4_Lim_EXTI_IRQn EXTI15_10_IRQn
#define J5_LIM_Pin GPIO_PIN_11
#define J5_LIM_GPIO_Port GPIOE
#define J5_LIM_EXTI_IRQn EXTI15_10_IRQn
#define J6_LIM_Pin GPIO_PIN_12
#define J6_LIM_GPIO_Port GPIOE
#define J6_LIM_EXTI_IRQn EXTI15_10_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
