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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define GPIO_IN2_Pin GPIO_PIN_2
#define GPIO_IN2_GPIO_Port GPIOA
#define GPIO_IN5_Pin GPIO_PIN_5
#define GPIO_IN5_GPIO_Port GPIOA
#define GPIO_IN0_Pin GPIO_PIN_0
#define GPIO_IN0_GPIO_Port GPIOA
#define GPIO_IN3_Pin GPIO_PIN_3
#define GPIO_IN3_GPIO_Port GPIOA
#define GPIO_IN6_Pin GPIO_PIN_6
#define GPIO_IN6_GPIO_Port GPIOA
#define GPIO_IN1_Pin GPIO_PIN_1
#define GPIO_IN1_GPIO_Port GPIOA
#define GPIO_IN4_Pin GPIO_PIN_4
#define GPIO_IN4_GPIO_Port GPIOA
#define GPIO_IN7_Pin GPIO_PIN_7
#define GPIO_IN7_GPIO_Port GPIOA

#define GPIO_OUT0_Pin GPIO_PIN_7
#define GPIO_OUT0_GPIO_Port GPIOE
#define GPIO_OUT1_Pin GPIO_PIN_8
#define GPIO_OUT1_GPIO_Port GPIOE
#define GPIO_OUT2_Pin GPIO_PIN_9
#define GPIO_OUT2_GPIO_Port GPIOE
#define GPIO_OUT3_Pin GPIO_PIN_10
#define GPIO_OUT3_GPIO_Port GPIOE
#define GPIO_OUT4_Pin GPIO_PIN_11
#define GPIO_OUT4_GPIO_Port GPIOE
#define GPIO_OUT5_Pin GPIO_PIN_12
#define GPIO_OUT5_GPIO_Port GPIOE
#define GPIO_OUT6_Pin GPIO_PIN_13
#define GPIO_OUT6_GPIO_Port GPIOE
#define GPIO_OUT7_Pin GPIO_PIN_14
#define GPIO_OUT7_GPIO_Port GPIOE
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

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
