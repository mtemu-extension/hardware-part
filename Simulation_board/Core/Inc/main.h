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

//макросы выходов последовательного интерфейса GPIOA
#define GPIO_A0_Pin GPIO_PIN_0
#define GPIO_A0_GPIO_Port GPIOA
#define GPIO_A1_Pin GPIO_PIN_1
#define GPIO_A1_GPIO_Port GPIOA
#define GPIO_A2_Pin GPIO_PIN_2
#define GPIO_A2_GPIO_Port GPIOA
#define GPIO_A3_Pin GPIO_PIN_3
#define GPIO_A3_GPIO_Port GPIOA
#define GPIO_A4_Pin GPIO_PIN_4
#define GPIO_A4_GPIO_Port GPIOA
#define GPIO_A5_Pin GPIO_PIN_5
#define GPIO_A5_GPIO_Port GPIOA
#define GPIO_A6_Pin GPIO_PIN_6
#define GPIO_A6_GPIO_Port GPIOA
#define GPIO_A7_Pin GPIO_PIN_7
#define GPIO_A7_GPIO_Port GPIOA


//макросы выходов последовательного интерфейса GPIOE
#define GPIO_E0_Pin GPIO_PIN_7
#define GPIO_E0_GPIO_Port GPIOE
#define GPIO_E1_Pin GPIO_PIN_8
#define GPIO_E1_GPIO_Port GPIOE
#define GPIO_E2_Pin GPIO_PIN_9
#define GPIO_E2_GPIO_Port GPIOE
#define GPIO_E3_Pin GPIO_PIN_10
#define GPIO_E3_GPIO_Port GPIOE
#define GPIO_E4_Pin GPIO_PIN_11
#define GPIO_E4_GPIO_Port GPIOE
#define GPIO_E5_Pin GPIO_PIN_12
#define GPIO_E5_GPIO_Port GPIOE
#define GPIO_E6_Pin GPIO_PIN_13
#define GPIO_E6_GPIO_Port GPIOE
#define GPIO_E7_Pin GPIO_PIN_14
#define GPIO_E7_GPIO_Port GPIOE


//макросы выходов последовательного интерфейса GPIOC
#define GPIO_C7_Pin GPIO_PIN_7
#define GPIO_C7_GPIO_Port GPIOC
#define GPIO_C6_Pin GPIO_PIN_6
#define GPIO_C6_GPIO_Port GPIOC
#define GPIO_C5_Pin GPIO_PIN_5
#define GPIO_C5_GPIO_Port GPIOC
#define GPIO_C4_Pin GPIO_PIN_4
#define GPIO_C4_GPIO_Port GPIOC
#define GPIO_C3_Pin GPIO_PIN_3
#define GPIO_C3_GPIO_Port GPIOC
#define GPIO_C2_Pin GPIO_PIN_2
#define GPIO_C2_GPIO_Port GPIOC
#define GPIO_C1_Pin GPIO_PIN_1
#define GPIO_C1_GPIO_Port GPIOC
#define GPIO_C0_Pin GPIO_PIN_0
#define GPIO_C0_GPIO_Port GPIOC


//макросы управляющих выходов (not SS) интерфейса SPI
#define not_SS7_Pin GPIO_PIN_12
#define not_SS7_GPIO_Port GPIOD
#define not_SS6_Pin GPIO_PIN_11
#define not_SS6_GPIO_Port GPIOD
#define not_SS5_Pin GPIO_PIN_10
#define not_SS5_GPIO_Port GPIOD
#define not_SS4_Pin GPIO_PIN_9
#define not_SS4_GPIO_Port GPIOD

#define not_SS3_Pin GPIO_PIN_4
#define not_SS3_GPIO_Port GPIOD
#define not_SS2_Pin GPIO_PIN_3
#define not_SS2_GPIO_Port GPIOD
#define not_SS1_Pin GPIO_PIN_2
#define not_SS1_GPIO_Port GPIOD
#define not_SS0_Pin GPIO_PIN_1
#define not_SS0_GPIO_Port GPIOD

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
