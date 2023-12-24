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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define SPI_MAX_DEVICES 4
#define I2C_MAX_DEVICES 16

uint8_t MTEMU_REQUEST[4] = { };
uint8_t UART_INTERFACE_RESPONSE[1] = { };
uint8_t I2C_INTERFACE_RESPONSE[16] = { };
uint8_t SPI_INTERFACE_RESPONSE[4] = { };

uint8_t UART_INTERFACE_DATA_RECIEVED = 0;
uint8_t SPI_INTERFACE_DATA_RECIEVED = 0;
uint8_t I2C_INTERFACE_DATA_RECIEVED = 0;

// GPIOA=0/UART=1 - BIT7
// GPIOC=0/SPI=1 - BIT6
// GPIOE=0/I2C=1 - BIT5
// не используется - BIT4
// GPIOA OUTPUT=0/GPIOA INPUT=1 - BIT3
// GPIOC OUTPUT=0/GPIOC INPUT=1 - BIT2
// GPIOE OUTPUT=0/GPIOE INPUT=1 - BIT1
// не используется - BIT0
uint8_t CONTROL_REGISTER = 0;

char *FIRST_ROW_LCD = "Welcome to";
char *SECOND_ROW_LCD = "PortExtender";

typedef enum {
	// Интерфейсы последовательной передачи данных
	INTERFACE_UART,
	INTERFACE_SPI,  //
	INTERFACE_I2C,
	// Параллельные интерфейсы передачи данных (параллельные ноги GPIO)
	INTERFACE_GPIOA,
	INTERFACE_GPIOC,
	INTERFACE_GPIOE,

	// Управляющий регистр
	CTRL_REG,

	INTERFACE_UNKNOWN
} AVAILABLE_INTERFACES;
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_SPI1_Init();
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_I2C3_Init();
	/* USER CODE BEGIN 2 */

	// Turn on backlight
	prepare_lcd();

	HAL_NVIC_SetPriority(USART2_IRQn, 1, 1);
	HAL_NVIC_EnableIRQ(USART2_IRQn);
	HAL_UART_Receive_IT(&huart2, MTEMU_REQUEST, sizeof(MTEMU_REQUEST));
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief I2C3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C3_Init(void) {

	/* USER CODE BEGIN I2C3_Init 0 */

	/* USER CODE END I2C3_Init 0 */

	/* USER CODE BEGIN I2C3_Init 1 */

	/* USER CODE END I2C3_Init 1 */
	hi2c3.Instance = I2C3;
	hi2c3.Init.ClockSpeed = 100000;
	hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c3.Init.OwnAddress1 = 0;
	hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c3.Init.OwnAddress2 = 0;
	hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C3_Init 2 */

	/* USER CODE END I2C3_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	HAL_GPIO_WritePin(GPIOD,
	not_SS0_Pin | not_SS1_Pin | not_SS2_Pin | not_SS3_Pin, GPIO_PIN_SET);
	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/*Configure GPIO pin Output Level */

	// GPIO управления SPI
	GPIO_InitStruct.Pin = not_SS3_Pin | not_SS2_Pin | not_SS0_Pin | not_SS1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOD,
	not_SS3_Pin | not_SS2_Pin | not_SS0_Pin | not_SS1_Pin, GPIO_PIN_SET);

	// GPIOE последовательный интерфейс
	GPIO_InitStruct.Pin = GPIO_E0_Pin | GPIO_E1_Pin | GPIO_E2_Pin | GPIO_E3_Pin
			| GPIO_E4_Pin | GPIO_E5_Pin | GPIO_E6_Pin | GPIO_E7_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOE,
			GPIO_E0_Pin | GPIO_E1_Pin | GPIO_E2_Pin | GPIO_E3_Pin | GPIO_E4_Pin
					| GPIO_E5_Pin | GPIO_E6_Pin | GPIO_E7_Pin, GPIO_PIN_RESET);

	// GPIOA последовательный интерфейс
	GPIO_InitStruct.Pin = GPIO_A0_Pin | GPIO_A1_Pin | GPIO_A2_Pin | GPIO_A3_Pin
			| GPIO_A4_Pin | GPIO_A5_Pin | GPIO_A6_Pin | GPIO_A7_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOA,
			GPIO_A0_Pin | GPIO_A1_Pin | GPIO_A2_Pin | GPIO_A3_Pin | GPIO_A4_Pin
					| GPIO_A5_Pin | GPIO_A6_Pin | GPIO_A7_Pin, GPIO_PIN_RESET);

	// GPIOС последовательный интерфейс
	GPIO_InitStruct.Pin = GPIO_С0_Pin | GPIO_С1_Pin | GPIO_С2_Pin | GPIO_С3_Pin
			| GPIO_С4_Pin | GPIO_С5_Pin | GPIO_С6_Pin | GPIO_С7_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOC,
			GPIO_С0_Pin | GPIO_С1_Pin | GPIO_С2_Pin | GPIO_С3_Pin | GPIO_С4_Pin
					| GPIO_С5_Pin | GPIO_С6_Pin | GPIO_С7_Pin, GPIO_PIN_RESET);

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
AVAILABLE_INTERFACES GetInterfaceByPort(const uint8_t port) {
	switch (port) {
	case 1:
	case 2:
	case 3:
		if (COTROL_REGISTER & 0b10000000 == 0) {
			return INTERFACE_GPIOA;
		}
		return INTERFACE_UART;
	case 5:
	case 6:
	case 7:
		if (COTROL_REGISTER & 0b01000000 == 0) {
			return INTERFACE_GPIOC;
		}
		return INTERFACE_SPI;
	case 9:
	case 10:
	case 11:
		if (COTROL_REGISTER & 0b00100000 == 0) {
			return INTERFACE_GPIOA;
		}
		return INTERFACE_I2C;
	case 13:
	case 14:
	case 15:
		return CTRL_REG;
	}
	return INTERFACE_UNKNOWN;
}

void SendRequestToInterface(AVAILABLE_INTERFACES interface, uint8_t address,
		uint8_t data) {
	switch (interface) {
	case INTERFACE_UART:
		// Отправить данные по интерфейсу юарт
		FIRST_ROW_LCD = "MT->UART";
		sprintf(SECOND_ROW_LCD, "0x%x", data);

		HAL_UART_Transmit(&huart1, &data, 1, HAL_MAX_DELAY);

		// Прервать асинхронных прием данных, так как была дана команда на новую отправку посылки
		if (HAL_UART_GetState(&huart1) == HAL_UART_STATE_BUSY_RX) {
			HAL_UART_AbortReceive_IT(&huart1);
		}

		UART_INTERFACE_DATA_RECIEVED = 0;
		// Асинхронно ожидаем прием данных
		HAL_UART_Receive_IT(&huart1, UART_INTERFACE_RESPONSE,
				sizeof(UART_INTERFACE_RESPONSE));

		break;
	case INTERFACE_SPI:
		sprintf(FIRST_ROW_LCD, "%s(%d)", "MT->SPI", address % SPI_MAX_DEVICES);
		sprintf(SECOND_ROW_LCD, "0x%x", data);
		switch (address % SPI_MAX_DEVICES) {
		case 0:
			HAL_GPIO_WritePin(not_SS0_GPIO_Port, not_SS0_Pin, GPIO_PIN_RESET);
			break;
		case 1:
			HAL_GPIO_WritePin(not_SS1_GPIO_Port, not_SS1_Pin, GPIO_PIN_RESET);
			break;
		case 2:
			HAL_GPIO_WritePin(not_SS2_GPIO_Port, not_SS2_Pin, GPIO_PIN_RESET);
			break;
		case 3:
			HAL_GPIO_WritePin(not_SS3_GPIO_Port, not_SS3_Pin, GPIO_PIN_RESET);
			break;
		}
		HAL_SPI_Transmit(&hspi1, &data, 1, HAL_MAX_DELAY);

		// Прервать асинхронных прием данных, так как была дана команда на новую отправку посылки
		/*if (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_RX) {
		 HAL_SPI_Abort_IT(&hspi1);
		 }

		 SPI_INTERFACE_DATA_RECIEVED = 0;
		 HAL_SPI_Receive_IT(&hspi1,
		 &SPI_INTERFACE_RESPONSE[address % SPI_MAX_DEVICES],
		 sizeof(SPI_INTERFACE_RESPONSE[address % SPI_MAX_DEVICES]));*/
		break;
	case INTERFACE_I2C:
		sprintf(FIRST_ROW_LCD, "%s(%d)", "MT->I2C", address % I2C_MAX_DEVICES);
		sprintf(SECOND_ROW_LCD, "0x%x", data);

		HAL_I2C_Master_Transmit(&hi2c1, 0x20, &data, 1,
		HAL_MAX_DELAY);

		// Синхронное говно (принимаем без прерываний)

		// Прервать асинхронных прием данных, так как была дана команда на новую отправку посылки
		// Надо ли, если 16 устройств?
		/*if (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_BUSY_RX
		 || HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_BUSY_RX_LISTEN) {
		 HAL_I2C_Master_Abort_IT(&hi2c1, address % I2C_MAX_DEVICES);
		 }

		 I2C_INTERFACE_DATA_RECIEVED = 0;

		 HAL_I2C_Master_Receive_IT(&hi2c1, address % I2C_MAX_DEVICES,
		 &I2C_INTERFACE_RESPONSE[address % I2C_MAX_DEVICES],
		 sizeof(I2C_INTERFACE_RESPONSE[address % I2C_MAX_DEVICES]));*/
		break;
	case INTERFACE_GPIO:
		FIRST_ROW_LCD = "MT->GPIO";
		sprintf(SECOND_ROW_LCD, "0x%x", data);

		HAL_GPIO_WritePin(GPIO_OUT0_GPIO_Port, GPIO_OUT0_Pin,
				(data >> 0) & 0b00000001);
		HAL_GPIO_WritePin(GPIO_OUT1_GPIO_Port, GPIO_OUT1_Pin,
				(data >> 1) & 0b00000001);
		HAL_GPIO_WritePin(GPIO_OUT2_GPIO_Port, GPIO_OUT2_Pin,
				(data >> 2) & 0b00000001);
		HAL_GPIO_WritePin(GPIO_OUT3_GPIO_Port, GPIO_OUT3_Pin,
				(data >> 3) & 0b00000001);
		HAL_GPIO_WritePin(GPIO_OUT4_GPIO_Port, GPIO_OUT4_Pin,
				(data >> 4) & 0b00000001);
		HAL_GPIO_WritePin(GPIO_OUT5_GPIO_Port, GPIO_OUT5_Pin,
				(data >> 5) & 0b00000001);
		HAL_GPIO_WritePin(GPIO_OUT6_GPIO_Port, GPIO_OUT6_Pin,
				(data >> 6) & 0b00000001);
		HAL_GPIO_WritePin(GPIO_OUT7_GPIO_Port, GPIO_OUT7_Pin,
				(data >> 7) & 0b00000001);
		break;
	}
}

void GetResponseFromInterface(AVAILABLE_INTERFACES interface, uint8_t address,
		uint8_t *answer) {
	uint8_t error_code;
	switch (interface) {
	case INTERFACE_UART:

		while (UART_INTERFACE_DATA_RECIEVED != 1)
			;
		error_code = HAL_UART_GetError(&huart1);

		FIRST_ROW_LCD = "MT<-UART";
		if (error_code == HAL_UART_ERROR_NONE) {
			*answer = UART_INTERFACE_RESPONSE[0];
			sprintf(SECOND_ROW_LCD, "0x%x", UART_INTERFACE_RESPONSE[0]);
		} else {
			// TODO Нужно указать на ошибку, хз, куда поместить
			*answer = error_code;
			sprintf(SECOND_ROW_LCD, "%s:0x%x", "Error code", error_code);
		}
		break;
	case INTERFACE_SPI:
		// Все принимается синхронно
		//while (SPI_INTERFACE_DATA_RECIEVED != 1)
		//	;

		// Синхронное говно (принимаем без прерываний)
		HAL_SPI_Receive(&hspi1,
				&SPI_INTERFACE_RESPONSE[address % SPI_MAX_DEVICES],
				sizeof(SPI_INTERFACE_RESPONSE[address % SPI_MAX_DEVICES]),
				HAL_MAX_DELAY);

		error_code = HAL_SPI_GetError(&hspi1);

		switch (address % SPI_MAX_DEVICES) {
		case 0:
			HAL_GPIO_WritePin(not_SS0_GPIO_Port, not_SS0_Pin, GPIO_PIN_SET);
			break;
		case 1:
			HAL_GPIO_WritePin(not_SS1_GPIO_Port, not_SS1_Pin, GPIO_PIN_SET);
			break;
		case 2:
			HAL_GPIO_WritePin(not_SS2_GPIO_Port, not_SS2_Pin, GPIO_PIN_SET);
			break;
		case 3:
			HAL_GPIO_WritePin(not_SS3_GPIO_Port, not_SS3_Pin, GPIO_PIN_SET);
			break;
		}

		sprintf(FIRST_ROW_LCD, "%s(%d)", "MT<-SPI", address % SPI_MAX_DEVICES);
		if (error_code == HAL_SPI_ERROR_NONE) {
			*answer = SPI_INTERFACE_RESPONSE[address % SPI_MAX_DEVICES];
			sprintf(SECOND_ROW_LCD, "0x%x",
					SPI_INTERFACE_RESPONSE[address % SPI_MAX_DEVICES]);
		} else {
			*answer = error_code;
			sprintf(SECOND_ROW_LCD, "%s:0x%x", "Error code", error_code);
		}
		break;
	case INTERFACE_I2C:
		// Все принимается синхронно
		//while (I2C_INTERFACE_DATA_RECIEVED != 1)
		//	;
		HAL_I2C_Master_Receive(&hi2c1, 0x20,
				&I2C_INTERFACE_RESPONSE[address % I2C_MAX_DEVICES],
				sizeof(I2C_INTERFACE_RESPONSE[address % I2C_MAX_DEVICES]),
				HAL_MAX_DELAY);

		error_code = HAL_I2C_GetError(&hi2c1);

		sprintf(FIRST_ROW_LCD, "%s(%d)", "MT<-I2C", address % I2C_MAX_DEVICES);
		if (error_code == HAL_I2C_ERROR_NONE) {
			*answer = I2C_INTERFACE_RESPONSE[address % I2C_MAX_DEVICES];
			sprintf(SECOND_ROW_LCD, "0x%x",
					I2C_INTERFACE_RESPONSE[address % I2C_MAX_DEVICES]);
		} else {
			*answer = error_code;
			sprintf(SECOND_ROW_LCD, "%s:0x%x", "Error code", error_code);
		}
		break;
	case INTERFACE_GPIO:
		*answer = 0;
		*answer = *answer
				| (HAL_GPIO_ReadPin(GPIO_IN0_GPIO_Port, GPIO_IN0_Pin) << 0);
		*answer = *answer
				| (HAL_GPIO_ReadPin(GPIO_IN1_GPIO_Port, GPIO_IN1_Pin) << 1);
		*answer = *answer
				| (HAL_GPIO_ReadPin(GPIO_IN2_GPIO_Port, GPIO_IN2_Pin) << 2);
		*answer = *answer
				| (HAL_GPIO_ReadPin(GPIO_IN3_GPIO_Port, GPIO_IN3_Pin) << 3);
		*answer = *answer
				| (HAL_GPIO_ReadPin(GPIO_IN4_GPIO_Port, GPIO_IN4_Pin) << 4);
		*answer = *answer
				| (HAL_GPIO_ReadPin(GPIO_IN5_GPIO_Port, GPIO_IN5_Pin) << 5);
		*answer = *answer
				| (HAL_GPIO_ReadPin(GPIO_IN6_GPIO_Port, GPIO_IN6_Pin) << 6);
		*answer = *answer
				| (HAL_GPIO_ReadPin(GPIO_IN7_GPIO_Port, GPIO_IN7_Pin) << 7);

		FIRST_ROW_LCD = "MT<-GPIO";
		sprintf(SECOND_ROW_LCD, "0x%x", *answer);
		break;
	}
}

// Соединение с мтему
void MtemuConnection() {

	uint8_t MTEMU_RESPONSE[18] = { "\x10TMBelka1234567EP\x90" };

	HAL_UART_Transmit(&huart2, MTEMU_RESPONSE, sizeof(MTEMU_RESPONSE),
	HAL_MAX_DELAY);
	HAL_UART_Receive_IT(&huart2, MTEMU_REQUEST, sizeof(MTEMU_REQUEST));
}

// Мтему хочет, чтобы мы ему выдали данные
void SendMtemuResponse() {
	uint8_t MTEMU_RESPONSE[4] = { 2, MTEMU_REQUEST[1], '\x44', '\x82' };

	// 4 старших бита - адрес девайса на интерфейсе, 4 младших бита - интерфейс
	AVAILABLE_INTERFACES interface = GetInterfaceByPort(
			MTEMU_REQUEST[1] & 0b00001111);
	uint8_t address = (MTEMU_REQUEST[1] >> 4) & 0b00001111;

	if (interface != INTERFACE_UNKNOWN) {
		GetResponseFromInterface(interface, address, &MTEMU_RESPONSE[2]);
	} else {
		// Ошибка - нераспознанный интерфейс
	}

	HAL_UART_Transmit(&huart2, MTEMU_RESPONSE, sizeof(MTEMU_RESPONSE),
	HAL_MAX_DELAY);
	HAL_UART_Receive_IT(&huart2, MTEMU_REQUEST, sizeof(MTEMU_REQUEST));
}

// Мтему хочет отослать нам данные
void GetMtemuRequest() {
	// 4 старших бита - адрес девайса на интерфейса, 4 младших бита - интерфейс
	AVAILABLE_INTERFACES interface = GetInterfaceByPort(
			MTEMU_REQUEST[1] & 0b00001111);
	uint8_t address = (MTEMU_REQUEST[1] >> 4) & 0b00001111;

	if (interface != INTERFACE_UNKNOWN) {
		SendRequestToInterface(interface, address, MTEMU_REQUEST[2]);
	} else {
		// Ошибка - нераспознанный интерфейс
	}

	uint8_t MTEMU_RESPONSE[2] = { '\x00', '\x80' };

	HAL_UART_Transmit(&huart2, MTEMU_RESPONSE, sizeof(MTEMU_RESPONSE),
	HAL_MAX_DELAY);
	HAL_UART_Receive_IT(&huart2, MTEMU_REQUEST, sizeof(MTEMU_REQUEST));
}

void MtemuUartHandler() {
	if (MTEMU_REQUEST[0] == 98 && MTEMU_REQUEST[1] == 0 && MTEMU_REQUEST[2] == 0
			&& MTEMU_REQUEST[3] == 226) {
		// Соединение с Mtemu
		MtemuConnection();

	}
	if (MTEMU_REQUEST[0] == 34 && MTEMU_REQUEST[3] == 162) {
		// Mtemu запросил ответ
		SendMtemuResponse();
	}
	if (MTEMU_REQUEST[0] == 66 && MTEMU_REQUEST[3] == 194) {
		// Mtemu плюнул информацию
		GetMtemuRequest();
	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart2) {
		MtemuUartHandler();
	} else {
		UART_INTERFACE_DATA_RECIEVED = 1;
	}

}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {

	SPI_INTERFACE_DATA_RECIEVED = 1;

}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {

	I2C_INTERFACE_DATA_RECIEVED = 1;

}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
