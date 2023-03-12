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
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include "modbus_crc.h"

uint8_t RxData[30];
uint8_t TxData[15];
uint16_t data;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
//	data = (RxData[4] << 8) |  RxData[5];
//	if(data == 0xFF00) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
//	else if (data == 0) HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1);
}


void sendData(uint8_t *data)
{
	HAL_GPIO_WritePin(Tx_Enable_GPIO_Port, Tx_Enable_Pin, GPIO_PIN_SET);
	HAL_UART_Transmit(&huart1, data, 15, 1000);
	HAL_GPIO_WritePin(Tx_Enable_GPIO_Port, Tx_Enable_Pin, GPIO_PIN_RESET);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, 32);
  /*
  Prepare data transmit single coil or register to slave
  || Slave ID 1byte || function Code 1byte|| Add wanna Write High 1byte|| Addr wanna write Low || Value High || Value Low || CRC Low || CRC High
  if data 0xFF00 -> write 1
  if data 0x0000 -> write 0
  */
  /*
  TxData[0] = 0x05;
  TxData[1] = 0x06;// Write single coil function code 0x05
                   // Write single Register function code 0x06

  TxData[2] = 0x00;// Addr wanna write High
  TxData[3] = 0x00;// Addr wanna write Low
  TxData[4] = 0x12; //Value High
  TxData[5] = 0x34; //Value Low

  uint16_t crc = crc16(TxData, 6);

  TxData[6] = crc & 0xFF;
  TxData[7] = (crc >> 8 ) & 0xFF;

  sendData(TxData);
  */

  /*
   * Write mutiple coilsm FC: 0x15, up to 1968 contigous memory bits
   * Write mutiple register FC: 0x16, up to 123 contiguous memory WORDs
   || Slave ID || Function code || Start Add || Number of coils wanna write || Number of byte will send to slave || Data || CRC Low || CRC High
       1byte		1byte			2byte						2byte					1byte						nbyte	1byte        1byte
       if we want to write 16 coils
        Number of coils wanna write : 15 ( 2 byte)
        Number of byte will send to slave : 2

        Data 0xFF01
        1  1  1  1  1  1  1  1  0  0  0  0  0  0  0  1
        coil 16 -> coil 1

     Data response from slave:
       || Slave ID || Function code || Start Add || Number of coils modify || CRC Low || CRC High
            1byte		 1byte			 2byte            2byte				   1byte		 1byte
		// Example for write mutiple coils
		      TxData[0] = 0x05;
			  TxData[1] = 0x0F;

			  TxData[2] = 0x00;
			  TxData[3] = 0x00;

			  TxData[4] = 0x00;
			  TxData[5] = 16;

			  TxData[6] = 2;
			  TxData[7] = 0x01;
			  TxData[8] = 0xFF;
			 // CRC number  =  9 bit trước đó
			  uint16_t crc = crc16(TxData, 9);
			  TxData[9] = crc & 0xFF;
			  TxData[10] = ( crc >> 8 ) & 0xFF;
			  sendData(TxData);
    */
  TxData[0] = 0x05;
  TxData[1] = 0x10;

  TxData[2] = 0x00;
  TxData[3] = 0x00;

  TxData[4] = 0x00;
  TxData[5] = 3;

  TxData[6] = 6;

  TxData[7] = 0x01;   // Register 1 High byte
  TxData[8] = 0xFF;   // Register 1 Low byte
  TxData[9] = 0x12;   // Register 2 High byte
  TxData[10] = 0x34;  // Register 2 Low byte
  TxData[11] = 0xAB;  // Register 3 High byte
  TxData[12] = 0xCD;  // Register 3 Low byte

  uint16_t crc = crc16(TxData, 13);
  TxData[13] = crc & 0xFF;
  TxData[14] = ( crc >> 8 ) & 0xFF;
  sendData(TxData);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Tx_Enable_GPIO_Port, Tx_Enable_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Tx_Enable_Pin */
  GPIO_InitStruct.Pin = Tx_Enable_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Tx_Enable_GPIO_Port, &GPIO_InitStruct);

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
