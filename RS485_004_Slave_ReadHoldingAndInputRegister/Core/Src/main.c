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

uint8_t funcReadHoldingRegister();
uint8_t funcReadInputRegister();
void exceptionCodeResponse(uint8_t functionCode);

uint8_t TxData[256];
uint8_t RxData[256];

#define Enable_Transmit    1
#define Enable_Recieve     0
#define funtionCode_ReadCoils                0x01  // Read up to 2000 contiguous memory bits
#define funtionCode_ReadDiscreteInputs       0x02  // Read up to 2000 contiguous input bits
#define funtionCode_ReadHoldingRegisters     0x03  // Read up to 125 contiguous memory words
#define funtionCode_ReadInputRegisters       0x04  // Read up to 125 contiguous input words
#define funtionCode_WriteSingleCoil          0x05  // Write one memory bit
#define funtionCode_WriteSingleRegister      0x06  // Write one memory word
#define funtionCode_WriteCoils               15 // Write up to 1968 contiguous memory bits
#define funtionCode_WriteHoldingRegisters    16 // Write up to 123 contiguous memory words


#define ILLEGAL_FUNCTION 		1
#define ILLEGAL_DATA_ADDRESS    2
#define ILLEGAL_DATA_VALUE      3

#define SLAVE_ID      5
// Create Database for Holding Register and Input Register

static uint16_t holdingRegistersDatabase[50] =
{
		0000, 1111 ,2222 ,3333, 4444, 5555, 6666, 7777, 8888, 9999,       // 0-9    40001-40010
		12345,15464,12254,14144,15425,15465,13232,15215,14541, 19998,     // 10-19  40011-40020
		22345,25464,22254,24144,25425,25465,23232,25215,24541, 29998,     // 20-29  40021-40030
		32345,35464,32254,38414,35425,35465,33232,35215,34541, 39998,     // 30-39  40031-40040
		42345,45464,42254,44144,45425,45465,43232,45215,44541, 49998,     // 40-49  40041-40050
};

static const uint16_t InputRegistersDatabase[50] =
{
		0000, 1111 ,2222 ,3333, 4444, 5555, 6666, 7777, 8888, 9999,     // 0-9    40001-40010
		12345,15464,12254,14144,15425,15465,13232,15215,14541, 19998,     // 10-19  40011-40020
		22345,25464,22254,24144,25425,25465,23232,25215,24541, 29998,     // 20-29  40021-40030
		32345,35464,32254,38414,35425,35465,33232,35215,34541, 39998,     // 30-39  40031-40040
		42345,45464,42254,44144,45425,45465,43232,45215,44541, 49998,     // 40-49  40041-40050
};


void sendData(uint8_t *data, int size)
{
	uint16_t crc =  crc16(TxData, size);
	data[size] =  crc & 0xFF;
	data[size+1] =  (crc >> 8 ) & 0xFF;
	HAL_GPIO_WritePin(Tx_Enable_GPIO_Port, Tx_Enable_Pin, Enable_Transmit);
	HAL_UART_Transmit(&huart1, TxData, size+2, 1000);
	HAL_GPIO_WritePin(Tx_Enable_GPIO_Port, Tx_Enable_Pin, Enable_Recieve);
}



/* LƯU Ý: Register đọc tối đa 125 word, tức 250 byte
 * Cấu trúc Data gửi từ master
 * Slave Addr || Function Code || StartAddr High-Low|| Number register wanna read High-Low|| CRC low || CRC high
 *  1 byte         1 byte            2 byte                      2 byte                        1 byte    1 byte
 *
 *  Cấu trúc data slave Response đến master
 *
 *  Slave Addr || Function Code || Byte Count Data Response || Data  || CRC Low || CRC High
 *   1 byte         1 byte               1 byte                n byte    1 byte     1 byte
 *   Nếu đọc 5 register, data  = 10 byte vì 1 data 2 byte(uint16_t) => byte count  = 10
 *
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	// check xem id slave
	uint8_t slaveAddr = RxData[0];
	uint8_t funtionCode = RxData[1];
	if(slaveAddr == SLAVE_ID)
	{
		switch(funtionCode)
		{
			case funtionCode_ReadHoldingRegisters:
				funcReadHoldingRegister();
				break;
			case funtionCode_ReadInputRegisters:
				funcReadInputRegister();
				break;
			default:
				exceptionCodeResponse(ILLEGAL_FUNCTION);
		}
	}
	HAL_GPIO_WritePin(Tx_Enable_GPIO_Port, Tx_Enable_Pin, Enable_Recieve);
	HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, 256);
}
/*
 * Cấu trúc phản hồi exception như sau:
 * Slave Addr  || funtionCode|0x10   ||   ExceptionCode || CRC low || CRC High
 *  1 byte              1 byte                  1 byte       1 byte    1 byte
 */
void exceptionCodeResponse(uint8_t exceptioncode)
{
	TxData[0] = RxData[0];       // slave ID
	TxData[1] = RxData[1]|0x80;  // adding 1 to the MSB of the function code
	TxData[2] = exceptioncode;   // Load the Exception code
	sendData(TxData, 3);
}

/*
*  Cấu trúc data slave Response đến master
*  Slave Addr || Function Code || Byte Count Data Response || Data  || CRC Low || CRC High
*   1 byte         1 byte               1 byte                n byte    1 byte     1 byte
*   Nếu đọc 5 register, data  = 10 byte vì 1 data 2 byte(uint16_t) => byte count  = 10
*/
uint8_t funcReadHoldingRegister()
{
	// Tính toán địa chỉ bắt đầu đọc
	uint16_t addrStartReadHighByte = RxData[2];
	uint16_t addrStartReadLowByte = RxData[3];
	uint16_t addrStartRead =  (addrStartReadHighByte << 8 ) | addrStartReadLowByte;

	// Tính toán số Register đọc
	uint16_t numberRegReadHigh =   RxData[4];
	uint16_t numberRegReadLow  =   RxData[5];
	uint16_t numberRegRead =  (numberRegReadHigh << 8 ) | numberRegReadLow;

	uint16_t addrEndRead = addrStartRead + numberRegRead - 1;
	// Check CRC
	if(numberRegRead < 1 || numberRegRead > 125)
	{
		exceptionCodeResponse(ILLEGAL_DATA_VALUE);
		return  0;
	}
	if(addrEndRead > 49)
	{
		exceptionCodeResponse(ILLEGAL_DATA_ADDRESS);
		return  0;
	}

	TxData[0] = SLAVE_ID;
	TxData[1] = RxData[1];
	TxData[2] = numberRegRead * 2;
	uint8_t count = 3;
	for(uint8_t i = 0; i < numberRegRead ; i++)
	{
		TxData[count] = (holdingRegistersDatabase[addrStartRead] >> 8) & 0xFF;
		count ++;
		TxData[count] = (holdingRegistersDatabase[addrStartRead] ) & 0xFF ;
		addrStartRead++;
		count ++;
	}
	sendData(TxData, count );

	return 1;
}

uint8_t funcReadInputRegister()
{
	// Tính toán địa chỉ bắt đầu đọc
	uint16_t addrStartReadHighByte = RxData[2];
	uint16_t addrStartReadLowByte = RxData[3];
	uint16_t addrStartRead =  (addrStartReadHighByte << 8 ) | addrStartReadLowByte;

	// Tính toán số Register đọc
	uint16_t numberRegReadHigh =   RxData[4];
	uint16_t numberRegReadLow  =   RxData[5];
	uint16_t numberRegRead =  (numberRegReadHigh << 8 ) | numberRegReadLow;

	uint16_t addrEndRead = addrStartRead + numberRegRead - 1;
	// Check CRC
	if(addrEndRead > 49)
	{
		exceptionCodeResponse(ILLEGAL_DATA_ADDRESS);
		return  0;
	}
	if(numberRegRead < 1 || numberRegRead > 125)
	{
		exceptionCodeResponse(ILLEGAL_DATA_VALUE);
		return  0;
	}
	TxData[0] = SLAVE_ID;
	TxData[1] = RxData[1];
	TxData[2] = numberRegRead * 2;
	uint8_t count = 3;
	for(int i = 0; i < numberRegRead ; i++)
	{
		TxData[count] = (InputRegistersDatabase[addrStartRead] >> 8) & 0xFF;
		count ++;
		TxData[count] = (InputRegistersDatabase[addrStartRead] ) & 0xFF ;
		addrStartRead++;
		count ++;
	}
	sendData(TxData, count );

	return 1;
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
  HAL_GPIO_WritePin(Tx_Enable_GPIO_Port, Tx_Enable_Pin, Enable_Recieve);
  HAL_UARTEx_ReceiveToIdle_IT(&huart1, RxData, 256);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Tx_Enable_GPIO_Port, Tx_Enable_Pin, GPIO_PIN_RESET);

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
