/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#include <string.h>
#include <stdio.h>
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define I2C_ADDRESS                                              0x76
#define I2C_ID_ADDRESS                                           0xD0
#define I2C_TIMEOUT                                              10
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
float pressure, temperature, humidity;

char *status = "";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int presence = 0, isRxed = 0;
uint8_t RxData[8], Temp_LSB = 0, Temp_MSB = 0;
int16_t Temp;
float temp1;

uint8_t SPI1_txProp_buf[4];

uint8_t SPI1_tx_buf[1];

uint8_t SPI1_rx_buf[3];
uint8_t SPI1_temp[4];
float temp2;

void uart_Init (uint32_t baud)
{

	  huart2.Instance = USART2;
	  huart2.Init.BaudRate = baud;
	  huart2.Init.WordLength = UART_WORDLENGTH_8B;
	  huart2.Init.StopBits = UART_STOPBITS_1;
	  huart2.Init.Parity = UART_PARITY_NONE;
	  huart2.Init.Mode = UART_MODE_TX_RX;
	  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	  if (HAL_HalfDuplex_Init(&huart2) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

int DS18B20_Start(void)
{
	uint8_t data = 0xF0;
	uart_Init(9600);
	HAL_UART_Transmit(&huart2, &data, 1, 50);  // low for 500+ms
	if (HAL_UART_Receive(&huart2, &data, 1, 1000) != HAL_OK) return -1;   // failed.. check connection
	uart_Init(115200);
	if (data == 0xFF) return -2;  // no response.. check connection
	return 1;  // response detected
}

void BME280_Start(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, SPI1_txProp_buf, 4, 100);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void DS18B20_Write (uint8_t data)
{
  uint8_t buffer[8];
  for (int i=0; i<8; i++)
  {
    if ((data & (1<<i))!=0)  // if the bit is high
    {
	buffer[i] = 0xFF;  // write 1
    }
    else  // if the bit is low
    {
	buffer[i] = 0x00;  // write 0
    }
  }
  HAL_UART_Transmit(&huart2, buffer, 8, 100);
}

uint8_t DS18B20_Read (void)
{
	uint8_t buffer[8];
	uint8_t value = 0;
	for (int i=0; i<8; i++)
	{
		buffer[i] = 0xFF;
	}
	HAL_UART_Transmit_DMA(&huart2, buffer, 8);
	HAL_UART_Receive_DMA(&huart2, RxData, 8);

	//while (isRxed == 0);
	HAL_Delay(1000);
	for (int i=0;i<8;i++)
	{
		if (RxData[i]==0xFF)  // if the pin is HIGH
		{
			value |= 1<<i;  // read = 1
		}
	}
	isRxed = 0;
	return value;
}

void BME280_Read(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, SPI1_tx_buf, 1, 100);
	HAL_SPI_Receive(&hspi1, SPI1_rx_buf, 1, 0.5);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	//HAL_UART_Transmit(&huart1, &SPI1_rx_buf[0], 1, 100);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	isRxed = 1;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	const uint8_t st_msg[] = "Comitas Akademya\r\n";
	const uint8_t err_msg[] = "err_msg\r\n";

	uint16_t I2C_adresses[] = {0x03, 0x00, 0x01, 0x02};
	uint8_t I2C_data[] = {0x11};
	uint8_t I2C_data_buf[] = {1, 0};
	uint8_t I2C_DATAh;
	uint8_t I2C_DATAl;
	uint16_t I2C_temp;
	HAL_StatusTypeDef res;
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Transmit(&huart1, st_msg, sizeof(st_msg) - 1, 100);

  SPI1_txProp_buf[0] = 0x74;	// 0x74 F4h register address
  SPI1_txProp_buf[1] = 0x23;
  SPI1_txProp_buf[2] = 0x75;	// 0x75 F5h register address
  SPI1_txProp_buf[3] = 0xA4;	// 0xA0 - disable IIR filter

  SPI1_tx_buf[0] = 0xFA;	// 0xFA 7A

  BME280_Start();
  BME280_Read();
  char msg[64];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //HAL_Delay(700);
	  presence = DS18B20_Start ();
	  //snprintf(msg, sizeof(msg), "Rep: %d\r\n", (int16_t)presence);
	  //HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	  DS18B20_Write (0xCC);  // skip ROM
	  DS18B20_Write (0x44);  // convert t
	  presence = DS18B20_Start ();
	  DS18B20_Write (0xCC);  // skip ROM
	  DS18B20_Write (0xBE);  // Read Scratch-pad
	  Temp_LSB = DS18B20_Read();
	  Temp_MSB = DS18B20_Read();
	  Temp = ((Temp_MSB<<8))|Temp_LSB;
	  temp1 = (float)Temp/16.0;  // resolution is 0.0625

	  BME280_Start();
	  BME280_Read();

	  //Поиск id i2c
	  /*for(uint16_t i = 0; i < 128; i++)
	  {
		  // проверяем, готово ли устройство по адресу i для связи
		  res = HAL_I2C_IsDeviceReady(&hi2c1, i << 1, 1, 10);
		  // если да, то
		  if(res != HAL_OK)
		  {
			  char msg[64];
			  // запись адреса i, на который откликнулись, в строку в виде
			  // 16тиричного значения:
			  snprintf(msg, sizeof(msg), "0x%02X", i);
			  // отправка номера откликнувшегося адреса
			  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 10);
			  // переход на новую строчку
			  HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, 10);
		  }
		  else HAL_UART_Transmit(&huart1, (uint8_t*)".", 1, 10);
	  }*/

	  HAL_I2C_Mem_Write(&hi2c1, (0x04 << 1), 0x03, I2C_MEMADD_SIZE_8BIT, &I2C_data[0], 1, 100);
//	  while (I2C_data_buf[0] != 0x0){
		  HAL_I2C_Mem_Read(&hi2c1, (0x04 << 1), 0x0, I2C_MEMADD_SIZE_8BIT, &I2C_data_buf[0], 1, 100);
		  HAL_UART_Transmit(&huart1, &I2C_data_buf[0], 1, 1000);
//	  }
	  HAL_Delay(1000);
	  HAL_I2C_Mem_Read(&hi2c1, (0x04 << 1), I2C_adresses[2], I2C_MEMADD_SIZE_8BIT, &I2C_DATAh, 1, 100);
	  HAL_I2C_Mem_Read(&hi2c1, (0x04 << 1), I2C_adresses[3], I2C_MEMADD_SIZE_8BIT, &I2C_DATAl, 1, 100);

	  //HAL_UART_Transmit(&huart1, &I2C_DATAh, 1, 1000);
	  //HAL_UART_Transmit(&huart1, &I2C_DATAl, 1, 1000);

	  HAL_Delay(1000);

	  I2C_temp += I2C_DATAh;
	  I2C_temp = I2C_temp << 8;
	  I2C_temp += I2C_DATAl;
	  I2C_temp = (I2C_temp / 32) - 50;	// значение температуры TH02
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  snprintf(msg, sizeof(msg), "T DS18B20: %d C\r\n", (int16_t)temp1);
	  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	  snprintf(msg, sizeof(msg), "T BME280: %d C\r\n", (SPI1_rx_buf[0] - 0x6D));
	  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
	  snprintf(msg, sizeof(msg), "T TH02: %d C\r\n ", (I2C_temp));
	  HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
      HAL_Delay(2000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

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
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

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
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
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
static void MX_USART1_UART_Init(void)
{

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
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
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_HalfDuplex_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USART1_CK_GPIO_Port, USART1_CK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(I2C_CLK_GPIO_Port, I2C_CLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 USART1_CK_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_4|USART1_CK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : I2C_CLK_Pin */
  GPIO_InitStruct.Pin = I2C_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(I2C_CLK_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
