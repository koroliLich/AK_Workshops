/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
	C4 = 0x00, // all next 0x00+1
	C5, D5, E5, F5, G5, A5, B5,
	C6, D6, E6, F6, G6, A6, B6,
	C7
} soundToneType;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define CS43L22_I2C_ADDRESS 	0x94
#define I2C_TIMEOUT 			10

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_tx;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */

int16_t dataI2S[100] = {0};

uint8_t led_state = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void CS43L22_Init(void)
{
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, 1);

	uint8_t buffer[2];

	buffer[0] = 0x0D;
	buffer[1] = 0x01;
	HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t *) &buffer, 2, I2C_TIMEOUT);

	buffer[0] = 0x00;
	buffer[1] = 0x99;
	HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t *) &buffer, 2, I2C_TIMEOUT);

	buffer[0] = 0x47;
	buffer[1] = 0x80;
	HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t *) &buffer, 2, I2C_TIMEOUT);

	buffer[0] = 0x32;
	buffer[1] = 0xFF;
	HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t *) &buffer, 2, I2C_TIMEOUT);

	buffer[0] = 0x32;
	buffer[1] = 0x7F;
	HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t *) &buffer, 2, I2C_TIMEOUT);

	buffer[0] = 0x00;
	buffer[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t *) &buffer, 2, I2C_TIMEOUT);

	buffer[0] = 0x04;
	buffer[1] = 0xAF;
	HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t *) &buffer, 2, I2C_TIMEOUT);

	buffer[0] = 0x0D;
	buffer[1] = 0x70;
	HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t *) &buffer, 2, I2C_TIMEOUT);

	buffer[0] = 0x05;
	buffer[1] = 0x81;
	HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t *) &buffer, 2, I2C_TIMEOUT);

	buffer[0] = 0x06;
	buffer[1] = 0x07;
	HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t *) &buffer, 2, I2C_TIMEOUT);

	buffer[0] = 0x0A;
	buffer[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t *) &buffer, 2, I2C_TIMEOUT);

	buffer[0] = 0x27;
	buffer[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t *) &buffer, 2, I2C_TIMEOUT);

	buffer[0] = 0x1A;
	buffer[1] = 0x0A;
	HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t *) &buffer, 2, I2C_TIMEOUT);

	buffer[0] = 0x1B;
	buffer[1] = 0x0A;
	HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t *) &buffer, 2, I2C_TIMEOUT);

	buffer[0] = 0x1F;
	buffer[1] = 0x0F;
	HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t *) &buffer, 2, I2C_TIMEOUT);

	buffer[0] = 0x02;
	buffer[1] = 0x9E;
	HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t *) &buffer, 2, I2C_TIMEOUT);

}

void CS43L22_Beep(soundToneType pitch, uint32_t delay)
{
	uint8_t buffer[2];

	// Set volume
	buffer[0] = 0x1D;
	buffer[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t *) &buffer, 2, I2C_TIMEOUT);

	// Set note
	buffer[0] = 0x1C;
	buffer[1] = ((uint8_t)pitch << 4); // value from enum to upper bits
	HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t *) &buffer, 2, I2C_TIMEOUT);

	// Enable continuous mode (start)
	buffer[0] = 0x1E;
	buffer[1] = 0xC0;
	HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t *) &buffer, 2, I2C_TIMEOUT);

	// PLaying delay ms
	HAL_Delay(delay);

	// Disable continuous mode (stop)
	buffer[0] = 0x1E;
	buffer[1] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, CS43L22_I2C_ADDRESS, (uint8_t *) &buffer, 2, I2C_TIMEOUT);
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{

	if (htim -> Instance == TIM3)
		{
			led_state = (led_state + 1) % 4;

			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, (led_state == 0) ? 1 : 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, (led_state == 1) ? 1 : 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, (led_state == 2) ? 1 : 0);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, (led_state == 3) ? 1 : 0);
		}
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  CS43L22_Init();

  HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t *) dataI2S, 100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Wait user button press
	  bool buttonIsPressed = false;

	  while (!buttonIsPressed)
	  {
	  	  if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1)
	  	  {
	  		  buttonIsPressed = true;
	  		  HAL_Delay(500);
	   	  }
	  }
	  // Start TIM
	  HAL_TIM_Base_Start_IT(&htim3);

	  // First pin ON
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 1);

	  // Beethoven symphony melody
	  CS43L22_Beep(E5, 300);
	  HAL_Delay(30);
	  CS43L22_Beep(E5, 300);
	  HAL_Delay(30);
	  CS43L22_Beep(F5, 300);
	  HAL_Delay(30);
	  CS43L22_Beep(G5, 300);
	  HAL_Delay(30);

	  CS43L22_Beep(G5, 300);
	  HAL_Delay(30);
	  CS43L22_Beep(F5, 300);
	  HAL_Delay(30);
	  CS43L22_Beep(E5, 300);
	  HAL_Delay(30);
	  CS43L22_Beep(D5, 300);
	  HAL_Delay(30);

	  CS43L22_Beep(C5, 300);
	  HAL_Delay(30);
	  CS43L22_Beep(C5, 300);
	  HAL_Delay(30);
	  CS43L22_Beep(D5, 300);
	  HAL_Delay(30);
	  CS43L22_Beep(E5, 300);
	  HAL_Delay(30);

	  CS43L22_Beep(E5, 300);
	  HAL_Delay(30);
	  CS43L22_Beep(D5, 300);
	  HAL_Delay(30);

	  // Stop timer (because timer 330ms), next manual led setting
	  HAL_TIM_Base_Stop_IT(&htim3);
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 1);
	  CS43L22_Beep(D5, 600);
	  HAL_Delay(30);

	  // Reset all
	  led_state = 0;
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, 0);
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, 0);
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, 0);
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, 0);


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

  /** Macro to configure the PLL multiplication factor
  */
  __HAL_RCC_PLL_PLLM_CONFIG(16);

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSI);

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
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
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 330;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15
                           PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

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
