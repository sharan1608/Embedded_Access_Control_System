/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include"i2c-lcd.h"
#include"rc522.h"
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

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t rxDATA[5];
uint8_t txDATA[25];
uchar id[] = {"\nbe4dc27b4a"};
uint8_t idlen, count;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void usDelay(uint32_t uSec);
void stepper_set_rpm (int rpm);
void stepper_half_drive (int step);
void stepper_step_angle (float angle, int direction, int rpm);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE BEGIN 0 */
// Speed of Sound in CM/uSec
const float speedofsound = 0.0343;
float distance;

char uartBuff[100];
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t numTicks;
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
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  lcd_init();
  MFRC522_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  lcd_send_cmd(0x80|0x00); //1st line
    lcd_send_string("Welcome - scan card");

  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //set TRIG pin to low for few sec
	  HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_RESET);
	  usDelay(3);

	  /**** START Ultrasonic measure routine*/
	  //1. Output 10 uSec TRIG
	  HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_SET);
	  usDelay(10);
	  HAL_GPIO_WritePin(US_TRIG_GPIO_Port, US_TRIG_Pin, GPIO_PIN_RESET);

	  //2.Wait for ECHO pin Rising Edge
	  while(HAL_GPIO_ReadPin(US_ECHO_GPIO_Port, US_ECHO_Pin) == GPIO_PIN_RESET);

	  //3.Start measuring ECHO pulse width in uSec
	  numTicks = 0;
	  while(HAL_GPIO_ReadPin(US_ECHO_GPIO_Port, US_ECHO_Pin) == GPIO_PIN_SET)
	  {
		  numTicks++;
		  usDelay(2);
	  };
	  //4. Estimate distance in CM
	  distance = (numTicks + 0.0f)*2.8*speedofsound;
	  if (distance <= 10)
	  {
		  lcd_send_cmd(0x80|0x40); //2nd line
		  lcd_send_string("Welcome Sharanyan");
	  }
	  else
	  {
		  lcd_send_cmd(0x80|0x40); //2nd line
		  lcd_send_string("Please move closer");
	  }
	  //5. Print to UART Termianal
	  HAL_Delay(500);
	  sprintf(uartBuff, "Distance (CM) = %0.1f\n", distance);
	  HAL_UART_Transmit(&huart2, (uint8_t *)uartBuff,strlen(uartBuff), 100);



	  /* RFID check*/

	  MFRC522_Request(PICC_REQIDL, &rxDATA);
	  HAL_Delay(300);
	  MFRC522_Anticoll(&rxDATA);
	  //HAL_Delay(300);

	  sprintf(txDATA, "\n%x%x%x%x%x", rxDATA[0],rxDATA[1],rxDATA[2],rxDATA[3],rxDATA[4]);
	  HAL_UART_Transmit(&huart2, (uint8_t *)txDATA,25, 500);
	  HAL_Delay(300);

	  idlen = strlen(id);
	  count = 0;

	  for(uint8_t i = 0; i <idlen;i++)
	  {
		  if(id[i] == txDATA[i])
		  {
			  count++;
		  }
	  }

	  if(count == 11)
	  {
		  lcd_send_cmd (0x80|0x14); //3rd line
		  lcd_send_string("Access Granted");

		  lcd_send_cmd (0x80|0x54); //4th line
		  lcd_send_string("Have a wonderful day");
		  HAL_Delay(1000);
	  }
	  if(count != 11)
	  {
		  //verify = 0;
		  lcd_send_cmd (0x80|0x14); //3rd line
		  lcd_send_string("Access Denied");

		  lcd_send_cmd (0x80|0x54); //4th line
		  lcd_send_string("Invalid user");
		  HAL_Delay(1000);
	  }

	  if(distance <= 10 && count == 11)
	  {
		  stepper_step_angle(90,0,10);
		  HAL_Delay(500);
		  stepper_step_angle(90,1,10);
		  HAL_Delay(500);
	  }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
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
  hi2c1.Init.Timing = 0x10909CEC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
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
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 80-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, NSS_Pin|US_TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IN4_Pin|IN1_Pin|LD3_Pin|IN2_Pin
                          |IN3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : NSS_Pin US_TRIG_Pin */
  GPIO_InitStruct.Pin = NSS_Pin|US_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IN4_Pin IN1_Pin LD3_Pin IN2_Pin
                           IN3_Pin */
  GPIO_InitStruct.Pin = IN4_Pin|IN1_Pin|LD3_Pin|IN2_Pin
                          |IN3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : US_ECHO_Pin */
  GPIO_InitStruct.Pin = US_ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(US_ECHO_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void stepper_set_rpm (int rpm)  // Set rpm--> max 13, min 1,,,  went to 14 rev/min
{
	usDelay(60000000/4096/rpm); // check this!!!!
}

void stepper_half_drive (int step)
{
	switch (step){
		case 0:
			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);   // IN1
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);   // IN2
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);   // IN3
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);   // IN4
			  break;

		case 1:
			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);   // IN1
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);   // IN2
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);   // IN3
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);   // IN4
			  break;

		case 2:
			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);   // IN1
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);   // IN2
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);   // IN3
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);   // IN4
			  break;

		case 3:
			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);   // IN1
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);   // IN2
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);   // IN3
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);   // IN4
			  break;

		case 4:
			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);   // IN1
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);   // IN2
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);   // IN3
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);   // IN4
			  break;

		case 5:
			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);   // IN1
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);   // IN2
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);   // IN3
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);   // IN4
			  break;

		case 6:
			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);   // IN1
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);   // IN2
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);   // IN3
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);   // IN4
			  break;

		case 7:
			  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);   // IN1
			  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);   // IN2
			  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);   // IN3
			  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);   // IN4
			  break;

		}
}

void stepper_step_angle (float angle, int direction, int rpm)
{
	float anglepersequence = 0.703125;  // 360 = 512 sequences
	int numberofsequences = (int) (angle/anglepersequence);

	for (int seq=0; seq<numberofsequences; seq++)
	{
		if (direction == 0)  // for clockwise
		{
			for (int step=7; step>=0; step--)
			{
				stepper_half_drive(step);
				stepper_set_rpm(rpm);
			}

		}

		else if (direction == 1)  // for anti-clockwise
		{
			for (int step=0; step<8; step++)
			{
				stepper_half_drive(step);
				stepper_set_rpm(rpm);
			}
		}
	}
}
void usDelay(uint32_t uSec)
{
	if(uSec < 2) uSec = 2;
		TIM1->ARR = uSec - 1; 	/*sets the value in the auto-reload register*/
		TIM1->EGR = 1; 			/*Re-initialises the timer*/
		TIM1->SR &= ~1; 		//Resets the flag
		TIM1->CR1 |= 1; 		//Enables the counter
		while((TIM1->SR&0x0001) != 1);
		TIM1->SR &= ~(0x0001);
}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
