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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	//------------------------------------------------------

	//*---------สร้างตัวแปร----------*//
	GPIO_PinState SwitchState[2];  		// Now, Previous ----> ใช้สำหรับรับค่า 0 Now, 1 Previous [GPIO_PinState สามารถเขียนเป็น uint8_t ได้ แค่ให้รับตัวเลข 0 1 ได้]
	uint16_t LED1_Half_Period = 1000;  	// 1 Hz (mS)
	uint32_t TimeStamp = 0;				;        //ตัวเก็บเวลา Typeเดียวกับ HAL_GetTick()
	uint32_t ButtonTimeStamp = 0;
	GPIO_PinState Check[2];
	uint32_t CheckS = 0;
	GPIO_PinState FIBO[2];
	uint32_t FIBOS = 0;
	uint16_t LED3_Half_Period = 500;
	uint32_t TimeStamp3 = 0;

	//------------------------------------------------------
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1) {
//*----------HAL_GPIO_ReadPin()---------*//
		HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10); //S1 PA10
		HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);  //S2 PB3
		HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);  //S3 PB5

//*---------------ลดความเบลอของปุ่ม---------------*//
		if (HAL_GetTick() - ButtonTimeStamp >= 100)
		{
			ButtonTimeStamp = HAL_GetTick(); 	// HAL_GetTick(); ---> อ่านเวลาที่เริ่มทำงานจนถึงปัจจุบัน(mS)
//-----------------อ่านค่าของPinต่างๆ-------------------*//
			SwitchState[0] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);
			Check[0] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
			FIBO[0] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
			//Press = Low , No = High

//------------------------------------PART 1-------------------------------------//
			if (SwitchState[1] == GPIO_PIN_SET && SwitchState[0] == GPIO_PIN_RESET)  // ถ้า Pre เป็น Hi Now เป็น Lo
			{
				// set = high , reset = low
				//Change Half Period of LED 1
				if (LED1_Half_Period == 1000)				//NOW
				{
					LED1_Half_Period = 500;   				//NEXT
				}
				else if (LED1_Half_Period == 500)
				{
					LED1_Half_Period = 250;
				}
				else if (LED1_Half_Period == 250)
				{
					LED1_Half_Period = 167;
				}
				else
				{
					LED1_Half_Period = 1000;
				}
			}


//------------------------------------PART 2-------------------------------------//
			if (Check[0] == GPIO_PIN_RESET && Check[1] == GPIO_PIN_SET) // ถ้า  Now เป็น Lo Pre เป็น Hi
			{
				if (CheckS == 0)
				{
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
					CheckS = 1;
				}
				else
				{
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
					CheckS = 0;
				}
			}

//------------------------------------PART 3-------------------------------------//
			if (FIBO[0] == GPIO_PIN_RESET && FIBO[1] == GPIO_PIN_SET) // ถ้า  Now เป็น Lo Pre เป็น Hi
			{
				if (FIBOS == 0)
				{
					LED3_Half_Period = 500;
					FIBOS = 1;
				}
				else
				{
					LED3_Half_Period = 3000;
					FIBOS = 0;
				}
			}
//----------------------Save Now State------------------------//
			SwitchState[1] = SwitchState[0];
			Check[1] = Check[0];
			FIBO[1] = FIBO[0];
		}

//Run LED ZONE!
//------------------------------------PART 1-------------------------------------//
		if (HAL_GetTick() - TimeStamp >= LED1_Half_Period) //millisecond now time
				{
			TimeStamp = HAL_GetTick();       									  //บันทึกเวลาปัจจุบัน เพื่อให้รู้ว่ารอบต่อไปต้องใช้กี่วินาที
			if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9) == GPIO_PIN_SET) 			 //D1 PA9
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);            //สามารถอ่าน (HAL_GPIO_ReadPin();) PIN state ปัจจุบันได้เสมอ ไม่เกี่ยวว่าเป็น Output/Input
			} 																	  //แต่จะเขียนได้ก็ต่อเมื่อเป็น Output เท่านั้น HAL_GPIO_Write();
			else
			{
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
			}
		}

		//------------------------------------PART 2-------------------------------------//
		if (FIBOS == 1)
		{
			if (HAL_GetTick() - TimeStamp3 >= LED3_Half_Period) //millisecond now time
					{
				TimeStamp3 = HAL_GetTick();
				if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET)
				{
					LED3_Half_Period = 500;
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
				}
				else
				{
					LED3_Half_Period = 3000;
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
				}
			}
		}

		//------------------------------------PART 3-------------------------------------//
		if (FIBOS == 0)
		{
			if (HAL_GetTick() - TimeStamp3 >= LED3_Half_Period) //millisecond now time
					{
				TimeStamp3 = HAL_GetTick();
				if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_SET)
				{
					LED3_Half_Period = 3000;
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
				}
				else
				{
					LED3_Half_Period = 500;
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
				}
			}
		}

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
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 16;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
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

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, LD2_Pin | GPIO_PIN_9, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LD2_Pin PA9 */
	GPIO_InitStruct.Pin = LD2_Pin | GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PC7 */
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PA10 */
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB3 PB5 */
	GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PB6 */
	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
