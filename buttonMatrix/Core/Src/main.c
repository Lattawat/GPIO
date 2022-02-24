/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

uint16_t buttonState = 0;
uint16_t out = 0;
uint8_t status[3] = {1,0,0};
uint8_t currentL = 0;
uint8_t currentL2 = 0;
uint16_t operateTime = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void TestButtonMatrixRead();
void buttonMatrixRead();
uint8_t valueDecoder(uint16_t);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint32_t clock = 0;
	  uint16_t buttonVal[4] = {0};
	  static enum {init, correct, incorrect, check, waitForClear} State = init;
	  int8_t pass[] = {6,3,3,4,0,5,0,0,0,5,6};
	  uint8_t pos = 0;

	  switch(State){
	  default:
	  case init:
		  pos = 0;
		  clock = HAL_GetTick();
		  State = incorrect;
		  break;
	  case incorrect:
		  if(HAL_GetTick() - clock > 100 && pos < 11){
			  buttonMatrixRead();
			  if (status[0] == 0){
				  State = check;
			  }
			  else{
				  State = incorrect;
			  }
			  break;
		  }
	  case check:
		  if(HAL_GetTick() - clock > 100){
			  if (pass[pos] == valueDecoder(out)){
				  if(pos != 10){
					  pos++;
				  }
			  } //// input number
			  else if (pass[pos] != valueDecoder(out)){

				  if(valueDecoder(out) == 12){
					  pos--;
					  State == incorrect;
				  } //// input bs
				  else if(valueDecoder(out) == 11){
					  pos = 0;
					  State == incorrect;
				  } //// input clr
				  else if(valueDecoder(out) == 10){
					  if(pos != 10){
						  State = incorrect;
					  }
				  } //// input ok
				  else if(valueDecoder(out) == 13){
					  State = incorrect
				  } //// none
				  else{
					  State = waitForClear;
				  }
			  }
			  break;
		  }
	  case correct:
		  // wait for clear and reset
		  break;

	  case waitForClear:
		  //wait for clear
		  break;
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
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|L4_Pin|L1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(L2_GPIO_Port, L2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(L3_GPIO_Port, L3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : L4_Pin L1_Pin */
  GPIO_InitStruct.Pin = L4_Pin|L1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : L2_Pin */
  GPIO_InitStruct.Pin = L2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(L2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : R1_Pin */
  GPIO_InitStruct.Pin = R1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(R1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : R2_Pin R4_Pin R3_Pin */
  GPIO_InitStruct.Pin = R2_Pin|R4_Pin|R3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : L3_Pin */
  GPIO_InitStruct.Pin = L3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(L3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

GPIO_TypeDef* BMPortR[4] = {R1_GPIO_Port, R2_GPIO_Port, R3_GPIO_Port, R4_GPIO_Port};
uint16_t BMPinR[4] = {R1_Pin, R2_Pin, R3_Pin, R4_Pin};

GPIO_TypeDef* BMPortL[4] = {L1_GPIO_Port, L2_GPIO_Port, L3_GPIO_Port, L4_GPIO_Port};
uint16_t BMPinL[4] = {L1_Pin, L2_Pin, L3_Pin, L4_Pin};

void TestButtonMatrixRead(){
	static uint32_t Timestamp = 0;
	//uint16_t out = 0;

	if(HAL_GetTick() - Timestamp >= 100){
		operateTime = HAL_GetTick(); ///// try to find a runtime of this function
		Timestamp = HAL_GetTick();
		for(int i = 0; i<4 ; i++){
			if(HAL_GPIO_ReadPin(BMPortR[i], BMPinR[i]) == GPIO_PIN_RESET){
				buttonState = buttonState | (1 << (i + 4 * currentL));
			}
			else{
				buttonState = buttonState & ~(1 << (i + 4 * currentL));
			}
		}
		HAL_GPIO_WritePin(BMPortL[currentL], BMPinL[currentL], GPIO_PIN_SET);
		uint8_t nextL = (currentL + 1)%4;
		HAL_GPIO_WritePin(BMPortL[nextL], BMPinL[nextL], GPIO_PIN_RESET);
		currentL = nextL;
		operateTime = HAL_GetTick() - operateTime;
	}
}

void buttonMatrixRead(){
	static uint32_t Timestamp = 0;


	if(HAL_GetTick() - Timestamp >= 50){
		Timestamp = HAL_GetTick();
		for(int i = 0 ; i< 4 ; i++){
			if(status[0] == 0 && status[1] == i && status[2] == currentL2){
				status[0] = 0x01;
			}
			if(HAL_GPIO_ReadPin(BMPortR[i], BMPinR[i]) == GPIO_PIN_RESET && status[0] == 0x01){
				out =  (0 | (1 << (i + 4 * currentL2)));
				status[0] = 0;
				status[1] = i;
				status[2] = currentL2;
			}
		}
		HAL_GPIO_WritePin(BMPortL[currentL2], BMPinL[currentL2], GPIO_PIN_SET);
		uint8_t nextL = (currentL2 + 1)%4;
		HAL_GPIO_WritePin(BMPortL[nextL], BMPinL[nextL], GPIO_PIN_RESET);
		currentL2 = nextL;

	}
}

uint8_t valueDecoder(uint16_t buttonState){
	if(buttonState == 0b1000000000000000){
		return 10; //ok
	}
	else if(buttonState == 0b0100000000000000){
		return 13; //none
	}
	else if(buttonState == 0b0010000000000000){
		return 13; //none
	}
	else if(buttonState == 0b0001000000000000){
		return 0;
	}
	else if(buttonState == 0b100000000000){
		return 13; //none
	}
	else if(buttonState == 0b010000000000){
		return 3;
	}
	else if(buttonState == 0b001000000000){
		return 2;
	}
	else if(buttonState == 0b000100000000){
		return 1;
	}
	else if(buttonState == 0b10000000){
		return 12; //backspace
	}
	else if(buttonState == 0b01000000){
		return 6;
	}
	else if(buttonState == 0b00100000){
		return 5;
	}
	else if(buttonState == 0b00010000){
		return 4;
	}
	else if(buttonState == 0b1000){
		return 11; //clear
	}
	else if(buttonState == 0b0100){
		return 9;
	}
	else if(buttonState == 0b0010){
		return 8;
	}
	else if(buttonState == 0b0001){
		return 7;
	}
	else{
		return 13; // none
	}

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

