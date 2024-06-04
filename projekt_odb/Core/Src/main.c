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
#include "tim.h"

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
UART_HandleTypeDef huart2;

uint16_t silniki[4] = {0};

/* USER CODE BEGIN PV */
struct us_sensor_str distance_sensor;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  hc_sr04_init(&distance_sensor, &htim1, &htim2, TIM_CHANNEL_3);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
      HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
      HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

      __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 0);
      __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);
      __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
      __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 0);

      HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(ENB_GPIO_Port, ENB_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
      uint8_t stan_init[5] = {0};
      uint8_t stan[5] = {0};
      char message[100] = {0};
      uint8_t D1, D2, D3, D4, D5 = 0;
      	  wyjscie:
      	 stan_init[0] = 0;
      	 stan[0] = 0;
      	 // Odczyt trybu pracy	b -- bluetooth, l -- Jazda po linii, o -- czujnik odbiciowy
      	 while(stan_init[0] != 'b' && stan_init[0] != 'l' && stan_init[0] != 'o') {
      		HAL_UART_Receive(&huart1, stan_init, 5, 100);
      	 }
            while (1) {
			// Ustawienie czujników listwy z czujnikami odbiciowymi
            D1 = HAL_GPIO_ReadPin(D1_GPIO_Port, D1_Pin);
            D2 = HAL_GPIO_ReadPin(D2_GPIO_Port, D2_Pin);
            D3 = HAL_GPIO_ReadPin(D3_GPIO_Port, D3_Pin);
            D4 = HAL_GPIO_ReadPin(D4_GPIO_Port, D4_Pin);
            D5 = HAL_GPIO_ReadPin(D5_GPIO_Port, D5_Pin);

            //Ustawienie sterowania pojazdem na bluetooth
            if (stan_init[0] == 'b') {
            	 HAL_UART_Receive(&huart1, stan, 5, 0);
            	 // Jazda do przodu
            	if (stan[0]=='w') {
					__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 0);
					__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 500);
					__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
					__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 500);
				}
            	// Jazda do tyłu
				if (stan[0]=='s') {
					__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 500);
					__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);
					__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 500);
					__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 0);
				}
				// Skręt w lewo
				if (stan[0]=='a') {
					__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 0);
					__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);
					__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
					__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 500);
				}
				// Skręt w prawo
				if (stan[0]=='d') {
					__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 0);
					__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 500);
					__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
					__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 0);
				}
				// Zatrzymaj się
				if (stan[0]=='x') {
					__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 0);
					__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);
					__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
					__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 0);
				}
				// Wyjście z trybu
				if (stan[0]=='e') {
					__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 0);
					__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);
					__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
					__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 0);
					goto wyjscie;
				}
            }

            //Ustawienie sterowania pojazdem z czujnikiem odbiciowym
            	if (stan_init[0]=='o'){
            		HAL_UART_Receive(&huart1, stan, 5, 0);
            		if (stan[0]=='e') {
            			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 0);
						__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);
						__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
						__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 0);
						goto wyjscie;
					}
            		if(distance_sensor.distance_cm<=10){
            			// Gdy dystans do przeszkoday mniejszy niż 10 cm to pojazd cofa
						__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 400);
						__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);
						__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 400);
						__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 0);
					} else if(distance_sensor.distance_cm<=40){
						// Gdy dystans do przeszkoday mniejszy niż 40 cm to pojazd skęreca w lewo
            			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 0);
            			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);
            			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
            			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 400);
            		} else if(distance_sensor.distance_cm<=70){
            			// Gdy dystans do przeszkoday mniejszy niż 70 cm to pojazd delikatnie skręca w lewo
						__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 0);
						__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 300);
						__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
						__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 400);
					}  else {
						// Jazda do przodu gdy inne warunki nie sa spełnione
            			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 0);
            			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 400);
            			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
            			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 400);
            		}

            	}
            	//Ustawienie sterowania pojazdem po linii
            	if(stan_init[0]=='l') {
            		HAL_UART_Receive(&huart1, stan, 5, 0);
            		if (stan[0]=='e') {\
            			__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 0);
						__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);
						__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
						__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 0);
						goto wyjscie;
					}
					D1 = HAL_GPIO_ReadPin(D1_GPIO_Port, D1_Pin);
					D2 = HAL_GPIO_ReadPin(D2_GPIO_Port, D2_Pin);
					D3 = HAL_GPIO_ReadPin(D3_GPIO_Port, D3_Pin);
					D4 = HAL_GPIO_ReadPin(D4_GPIO_Port, D4_Pin);
					D5 = HAL_GPIO_ReadPin(D5_GPIO_Port, D5_Pin);

            /*snprintf(message, sizeof(message), " %d %d %d %d %d", D1, D2, D3, D4, D5);
                        HAL_UART_Transmit(&huart2, message, 100, 100);
                      	HAL_UART_Transmit(&huart1, message, 100, 100);*/
			  if (D2==1 && D3==1 && D4==1 || D2==1 && D3==1 || D3 == 1 && D4==1 || D3 == 1) {
				__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
				__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 500);
				__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 0);
				__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 500);
				silniki[0] = 0;
				silniki[1] = 500;
				silniki[2] = 0;
				silniki[3] = 500;
			  }
			  else if (D1==1 && D2==1 && D3==1 && D4 == 1 || D1==1 && D2==1 && D3==1 || D1==1 && D2==1 || D1 == 1) {
				__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
				__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 500);
				__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 0);
				__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);
				silniki[0] = 0;
				silniki[1] = 500;
				silniki[2] = 0;
				silniki[3] = 0;
			  }
			  else if (D2==1 && D3==1 && D4==1 && D5==1 || D3==1 && D4==1 && D5==1 || D4==1 && D5==1 || D5 == 1) {
				__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 0);
				__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, 0);
				__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 0);
				__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 500);
				silniki[0] = 0;
				silniki[1] = 0;
				silniki[2] = 0;
				silniki[3] = 500;
			  }
			  else {
				__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, silniki[0]);
				__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, silniki[1]);
				__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, silniki[2]);
				__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, silniki[3]);
			  }

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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
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
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */


/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */


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
  HAL_GPIO_WritePin(ENA_GPIO_Port, ENA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ENB_GPIO_Port, ENB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ENA_Pin */
  GPIO_InitStruct.Pin = ENA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ENA_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = D5_Pin|D4_Pin|D3_Pin|D2_Pin
                          |D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ENB_Pin */
  GPIO_InitStruct.Pin = ENB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ENB_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(TIM1 == htim->Instance)
	{
		uint32_t echo_us;

		echo_us = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		distance_sensor.distance_cm = hc_sr04_convert_us_to_cm(echo_us);
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
