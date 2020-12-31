/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
GPIO_PinState translate(int bin);// defining function that translates from binary to SET 
//and RESET functions for the step motor

//Full step binaries; sequences used for conducting full steps 
int fullA[4] = { 1,1,0,0 };
int fullB[4] = { 0,0,1,1 };
int fullC[4] = { 0,1,1,0 };
int fullD[4] = { 1,0,0,1 };

//Half step binaries; sequences used for conducting half steps 
int halfA[8] = { 1,1,1,0,0,0,0,0 };
int halfB[8] = { 0,0,0,0,1,1,1,0 };
int halfC[8] = { 0,0,1,1,1,0,0,0 };
int halfD[8] = { 1,0,0,0,0,0,1,1 };


int i = 0, direction = 1, PSC=399;// i variable used for iterating through arrays above
//default direction is CW // Prescaler set to 399 as default
char lcd_buffer[6];    // LCD display buffer

//enumerator used for defining which step to use
typedef enum {FullStep, HalfStep} Step; 
Step step= FullStep;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
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
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	BSP_LCD_GLASS_Init();
	HAL_TIM_Base_Start_IT(&htim3);
	BSP_LCD_GLASS_Clear();
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  htim3.Init.Prescaler = 399;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 4999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LEDR_GPIO_Port, LEDR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LEDG_Pin|OUT_A_Pin|OUT_B_Pin|OUT_C_Pin
                          |OUT_D_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : JOY_CENTER_Pin */
  GPIO_InitStruct.Pin = JOY_CENTER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(JOY_CENTER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : JOY_LEFT_Pin JOY_RIGHT_Pin JOY_UP_Pin JOY_DOWN_Pin */
  GPIO_InitStruct.Pin = JOY_LEFT_Pin|JOY_RIGHT_Pin|JOY_UP_Pin|JOY_DOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LEDR_Pin */
  GPIO_InitStruct.Pin = LEDR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LEDR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LEDG_Pin OUT_A_Pin OUT_B_Pin OUT_C_Pin
                           OUT_D_Pin */
  GPIO_InitStruct.Pin = LEDG_Pin|OUT_A_Pin|OUT_B_Pin|OUT_C_Pin
                          |OUT_D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
int i_update(int i, int period) { //Updates i value according to direction and period
	
	// in the effects of a negative direction (opposite rotation)
	i+=direction;
	if (i>=0) return (i%period);//if i is positive will iterate through period normally
	else return period-1;//else will reverse iteration pattern to accomodate for opposite direction
}
GPIO_PinState translate(int bin){ // translates binary to pin SET and RESET
	if (bin==1) return GPIO_PIN_SET;// if binary ==1, will set pin
	else return GPIO_PIN_RESET;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  if((*htim).Instance==TIM3) HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_8);//every step is a new action and LED 
	//goes off
		
		switch(step){//switch for fullstep and half step
		
		case FullStep:
		HAL_GPIO_WritePin(OUT_A_GPIO_Port, OUT_A_Pin, translate(fullA[i]));
		HAL_GPIO_WritePin(OUT_B_GPIO_Port, OUT_B_Pin, translate(fullB[i]));
		HAL_GPIO_WritePin(OUT_C_GPIO_Port, OUT_C_Pin, translate(fullC[i]));
		HAL_GPIO_WritePin(OUT_D_GPIO_Port, OUT_D_Pin, translate(fullD[i]));
		i = i_update(i, 4); // recursive function used to iterate through full step 
		//binary sequence mentioned above
		
			break;
		case HalfStep:	// same as full step 		
		HAL_GPIO_WritePin(OUT_A_GPIO_Port, OUT_A_Pin, translate(halfA[i]));
		HAL_GPIO_WritePin(OUT_B_GPIO_Port, OUT_B_Pin, translate(halfB[i]));
		HAL_GPIO_WritePin(OUT_C_GPIO_Port, OUT_C_Pin, translate(halfC[i]));
		HAL_GPIO_WritePin(OUT_D_GPIO_Port, OUT_D_Pin, translate(halfD[i]));
		i = i_update(i, 8);
			break;
	}

}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	switch (GPIO_Pin) {
		case JOY_CENTER_Pin:      //Press select to move from fullstep to halfstep and viceversa 

			if (step==HalfStep) {
			BSP_LCD_GLASS_Clear();
			htim3.Instance->ARR = 13750;// setting autoreload register to full step ticks/second 
			BSP_LCD_GLASS_DisplayString((uint8_t *) "FULL");
			step=FullStep;
			}
			else { 
			BSP_LCD_GLASS_Clear();
			htim3.Instance->ARR = 6875;// setting autoreload register to halfstep ticks/second 
			BSP_LCD_GLASS_DisplayString((uint8_t *) "HALF");
			step=HalfStep;
				}
			break;    
		case JOY_LEFT_Pin:     //CCW rotation
			BSP_LCD_GLASS_Clear(); 
			BSP_LCD_GLASS_DisplayString((uint8_t *) "CCW");
			direction = 1;// 
			break;
		case JOY_RIGHT_Pin:    //CW rotation
			BSP_LCD_GLASS_Clear(); 
			BSP_LCD_GLASS_DisplayString((uint8_t *) "CW");
			direction = -1;
			break;
		
		case JOY_UP_Pin:   // increase speed
			PSC-=50;//decreasing prescaler increases frequency
			if (PSC<100) PSC=100;
		  __HAL_TIM_SET_PRESCALER(&htim3, PSC);
			sprintf(lcd_buffer, "PSC%d",PSC);
			BSP_LCD_GLASS_DisplayString((uint8_t *) lcd_buffer);
			break;
		
		case JOY_DOWN_Pin:    // decrease speed
			PSC+=50;//increasing prescaler decreases frequency
			if (PSC>1000) PSC=1000;
		  __HAL_TIM_SET_PRESCALER(&htim3, PSC);
			BSP_LCD_GLASS_Clear(); 
			sprintf(lcd_buffer, "PSC%d",PSC);
			BSP_LCD_GLASS_DisplayString((uint8_t *) lcd_buffer);		
			break;
		
		default:
			break;
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
