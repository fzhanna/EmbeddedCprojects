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
I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

/* USER CODE BEGIN PV */

//Defining Time and Date with RTC 
RTC_TimeTypeDef Time;
RTC_DateTypeDef Date;

char active[64];// defining variable for use throughout
typedef enum {second, minute, hour, weekday, month, date, year} Fields; //these fields will help us configuring all variables inclued
typedef enum {displayTime, displayDate, timeSet, saveTime, readSave} State; // states in our system 
State state = displayTime; // start point
Fields field = second; // start point
int StartTime = -1; // start time used for held SELECT
int IncrementSet = 0; //incrementing flag for timeSet



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//------------------ Display functions ----------------------------------------\\
// function for displaying current time 
void displayCTime(RTC_TimeTypeDef ts) {
	char timeString[8]; 
	sprintf(timeString, "%02d%02d%02d", ts.Hours, ts.Minutes, ts.Seconds);
	BSP_LCD_GLASS_DisplayString((uint8_t *) timeString);
	BSP_LCD_GLASS_DisplayChar( (uint8_t*) &timeString[1], POINT_OFF, DOUBLEPOINT_ON, LCD_DIGIT_POSITION_2);
	BSP_LCD_GLASS_DisplayChar( (uint8_t*) &timeString[3], POINT_OFF, DOUBLEPOINT_ON, LCD_DIGIT_POSITION_4);
}

//function for displaying current date
void displayCDate(RTC_DateTypeDef ds) {
	char dateString[35]; 
	sprintf(dateString, "YEAR %02d MONTH %02d DATE %02d WEEKDAY %02d", ds.Year, ds.Month, ds.Date, ds.WeekDay);
	BSP_LCD_GLASS_Clear();
	BSP_LCD_GLASS_ScrollSentence((uint8_t *)dateString, (uint16_t) 1, (uint16_t) 200);
}

//function for displaying saved time in EEPROM
void displayEEPROM( uint8_t seconds, uint8_t minutes, uint8_t hours,
	uint8_t seconds1, uint8_t minutes1, uint8_t hours1){
	char completeTimeString[32];
	sprintf(completeTimeString, "TIME1  %02d %02d %02d  TIME2  %02d %02d %02d", 
					hours , minutes, seconds, hours1 , minutes1, seconds1);
	BSP_LCD_GLASS_Clear();
	BSP_LCD_GLASS_ScrollSentence((uint8_t *) completeTimeString, (uint16_t) 1, (uint16_t) 400);

}
	
//-------------------------EEPROM-------------------------------------------------------- \\

#define EEPROM_ADDRESS  0xA0
__IO uint16_t memLocation = 0x000A;
uint16_t EE_status;

//Saving time data in EEPROM memory
void WriteEEPROM() {
	uint8_t oldSec = I2C_ByteRead(&hi2c1, EEPROM_ADDRESS, memLocation);
	uint8_t oldMin= I2C_ByteRead(&hi2c1, EEPROM_ADDRESS, memLocation+1);
	uint8_t oldHours = I2C_ByteRead(&hi2c1, EEPROM_ADDRESS, memLocation+2);
	I2C_ByteWrite(&hi2c1,EEPROM_ADDRESS, (uint16_t) memLocation, (uint8_t)Time.Seconds);
	I2C_ByteWrite(&hi2c1,EEPROM_ADDRESS, (uint16_t) memLocation+1, (uint8_t) Time.Minutes);
	I2C_ByteWrite(&hi2c1,EEPROM_ADDRESS, (uint16_t) memLocation+2, (uint8_t) Time.Hours);
	I2C_ByteWrite(&hi2c1,EEPROM_ADDRESS, (uint16_t) memLocation+3, (uint8_t) oldSec);
	I2C_ByteWrite(&hi2c1,EEPROM_ADDRESS, (uint16_t) memLocation+4, (uint8_t) oldMin);
	I2C_ByteWrite(&hi2c1,EEPROM_ADDRESS, (uint16_t) memLocation+5, (uint8_t) oldHours);
}
//Reading time data from EEPROM 
void ReadEEPROM(){
	uint8_t seconds1 = I2C_ByteRead(&hi2c1,EEPROM_ADDRESS, memLocation);
	uint8_t minutes1 = I2C_ByteRead(&hi2c1,EEPROM_ADDRESS, memLocation+1);
	uint8_t hours1 = I2C_ByteRead(&hi2c1,EEPROM_ADDRESS, memLocation+2);
	uint8_t seconds2 = I2C_ByteRead(&hi2c1,EEPROM_ADDRESS, memLocation+3);
	uint8_t minutes2 = I2C_ByteRead(&hi2c1,EEPROM_ADDRESS, memLocation+4);
	uint8_t hours2 = I2C_ByteRead(&hi2c1,EEPROM_ADDRESS, memLocation+5);
	displayEEPROM(seconds1,minutes1,hours1,seconds2,minutes2,hours2);
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

  /* USER CODE BEGIN Init */
	
	HAL_Init();
	BSP_LCD_GLASS_Init();
	BSP_JOY_Init(JOY_MODE_EXTI);
	BSP_LED_Init(LED4);
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//To differentiate  between a hold and a press for SELECT
			if (StartTime > -1 && state == displayTime){
				if (BSP_JOY_GetState() == JOY_SEL && HAL_GetTick()- StartTime > 1000)// if SELECT held for longer than 1000/1000 sec
				{
					StartTime = -1;//exit while loop
					state = displayDate;// shows date when SELECT held
				}
				else if (BSP_JOY_GetState() != JOY_SEL)// if select pressed, will save time in EEPROM
				{
					StartTime = -1;
					state = saveTime;
				}
		}
	

			switch(state){
				case displayTime:
					displayCTime(Time);// displays current time - default
					break;
			
				case displayDate:
					displayCDate(Date);// displays current date
					state=displayTime;// goes back current time state
					break;
				
				case timeSet:
				{
						//Gets current time and date
						HAL_RTC_GetTime(&hrtc , &Time, RTC_FORMAT_BIN);
						HAL_RTC_GetDate(&hrtc, &Date, RTC_FORMAT_BIN);
					// displays field being changed	
					switch (field){
							case (second):
								sprintf(active, "SEC %d", Time.Seconds);		
								break;
							case (minute):
								sprintf(active, "MIN %d", Time.Minutes);		
								break;
							case (hour):
								sprintf(active, "HR %d", Time.Hours);		
								break;
							case (weekday):
								sprintf(active, "WKD %d", Date.WeekDay);		
								break;
							case (month):
								sprintf(active, "MON %d", Date.Month);		
								break;
							case (date):
								sprintf(active, "DTE %d", Date.Date);		
								break;
							case (year):
								sprintf(active, "YR %d", Date.Year);		
								break;
							default:
								break;
						}
						//if incrementing is sensed will add increment by 1
						if ((IncrementSet==1) && (state==timeSet)){
	
							
							IncrementSet=0;
	
							//HAL_RTC_GetTime(&hrtc , &Time, RTC_FORMAT_BIN);
							//HAL_RTC_GetDate(&hrtc, &Date, RTC_FORMAT_BIN);
							
							switch (field){
								case (second):
									Time.Seconds = (Time.Seconds+1)%60;
									sprintf(active, "SEC %d", Time.Seconds);			
								case (minute):
									Time.Minutes = (Time.Minutes+1)%60;
									sprintf(active, "MIN %d", Time.Minutes);		
									break;
								case (hour):
									Time.Hours = (Time.Hours+1)%24;
									sprintf(active, "HR %d", Time.Hours);		
									break;
								case (weekday):
									Date.WeekDay = (Date.WeekDay+0x01U)%0x7U;
									sprintf(active, "WKD %d", Date.WeekDay);		
									break;
								case (month):
									Date.Month = (Date.Month+0x01U)%0x0CU;
									sprintf(active, "MON %d", Date.Month);		
									break;
								case (date):
									Date.Date = (Date.Date+1)%32;
									sprintf(active, "DTE %d", Date.Date);		
									break;
								case (year):
									Date.Year = (Date.Year+1)%100;
									sprintf(active, "YR %d", Date.Year);		
									break;
								default:
									break;}
							
							//clear and displays time
							BSP_LCD_GLASS_Clear();
							BSP_LCD_GLASS_DisplayString((uint8_t *) active);
							HAL_RTC_SetTime(&hrtc, &Time, RTC_FORMAT_BIN);
							HAL_RTC_SetDate(&hrtc, &Date, RTC_FORMAT_BIN);
		}

						BSP_LCD_GLASS_Clear();
						BSP_LCD_GLASS_DisplayString((uint8_t *) active);

								break;
				}
								
				case saveTime:{ 
						WriteEEPROM();// saves time in EEPROM 
						BSP_LCD_GLASS_Clear();
						BSP_LCD_GLASS_DisplayString((uint8_t*) "SAVE");
						HAL_Delay(1000);// display SAVE for 1 sec
						state = displayTime;
						break;
				  }
				
				case readSave:
					ReadEEPROM();
					state=displayTime;
					break;	
		}	
	}
	}



  /* USER CODE END 3 */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
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
  hi2c1.Init.Timing = 0x00000E14;
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 12;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_TUESDAY;
  sDate.Month = RTC_MONTH_NOVEMBER;
  sDate.Date = 10;
  sDate.Year = 20;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0;
  sAlarm.AlarmTime.Minutes = 0;
  sAlarm.AlarmTime.Seconds = 0;
  sAlarm.AlarmTime.SubSeconds = 0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_ALL;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
{
	BSP_LED_Toggle(LED4);// RED LEDsys blinks every 1s
	HAL_RTC_GetTime(hrtc , &Time, RTC_FORMAT_BIN); //time and date being changed every 1s
	HAL_RTC_GetDate(hrtc, &Date, RTC_FORMAT_BIN);
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
 switch (GPIO_Pin) {
 case GPIO_PIN_0: //SELECT button
	if (state == displayTime) StartTime = HAL_GetTick();// when SELECT held during displayTime state we get StartTime to be counting milliseconds of how long SELECT is held
	else if (state == timeSet)IncrementSet = 1; //when on timeSET, triggers increment flag to shift field by 1
	break;
 
 case GPIO_PIN_1: //left button
   if (state==displayTime)state=readSave;//Reads saved times in EEPROM when in displayTime state and left pressed
	 else if (state==readSave)state=displayTime;//go back to displayTime when reading EEPROM
	 else if (state== timeSet)field= (field+1)%7;// shifting in Fields
   break;
 
 case GPIO_PIN_2: //right button
	 if (state==displayTime)state=timeSet;// timeSet from displayTime and right pressed
	 else if (state==timeSet)state = displayTime;//goes back to displayTime from timeSet when right pressed
	 break;
	
 default://
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
