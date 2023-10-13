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
#include "i2c.h"
#include "rng.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "ILI9341_Touchscreen.h"

#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"

#include "snow_tiger.h"

#include "DFPLAYER_MINI.h"
#include "ds3231_for_stm32_hal.h"
#include "string.h"
//#include "dfr0299.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint32_t count = 0;
uint32_t starttime = 0;
uint32_t alarmtime = 0;
uint8_t screen = 1;
uint8_t sec = 0,min = 0,hur = 0,dow = 0,date = 0,month = 0;
uint32_t year = 0;
uint8_t alarmsec = 0,alarmmin = 0,alarmhour = 0;

char *day[7] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };
char *months[12] = { "January", "February", "March", "April", "May", "June", "July", "August", "September", "October", "November", "December" };

uint8_t seconds ;
uint8_t minutes;
uint8_t hours;

int alarmtrigger = 0;
float temp = 0.0;
float RH = 0.0;
uint8_t step = 0;
HAL_StatusTypeDef status;

uint8_t cmdBuffer[3];
uint8_t dataBuffer[8];

char strS[] ="";
char strM[] ="";
char strH[] ="";
char timedate[50];
char strdow[] ="";
char strdate[] ="";
char strmonth[] ="";
char stryear[] ="";
char strsensor[] ="";

uint16_t CRC16_2(uint8_t * , uint8_t);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

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

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_USART3_UART_Init();
  MX_SPI5_Init();
  MX_TIM1_Init();
  MX_RNG_Init();
  MX_I2C4_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  ILI9341_Init();//initial driver setup to drive ili9341
  HAL_TIM_Base_Start_IT(&htim2);
  DF_Init(30);
  /* USER CODE END 2 */
     DF_PlayFromStart();
     DF_SetEQ(3);
     DS3231_Init(&hi2c4);
    __disable_irq();
    	//Set interrupt mode to square wave mode, enable square wave interrupt at pin 3.
    	DS3231_SetInterruptMode(DS3231_SQUARE_WAVE_INTERRUPT);
    	//Set interrupting frequency to 1 Hz.
    	DS3231_SetRateSelect(DS3231_1HZ);
    	//Enable interrupts after finishing.
    	__enable_irq();

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    ILI9341_Fill_Screen(WHITE);
    ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
     starttime = count;

     cmdBuffer[0] = 0x03;
     cmdBuffer[1] = 0x00;
     cmdBuffer[2] = 0x04;
     //DS3231_SetFullDate(13, 10, 5, 2023);
     DS3231_SetInterruptMode(DS3231_ALARM_INTERRUPT);
     	DS3231_ClearAlarm1Flag();

     	DS3231_EnableAlarm1(DS3231_ENABLED);
     		DS3231_SetAlarm1Mode(DS3231_A1_MATCH_S_M_H);
     		DS3231_SetAlarm1Second(0);
     		DS3231_SetAlarm1Minute(29);
     		DS3231_SetAlarm1Hour(19);
     ILI9341_Draw_Rectangle(0,0, 320,200,  WHITE);
   while (1)
   {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	   if((count-starttime) > 1000 && screen == 1){
	    		updateT();
	    		showT();
	    		starttime = count;
	    	  }

	   Check_Key();


	    if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET){
	    		 HAL_Delay(50);
	    		if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET){
	    			DS3231_SetFullTime(21 , 12, 30);
	    		}
	    	 }
		if(alarmtrigger)
			   {
					ILI9341_Fill_Screen(WHITE);
				   alarm();
				   screen = 2;
				   alarmtime = count;
				   alarmtrigger = 0;
				   //HAL_Delay(3000)
			   }
		if(count - alarmtime > 3000 && screen == 2){
			ILI9341_Fill_Screen(WHITE);
			screen = 1;
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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint16_t CRC16_2(uint8_t *ptr, uint8_t lenght){
	uint16_t crc = 0xffff;
	uint8_t s = 0x00;

	while (lenght--){
		crc ^= *ptr++;

		for (s = 0 ; s <8 ;s++){
			if ((crc & 0x01) != 0){
				crc >>= 1;
				crc ^= 0xA001;

			}
			else{
				crc >>= 1;
			}
		}
	}
	return crc;
}

void updateSensor() {
			HAL_I2C_Master_Transmit(&hi2c1, 0x5c << 1 , cmdBuffer, 3, 200);

		  	HAL_I2C_Master_Transmit(&hi2c1, 0x5c << 1 , cmdBuffer, 3, 200);

		  	HAL_Delay(1);

		  	// receive sensor data

		  	HAL_I2C_Master_Receive(&hi2c1, 0x5c << 1 , dataBuffer , 8, 200);

		  	uint16_t Rcrc = dataBuffer[7] << 8 ;
		  	Rcrc += dataBuffer[6];

		  	if (Rcrc == CRC16_2(dataBuffer,6)){
		  		uint16_t temperature = ((dataBuffer[4] & 0x7F) << 8 ) + dataBuffer[5];

		  		temp = temperature  / 10.0 ;

		  		temp = (((dataBuffer[4] & 0x80) >> 7 ) == 1)? (temp * (-1)) : temp ;

		  		uint16_t humi = (dataBuffer[2] << 8) + dataBuffer[3] ;
		  		RH = humi / 10.0 ;
		  	}
}

void alarm(){
	ILI9341_Draw_Text("ALARM TRIGGER", 50,50, BLACK, 4, WHITE);
}

 void updateT(){
 	sec = DS3231_GetSecond();
 	min = DS3231_GetMinute();
 	hur = DS3231_GetHour();
 	dow = DS3231_GetDayOfWeek();
 	date = DS3231_GetDate();
 	month = DS3231_GetMonth()-1;
 	year = DS3231_GetYear();

 }
 void showT(){

	updateSensor();

 	sprintf(strH,"%02d",hur);
 	sprintf(strM,"%02d",min);
 	sprintf(strS,"%02d",sec);

 	sprintf(strsensor,"%.1f C %.1f %%RH",temp,RH);
 	//sprintf(strdow,"%s",day[1]);
	//sprintf(strdate,"%d",date);
//	sprintf(strmonth,"%s",months[month]);
	//sprintf(stryear,"%d",year);

 	snprintf(timedate,50,"%d %s %d",date, months[month], year);

 	ILI9341_Draw_Text(strH, 50,50, BLACK, 7, WHITE);
 	ILI9341_Draw_Text(strM, 120,110, BLACK, 7, WHITE);
 	ILI9341_Draw_Text(strS, 190,170, BLACK, 4, WHITE);
 	ILI9341_Draw_Text(day[dow], 10,10 , BLACK, 4, WHITE);
 	ILI9341_Draw_Text(strsensor, 80,40 , BLACK, 2, WHITE);
//	ILI9341_Draw_Text(strdate, 50,210, BLACK, 2, WHITE);
//	ILI9341_Draw_Text(months[month], 80,210, BLACK, 2, WHITE);
//	ILI9341_Draw_Text(stryear, 200,210, BLACK, 2, WHITE);
 	ILI9341_Draw_Text(timedate, 50,210, BLACK, 2, WHITE);

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
  while(1)
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
