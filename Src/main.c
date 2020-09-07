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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */
extern TIM_HandleTypeDef htim1, htim3, htim6, htim15;
extern UART_HandleTypeDef huart3;
extern ADC_HandleTypeDef hadc;
uint8_t working = 1;
uint8_t buffer[100];
uint32_t bufferIdx = 0;
uint32_t adcVal = 0;//*0.025=pad Voltage
uint32_t adcBuffer[10];
uint32_t adcBufferIdx = 0;
static uint32_t max_mV = 45000, min_mV = 5000;
uint32_t target_mV = 15000;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
uint16_t strLen(uint8_t *str){
	uint16_t i;
	for(i = 0; str[i] != '\r'; i++);
	return i + 1;
}
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
  MX_DMA_Init();
  MX_ADC_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM15_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_ADC_Start_DMA(&hadc, &adcVal, 1);
//  HAL_UART_Receive_IT(&huart3, &buffer[bufferIdx++], 1);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  for(int i = 0; i < 50000; i++);
  TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_4, TIM_CCx_DISABLE);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (working)
  {
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
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
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART3)
	{
		  HAL_UART_Receive_IT(&huart3, &buffer[bufferIdx], 1);
		  bufferIdx = ++bufferIdx % 100;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{
		case GPIO_PIN_0:
			target_mV += 5000;
			if(target_mV > max_mV)
				target_mV = max_mV;
			break;
		case GPIO_PIN_1:
			target_mV -= 5000;
			if(target_mV < min_mV)
				target_mV = min_mV;
			break;
		case GPIO_PIN_2:
			htim6.Instance->CNT = 0;
			htim6.Instance->SR = 0;
			HAL_TIM_Base_Start_IT(&htim6);
			TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_4, TIM_CCx_ENABLE);
			for(int i = 0; i < 30000; i++);
			TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_4, TIM_CCx_DISABLE);
			break;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM6)
	{
		htim6.Instance->CNT = 0;
		HAL_TIM_Base_Stop_IT(&htim6);
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_SET)
		{
			TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_4, TIM_CCx_ENABLE);
			for(int i = 0; i < 30000; i++);
			TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_4, TIM_CCx_DISABLE);
			for(int i = 0; i < 30000; i++);
			TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_4, TIM_CCx_ENABLE);
			for(int i = 0; i < 30000; i++);
			TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_4, TIM_CCx_DISABLE);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			working = 0;
		}
	}
	else if(htim->Instance == TIM7)
	{
		htim15.Instance->CCR1 = htim15.Instance->CCR2 = 10;
		while(htim7.Instance->CR1 & 0x01 && htim7.Instance->CNT < 5);
		htim15.Instance->CCR1 = 0;
		htim15.Instance->CCR2 = 20;
		adcBuffer[adcBufferIdx++] = adcVal * 25;
		if(adcBufferIdx >= 10)
		{
			uint32_t avg = 0;
			for(uint32_t i = 0; i < 10; i++)
			{
				avg += adcBuffer[i];
			}
			avg /= 10;
			uint32_t pulse = htim3.Instance->CCR1;
			if(target_mV > avg)
			{
				if(target_mV - avg > 5000)
					pulse += 5;
				else
					pulse += 1 + (target_mV - avg)/1000;
				if(pulse > 200)
					pulse = 200;
			}
			else
			{
				if(avg - target_mV > 5000)
					pulse -= 5;
				else
					pulse -= (avg - target_mV)/1000;
				if(pulse < 10)
					pulse = 10;
			}
			htim3.Instance->CCR1 = pulse;
			adcBufferIdx = 0;
		}
	}
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
