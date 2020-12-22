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
#include "crc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "stdio.h"
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
extern CRC_HandleTypeDef hcrc;
PAD_ControlData pad;
UART_ReceiveDataTypeDef rduart3;
ADC_Result adc;

uint8_t working = 1;
uint8_t padState = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Beep()
{
	TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_4, TIM_CCx_ENABLE);//buzzer on
	for(int i = 0; i < 10000; i++);
	TIM_CCxChannelCmd(htim1.Instance, TIM_CHANNEL_4, TIM_CCx_DISABLE);//buzzer off
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
uint8_t tx_str[] = "ATZ\r";
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  UART_ReceiveData_Init(&rduart3, &huart3);
  PAD_ControlData_Init(&pad);
  /* USER CODE END Init */


  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();//adc
  MX_ADC_Init();//pad voltage
  MX_TIM1_Init();//buzzer pwm
  MX_TIM3_Init();//pad vdc pwm
  MX_TIM6_Init();//power button timer interrupt
  //MX_TIM7_Init();//pad timer interrupt
  MX_TIM15_Init();//pad pwm
  MX_USART3_UART_Init();
  ADC_Result_Init(&adc, &hadc);
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  HAL_TIM_Base_Start_IT(&htim3);
  //HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start_IT(&htim15);

  UART_ReceiveData(&rduart3);

  Beep();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);//init ble module
  for(int i = 0; i < 10000; ++i);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
  for(int i = 0; i < 10000; ++i);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
  //HAL_UART_Transmit_IT(&huart3, tx_str, sizeof(tx_str) - 1);


  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (working)
  {
	  if(rduart3.cmdUF)
	  {
		  Beep();
		  HAL_UART_Transmit(&huart3, rduart3.cmdBuffer, rduart3.cmdBufferLen, 100);
		  if(CmdCmp(&rduart3, "+", 1))
		  {
			  PAD_VoltageUp(&pad);
		  }
		  else if(CmdCmp(&rduart3, "-", 1))
		  {
			  PAD_VoltageDown(&pad);
		  }
		  else if(CmdCmp(&rduart3, "O", 1))
		  {
			  padState = 1;
		  }
		  else if(CmdCmp(&rduart3, "X", 1))
		  {
			  padState = 0;
		  }
		  else if(CmdCmp(&rduart3, "OFF", 3))
		  {
			  working = 0;
		  }
		  /*else if(CmdCmp(&rduart3, "S", 1))
		  {
			  pad.pulseType = SQUARE_WAVE;
		  }
		  else if(CmdCmp(&rduart3, "R", 1))
		  {
			  pad.pulseType = RAMP_WAVE;
		  }*/
		  else if(rduart3.cmdBuffer[0] == 'C')
		  {
			  int tmp;
			  sscanf((char *)&rduart3.cmdBuffer[1], "%d", &tmp);
			  PAD_ChangeCount(&pad, tmp);
		  }
		  else if(rduart3.cmdBuffer[0] == 'W')
		  {
			  int tmp;
			  sscanf((char *)&rduart3.cmdBuffer[1], "%d", &tmp);
			  PAD_ChangeWidth(&pad, tmp);
		  }
		  else if(rduart3.cmdBuffer[0] == 'I')
		  {
			  int tmp;
			  sscanf((char *)&rduart3.cmdBuffer[1], "%d", &tmp);
			  PAD_ChangeInterval(&pad, tmp);
		  }
		  rduart3.cmdUF = 0;
	  }
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_2);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);


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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
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
		if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8) == GPIO_PIN_SET)//Connected
		{
			if(UART_LastInputData(&rduart3) == '\r')
			{
				if(rduart3.uartORF)
				{
					rduart3.uartORF = 0;
					rduart3.uartBufferStartIdx = rduart3.uartBufferEndIdx;
				}
				else
				{
					UART_CopyUartBuf2CmdBuf(&rduart3);
				}
			}
		}
		else
		{
			rduart3.uartBufferStartIdx = rduart3.uartBufferEndIdx;
		}

		UART_ReceiveData(&rduart3);
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{
		case GPIO_PIN_0://pad voltage down
			PAD_VoltageDown(&pad);
			Beep();
			for(int i = 0; i < 30000; i++);
			Beep();
			break;
		case GPIO_PIN_1://pad voltage up
			PAD_VoltageUp(&pad);
			Beep();
			break;
		case GPIO_PIN_2://power button interrupt
			htim6.Instance->CNT = 0;
			htim6.Instance->SR = 0;
			HAL_TIM_Base_Start_IT(&htim6);
			Beep();
			break;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3)
	{
		static uint32_t pwm_count = 0;
		ADC_ReadDMA(&adc);
		if(++pwm_count >= 10)
		{
			if(adc.adcBufferOVF)//pad voltage control
			{
				PAD_ChangeVoltage(&pad, (double)ADC_Average_mV(&adc));
			}
			pwm_count = 0;
		}
	}
	else if(htim->Instance == TIM6)
	{
		htim6.Instance->CNT = 0;
		HAL_TIM_Base_Stop_IT(&htim6);
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_SET)//power off
		{
			Beep();
			for(int i = 0; i < 30000; i++);
			Beep();
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
			working = 0;
		}
	}
	else if(htim->Instance == TIM15)
	{
		if(htim15.Instance->CCR1 == 0)
		{
			if(padState)
			{
				htim15.Instance->CCR1 = htim15.Instance->ARR / 2 + 1;
				htim15.Instance->CCR2 = htim15.Instance->ARR / 2 + 1;//pulse on
			}
			htim15.Instance->RCR = pad.pulseCount - 1;
		}
		else
		{
			htim15.Instance->CCR1 = 0;
			htim15.Instance->CCR2 = htim15.Instance->ARR + 1;
			htim15.Instance->RCR  = pad.pulsePeriod_us / pad.pulseWidth_us - pad.pulseCount;
		}
	}
	/*else if(htim->Instance == TIM7)
	{
		if(padState)
		{
			htim15.Instance->CCR1 = htim15.Instance->ARR / 2;
			htim15.Instance->CCR2 = htim15.Instance->ARR / 2;//pulse on
		}
		htim15.Instance->RCR = pad.pulseCount - 1;
		htim15.Instance->EGR |= 0x20;
		htim15.Instance->SR |= 0x1;//interrupt enable
	}
	else if(htim->Instance == TIM15)//pulse off
	{
		htim15.Instance->CCR1 = 0;
		htim15.Instance->CCR2 = htim15.Instance->ARR + 1;
		htim15.Instance->EGR |= 0x20;
		htim15.Instance->SR &= ~0x1;//interrupt disable
	}*/
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
