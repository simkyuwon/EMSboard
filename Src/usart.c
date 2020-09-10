/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
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

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

UART_HandleTypeDef huart3;

/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_8;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART3_6_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_6_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void UART_ReceiveData_Init(UART_ReceiveDataTypeDef *rduart, UART_HandleTypeDef* uartHandle)
{
	rduart->huart = uartHandle;
	rduart->cmdUF = 0;
	rduart->uartORF = 0;
	rduart->uartBufferStartIdx = 0;
	rduart->uartBufferEndIdx = 0;
}

HAL_StatusTypeDef UART_ReceiveData(UART_ReceiveDataTypeDef *rduart)
{
	HAL_StatusTypeDef ret = HAL_UART_Receive_IT(rduart->huart, &rduart->uartBuffer[rduart->uartBufferEndIdx++], 1);
	if(rduart->uartBufferEndIdx - rduart->uartBufferStartIdx >= uartBufferSize)
	{
		rduart->uartBufferEndIdx -= 1;
		rduart->uartORF = 1;
	}
	return ret;
}

_Bool CmdCmp(UART_ReceiveDataTypeDef *rduart, char *str, uint32_t size)
{
	if(rduart->cmdBufferLen != size)
	{
		return 0;
	}

	for(uint32_t i = 0; i < rduart->cmdBufferLen && i < size; i++)
	{
		if(rduart->cmdBuffer[i] != str[i])
		{
			return 0;
		}
	}

	return 1;
}

char UART_LastInputData(UART_ReceiveDataTypeDef *rduart)
{
	return rduart->uartBuffer[(rduart->uartBufferEndIdx + uartBufferSize - 1) % uartBufferSize];
}

_Bool UART_CopyUartBuf2CmdBuf(UART_ReceiveDataTypeDef *rduart)
{
	if(rduart->cmdUF)
	{
		return 0;
	}

	for(rduart->cmdBufferLen = 0; rduart->uartBufferStartIdx + rduart->cmdBufferLen < rduart->uartBufferEndIdx; rduart->cmdBufferLen++)
	{
		rduart->cmdBuffer[rduart->cmdBufferLen] = rduart->uartBuffer[(rduart->uartBufferStartIdx + rduart->cmdBufferLen) % uartBufferSize];
	}
	rduart->cmdBufferLen -= 1;
	rduart->cmdUF = 1;
	rduart->uartBufferStartIdx = rduart->uartBufferEndIdx;

	if(rduart->uartBufferStartIdx >= uartBufferSize)
	{
		rduart->uartBufferStartIdx -= uartBufferSize;
		rduart->uartBufferEndIdx -= uartBufferSize;
	}

	return 1;
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
