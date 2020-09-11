/**
  ******************************************************************************
  * File Name          : USART.h
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */
#define uartBufferSize 1024

typedef struct __UART_ReceiveDataTypeDef
{
	UART_HandleTypeDef *huart;
	uint8_t uartBuffer[uartBufferSize];
	uint8_t cmdBuffer[uartBufferSize];
	uint32_t uartBufferStartIdx;
	uint32_t uartBufferEndIdx;
	uint32_t cmdBufferLen;
	_Bool uartORF;
	_Bool cmdUF;
} UART_ReceiveDataTypeDef;
/* USER CODE END Private defines */

void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */
void UART_ReceiveData_Init(UART_ReceiveDataTypeDef *rduart, UART_HandleTypeDef* uartHandle);
HAL_StatusTypeDef UART_ReceiveData(UART_ReceiveDataTypeDef *rduart);
_Bool CmdCmp(UART_ReceiveDataTypeDef *rduart, char *str, uint32_t size);
char UART_LastInputData(UART_ReceiveDataTypeDef *rduart);
_Bool UART_CopyUartBuf2CmdBuf(UART_ReceiveDataTypeDef *rduart);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
