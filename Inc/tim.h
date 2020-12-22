/**
  ******************************************************************************
  * File Name          : TIM.h
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
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
#ifndef __tim_H
#define __tim_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim15;

/* USER CODE BEGIN Private defines */
#define SQUARE_WAVE 0
#define RAMP_WAVE 1

static uint32_t PAD_AMPLITUDE_VOLTAGE[] =
{
		3300,
		5000,
		7500,
		10000,
		12500,
		15000,
		17500,
		20000,
		22500,
		25000,
		27500,
		30000,
		32500,
		35000,
		37500,
		40000,
		42500,
		45000,
		47500,
		50000,
};

#define VOLTAGE_MAX_LEVEL (sizeof(PAD_AMPLITUDE_VOLTAGE)/sizeof(uint32_t))

typedef struct __PAD_ControlData
{
	uint32_t pulseAmplitude;
	uint32_t pulseCount;
	uint32_t pulseWidth_us;
	uint32_t pulsePeriod_us;
	uint8_t  pulseType;
}PAD_ControlData;

/* USER CODE END Private defines */

void MX_TIM1_Init(void);
void MX_TIM3_Init(void);
void MX_TIM6_Init(void);
void MX_TIM15_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */
void PAD_ControlData_Init(PAD_ControlData *pad);
void PAD_ChangeCount(PAD_ControlData *pad, uint32_t cnt);
void PAD_ChangeWidth(PAD_ControlData *pad, uint32_t time_us);
void PAD_ChangeInterval(PAD_ControlData *pad, uint32_t period_us);
void PAD_ChangeVoltage(PAD_ControlData *pad, double now_mV);
void PAD_VoltageUp(PAD_ControlData *pad);
void PAD_VoltageDown(PAD_ControlData *pad);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ tim_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
