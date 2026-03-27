/**
  ******************************************************************************
  * @file    adc.h
  * @brief   This file contains all the function prototypes for
  *          the adc.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "foc.h"
/* USER CODE END Includes */

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

/* USER CODE BEGIN Private defines */

#define CURR_AMP_GAIN_RES_1mOhm 				(5.0f)
#define CURR_AMP_GAIN_RES_2mOhm 				(20.0f)
#define CURR_AMP_GAIN_RES_500uOhm       (20.0f)

#define VOL_AMP_DIV_RES_1               (3.f / 103.f)
#define VOL_AMP_DIV_RES_2               (3.f / 33.f)
#define VOL_AMP_DIV_RES_3               (3.f / 54.f)

#define HALL_ACS781_50A_GAIN						0.0264f	// V/A
#define HALL_ACS781_150A_GAIN						0.0088f	// V/A	

#define OverCurrent_Time      					(100)  

/* USER CODE END Private defines */

void MX_ADC1_Init(void);
void MX_ADC2_Init(void);
void MX_ADC3_Init(void);

/* USER CODE BEGIN Prototypes */

void ADC_CMD(FunctionalState adcState);
void GetPhaseCurrent(void);
void ADCSelfCalibration(void);
void ADCOvercurrentProtection(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
