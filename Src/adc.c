/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
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

/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* USER CODE BEGIN 0 */
uint32_t ADC_CurrentConvertedValue_PhaseA = 0;
uint32_t ADC_CurrentConvertedValue_PhaseB = 0;
uint32_t ADC_CurrentConvertedValue_PhaseC = 0;
uint32_t ADC_CurrentConvertedValue_Generatrix = 0;
int ADC_flag = 0;

 extern struct CoordTrans_t CoordTrans;

/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc3;

/* ADC1 init function */
void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_11;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 2;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T8_CC4;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = ENABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_4;
  sConfigInjected.InjectedRank = 2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

}
/* ADC2 init function */
void MX_ADC2_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_12;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T8_CC4;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = ENABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

}
/* ADC3 init function */
void MX_ADC3_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ENABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_13;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_T8_CC4;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = ENABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PC1     ------> ADC1_IN11
    PA4     ------> ADC1_IN4
    */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC1 interrupt Init */
    HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
  else if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspInit 0 */

  /* USER CODE END ADC2_MspInit 0 */
    /* ADC2 clock enable */
    __HAL_RCC_ADC2_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**ADC2 GPIO Configuration
    PC2     ------> ADC2_IN12
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* ADC2 interrupt Init */
    HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* USER CODE BEGIN ADC2_MspInit 1 */

  /* USER CODE END ADC2_MspInit 1 */
  }
  else if(adcHandle->Instance==ADC3)
  {
  /* USER CODE BEGIN ADC3_MspInit 0 */

  /* USER CODE END ADC3_MspInit 0 */
    /* ADC3 clock enable */
    __HAL_RCC_ADC3_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**ADC3 GPIO Configuration
    PC0     ------> ADC3_IN10
    PC3     ------> ADC3_IN13
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* ADC3 DMA Init */
    /* ADC3 Init */
    hdma_adc3.Instance = DMA2_Stream0;
    hdma_adc3.Init.Channel = DMA_CHANNEL_2;
    hdma_adc3.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc3.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc3.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc3.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc3.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc3.Init.Mode = DMA_CIRCULAR;
    hdma_adc3.Init.Priority = DMA_PRIORITY_LOW;
    hdma_adc3.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_adc3) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc3);

    /* ADC3 interrupt Init */
    HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC_IRQn);
  /* USER CODE BEGIN ADC3_MspInit 1 */

  /* USER CODE END ADC3_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PC1     ------> ADC1_IN11
    PA4     ------> ADC1_IN4
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_1);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);

    /* ADC1 interrupt Deinit */
  /* USER CODE BEGIN ADC1:ADC_IRQn disable */
    /**
    * Uncomment the line below to disable the "ADC_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(ADC_IRQn); */
  /* USER CODE END ADC1:ADC_IRQn disable */

  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
  else if(adcHandle->Instance==ADC2)
  {
  /* USER CODE BEGIN ADC2_MspDeInit 0 */

  /* USER CODE END ADC2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC2_CLK_DISABLE();

    /**ADC2 GPIO Configuration
    PC2     ------> ADC2_IN12
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_2);

    /* ADC2 interrupt Deinit */
  /* USER CODE BEGIN ADC2:ADC_IRQn disable */
    /**
    * Uncomment the line below to disable the "ADC_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(ADC_IRQn); */
  /* USER CODE END ADC2:ADC_IRQn disable */

  /* USER CODE BEGIN ADC2_MspDeInit 1 */

  /* USER CODE END ADC2_MspDeInit 1 */
  }
  else if(adcHandle->Instance==ADC3)
  {
  /* USER CODE BEGIN ADC3_MspDeInit 0 */

  /* USER CODE END ADC3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC3_CLK_DISABLE();

    /**ADC3 GPIO Configuration
    PC0     ------> ADC3_IN10
    PC3     ------> ADC3_IN13
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_0|GPIO_PIN_3);

    /* ADC3 DMA DeInit */
    HAL_DMA_DeInit(adcHandle->DMA_Handle);

    /* ADC3 interrupt Deinit */
  /* USER CODE BEGIN ADC3:ADC_IRQn disable */
    /**
    * Uncomment the line below to disable the "ADC_IRQn" interrupt
    * Be aware, disabling shared interrupt may affect other IPs
    */
    /* HAL_NVIC_DisableIRQ(ADC_IRQn); */
  /* USER CODE END ADC3:ADC_IRQn disable */

  /* USER CODE BEGIN ADC3_MspDeInit 1 */

  /* USER CODE END ADC3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void ADC_CMD(FunctionalState adcState)
{
	  if (adcState == ENABLE)
	  {
		__HAL_ADC_ENABLE(&hadc1);
		__HAL_ADC_ENABLE(&hadc2);
		__HAL_ADC_ENABLE(&hadc3);
		HAL_ADCEx_InjectedStart_IT(&hadc2);
		HAL_ADCEx_InjectedStart_IT(&hadc3);
		HAL_ADCEx_InjectedStart_IT(&hadc1);
		
	  }
	  else if (adcState == DISABLE)
	  {
		__HAL_ADC_DISABLE(&hadc1);
		__HAL_ADC_DISABLE(&hadc2);
		__HAL_ADC_DISABLE(&hadc3);
		HAL_ADCEx_InjectedStop_IT(&hadc2);
		HAL_ADCEx_InjectedStop_IT(&hadc3);
		HAL_ADCEx_InjectedStop_IT(&hadc1);
	  }
	  /*失能DMA中断*/
	  __HAL_DMA_DISABLE_IT(hadc1.DMA_Handle,DMA_IT_TC|DMA_IT_HT);
	  __HAL_DMA_DISABLE_IT(hadc2.DMA_Handle,DMA_IT_TC|DMA_IT_HT);
	  __HAL_DMA_DISABLE_IT(hadc3.DMA_Handle,DMA_IT_TC|DMA_IT_HT);
	  
}

void GetPhaseCurrent(void)
{
	#if PHASE_SEQUENCE == POSITIVE_SEQUENCE
		ADC_CurrentConvertedValue_PhaseA = hadc3.Instance->JDR1;
		ADC_CurrentConvertedValue_PhaseB = hadc2.Instance->JDR1;
		ADC_CurrentConvertedValue_PhaseC = hadc1.Instance->JDR1;
	#elif PHASE_SEQUENCE == NEGATIVE_SEQUENCE
		ADC_CurrentConvertedValue_PhaseA = hadc1.Instance->JDR1;
		ADC_CurrentConvertedValue_PhaseB = hadc2.Instance->JDR1;
		ADC_CurrentConvertedValue_PhaseC = hadc3.Instance->JDR1;
	#else
	#error "Phase Sequence Invalid"
	#endif
	#if CURRENT_SENSOR == RES_1mOhm

//	CoordTrans.CurrA = 330.f- 0.1611328125f * (float)ADC_CurrentConvertedValue_PhaseA;
//			CoordTrans.CurrA = ((1.65f - 3.3f * (float)ADC_CurrentConvertedValue_PhaseA / 4096.f) / CURR_AMP_GAIN_RES_1mOhm) / 0.001f;
//			CoordTrans.CurrB = ((1.65f - 3.3f * (float)ADC_CurrentConvertedValue_PhaseB / 4096.f) / CURR_AMP_GAIN_RES_1mOhm) / 0.001f;
//			CoordTrans.CurrC = ((1.65f - 3.3f * (float)ADC_CurrentConvertedValue_PhaseC / 4096.f) / CURR_AMP_GAIN_RES_1mOhm) / 0.001f;
		CoordTrans.CurrA = - ((1.65f -  3.3f * (float)ADC_CurrentConvertedValue_PhaseA / 4096.f) / CURR_AMP_GAIN_RES_1mOhm) / 0.001f - CoordTrans.CurrA_Offset;
		CoordTrans.CurrB = - ((1.65f - 3.3f * (float)ADC_CurrentConvertedValue_PhaseB / 4096.f) / CURR_AMP_GAIN_RES_1mOhm) / 0.001f - CoordTrans.CurrB_Offset;
		CoordTrans.CurrC = - ((1.65f - 3.3f * (float)ADC_CurrentConvertedValue_PhaseC / 4096.f) / CURR_AMP_GAIN_RES_1mOhm) / 0.001f - CoordTrans.CurrC_Offset;

//	CoordTrans.CurrB =  330.f- 0.1611328125f * (float)ADC_CurrentConvertedValue_PhaseB;
//			CoordTrans.CurrC =  330.f- 0.1611328125f * (float)ADC_CurrentConvertedValue_PhaseC;
//		CoordTrans.CurrA = -CoordTrans.CurrB-CoordTrans.CurrC;
	
	#elif CURRENT_SENSOR == RES_2mOhm
		#if GATE_DRIVER_TYPE == GATE_DRIVER_DRV8323
			CoordTrans.CurrA = ((1.65f - 3.3f * (float)ADC_CurrentConvertedValue_PhaseA / 4096.f) / CURR_AMP_GAIN_RES_1mOhm) / 0.002f;
			CoordTrans.CurrB = ((1.65f - 3.3f * (float)ADC_CurrentConvertedValue_PhaseB / 4096.f) / CURR_AMP_GAIN_RES_1mOhm) / 0.002f;
			CoordTrans.CurrC = ((1.65f - 3.3f * (float)ADC_CurrentConvertedValue_PhaseC / 4096.f) / CURR_AMP_GAIN_RES_1mOhm) / 0.002f;
		#elif GATE_DRIVER_TYPE == GATE_DRIVER_DISCRETE
			CoordTrans.CurrA = ((1.65f - 3.3f * (float)ADC_CurrentConvertedValue_PhaseA / 4096.f) / CURR_AMP_GAIN_RES_1mOhm) / 0.002f;
			CoordTrans.CurrB = ((1.65f - 3.3f * (float)ADC_CurrentConvertedValue_PhaseB / 4096.f) / CURR_AMP_GAIN_RES_1mOhm) / 0.002f;
			CoordTrans.CurrC = ((1.65f - 3.3f * (float)ADC_CurrentConvertedValue_PhaseC / 4096.f) / CURR_AMP_GAIN_RES_1mOhm) / 0.002f;
		#endif 
	#elif CURRENT_SENSOR == RES_200uOhm
			CoordTrans.CurrB = ((0.20141601562f * (float)ADC_CurrentConvertedValue_PhaseB) - 412.5f);
			CoordTrans.CurrC = ((0.20141601562f * (float)ADC_CurrentConvertedValue_PhaseC) - 412.5f);
			CoordTrans.CurrA = ((0.20141601562f * (float)ADC_CurrentConvertedValue_PhaseA) - 412.5f);

	#endif
	
	ADC_CurrentConvertedValue_Generatrix = hadc1.Instance->JDR2;
	CoordTrans.CurrG = ((0.20141601562f * (float)ADC_CurrentConvertedValue_Generatrix));
	ADCOvercurrentProtection();
	
}


/*ADC校准程序，在2000个内环周期内进行ADC补偿*/
void ADCSelfCalibration(void)
{
	static uint16_t SelfCalibrationPeriod = 0;
	 
	static float currAoffset = 0;
	static float currBoffset = 0;
	static float currCoffset = 0;
	if(!CoordTrans.ADCSelfCalibration_Status)
	{
		
		if (SelfCalibrationPeriod >= 2000)
		{
			CoordTrans.CurrA_Offset = currAoffset;
			CoordTrans.CurrB_Offset = currBoffset;
			CoordTrans.CurrC_Offset = currCoffset;
//			PWM_CMD(ENABLE);
			CoordTrans.ADCSelfCalibration_Status = 1;
			PosSensor_Init();
		}else
		{
			currAoffset += CoordTrans.CurrA / 2000.f;
			currBoffset += CoordTrans.CurrB / 2000.f;
			currCoffset += CoordTrans.CurrC / 2000.f;
			SelfCalibrationPeriod ++;
		}
    }
}
/*过流保护*/
void ADCOvercurrentProtection(void)
{
	if(CoordTrans.CurrG >= CurrenIN_Max)
	{
		ADC_flag ++;
		if(ADC_flag >= OverCurrent_Time)
		{
			PWM_CMD(DISABLE);
		}
	}
	else if (CoordTrans.CurrG < CurrenIN_Max)
	{
		ADC_flag = 0;
	}
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
