/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MotorConfig.h"
#include "can.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc3;
extern CAN_HandleTypeDef hcan1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart1;
/* USER CODE BEGIN EV */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

extern struct CAN_t CAN;
extern struct PosLoop_t PosLoop;
extern struct SpdLoop_t SpdLoop;
extern struct CurrLoop_t CurrLoop;
extern struct TorqueCtrl_t TorqueCtrl;
extern struct MainCtrl_t MainCtrl;
extern struct CoordTrans_t	CoordTrans;
extern struct PosSensor_t PosSensor;
extern struct Driver_t Driver;


/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
	  PWM_CMD(DISABLE);
	  
	  PutStr("Hard Fault!!\r\n");SendBuf();
	  
	  CAN.Identifier = IDENTIFIER_HARD_FAULT;
	  CAN.TransmitData = 0x00;
	  CAN_Transmit(CAN.Identifier, CAN.TransmitData, 1 ,DRIVER_CLIENT_CAN_ID);
	  
	  LL_mDelay(1);
	  
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line 3 interrupt.
  */
void EXTI3_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI3_IRQn 0 */
  
  /* USER CODE END EXTI3_IRQn 0 */
  if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_3) != RESET)
  {
    LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_3);
    /* USER CODE BEGIN LL_EXTI_LINE_3 */
		/*轮子电机向航向电机发*/

		
    /* USER CODE END LL_EXTI_LINE_3 */
  }
  /* USER CODE BEGIN EXTI3_IRQn 1 */

  /* USER CODE END EXTI3_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupt.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */
	
  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX1 interrupt.
  */
void CAN1_RX1_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX1_IRQn 0 */

  /* USER CODE END CAN1_RX1_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX1_IRQn 1 */

  /* USER CODE END CAN1_RX1_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream0 global interrupt.
  */
void DMA2_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

  /* USER CODE END DMA2_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc3);
  /* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

  /* USER CODE END DMA2_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream7 global interrupt.
  */
void DMA2_Stream7_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream7_IRQn 0 */

  /* USER CODE END DMA2_Stream7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_tx);
  /* USER CODE BEGIN DMA2_Stream7_IRQn 1 */

  /* USER CODE END DMA2_Stream7_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void ADC_IRQHandler(void)
{
	
	MainCtrl.CAN_FaultSign++;
	if(MainCtrl.CAN_FaultSign >= 3000)
	{

		MainCtrl.CAN_FaultSign = 200;
	}
	
	if((MainCtrl.CAN_FaultSign % 200) == 0 && MainCtrl.CAN_FaultSign >= 200)
	{
		HAL_CAN_Stop(&hcan1);
		CAN_Enable();
	}
											
	/*获取实际的三相电流�??*/
	GetPhaseCurrent();
	
/*获取实时母线电压*/
//	Get_Generatrix_Vol();
	

	
//使用旧驱动器的时候开启里面的pwmenable
	/*ADC校准*/
	ADCSelfCalibration();

	if(Driver.UnitMode == WORK_MODE && CoordTrans.ADCSelfCalibration_Status)
	{		
		GetEleImformation();
//		LoadObserver();
		
		switch(Driver.ControlMode)
		{
			case SPD_CURR_CTRL_MODE :	/*速度-电流控制模式*/
										SpdCurrController();
										

										/*计算电磁转矩*/
//										CalculateEleTorque(CoordTrans.CurrQ, &TorqueCtrl.EleTorque_Nm);

										/*慎用, 每五个周期发送一次数据可能导致错过异常数�?*/
										static int so;
										so++;
										if(so>50)
										{
//							  			UART_Transmit_DMA("%d\t", (int)(CoordTrans.CurrG * 1e3));
											
//											UART_Transmit_DMA("%d\t", (int)(CoordTrans.CurrD * 1e3f));
//											UART_Transmit_DMA("%d\t", (int)(CurrLoop.ExptCurrQ * 1e3));
//											UART_Transmit_DMA("%d\t", (int)(CoordTrans.CurrQ * 1e3f));											
//											UART_Transmit_DMA("%d\t",(int)(SpdLoop.ExptMecAngularSpeed_rad * 100 / 2.f / PI));
//											UART_Transmit_DMA("%d\r\n",(int)(PosSensor.MecAngularSpeed_rad / 2 / PI * 100));

//											UART_Transmit_DMA("%d\t", (int)(CoordTrans.CurrA * 1e2));
//											UART_Transmit_DMA("%d\t", (int)(CoordTrans.CurrB * 1e2));
//											UART_Transmit_DMA("%d\r\n", (int)(CoordTrans.CurrC * 1e2));
											
//											UART_Transmit_DMA("%d\t", (int)(AngEst.omega_e_enc_rad * 1e2));
//											UART_Transmit_DMA("%d\t", (int)(AngEst.omega_pll_rad * 1e2));
//											UART_Transmit_DMA("%d\t", (int)(AngEst.theta_pll_rad * 1e2));
//											
											UART_Transmit_DMA("%d\r\n",(int)(PosSensor.MecAngularSpeed_rad / 2 / PI * 100));
//											UART_Transmit_DMA("%d\t",(int)(PosSensor.MecAngularSpeed_rad / 2 / PI * 100));
//											UART_Transmit_DMA("%d\t", (int)(AngEst.e_hat_alpha * 1e2));
//											UART_Transmit_DMA("%d\t", (int)(AngEst.e_hat_beta * 1e2));
//											UART_Transmit_DMA("%d\t", (int)(AngEst.omega_pll_rad * 1e2));
//											UART_Transmit_DMA("%d\r\n", (int)(PosSensor.EleAngularSpeed_rad * 1e2));
//											
//											UART_Transmit_DMA("%d\t", (int)(AngEst.i_hat_alpha * 1e2));
//											UART_Transmit_DMA("%d\t", (int)(AngEst.i_hat_beta * 1e2));
//											UART_Transmit_DMA("%d\t", (int)(CoordTrans.CurrAlpha * 1e2));
//											UART_Transmit_DMA("%d\t", (int)(CoordTrans.CurrBeta * 1e2));
//											UART_Transmit_DMA("%d\t", (int)(CoordTrans.last_VolAlpha * 1e2));
//											UART_Transmit_DMA("%d\t", (int)(CoordTrans.last_VolBeta * 1e2));
//											UART_Transmit_DMA("%d\t", (int)(AngEst.e_hat_alpha * 1e2));
//											UART_Transmit_DMA("%d\r\n", (int)(AngEst.e_hat_beta * 1e2));
											
//											UART_Transmit_DMA("%d\t", (int)(CurrLoop.CtrlVolQ * 1e3f));
											
//											UART_Transmit_DMA("%d\t", (int)(CurrLoop.CtrlVolD * 1e3f));
//											UART_Transmit_DMA("%d\t", (int)(CoordTrans.CurrQ * 1e3f));
//											UART_Transmit_DMA("%d\r\n", (int)(CurrLoop.ExptCurrQ * 1e3f));										
											so=0;
										}
			
			

										break;
			
			case POS_SPD_CURR_CTRL_MODE :/*位置-速度-电流控制模式, 即常规位置控制模�?*/
				
										PosSpdCurrController();
										/*计算电磁转矩*/
										CalculateEleTorque(CoordTrans.CurrQ, &TorqueCtrl.EleTorque_Nm);
										static int ho;
										ho++;
										if(ho>50)
										{
//											UART_Transmit_DMA("%d\t",(int)(SpdLoop.ExptMecAngularSpeed_rad * 100 / 2.f / PI));
//											UART_Transmit_DMA("%d\t",(int)(PosSensor.MecAngularSpeed_rad / 2 / PI * 100));
//							  			UART_Transmit_DMA("%d\t", (int)(CurrLoop.ExptCurrD * 1e3));
//											UART_Transmit_DMA("%d\t", (int)(CurrLoop.CtrlVolD * 1e3));
//											UART_Transmit_DMA("%d\t", (int)(CurrLoop.ExptCurrQ * 1e3));
//											UART_Transmit_DMA("%d\r\n", (int)(CurrLoop.CtrlVolQ * 1e3));

//											UART_Transmit_DMA("%d\t",(int)(SpdLoop.ExptMecAngularSpeed_rad * 100 / 2.f / PI));
//											UART_Transmit_DMA("%d\t",(int)(PosSensor.MecAngularSpeed_rad / 2 / PI * 100));
//											UART_Transmit_DMA("%d\t",(int)(MainCtrl.RefMecAngle_pulse));
//											UART_Transmit_DMA("%d\r\n",(int)(MainCtrl.ExptMecAngle_pulse));
											UART_Transmit_DMA("%d\r\n",(int)(PosSensor.MecAngle_15bit));
//     									UART_Transmit_DMA("%d\t", (int)(CoordTrans.CurrG * 1e3));
//											UART_Transmit_DMA("%d\t", (int)(PosSensor.MecAngularSpeed_rad / 2 / PI *100.f));
//											UART_Transmit_DMA("%d\t", (int)(SpdLoop.ExptMecAngularSpeed_rad / 2 / PI * 100.f));
//							  			UART_Transmit_DMA("%d\t", (int)(CurrLoop.ExptCurrD * 1e3));
//											UART_Transmit_DMA("%d\t", (int)(CurrLoop.CtrlVolD * 1e3));
//											UART_Transmit_DMA("%d\t", (int)(CurrLoop.ExptCurrQ * 1e3));
//											UART_Transmit_DMA("%d\r\n", (int)(CurrLoop.CtrlVolQ * 1e3));
											ho=0;
										}
										break;
			
			case POS_CURR_CTRL_MODE :	/*位置-电流控制模式*/
										PosCurrController();
		
										/*计算电磁转矩*/
										CalculateEleTorque(CoordTrans.CurrQ, &TorqueCtrl.EleTorque_Nm);
										static int co;
										co++;
										if(co>50)	
										{
//											UART_Transmit_DMA("%d\t", (int)(CoordTrans.CurrA * 1e2));
//											UART_Transmit_DMA("%d\t", (int)(CoordTrans.CurrB * 1e2));
//											UART_Transmit_DMA("%d\r\n", (int)(CoordTrans.CurrC * 1e2));
//											UART_Transmit_DMA("%d\t",(int)(SpdLoop.ExptMecAngularSpeed_rad * 100 / 2.f / PI));
//											UART_Transmit_DMA("%d\t",(int)(PosSensor.MecAngularSpeed_rad / 2 / PI * 100));
											UART_Transmit_DMA("%d\t",(int)(MainCtrl.RefMecAngle_pulse));
//											UART_Transmit_DMA("%d\t",(int)(PosLoop.Err*1000));
											UART_Transmit_DMA("%d\r\n",(int)(PosSensor.MecAngle_15bit));
//										UART_Transmit_DMA("%d\t", (int)(CoordTrans.CurrQ * 1e3));
//										UART_Transmit_DMA("%d\r\n", (int)(CurrLoop.ExptCurrQ * 1e3));
											co=0;
										}                          
										break;
			
			case TORQUE_CTRL_MODE :		/*转矩控制模式*/
										TorqueController();
		
										/*计算电磁转矩*/
										CalculateEleTorque(CoordTrans.CurrQ, &TorqueCtrl.EleTorque_Nm);
										static int wo;
										wo++;
										if(wo>50)
										{
//							  			UART_Transmit_DMA("%d\t", (int)(CoordTrans.CurrG * 1e3));
											
											UART_Transmit_DMA("%d\t", (int)(CoordTrans.CurrD * 1e3f));
											UART_Transmit_DMA("%d\t", (int)(CurrLoop.ExptCurrQ * 1e3));
											UART_Transmit_DMA("%d\r\n", (int)(CoordTrans.CurrQ * 1e3f));
////											
//											UART_Transmit_DMA("%d\t",(int)(SpdLoop.ExptMecAngularSpeed_rad * 100 / 2.f / PI));
//											UART_Transmit_DMA("%d\r\n",(int)(PosSensor.MecAngularSpeed_rad / 2 / PI * 100));

//											UART_Transmit_DMA("%d\t", (int)(CoordTrans.CurrA * 1e2));
//											UART_Transmit_DMA("%d\t", (int)(CoordTrans.CurrB * 1e2));
//											UART_Transmit_DMA("%d\r\n", (int)(CoordTrans.CurrC * 1e2));
											
											
											
//											UART_Transmit_DMA("%d\t", (int)(CurrLoop.CtrlVolQ * 1e3f));
											
//											UART_Transmit_DMA("%d\t", (int)(CurrLoop.CtrlVolD * 1e3f));
//											UART_Transmit_DMA("%d\t", (int)(CoordTrans.CurrQ * 1e3f));
//											UART_Transmit_DMA("%d\r\n", (int)(CurrLoop.ExptCurrQ * 1e3f));										
											wo=0;
										}
										break;
		}
	}
	else if(Driver.UnitMode == MEASURE_PARAM_MODE)
	{
		/*参数辨识*/
		MeasureParameters();
	}
	else if(Driver.UnitMode == MEASURE_INERTIA_MODE)
	{
		RotateInertiaTest(1.f);
		
		GetEleImformation();
		
		SpdCurrController();
		
		/*计算电磁转矩*/
		CalculateEleTorque(CoordTrans.CurrQ, &TorqueCtrl.EleTorque_Nm);		

	}
	/*清除ADC注入模式中断标志�?*/
	__HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_JEOC);
	__HAL_ADC_CLEAR_FLAG(&hadc2, ADC_FLAG_JEOC);
	__HAL_ADC_CLEAR_FLAG(&hadc3, ADC_FLAG_JEOC);
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
