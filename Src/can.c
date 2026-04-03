/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"
#include "droncan_esc_rpm.h"

/* USER CODE BEGIN 0 */
#ifndef CAN_ID_NUM
 #error "CAN_ID_NUMBER is not undefined."
#endif

/* DroneCAN Protocol defines */
#define DRONCAN_PRIORITY_MASK        0xE0000000
#define DRONCAN_MSG_TYPE_MASK        0x1FFFC000
#define DRONCAN_SRC_NODE_ID_MASK     0x00003F80
#define DRONCAN_TRANSFER_ID_MASK     0x00000060
#define DRONCAN_FRAME_TYPE_MASK      0x00000010
#define DRONCAN_LAST_FRAME_MASK      0x00000008
#define DRONCAN_FRAME_IDX_MASK       0x00000007

#define DRONCAN_PRIORITY_SHIFT       29
#define DRONCAN_MSG_TYPE_SHIFT       14
#define DRONCAN_SRC_NODE_ID_SHIFT    7
#define DRONCAN_TRANSFER_ID_SHIFT    5
#define DRONCAN_FRAME_TYPE_SHIFT     4
#define DRONCAN_LAST_FRAME_SHIFT     3
#define DRONCAN_FRAME_IDX_SHIFT      0

typedef struct {
    uint8_t priority;
    uint16_t message_type;
    uint8_t source_node_id;
    uint8_t transfer_id;
    uint8_t frame_type;
    uint8_t last_frame;
    uint8_t frame_index;
} DroneCAN_ID_t;

CAN_TxHeaderTypeDef TxMessage = { 0 };
CAN_RxHeaderTypeDef RxMessage0 = { 0 };

struct CAN_t CAN;
extern struct Driver_t Driver;
extern struct CurrLoop_t CurrLoop;
extern struct SpdLoop_t SpdLoop;
extern struct PosLoop_t PosLoop;
extern struct TorqueCtrl_t TorqueCtrl;
extern struct MainCtrl_t MainCtrl;
extern struct CoordTrans_t CoordTrans;
extern struct PosSensor_t PosSensor;
extern struct LoadObserver_t LoadOb;

/* DroneCAN helper functions */
void DroneCAN_ParseID(uint32_t can_id, DroneCAN_ID_t *parsed_id)
{
    parsed_id->priority = (can_id & DRONCAN_PRIORITY_MASK) >> DRONCAN_PRIORITY_SHIFT;
    parsed_id->message_type = (can_id & DRONCAN_MSG_TYPE_MASK) >> DRONCAN_MSG_TYPE_SHIFT;
    parsed_id->source_node_id = (can_id & DRONCAN_SRC_NODE_ID_MASK) >> DRONCAN_SRC_NODE_ID_SHIFT;
    parsed_id->transfer_id = (can_id & DRONCAN_TRANSFER_ID_MASK) >> DRONCAN_TRANSFER_ID_SHIFT;
    parsed_id->frame_type = (can_id & DRONCAN_FRAME_TYPE_MASK) >> DRONCAN_FRAME_TYPE_SHIFT;
    parsed_id->last_frame = (can_id & DRONCAN_LAST_FRAME_MASK) >> DRONCAN_LAST_FRAME_SHIFT;
    parsed_id->frame_index = (can_id & DRONCAN_FRAME_IDX_MASK) >> DRONCAN_FRAME_IDX_SHIFT;
}

uint32_t DroneCAN_BuildID(DroneCAN_ID_t *parsed_id)
{
    uint32_t can_id = 0;
    can_id |= ((parsed_id->priority & 0x07) << DRONCAN_PRIORITY_SHIFT);
    can_id |= ((parsed_id->message_type & 0x3FFF) << DRONCAN_MSG_TYPE_SHIFT);
    can_id |= ((parsed_id->source_node_id & 0x7F) << DRONCAN_SRC_NODE_ID_SHIFT);
    can_id |= ((parsed_id->transfer_id & 0x03) << DRONCAN_TRANSFER_ID_SHIFT);
    can_id |= ((parsed_id->frame_type & 0x01) << DRONCAN_FRAME_TYPE_SHIFT);
    can_id |= ((parsed_id->last_frame & 0x01) << DRONCAN_LAST_FRAME_SHIFT);
    can_id |= ((parsed_id->frame_index & 0x07) << DRONCAN_FRAME_IDX_SHIFT);
    return can_id;
}

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 5;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_4TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* DroneCAN message handler - maps DroneCAN messages to standard identifiers */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{	
	first_line:
	
	DroneCAN_ID_t droncan_id;
	
	CAN_Receive(&CAN.StdID, &CAN.Identifier, &CAN.ReceiveData);
	
	/* Parse DroneCAN ID structure */
	DroneCAN_ParseID(CAN.StdID, &droncan_id);
	
	/* Map DroneCAN message_type to standard CAN identifier */
	CAN.Identifier = droncan_id.message_type & 0xFF;
	
	/*ACTION指令*/
	/*点对点模式 - 使用 source_node_id 区分*/
	if (droncan_id.source_node_id == DRIVER_SERVER_CAN_ID)
	{
		switch(CAN.Identifier)
		{
			case IDENTIFIER_DRIVER_STATE:

				if (CAN.ReceiveData == 0x00000001)
				{
					/*PWM输出使能*/
					SpdLoop.ExptMecAngularSpeed_rad = 0.f;
					CurrLoop.ErrD = 0;
					CurrLoop.ErrQ = 0;
					CurrLoop.ExptCurrD = 0;
					CurrLoop.ExptCurrQ = 0;
					CurrLoop.IntegralErrD = 0;
					CurrLoop.IntegralErrQ = 0;
					SpdLoop.Err = 0;
					SpdLoop.IntegralErr = 0;
					PWM_CMD(ENABLE);
					
					CAN.Identifier = IDENTIFIER_ENABLE_DONE;
					CAN.TransmitData = 0x01;
					CAN_Transmit(CAN.Identifier, CAN.TransmitData, 4, droncan_id.source_node_id, DRONCAN_PRIORITY_NORMAL);
					
				}
				else if(CAN.ReceiveData == 0x00000000)
				{
					/*PWM输出失能*/
					SpdLoop.ExptMecAngularSpeed_rad = 0.f;
					PWM_CMD(DISABLE);
				}
				
				break;
	
			case IDENTIFIER_CURR_KP_Q:
				
				/*设置q轴Kp, 主控乘以1000后发给驱动器*/
				CurrLoop.Kp_Q = CAN.ReceiveData * 1e-3;
			
				break;
			
			case IDENTIFIER_CURR_KI_Q:
				
				/*设置q轴Ki, 主控乘以1000后发给驱动器*/
				CurrLoop.Ki_Q = CAN.ReceiveData * 1e-3;
			
				break;
			
			case IDENTIFIER_SPD_KP:
				
				/*设置速度环Kp, 主控乘以1000后发给驱动器*/
				SpdLoop.Kp = CAN.ReceiveData * 1e-3;
			
				break;
			
			case IDENTIFIER_SPD_KI:
			
				/*设置速度环Ki, 主控乘以1000后发给驱动器*/			
				SpdLoop.Ki = CAN.ReceiveData * 1e-3;
			
				break;
			
			case IDENTIFIER_POS_KP:
				
				/*设置位置环Kp, 主控乘以1000后发给驱动器*/
				PosLoop.Kp = CAN.ReceiveData * 1e-3;
			
				break;
			
			case IDENTIFIER_POS_KD:

				/*设置位置环Kd, 主控乘以1000后发给驱动器*/
				PosLoop.Kd = CAN.ReceiveData * 1e-3;
			
				break;
			
			case IDENTIFIER_TORQUE_CTRL:
				
				/*转矩控制模式, 期望转矩, 主控以mN*M为单位发给驱动器*/
				TorqueCtrl.ExptTorque_Nm =  (float)CAN.ReceiveData * 1e-3;
			
				break;
				
			case IDENTIFIER_VEL_CTRL:
				
				/*速度控制模式, 期望机械角速度*/
				MainCtrl.ExptMecAngularSpeed_pulse =  MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
				SpdLoop.ExptMecAngularSpeed_rad = DRV_PULSE_TO_RAD(MainCtrl.ExptMecAngularSpeed_pulse);
			
				break;
			
			case IDENTIFIER_POS_CTRL_ABS:
			
				/*位置控制模式, 绝对位置模式, 期望机械角度*/
				MainCtrl.ExptMecAngle_pulse = MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
			
				break;

			case IDENTIFIER_POS_CTRL_REL:
				
				/*位置控制模式, 相对位置模式, 期望机械角度*/
				MainCtrl.ExptMecAngle_pulse = MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
			
				break;
						
			case IDENTIFIER_SET_CTRL_MODE:
				
				/*设置控制模式*/
				if(CAN.ReceiveData == SPD_CURR_CTRL_MODE)
				{
					Driver.ControlMode = SPD_CURR_CTRL_MODE;
				}
				else if(CAN.ReceiveData == POS_SPD_CURR_CTRL_MODE)
				{
					Driver.ControlMode = POS_SPD_CURR_CTRL_MODE;
				}
				else if(CAN.ReceiveData == POS_CURR_CTRL_MODE)
				{
					Driver.ControlMode = POS_SPD_CURR_CTRL_MODE;
				}
				else if(CAN.ReceiveData == TORQUE_CTRL_MODE)
				{
					Driver.ControlMode = TORQUE_CTRL_MODE;
				}
				
				DriverCtrlModeInit();
				
				break;

			case IDENTIFIER_SET_ACC:
				
				/*设置加速度*/
				MainCtrl.Acceleration_pulse = MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
				SpdLoop.Acceleration = DRV_PULSE_TO_RAD(MainCtrl.Acceleration_pulse);
				
				break;

			case IDENTIFIER_SET_DEC:

				/*设置减速度*/
				MainCtrl.Deceleration_pulse = MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
				SpdLoop.Deceleration = DRV_PULSE_TO_RAD(MainCtrl.Deceleration_pulse);

				break;
					
			case IDENTIFIER_SET_TORQUE_LIMIT:
				
				/*设置转矩限幅*/
				TorqueCtrl.MaxTorque_Nm = (float)CAN.ReceiveData * 1e-3;
				CurrLoop.LimitCurrQ = TorqueCtrl.MaxTorque_Nm / (1.5f * MOTOR_POLE_PAIRS_NUM * ROTATOR_FLUX_LINKAGE);
			
				break;
			case IDENTIFIER_SET_VEL_LIMIT:
				
				/*设置速度限幅*/
				MainCtrl.MaxMecAngularSpeed_pulse =  MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
			
				/*速度控制模式和位置控制模式下速度限幅*/
				if(Driver.ControlMode == SPD_CURR_CTRL_MODE || Driver.ControlMode == POS_SPD_CURR_CTRL_MODE)
				{
					SpdLoop.MaxExptMecAngularSpeed_rad = DRV_PULSE_TO_RAD(MainCtrl.MaxMecAngularSpeed_pulse);
					Saturation_float(&SpdLoop.MaxExptMecAngularSpeed_rad,  MAX_SPD(PosSensor.Generatrix_Vol), - MAX_SPD(PosSensor.Generatrix_Vol));
				}
				/*转矩控制模式下速度限幅*/
				else if(Driver.ControlMode == TORQUE_CTRL_MODE)
				{
					TorqueCtrl.MaxMecSpd_rad = DRV_PULSE_TO_RAD(MainCtrl.MaxMecAngularSpeed_pulse);
					Saturation_float(&TorqueCtrl.MaxMecSpd_rad, MAX_SPD(PosSensor.Generatrix_Vol), - MAX_SPD(PosSensor.Generatrix_Vol));
				}
				
				break;
			
			case IDENTIFIER_SET_POS_LIMIT_UP:
				
				/*设置机械角度上限*/
				MainCtrl.MecAngleUpperLimit_pulse =  MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
				PosLoop.MecAngleUpperLimit_rad = (float)DRV_PULSE_TO_RAD(MainCtrl.MecAngleUpperLimit_pulse);
			
				break;
			
			case IDENTIFIER_SET_POS_LIMIT_LOW:
				
				/*设置机械角度下限*/
				MainCtrl.MecAngleLowerLimit_pulse =  MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
				PosLoop.MecAngleLowerLimit_rad = (float)DRV_PULSE_TO_RAD(MainCtrl.MecAngleLowerLimit_pulse);
			
				break;

			case IDENTIFIER_CORRECT_POS_OFFSET:
				
				/*失能ADC, 停止进入ADC中断*/
				ADC_CMD(DISABLE);

				/*矫正编码器偏移量, 在中断中执行以阻塞程序*/
				Driver.UnitMode = CORRECT_POS_OFFSET_MODE;
				CorrectPosOffset_Encoder(0.8f);
			
			    PWM_CMD(ENABLE);
				/*进入工作模式*/
				Driver.UnitMode = WORK_MODE;
				ADC_CMD(ENABLE);
			
				break;
			
			case IDENTIFIER_HALL_TEST_ON:
					
				EXTI ->IMR = 0x00000008;
				
				break;
			
			/*关闭霍尔外部中断*/
			case IDENTIFIER_HALL_TEST_OFF:
					
				EXTI ->IMR = 0x00000000;
				break;
			
		  case IDENTIFIER_SET_CLEAR_INTEGRAL:
				
				SpdLoop.IntegralErr = 0.f;
				
				break;
			
			case (0x40 + IDENTIFIER_READ_TORQUE):
				
				/*读取电磁转矩*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_TORQUE);
				
				break;
						
			case (0x40 + IDENTIFIER_READ_VEL):
				
				/*读取机械角速度*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_VEL);
			
				break;

			case (0x40 + IDENTIFIER_READ_POS):
				
				/*读取绝对机械角度*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_POS);
		
				break; 
			
			case (0x40 + IDENTIFIER_READ_ENCODER_POS):
				
				/*读取编码器*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_ENCODER_POS);
		
				break; 
						
			case (0x40 + IDENTIFIER_READ_VOL_D):
				
				/*读取Vd*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_VOL_D);
			
				break;
			
			case (0x40 + IDENTIFIER_READ_CURR_D):
				
				/*读取Id*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_CURR_D);
				
				break;
			
			case (0x40 + IDENTIFIER_READ_VOL_Q):
				
				/*读取Vq*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_VOL_Q);
			
				break;
			
			case (0x40 + IDENTIFIER_READ_CURR_Q):
				
				/*读取Iq*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_CURR_Q);
				
				break;
			
			case (0x40 + IDENTIFIER_READ_SPD_LOOP_OUTPUT):
				
				/*读取速度环输出*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_SPD_LOOP_OUTPUT);
			
				break;
			
			case (0x40 + IDENTIFIER_READ_POS_LOOP_OUTPUT):
				
				/*读取位置环输出*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_POS_LOOP_OUTPUT);
				
				break;
			case (0x40 + IDENTIFIER_READ_LOAD_OBSERVER):
				
				/*读取负载观测器*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_LOAD_OBSERVER);
				
				break;		
	
			case (0x40 + IDENTIFIER_READ_ACC):
				
				/*读取加速度*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_ACC);
			

						
			default:
				
				break;
		}	
	}
	/*广播模式 - 使用 priority 和 message_type 区分*/
	else if(droncan_id.priority <= DRONCAN_PRIORITY_NORMAL && droncan_id.message_type == DRIVER_BROADCAST_ID)
	{
		switch(CAN.Identifier)
		{
			case IDENTIFIER_DRIVER_STATE:
				
				if (CAN.ReceiveData == 0x00000001)
				{
					/*PWM输出使能*/
					SpdLoop.ExptMecAngularSpeed_rad = 0.f;
					PWM_CMD(ENABLE);
				}
				else if(CAN.ReceiveData == 0x00000000)
				{
					/*PWM输出失能*/
					SpdLoop.ExptMecAngularSpeed_rad = 0.f;
					PWM_CMD(DISABLE);
				}
				
				break;
	
			case IDENTIFIER_CURR_KP_Q:
				
				/*设置q轴Kp, 主控乘以1000后发给驱动器*/
				CurrLoop.Kp_Q = CAN.ReceiveData * 1e-3;
			
				break;
			
			case IDENTIFIER_CURR_KI_Q:
				
				/*设置q轴Ki, 主控乘以1000后发给驱动器*/
				CurrLoop.Ki_Q = CAN.ReceiveData * 1e-3;
			
				break;
			
			case IDENTIFIER_SPD_KP:
				
				/*设置速度环Kp, 主控乘以1000后发给驱动器*/
				SpdLoop.Kp = CAN.ReceiveData * 1e-3;
			
				break;
			
			case IDENTIFIER_SPD_KI:
			
				/*设置速度环Ki, 主控乘以1000后发给驱动器*/			
				SpdLoop.Ki = CAN.ReceiveData * 1e-3;
			
				break;
			
			case IDENTIFIER_POS_KP:
				
				/*设置位置环Kp, 主控乘以1000后发给驱动器*/
				PosLoop.Kp = CAN.ReceiveData * 1e-3;
			
				break;
			
			case IDENTIFIER_POS_KD:

				/*设置位置环Kd, 主控乘以1000后发给驱动器*/
				PosLoop.Kd = CAN.ReceiveData * 1e-3;
			
				break;
			
			case IDENTIFIER_TORQUE_CTRL:
				
				/*转矩控制模式, 期望转矩, 主控以毫牛米为单位发给驱动器*/
				TorqueCtrl.ExptTorque_Nm =  (float)CAN.ReceiveData * 1e-3;
			
				break;
				
			case IDENTIFIER_VEL_CTRL:
				
				/*速度控制模式, 期望机械角速度*/
				MainCtrl.ExptMecAngularSpeed_pulse =  MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
				SpdLoop.ExptMecAngularSpeed_rad = DRV_PULSE_TO_RAD(MainCtrl.ExptMecAngularSpeed_pulse);
			
				break;
			
			case IDENTIFIER_POS_CTRL_ABS:
			
				/*位置控制模式, 绝对位置模式, 期望机械角度*/
				MainCtrl.ExptMecAngle_pulse = MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
			
				break;

			case IDENTIFIER_POS_CTRL_REL:
				
				/*位置控制模式, 相对位置模式，期望机械角速度*/
				MainCtrl.ExptMecAngle_pulse = MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
			
				break;
						
			case IDENTIFIER_SET_CTRL_MODE:
				
				/*设置控制模式*/
				if(CAN.ReceiveData == SPD_CURR_CTRL_MODE)
				{
					Driver.ControlMode = SPD_CURR_CTRL_MODE;
				}
				else if(CAN.ReceiveData == POS_SPD_CURR_CTRL_MODE)
				{
					Driver.ControlMode = POS_SPD_CURR_CTRL_MODE;
				}
				else if(CAN.ReceiveData == POS_CURR_CTRL_MODE)
				{
					Driver.ControlMode = POS_SPD_CURR_CTRL_MODE;
				}
				else if(CAN.ReceiveData == TORQUE_CTRL_MODE)
				{
					Driver.ControlMode = TORQUE_CTRL_MODE;
				}
				
				DriverCtrlModeInit();
				
				break;

			case IDENTIFIER_SET_ACC:
				
				/*设置加速度*/
				MainCtrl.Acceleration_pulse = MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
				SpdLoop.Acceleration = DRV_PULSE_TO_RAD(MainCtrl.Acceleration_pulse);
				
				break;

			case IDENTIFIER_SET_DEC:

				/*设置减速度*/
				MainCtrl.Deceleration_pulse = MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
				SpdLoop.Deceleration = DRV_PULSE_TO_RAD(MainCtrl.Deceleration_pulse);

				break;
					
			case IDENTIFIER_SET_TORQUE_LIMIT:
				
				/*设置转矩限幅, 在转矩控制模式用*/
				TorqueCtrl.MaxTorque_Nm = (float)CAN.ReceiveData * 1e-3;
				CurrLoop.LimitCurrQ = TorqueCtrl.MaxTorque_Nm / (1.5f * MOTOR_POLE_PAIRS_NUM * ROTATOR_FLUX_LINKAGE);
			
				break;
						
			case IDENTIFIER_SET_VEL_LIMIT:
				
				/*设置速度限幅*/
				MainCtrl.MaxMecAngularSpeed_pulse =  MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
			
				/*速度控制模式和位置控制模式下速度限幅*/
				if(Driver.ControlMode == SPD_CURR_CTRL_MODE || Driver.ControlMode == POS_SPD_CURR_CTRL_MODE)
				{
					SpdLoop.MaxExptMecAngularSpeed_rad = DRV_PULSE_TO_RAD(MainCtrl.MaxMecAngularSpeed_pulse);
					Saturation_float(&SpdLoop.MaxExptMecAngularSpeed_rad,  MAX_SPD(PosSensor.Generatrix_Vol), - MAX_SPD(PosSensor.Generatrix_Vol));
				}
				/*转矩控制模式下速度限幅*/
				else if(Driver.ControlMode == TORQUE_CTRL_MODE)
				{
					TorqueCtrl.MaxMecSpd_rad = DRV_PULSE_TO_RAD(MainCtrl.MaxMecAngularSpeed_pulse);
					Saturation_float(&TorqueCtrl.MaxMecSpd_rad,  MAX_SPD(PosSensor.Generatrix_Vol), - MAX_SPD(PosSensor.Generatrix_Vol));
				}
				
				break;
			
			case IDENTIFIER_SET_POS_LIMIT_UP:
				
				/*设置机械角度上限*/
				MainCtrl.MecAngleUpperLimit_pulse =  MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
				PosLoop.MecAngleUpperLimit_rad = (float)DRV_PULSE_TO_RAD(MainCtrl.MecAngleUpperLimit_pulse);
			
				break;
			
			case IDENTIFIER_SET_POS_LIMIT_LOW:
				
				/*设置机械角度下限*/
				MainCtrl.MecAngleLowerLimit_pulse =  MC_PULSE_TO_DRV_PULSE(CAN.ReceiveData);
				PosLoop.MecAngleLowerLimit_rad = (float)DRV_PULSE_TO_RAD(MainCtrl.MecAngleLowerLimit_pulse);
			
				break;

			case IDENTIFIER_CORRECT_POS_OFFSET:
				
				/*失能ADC, 停止进入ADC中断*/
				ADC_CMD(DISABLE);

				/*矫正编码器偏移量, 在中断中执行以阻塞程序*/
				Driver.UnitMode = CORRECT_POS_OFFSET_MODE;
				CorrectPosOffset_Encoder(ENCODER_CORRECT_VOL_D);
			
				/*进入工作模式*/
				Driver.UnitMode = WORK_MODE;
				ADC_CMD(ENABLE);
			
				break;		
			
			case (0x40 + IDENTIFIER_READ_TORQUE):
				
				/*读取电磁转矩*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_TORQUE);
				
				break;
						
			case (0x40 + IDENTIFIER_READ_VEL):
				
				/*读取机械角速度*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_VEL);
			
				break;

			case (0x40 + IDENTIFIER_READ_POS):
				
				/*读取绝对机械角度*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_POS);
		
				break; 
			
			case (0x40 + IDENTIFIER_READ_ENCODER_POS):
				
				/*读取编码器*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_ENCODER_POS);
		
				break; 
						
			case (0x40 + IDENTIFIER_READ_VOL_D):
				
				/*读取Vd*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_VOL_D);
			
				break;
			
			case (0x40 + IDENTIFIER_READ_CURR_D):
				
				/*读取Id*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_CURR_D);
				
				break;
			
			case (0x40 + IDENTIFIER_READ_VOL_Q):
				
				/*读取Vq*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_VOL_Q);
			
				break;
			
			case (0x40 + IDENTIFIER_READ_CURR_Q):
				
				/*读取Iq*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_CURR_Q);
				
				break;
			
			case (0x40 + IDENTIFIER_READ_SPD_LOOP_OUTPUT):
				
				/*读取速度环输出*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_SPD_LOOP_OUTPUT);
			
				break;
			
			case (0x40 + IDENTIFIER_READ_POS_LOOP_OUTPUT):
				
				/*读取位置环输出*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_POS_LOOP_OUTPUT);
				
				break;
			case (0x40 + IDENTIFIER_READ_LOAD_OBSERVER):
				
				/*读取负载观测器*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_LOAD_OBSERVER);
				
				break;	
			
			case (0x40 + IDENTIFIER_READ_ACC):
				
				/*读取加速度*/
				CAN.RecieveStatus = (0x40 + IDENTIFIER_READ_ACC);
				
				break;
			

						
			default:
				
				break;
		}	
	}
	/*DroneCAN ESC RawCommand - 接收PX4发送的转速指令*/
	else if(droncan_id.message_type == DRONCAN_ESC_RAWCMD_ID)
	{
		DroneCAN_ESC_RawCommandHandler(CAN.Receive.data_uint8);
	}
	
	CAN_Respond();
	
	MainCtrl.CAN_FaultSign = 0;
	
	/*判断RX FIFO0中是否还有消息*/
	if(HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0))
	{
		goto first_line;
	}
}

void CAN_Respond(void)
{
	switch (CAN.RecieveStatus)
	{
		/*发送电磁转矩*/	
		case (0x40 + IDENTIFIER_READ_TORQUE):
			
			CAN.Identifier = IDENTIFIER_READ_TORQUE;
			CAN.TransmitData = (int32_t)(TorqueCtrl.EleTorque_Nm * 1e3);
		
			CAN_Transmit(CAN.Identifier, CAN.TransmitData, 4, DRIVER_CLIENT_CAN_ID, DRONCAN_PRIORITY_NORMAL);
		
			CAN.RecieveStatus = 0;
		
			break;
			
		/*发送机械角速度*/			
		case (0x40 + IDENTIFIER_READ_VEL):
			
			CAN.Identifier = IDENTIFIER_READ_VEL;
			CAN.TransmitData = (int32_t)RAD_TO_MC_PULSE(PosSensor.MecAngularSpeed_rad);
			CAN_Transmit(CAN.Identifier, CAN.TransmitData, 4, DRIVER_CLIENT_CAN_ID, DRONCAN_PRIORITY_NORMAL);
			CAN.RecieveStatus = 0;
		
			break;		
		
		/*发送机械角度*/
		case (0x40 + IDENTIFIER_READ_POS):
			
			CAN.Identifier = IDENTIFIER_READ_POS;
			CAN.TransmitData = (int32_t)DRV_PULSE_TO_MC_PULSE(MainCtrl.RefMecAngle_pulse);
		
			CAN_Transmit(CAN.Identifier, CAN.TransmitData, 4, DRIVER_CLIENT_CAN_ID, DRONCAN_PRIORITY_NORMAL);
		
			CAN.RecieveStatus = 0;
		
			break;
		
		/*发送编���器位置*/
		case (0x40 + IDENTIFIER_READ_ENCODER_POS):
			
			CAN.Identifier = IDENTIFIER_READ_ENCODER_POS;
			CAN.TransmitData = (int32_t)DRV_PULSE_TO_MC_PULSE(PosSensor.MecAngle_15bit);
		
			CAN_Transmit(CAN.Identifier, CAN.TransmitData, 4, DRIVER_CLIENT_CAN_ID, DRONCAN_PRIORITY_NORMAL);
		
			CAN.RecieveStatus = 0;
		
			break;

		/*发送Vd*/		
		case (0x40 + IDENTIFIER_READ_VOL_D):
			
			CAN.Identifier = IDENTIFIER_READ_VOL_D;
			CAN.TransmitData = (int32_t)(CurrLoop.CtrlVolD * 1e3);
		
			CAN_Transmit(CAN.Identifier, CAN.TransmitData, 4, DRIVER_CLIENT_CAN_ID, DRONCAN_PRIORITY_NORMAL);
		
			CAN.RecieveStatus = 0;
		
			break;

		/*发送Id*/
		case (0x40 + IDENTIFIER_READ_CURR_D):
			
			CAN.Identifier = IDENTIFIER_READ_CURR_D;
			CAN.TransmitData = (int32_t)(CoordTrans.CurrD * 1e3);
		
			CAN_Transmit(CAN.Identifier, CAN.TransmitData, 4, DRIVER_CLIENT_CAN_ID, DRONCAN_PRIORITY_NORMAL);
		
			CAN.RecieveStatus = 0;
		
			break;
		
		/*发送Vq*/		
		case (0x40 + IDENTIFIER_READ_VOL_Q):
			
			CAN.Identifier = IDENTIFIER_READ_VOL_Q;
			CAN.TransmitData = (int32_t)(CurrLoop.CtrlVolQ * 1e3);
		
			CAN_Transmit(CAN.Identifier, CAN.TransmitData, 4, DRIVER_CLIENT_CAN_ID, DRONCAN_PRIORITY_NORMAL);
		
			CAN.RecieveStatus = 0;
		
			break;

		/*发送Iq*/
		case (0x40 + IDENTIFIER_READ_CURR_Q):
			
			CAN.Identifier = IDENTIFIER_READ_CURR_Q;
			CAN.TransmitData = (int32_t)(CoordTrans.CurrQ * 1e3);
		
			CAN_Transmit(CAN.Identifier, CAN.TransmitData, 4, DRIVER_CLIENT_CAN_ID, DRONCAN_PRIORITY_NORMAL);
		
			CAN.RecieveStatus = 0;
		
			break;
		
		/*发送速度环输出*/		
		case (0x40 + IDENTIFIER_READ_SPD_LOOP_OUTPUT):
			
			CAN.Identifier = IDENTIFIER_READ_SPD_LOOP_OUTPUT;
			CAN.TransmitData = (int32_t)(CurrLoop.ExptCurrQ * 1e3);
		
			CAN_Transmit(CAN.Identifier, CAN.TransmitData, 4, DRIVER_CLIENT_CAN_ID, DRONCAN_PRIORITY_NORMAL);
		
			CAN.RecieveStatus = 0;
		
			break;

		/*发送位置环输出*/
		case (0x40 + IDENTIFIER_READ_POS_LOOP_OUTPUT):
			
			CAN.Identifier = IDENTIFIER_READ_POS_LOOP_OUTPUT;
			CAN.TransmitData = (int32_t)(SpdLoop.ExptMecAngularSpeed_rad * 1e3);
		
			CAN_Transmit(CAN.Identifier, CAN.TransmitData, 4, DRIVER_CLIENT_CAN_ID, DRONCAN_PRIORITY_NORMAL);
		
			CAN.RecieveStatus = 0;
		
			break;
        
		case (0x40 +IDENTIFIER_HALL_TEST_OFF):
			
			CAN.Identifier = IDENTIFIER_HALL_TEST_OFF;
		
			CAN_Transmit(CAN.Identifier, CAN.TransmitData, 1, DRIVER_CLIENT_CAN_ID - 1, DRONCAN_PRIORITY_NORMAL);
		
			CAN.RecieveStatus = 0;
		
			break;

		case (0x40 +IDENTIFIER_READ_ACC):
			
			CAN.Identifier = IDENTIFIER_READ_ACC;
			CAN.TransmitData = (int32_t)DRV_PULSE_TO_MC_PULSE(MainCtrl.Acceleration_pulse);
			CAN_Transmit(CAN.Identifier, CAN.TransmitData, 4, DRIVER_CLIENT_CAN_ID, DRONCAN_PRIORITY_NORMAL);
		
			CAN.RecieveStatus = 0;

			break;	
		

		
		default:
			
			break;
	}
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan)
{
	CAN_Respond();
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan)
{
	CAN_Respond();
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan)
{
	CAN_Respond();
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	static uint16_t errTimes = 0;
	
	UART_Transmit_DMA("CAN ERROR: %d\r\n", (uint32_t)hcan->ErrorCode);
	SendBuf();
	
	if(errTimes++ <= 500)           //100
	{
		errTimes++;
		
		HAL_CAN_DeInit(&hcan1);
		HAL_CAN_Init(&hcan1);
		
		__HAL_CAN_CLEAR_FLAG(&hcan1, CAN_FLAG_FOV0);
		
		HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	}
	else
	{
//		PWM_CMD(DISABLE);
	}
}

/* Modified CAN_Transmit function to support DroneCAN */
void CAN_Transmit(uint8_t identifier, int32_t transmitData, uint8_t length, 
                   uint32_t dest_node_id, uint8_t priority)
{
	uint32_t absData = abs(transmitData);
	DroneCAN_ID_t tx_id;
	
	/* Build DroneCAN extended ID */
	tx_id.priority = priority;
	tx_id.message_type = identifier;  /* Map identifier to message_type */
	tx_id.source_node_id = dest_node_id;
	tx_id.transfer_id = 0;
	tx_id.frame_type = 0;  /* Single frame */
	tx_id.last_frame = 1;
	tx_id.frame_index = 0;
	
	TxMessage.StdId = 0;
	TxMessage.ExtId = DroneCAN_BuildID(&tx_id);
	TxMessage.IDE   = CAN_ID_EXT;
	TxMessage.RTR   = CAN_RTR_DATA;
	TxMessage.DLC   = length;
	
	CAN.Transmit.data_uint8[0] = (identifier>>0)&0xFF;
	
	if(transmitData >= 0)
	{
		for(uint8_t bytes = 1; bytes <= (length - 1); bytes++)
		{
			CAN.Transmit.data_uint8[bytes] = (absData>>((bytes - 1) * 8))&0xFF;
		}	
	}
	else if(transmitData < 0)
	{
		for(uint8_t bytes = 1; bytes <= (length - 1); bytes++)
		{

			CAN.Transmit.data_uint8[bytes] = (absData>>((bytes - 1) * 8))&0xFF;
			if(bytes == (length - 1))
			{
				CAN.Transmit.data_uint8[bytes] = ((absData>>((bytes - 1) * 8))&0xFF) | 0x80; /*加负号*/
			}
		}	
	}
	
	HAL_CAN_AddTxMessage(&hcan1, &TxMessage, (uint8_t *)&CAN.Transmit, &CAN.MailBox);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);
}

/**
 * @brief  Send raw bytes as a DroneCAN extended-frame message.
 *
 * Unlike CAN_Transmit(), this function sends the caller-supplied byte
 * array verbatim (no proprietary identifier byte prepended).  It uses
 * its own local TX-header and a private data buffer so it is safe to
 * call from a different ISR context than CAN_Transmit().
 *
 * @param data         Pointer to payload bytes (up to 8).
 * @param length       Number of bytes to send (1-8).
 * @param message_type DroneCAN message-type ID placed in the extended CAN ID.
 * @param priority     DroneCAN priority (DRONCAN_PRIORITY_*).
 */
void CAN_Transmit_Raw(const uint8_t *data, uint8_t length,
                      uint16_t message_type, uint8_t priority)
{
	CAN_TxHeaderTypeDef txHeader = {0};
	uint32_t mailbox;
	uint8_t  txBuf[8] = {0};
	DroneCAN_ID_t tx_id;

	if (data == NULL || length == 0U || length > 8U)
	{
		return;
	}

	/* Build DroneCAN extended ID for an ESC status (single-frame) */
	tx_id.priority     = priority;
	tx_id.message_type = message_type;
	tx_id.source_node_id = (uint8_t)(DRIVER_CLIENT_CAN_ID & 0x7FU);
	tx_id.transfer_id  = 0U;
	tx_id.frame_type   = 0U;
	tx_id.last_frame   = 1U;
	tx_id.frame_index  = 0U;

	txHeader.StdId = 0U;
	txHeader.ExtId = DroneCAN_BuildID(&tx_id);
	txHeader.IDE   = CAN_ID_EXT;
	txHeader.RTR   = CAN_RTR_DATA;
	txHeader.DLC   = length;

	for (uint8_t i = 0U; i < length; i++)
	{
		txBuf[i] = data[i];
	}

	HAL_CAN_AddTxMessage(&hcan1, &txHeader, txBuf, &mailbox);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY);
}



void CAN_Receive(uint32_t *stdId, uint8_t *identifier, int32_t *receiveData)
{	
	HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxMessage0, (uint8_t *)&CAN.Receive);
	
	/* 支持 DroneCAN 扩展 ID */
	if (RxMessage0.IDE == CAN_ID_EXT)
	{
		*stdId = RxMessage0.ExtId;
	}
	else
	{
		*stdId = RxMessage0.StdId;
	}
	
	*identifier = CAN.Receive.data_uint8[0];
	
	if(((CAN.Receive.data_uint8[3]&0x80)>>7) == 0)
	{
		/*当数据为正数时*/
		*receiveData = (int32_t)((CAN.Receive.data_uint8[3]<<16) | (CAN.Receive.data_uint8[2]<<8) | (CAN.Receive.data_uint8[1]<<0));
	}
	else if(((CAN.Receive.data_uint8[3]&0x80)>>7) == 1)
	{
		/*当数据为负数时*/
		*receiveData = -(int32_t)(((CAN.Receive.data_uint8[3]&0x7F)<<16) | (CAN.Receive.data_uint8[2]<<8) | (CAN.Receive.data_uint8[1]<<0));
	}
}

/*设置DroneCAN滤波器，接收指定优先级和消息类型的消息*/	
void CAN_Enable(void)
{
	CAN_FilterTypeDef CAN1_FilerConf = {0};
	
	/* 配置CAN滤波器为32位宽的掩码模式 (支持扩展ID) */
	CAN1_FilerConf.FilterBank           = 0;
	CAN1_FilerConf.FilterMode           = CAN_FILTERMODE_IDMASK;
	CAN1_FilerConf.FilterScale          = CAN_FILTERSCALE_32BIT;
	CAN1_FilerConf.FilterActivation     = ENABLE;
	CAN1_FilerConf.SlaveStartFilterBank = 14;
	
	/* 
	 * DroneCAN ID: [Priority(3) | MessageType(14) | SourceNodeID(7) | TransferID(2) | FrameType(1) | LastFrame(1) | FrameIndex(3)]
	 * 接收所有优先级和消息类型的消息
	 * FilterIdHigh/FilterIdLow: 扩展ID的高32位
	 * FilterMaskIdHigh/FilterMaskIdLow: 掩码
	 */
	CAN1_FilerConf.FilterIdHigh         = 0x0000;
	CAN1_FilerConf.FilterIdLow          = 0x0000;
	CAN1_FilerConf.FilterMaskIdHigh     = 0x0000;
	CAN1_FilerConf.FilterMaskIdLow      = 0x0000;
	CAN1_FilerConf.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	CAN1_FilerConf.FilterActivation     = ENABLE;

	if(HAL_CAN_ConfigFilter(&hcan1, &CAN1_FilerConf) != HAL_OK) 
	{
		Error_Handler();
	}
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_LAST_ERROR_CODE);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_BUSOFF);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_ERROR);
}

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/