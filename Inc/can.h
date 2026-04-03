/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "adc.h"
#include "foc.h"
#include "PositionSensor.h"
#include "MotorConfig.h"
#include "usart.h"
#include "observer.h"
/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */

#define DRIVER_CLIENT_BASE_ID		0x280
#define DRIVER_SERVER_BASE_ID		0x300

#define DRIVER_CLIENT_CAN_ID		(DRIVER_CLIENT_BASE_ID + CAN_ID_NUM)
#define DRIVER_SERVER_CAN_ID		(DRIVER_SERVER_BASE_ID + CAN_ID_NUM)
#define DRIVER_BROADCAST_ID			(DRIVER_SERVER_BASE_ID + 0x40 + GROUP_NUM)

/*各标识符宏定义需与主控同�?*/
/*控制标识�?*/
#define IDENTIFIER_DRIVER_STATE				0x01
#define IDENTIFIER_CURR_KP_Q			  	0x02
#define IDENTIFIER_CURR_KI_Q			   	0x03
#define IDENTIFIER_SPD_KP					    0x04
#define IDENTIFIER_SPD_KI					    0x05
#define IDENTIFIER_POS_KP					    0x06
#define IDENTIFIER_POS_KD				    	0x07
#define IDENTIFIER_TORQUE_CTRL				0x08
#define IDENTIFIER_VEL_CTRL					  0x09
#define IDENTIFIER_POS_CTRL_ABS				0x0A
#define IDENTIFIER_POS_CTRL_REL				0x0B
#define IDENTIFIER_SET_CTRL_MODE			0x0C
#define IDENTIFIER_SET_ACC				    	0x0D
#define IDENTIFIER_SET_DEC					    0x0E
#define IDENTIFIER_SET_TORQUE_LIMIT			0x0F
#define IDENTIFIER_SET_VEL_LIMIT			  0x10
#define IDENTIFIER_SET_POS_LIMIT_UP			0x11
#define IDENTIFIER_SET_POS_LIMIT_LOW		0x12
#define IDENTIFIER_CORRECT_POS_OFFSET		0x13
#define IDENTIFIER_SET_INTEGRAL_CLEARED	0x14
#define IDENTIFIER_SET_LOAD_GAIN_1      0x17
#define IDENTIFIER_SET_LOAD_GAIN_2      0x18
#define IDENTIFIER_SET_CONTROL_MODE		  0x19
#define IDENTIFIER_SET_CLEAR_INTEGRAL		0x1A

/*读取标识�?*/	
#define IDENTIFIER_ENABLE_DONE			0x50
#define IDENTIFIER_READ_EXVEL       0x18
#define IDENTIFIER_READ_ACC				  0x19
#define IDENTIFIER_READ_TORQUE			0x20
#define IDENTIFIER_READ_VEL					0x21
#define IDENTIFIER_READ_POS					0x22
#define IDENTIFIER_READ_ENCODER_POS	0x23
#define IDENTIFIER_READ_VOL_D				0x24
#define IDENTIFIER_READ_CURR_D			0x25
#define IDENTIFIER_READ_VOL_Q				0x26
#define IDENTIFIER_READ_CURR_Q			0x27
#define IDENTIFIER_READ_SPD_LOOP_OUTPUT		0x28
#define IDENTIFIER_READ_POS_LOOP_OUTPUT		0x29
#define IDENTIFIER_READ_LOAD_OBSERVER   	0x60

/*错误标识�??*/
#define IDENTIFIER_ENCODER_ERROR		0xEE
#define IDENTIFIER_HARD_FAULT				0xFF

/*霍尔标识�?*/
#define IDENTIFIER_HALL_CATCH     0x31 

/*寻找舵轮磁铁位置指令*/
#define IDENTIFIER_HALL_TEST_ON      0x32
#define IDENTIFIER_HALL_TEST_OFF     0x33



typedef union
{
	uint32_t  data_uint32[2];
	int32_t   data_int32[2];
	uint8_t   data_uint8[8];
} CAN_Data_t;

struct CAN_t
{
	uint32_t StdID;
	uint32_t MailBox;
	uint8_t Identifier;
	int32_t ReceiveData;
	uint32_t RecieveStatus;
	int32_t TransmitData;
	CAN_Data_t Receive;
	CAN_Data_t Transmit;
};

/* USER CODE END Private defines */


void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */
void CAN_Respond(void);
void CAN_Transmit(uint8_t identifier, int32_t transmitData, uint8_t length, uint32_t dest_node_id, uint8_t priority);
void CAN_Receive(uint32_t *stdId, uint8_t *identifier, int32_t *receiveData);
void CAN_Enable(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
