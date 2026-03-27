/**
 ******************************************************************************
 * @file		MotorConfig.h
 * @author		WrathWings
 * @version 	V1.0
 * @date		2019.1.17
 * @brief		The header file of MotorConfig.c
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
 /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTORCONFIG_H
#define __MOTORCONFIG_H

/* Private includes ----------------------------------------------------------*/
/* CODE BEGIN Includes */
#include "main.h"
/* CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/****************************************Type Define Begin****************************************/
/*Robot ID*/
#define R1                      1U        //第一底盘和第一射箭机构
#define R2     									2U        //第二底盘和第二射箭机构
#define Test_Robot              3U

/* Phase Sequence */
#define POSITIVE_SEQUENCE				1
#define NEGATIVE_SEQUENCE				2

/* Motor Type */
#define MAD_XC5500_KV505				          1
#define N5055_KV400						            2
#define TMOTOR_U3_KV700					          3
#define TMOTOR_P80_KV100				          4	
#define TMOTOR_MN505_S_KV320			        5	
#define SUNNYSKY_X4125_9_KV350			      6
#define SUNNYSKY_X4125_3_KV210			      7
#define IFLIGHT_T4214_KV660				        8
#define LEOPARD_HOBBY_PH2820_KV780		    9
#define CRAZY_MOTOR_5025_KV200			     10
#define MAD_XC5000_KV380				         11
#define SUNNYSKY_X4014_KV330					   12
#define SUNNYSKY_X3520_KV560             13
#define SUNNYSKY_X4130_KV520             14
#define TMOTOR_F1000_KV300               15
#define MAD5015_KV320                    16
#define MAD_3506_KV400                   17
#define SUNNYSKY_X2212_13                18


/* Position Sensor Type */
#define ENCODER_TLE5012										1
#define HALL_SENSOR_DRV5053								2

/* Gate Driver Type */
#define GATE_DRIVER_DRV8323								1
#define GATE_DRIVER_DISCRETE							2

/* MOSFET Type */
#define CDS18535_63nC_1mOhm6							1
#define IPD053N08N3G_52nC_5mOhm3					2
#define CSD88584Q5DC_137nC_0mOhm68				3
#define IPT015N10N5ATMA1_169nC_1mOhm5			4

/* Current Sensor */
#define RES_1mOhm						1
#define RES_2mOhm						2
#define RES_200uOhm         3

/****************************************Type Define End****************************************/
																   
#define ROBOT_ID		     			R1
#define CAN_ID_NUM		        1


#if ROBOT_ID == R1        
	#if CAN_ID_NUM == 1       
	#define MOTOR_TYPE 							SUNNYSKY_X2212_13	
	#define	PHASE_SEQUENCE					NEGATIVE_SEQUENCE	
	#define POSITION_SENSOR_TYPE		ENCODER_TLE5012
	#define GATE_DRIVER_TYPE				GATE_DRIVER_DISCRETE
	#define MOSFET_TYPE							IPT015N10N5ATMA1_169nC_1mOhm5
	#define CURRENT_SENSOR					RES_200uOhm
	#define GROUP_NUM           		1
	#endif
	#if CAN_ID_NUM == 2         
	#define MOTOR_TYPE 		 					MAD_XC5500_KV505	
	#define	PHASE_SEQUENCE					NEGATIVE_SEQUENCE	
	#define POSITION_SENSOR_TYPE		ENCODER_TLE5012
	#define GATE_DRIVER_TYPE				GATE_DRIVER_DISCRETE
	#define MOSFET_TYPE							IPT015N10N5ATMA1_169nC_1mOhm5
	#define CURRENT_SENSOR					RES_500uOhm
	#define GROUP_NUM           		1
	#endif
	#if CAN_ID_NUM == 3       
	#define MOTOR_TYPE 							MAD_XC5500_KV505	
	#define	PHASE_SEQUENCE					NEGATIVE_SEQUENCE
	#define POSITION_SENSOR_TYPE		ENCODER_TLE5012
	#define GATE_DRIVER_TYPE				GATE_DRIVER_DISCRETE
	#define MOSFET_TYPE							IPT015N10N5ATMA1_169nC_1mOhm5
	#define CURRENT_SENSOR					RES_500uOhm
	#define GROUP_NUM           		1
	#endif
	#if CAN_ID_NUM == 4       
	#define MOTOR_TYPE 							MAD5015_KV320	
	#define	PHASE_SEQUENCE					POSITIVE_SEQUENCE	
	#define POSITION_SENSOR_TYPE		ENCODER_TLE5012
	#define GATE_DRIVER_TYPE				GATE_DRIVER_DISCRETE
	#define MOSFET_TYPE							IPT015N10N5ATMA1_169nC_1mOhm5
	#define CURRENT_SENSOR					RES_500uOhm
	#define GROUP_NUM           		1
	#endif
	#if CAN_ID_NUM == 5       
	#define MOTOR_TYPE 							MAD_XC5500_KV505	
	#define	PHASE_SEQUENCE					NEGATIVE_SEQUENCE
	#define POSITION_SENSOR_TYPE		ENCODER_TLE5012
	#define GATE_DRIVER_TYPE				GATE_DRIVER_DISCRETE
	#define MOSFET_TYPE							IPT015N10N5ATMA1_169nC_1mOhm5
	#define CURRENT_SENSOR					RES_500uOhm
	#define GROUP_NUM           		1
	#endif

#endif
#if ROBOT_ID == R2	
	#if CAN_ID_NUM == 2		   			  //夹块
	#define MOTOR_TYPE 							SUNNYSKY_X3520_KV560	
	#define	PHASE_SEQUENCE					POSITIVE_SEQUENCE
	#define POSITION_SENSOR_TYPE		ENCODER_TLE5012
	#define GATE_DRIVER_TYPE				GATE_DRIVER_DISCRETE
	#define MOSFET_TYPE							IPT015N10N5ATMA1_169nC_1mOhm5
	#define CURRENT_SENSOR					RES_500uOhm
	#define GROUP_NUM           		1
	#endif
	#if CAN_ID_NUM == 3		    		  //左转块
	#define MOTOR_TYPE 							RM2006_KV1187	
	#define	PHASE_SEQUENCE					NEGATIVE_SEQUENCE
	#define POSITION_SENSOR_TYPE		ENCODER_TLE5012
	#define GATE_DRIVER_TYPE				GATE_DRIVER_DISCRETE
	#define MOSFET_TYPE							IPT015N10N5ATMA1_169nC_1mOhm5
	#define CURRENT_SENSOR					RES_500uOhm	
	#define GROUP_NUM           		1
	#endif		
	#if CAN_ID_NUM == 4          		//右转块
	#define MOTOR_TYPE 							RM2006_KV1187	
	#define	PHASE_SEQUENCE					NEGATIVE_SEQUENCE
	#define POSITION_SENSOR_TYPE		ENCODER_TLE5012
	#define GATE_DRIVER_TYPE				GATE_DRIVER_DISCRETE
	#define MOSFET_TYPE							IPT015N10N5ATMA1_169nC_1mOhm5
	#define CURRENT_SENSOR					RES_500uOhm
	#define GROUP_NUM           		1
	#endif
#endif
#if ROBOT_ID == Test_Robot        
	#if CAN_ID_NUM == 1       
	#define MOTOR_TYPE 							MAD_XC5500_KV505	
	#define	PHASE_SEQUENCE					POSITIVE_SEQUENCE	
	#define POSITION_SENSOR_TYPE		ENCODER_TLE5012
	#define GATE_DRIVER_TYPE				GATE_DRIVER_DISCRETE
	#define MOSFET_TYPE							IPT015N10N5ATMA1_169nC_1mOhm5
	#define CURRENT_SENSOR					RES_500uOhm
	#define GROUP_NUM           		1         		
	#endif
#endif

/* Motor Type*/
#if MOTOR_TYPE == MAD_XC5500_KV505	  
	#define MOTOR_POLE_PAIRS_NUM				7
	#define	MOTOR_KV										505.f
	#define ROTATOR_FLUX_LINKAGE				(5.513288954f / (MOTOR_KV * MOTOR_POLE_PAIRS_NUM))
	#define PHASE_RES										(20.0f / 2.f * (float)1e-3)	//(Ohm)
	#define INDUCTANCE_D								(12.2f / 2.f * (float)1e-6)	//(H)
	#define INDUCTANCE_Q								(19.0f / 2.f * (float)1e-6)	//(H)
	#define INERTIA											(160168.f * 1e-9)
	#define MAX_SPD(a)									(a * MOTOR_KV * 0.10471975)	//rad/s
	#define CurrenIN_Max                100.0f
#elif	MOTOR_TYPE == N5055_KV400
	#define MOTOR_POLE_PAIRS_NUM				7
	#define	MOTOR_KV										400.f
	#define ROTATOR_FLUX_LINKAGE				(5.513288954f / (MOTOR_KV * MOTOR_POLE_PAIRS_NUM))
	#define PHASE_RES										(22.f * (float)1e-3)	//(Ohm)
	#define INDUCTANCE_D								(15.f * (float)1e-6)	//(H)
	#define INDUCTANCE_Q								(15.f * (float)1e-6)	//(H)
	#define INERTIA											(160168.f * 1e-9)
	#define MAX_SPD(a)									(a * MOTOR_KV * 0.10471975)	//rad/s
	#define CurrenIN_Max                20.0f
#elif	MOTOR_TYPE == TMOTOR_U3_KV700
	#define MOTOR_POLE_PAIRS_NUM				7
	#define	MOTOR_KV										700.f
	#define ROTATOR_FLUX_LINKAGE				(5.513288954f / (MOTOR_KV * MOTOR_POLE_PAIRS_NUM))
	#define PHASE_RES										(85.f * (float)1e-3)	//(Ohm)
	#define INDUCTANCE_D								(22.f * (float)1e-6)	//(H)
	#define INDUCTANCE_Q								(22.f * (float)1e-6)	//(H)
	#define INERTIA											(160168.f * 1e-9)
	#define MAX_SPD(a)									(a * MOTOR_KV * 0.10471975)	//rad/s
	#define CurrenIN_Max                20.0f
#elif	MOTOR_TYPE == TMOTOR_P80_KV100
	#define MOTOR_POLE_PAIRS_NUM				21
	#define	MOTOR_KV										100.f
	#define ROTATOR_FLUX_LINKAGE				(5.513288954f / (MOTOR_KV * MOTOR_POLE_PAIRS_NUM))
	#define PHASE_RES										(20.f * (float)1e-3)	//(Ohm)
	#define INDUCTANCE_D								(24.f * (float)1e-6)	//(H)
	#define INDUCTANCE_Q								(24.f * (float)1e-6)	//(H)
	#define INERTIA											(160168.f * 1e-9)
	#define MAX_SPD(a)									(a * MOTOR_KV * 0.10471975)	//rad/s
	#define CurrenIN_Max                20.0f
#elif	MOTOR_TYPE == TMOTOR_MN505_S_KV320
	#define MOTOR_POLE_PAIRS_NUM				14
	#define	MOTOR_KV										320.f
	#define ROTATOR_FLUX_LINKAGE				(5.513288954f / (MOTOR_KV * MOTOR_POLE_PAIRS_NUM))
	#define PHASE_RES										(19.f * (float)1e-3)	//(Ohm)
	#define INDUCTANCE_D								(15.f * (float)1e-6)	//(H)
	#define INDUCTANCE_Q								(15.f * (float)1e-6)	//(H)
	#define INERTIA											(160168.f * 1e-9)
	#define MAX_SPD(a)									(a * MOTOR_KV * 0.10471975)	//rad/s
	#define CurrenIN_Max                20.0f
#elif	MOTOR_TYPE == SUNNYSKY_X4125_9_KV350
	#define MOTOR_POLE_PAIRS_NUM				7
	#define	MOTOR_KV										350.f
	#define ROTATOR_FLUX_LINKAGE				(5.513288954f / (MOTOR_KV * MOTOR_POLE_PAIRS_NUM))
	#define PHASE_RES										(18.2f * (float)1e-3)	//(Ohm)
	#define INDUCTANCE_D								(12.f * (float)1e-6)	//(H)
	#define INDUCTANCE_Q								(12.f * (float)1e-6)	//(H)
	#define INERTIA											(160168.f * 1e-9)
	#define MAX_SPD(a)									(a * MOTOR_KV * 0.10471975)	//rad/s
	#define CurrenIN_Max                20.0f
#elif	MOTOR_TYPE == SUNNYSKY_X4125_3_KV210	
	#define MOTOR_POLE_PAIRS_NUM				7
	#define	MOTOR_KV										210.f
	#define ROTATOR_FLUX_LINKAGE				(5.513288954f / (MOTOR_KV * MOTOR_POLE_PAIRS_NUM))
	#define PHASE_RES										(18.2f * (float)1e-3)	//(Ohm)
	#define INDUCTANCE_D								(15.f * (float)1e-6)	//(H)
	#define INDUCTANCE_Q								(15.f * (float)1e-6)	//(H)
	#define INERTIA											(160168.f * 1e-9)
	#define MAX_SPD(a)									(a * MOTOR_KV * 0.10471975)	//rad/s
	#define CurrenIN_Max                20.0f
#elif	MOTOR_TYPE == IFLIGHT_T4214_KV660	
	#define MOTOR_POLE_PAIRS_NUM				7
	#define	MOTOR_KV										660.f
	#define ROTATOR_FLUX_LINKAGE				(5.513288954f / (MOTOR_KV * MOTOR_POLE_PAIRS_NUM))
	#define PHASE_RES										(18.2f * (float)1e-3)	//(Ohm)
	#define INDUCTANCE_D								(15.f * (float)1e-6)	//(H)
	#define INDUCTANCE_Q								(15.f * (float)1e-6)	//(H)
	#define INERTIA											(160168.f * 1e-9)
	#define MAX_SPD(a)									(a * MOTOR_KV * 0.10471975)	//rad/s
	#define CurrenIN_Max                20.0f
#elif	MOTOR_TYPE == LEOPARD_HOBBY_PH2820_KV780	
	#define MOTOR_POLE_PAIRS_NUM				7
	#define	MOTOR_KV										780.f
	#define ROTATOR_FLUX_LINKAGE				(5.513288954f / (MOTOR_KV * MOTOR_POLE_PAIRS_NUM))
	#define PHASE_RES										(35.f * (float)1e-3)	//(Ohm)
	#define INDUCTANCE_D								(11.f * (float)1e-6)	//(H)
	#define INDUCTANCE_Q								(11.f * (float)1e-6)	//(H)
	#define INERTIA											(160168.f * 1e-9)
	#define MAX_SPD(a)									(a * MOTOR_KV * 0.10471975)	//rad/s
	#define CurrenIN_Max                20.0f
#elif	MOTOR_TYPE == CRAZY_MOTOR_5025_KV200	
	#define MOTOR_POLE_PAIRS_NUM				7
	#define	MOTOR_KV										200.f
	#define ROTATOR_FLUX_LINKAGE				(5.513288954f / (MOTOR_KV * MOTOR_POLE_PAIRS_NUM))
	#define PHASE_RES										(30.f * (float)1e-3)	//(Ohm)
	#define INDUCTANCE_D								(25.f * (float)1e-6)	//(H)
	#define INDUCTANCE_Q								(25.f * (float)1e-6)	//(H)
	#define INERTIA											(160168.f * 1e-9)
	#define MAX_SPD(a)									(a * MOTOR_KV * 0.10471975)	//rad/s
	#define CurrenIN_Max                20.0f
#elif	MOTOR_TYPE == MAD_XC5000_KV380	
	#define MOTOR_POLE_PAIRS_NUM				7
	#define	MOTOR_KV										380.f
	#define ROTATOR_FLUX_LINKAGE				(5.513288954f / (MOTOR_KV * MOTOR_POLE_PAIRS_NUM))
	#define PHASE_RES										(29.f * (float)1e-3)	//(Ohm)
	#define INDUCTANCE_D								(24.f * (float)1e-6)	//(H)
	#define INDUCTANCE_Q								(24.f * (float)1e-6)	//(H)
	#define INERTIA											(160168.f * 1e-9)
	#define MAX_SPD(a)									(a * MOTOR_KV * 0.10471975)	//rad/s
	#define CurrenIN_Max                20.0f
#elif	MOTOR_TYPE == SUNNYSKY_X4014_KV330	
	#define MOTOR_POLE_PAIRS_NUM				12
	#define	MOTOR_KV										330.f
	#define ROTATOR_FLUX_LINKAGE				(5.513288954f / (MOTOR_KV * MOTOR_POLE_PAIRS_NUM))
	#define PHASE_RES										(88.0f / 2.f * (float)1e-3)	//(Ohm)
	#define INDUCTANCE_D								(32.1f / 2.f * (float)1e-6)	//(H)
	#define INDUCTANCE_Q								(40.3f / 2.f * (float)1e-6)	//(H)
	#define INERTIA											(160168.f * 1e-9)
	#define MAX_SPD(a)									(a * MOTOR_KV * 0.10471975)	//rad/s	
	#define CurrenIN_Max                20.0f
#elif	MOTOR_TYPE == SUNNYSKY_X3520_KV560	
	#define MOTOR_POLE_PAIRS_NUM				7
	#define	MOTOR_KV										560.f
	#define ROTATOR_FLUX_LINKAGE				(5.513288954f / (MOTOR_KV * MOTOR_POLE_PAIRS_NUM))
	#define PHASE_RES										(23.f * (float)1e-3)	//(Ohm)
	#define INDUCTANCE_D								(10.6f / 2 * (float)1e-6)	//(H)
	#define INDUCTANCE_Q								(18.3f / 2 * (float)1e-6)	//(H)
	#define INERTIA											(160168.f * 1e-9)
	#define MAX_SPD(a)									(a * MOTOR_KV * 0.10471975)	//rad/s
	#define CurrenIN_Max                40.0f
#elif	MOTOR_TYPE == SUNNYSKY_X4130_KV520	
	#define MOTOR_POLE_PAIRS_NUM				7
	#define	MOTOR_KV										520.f
	#define ROTATOR_FLUX_LINKAGE				(5.513288954f / (MOTOR_KV * MOTOR_POLE_PAIRS_NUM))
	#define PHASE_RES										(12.9f * (float)1e-3)	//(Ohm)
	#define INDUCTANCE_D								(5.2f / 2 * (float)1e-6)	//(H)
	#define INDUCTANCE_Q								(8.6f / 2 * (float)1e-6)	//(H)
	#define INERTIA											(160168.f * 1e-9)
	#define MAX_SPD(a)									(a * MOTOR_KV * 0.10471975)	//rad/s	
	#define CurrenIN_Max                20.0f
#elif	MOTOR_TYPE == TMOTOR_F1000_KV300	
	#define MOTOR_POLE_PAIRS_NUM				7
	#define	MOTOR_KV										300.f
	#define ROTATOR_FLUX_LINKAGE				(5.513288954f / (MOTOR_KV * MOTOR_POLE_PAIRS_NUM))
	#define PHASE_RES										(27.f * (float)1e-3)	//(Ohm)
	#define INDUCTANCE_D								(21.f * (float)1e-6)	//(H)
	#define INDUCTANCE_Q								(21.f * (float)1e-6)	//(H)
	#define INERTIA											(160168.f * 1e-9)
	#define MAX_SPD(a)									(a * MOTOR_KV * 0.10471975)	//rad/s
	#define CurrenIN_Max                80.0f
#elif	MOTOR_TYPE == MAD5015_KV320	
	#define MOTOR_POLE_PAIRS_NUM				14
	#define	MOTOR_KV										320.f
	#define ROTATOR_FLUX_LINKAGE				(5.513288954f / (MOTOR_KV * MOTOR_POLE_PAIRS_NUM))
	#define PHASE_RES										(36.8f / 2.f* (float)1e-3)	//(Ohm)
	#define INDUCTANCE_D								(13.0f / 2.f * (float)1e-6)	//(H)
	#define INDUCTANCE_Q								(18.3f / 2.f * (float)1e-6)	//(H)
	#define INERTIA											(160168.f * 1e-9)
	#define MAX_SPD(a)									(a * MOTOR_KV * 0.10471975)	//rad/s
	#define CurrenIN_Max                50.0f
#elif	MOTOR_TYPE == RM2006_KV1187	
	#define MOTOR_POLE_PAIRS_NUM				7
	#define	MOTOR_KV										1187.f
	#define ROTATOR_FLUX_LINKAGE				(5.513288954f / (MOTOR_KV * MOTOR_POLE_PAIRS_NUM))
	#define PHASE_RES										(285.f / 2.f* (float)1e-3)	//(Ohm)
	#define INDUCTANCE_D								(41.0f / 2.f * (float)1e-6)	//(H)
	#define INDUCTANCE_Q								(49.3f / 2.f * (float)1e-6)	//(H)
	#define INERTIA											(160168.f * 1e-9)
	#define MAX_SPD(a)									(a * MOTOR_KV * 0.10471975)	//rad/s
	#define CurrenIN_Max                20.f
#elif	MOTOR_TYPE == MAD_3506_KV400
	#define MOTOR_POLE_PAIRS_NUM				6
	#define	MOTOR_KV										400.f
	#define ROTATOR_FLUX_LINKAGE				(5.513288954f / (MOTOR_KV * MOTOR_POLE_PAIRS_NUM))
	#define PHASE_RES										(234.f / 2.f* (float)1e-3)	//(Ohm)
	#define INDUCTANCE_D								(141.2f / 2.f * (float)1e-6)	//(H)
	#define INDUCTANCE_Q								(229.f / 2.f * (float)1e-6)	//(H)
	#define INERTIA											(160168.f * 1e-9)
	#define MAX_SPD(a)									(a * MOTOR_KV * 0.10471975)	//rad/s	
	#define CurrenIN_Max                20.0f
#elif	MOTOR_TYPE == SUNNYSKY_X2212_13
	#define MOTOR_POLE_PAIRS_NUM				7
	#define	MOTOR_KV										980.f
	#define ROTATOR_FLUX_LINKAGE				(5.513288954f / (MOTOR_KV * MOTOR_POLE_PAIRS_NUM))
	#define PHASE_RES										(12.5f / 2 * (float)1e-3)	//(Ohm) 包含采样电阻
	#define INDUCTANCE_D								(27.2f / 2.f * (float)1e-6)	//(H)
	#define INDUCTANCE_Q								(24.3f / 2.f * (float)1e-6)	//(H)
	#define INERTIA											(160168.f * 1e-9)
	#define MAX_SPD(a)									(a * MOTOR_KV * 0.10471975)	//rad/s		
	#define CurrenIN_Max                100.0f

#else
#error "Motor Type Invalid"
#endif



/* USER CODE END PD */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */


#endif

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/
