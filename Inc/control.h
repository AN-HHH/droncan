/**
 ******************************************************************************
 * @file		control.h
 * @author		WrathWings
 * @version 	V1.0
 * @date		2019.1.17
 * @brief		The header file of control.c
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
 /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CONTROL_H
#define __CONTROL_H

/* Private includes ----------------------------------------------------------*/
/* CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "MotorConfig.h"
#include "math.h"
#include "foc.h"
#include "observer.h"
/* CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */




struct CurrLoop_t
{
	float LimitCurrQ;
	float LimitVolD;
	float LimitVolQ;
	float ExptCurrD;
	float ExptCurrQ;
	float CtrlVolD;
	float CtrlVolQ;
	float ErrD;
	float ErrQ;
	float IntegralErrD;
	float IntegralErrQ;
	float Kp_D;
	float Ki_D;
	float Kp_Q;
	float Ki_Q;
};

struct SpdLoop_t
{
	float ExptMecAngularSpeed_rad;		//期望机械角速度(rad/s)
	float MaxExptMecAngularSpeed_rad;	//最大期望角速度(rad/s)，用于限幅
	float Acceleration;					//加速度(rad/s2)
	float Deceleration;					//减速度(rad/s2)
	float Err;
	float IntegralErr;
	float Kp;
	float Ki;
};

struct PosLoop_t
{
	int32_t RefMecAngle_pulse; 
	float ExptMecAngle_rad;			//期望机械角度(rad)
	float MecAngleUpperLimit_rad;	//机械角度上限(rad)
	float MecAngleLowerLimit_rad;	//机械角度下限(rad)
	float RefMecAngle_rad;			//参考机械角度(rad), 主控的参考零位置
	float Err;
	float LastErr;
	float DiffErr;
	float Kp;
	float Kd;
	float BufferInterval;
	float Acceleration;					//加速度(rad/s2)
	float Deceleration;					//减速度(rad/s2)
	float real_ExptAngle_rad;
	uint8_t PosStatusSign;
};

struct TorqueCtrl_t
{
	float ExptTorque_Nm;	//期望转矩(脉冲)
	float EleTorque_Nm;		//电磁转矩(脉冲)
	float MaxTorque_Nm;		//最大转矩(脉冲)
	float MaxMecSpd_rad;	//最大机械角速度(脉冲), 该转速限幅仅用于转矩控制模式
};

struct MainCtrl_t
{
	int32_t ExptMecAngularSpeed_pulse;			//期望机械角速度(脉冲)
	uint32_t MaxMecAngularSpeed_pulse;			//最大机械角速度(脉冲), 用于限幅
	uint32_t Acceleration_pulse;				//加速度(脉冲)
	uint32_t Deceleration_pulse;				//加速度(脉冲)
	int32_t ExptMecAngle_pulse;					//期望机械角度(脉冲)
	int32_t MecAngleUpperLimit_pulse;			//机械角度上限(脉冲), 用于限幅
	int32_t MecAngleLowerLimit_pulse;			//机械角度下限(脉冲), 用于限幅
	int32_t RefMecAngle_pulse;					
	int32_t PresentMecAngle_pulse;
	int32_t LastMecAngle_pulse;
	uint16_t CAN_FaultSign;
	uint8_t  RefMecAngle_Mode;
	uint16_t PosOffset;
};

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#if ROBOT_ID == R1
	#define PERIOD_MULTIPLE								10	//(外环周期 / 电流环周期)
#elif ROBOT_ID == R2
	#if CAN_ID_NUM  == 3
	#define PERIOD_MULTIPLE								2	//(外环周期 / 电流环周期)
	#elif CAN_ID_NUM == 4
	#define PERIOD_MULTIPLE					 			2	//(外环周期 / 电流环周期)
	#else  
	#define PERIOD_MULTIPLE								10	//(外环周期 / 电流环周期)
	#endif
#endif

#define OUTER_LOOP_PERIOD							(DEFAULT_CARRIER_PERIOD_S * PERIOD_MULTIPLE)	//外环周期

#define CURR_INTEGRAL_ERR_LIM_D 			(12.0f / CurrLoop.Ki_D)	//Id积分限幅
#define CURR_INTEGRAL_ERR_LIM_Q 			(6.0f / CurrLoop.Ki_Q)	//Iq积分限幅
#define SPD_INTEGRAL_ERR_LIM					(150 / SpdLoop.Ki)  //积分限幅150A

/*电流环参数*/
#define CURRENT_CONTROL_KP_D			(7500.f*INDUCTANCE_D)//d轴电流环采用的传统PI控制器, Kp = d轴电感 * 电流环带宽  //90
#define CURRENT_CONTROL_KI_D			(3700.f*PHASE_RES)	//d轴电流环采用的传统PI控制器, Ki = 相电阻 * 电流环带宽     //280
#define CURRENT_CONTROL_KP_Q			(7500.f*INDUCTANCE_Q)
#define CURRENT_CONTROL_KI_Q			(3700.f*PHASE_RES)	//q轴电流环采用基于模型改进的PI控制器, Ki待调节  

#define SPEED_CONTROL_KP				1.0f
#define SPEED_CONTROL_KI				1.0f

#define POSITION_CONTROL_KP				60.0f
#define POSITION_CONTROL_KD				0.1f

#define MC_CTRL_RESOLUTION				4096.f	//变量分辨率

/*控制模式宏定义, 需与主控同步*/
#define SPD_CURR_CTRL_MODE 				1
#define POS_SPD_CURR_CTRL_MODE 			2
#define POS_CURR_CTRL_MODE 				3
#define TORQUE_CTRL_MODE 				4
#define	SPD_VOL_CTRL_MODE 				5
#define	POS_SPD_VOL_CTRL_MODE 			6

#define WORK_MODE						1
#define CORRECT_POS_OFFSET_MODE			2
#define MEASURE_PARAM_MODE				3
#define MEASURE_INERTIA_MODE			4

#define ABSOLUTE_POS_MODE			    1    //绝对位置模式，一上电就是主控的零位置，会累加
#define RELATIVE_POS_MODE			    2    //相对位置模式，电机相对于补偿值(规定一个编码值为主控的零)的脉冲值，会累加

/* USER CODE END PD */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

void DriverInit(void);
void CurrentLoopInit(void);
void SpeedLoopInit(void);
void PositionLoopInit(void);
void TorqueCtrlInit(void);
void RefAngleInit(void);
void CurrentLoop(float exptCurrD, float exptCurrQ, float realCurrD, float realCurrQ, float *ctrlVolD, float *ctrlVolQ);
void SpeedLoop(float expectedMecAngularSpeed, float realMecAngularSpeed, float *ctrlCurrQ);
void PositionLoop(float exptMecAngle, float realMecAngle, float *ctrlAngularSpeed);
void SpdCurrController(void);
void PosSpdCurrController(void);
void PosCurrController(void);
void TorqueController(void);
float VelSlopeGenerator(float exptVelocity);
float PosSlopeGenerator(float exptpos, float realpos);
void DriverCtrlModeInit(void);
/* USER CODE END PFP */


#endif

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/
