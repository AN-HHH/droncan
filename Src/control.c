/**
 ******************************************************************************
 * @file		control.c
 * @author		WrathWings
 * @version 	V1.0
 * @date		2019.1.17
 * @brief		Algorithm of control
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
 
 /* Private includes ----------------------------------------------------------*/
/* CODE BEGIN Includes */
#include "control.h"
/* CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* CODE BEGIN PTD */

/* CODE END PTD */

/* Private variables ---------------------------------------------------------*/
/* CODE BEGIN PV */

struct CurrLoop_t CurrLoop;
struct SpdLoop_t SpdLoop;
struct PosLoop_t PosLoop;
struct TorqueCtrl_t TorqueCtrl;
struct MainCtrl_t MainCtrl;
/* CODE END PV */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
extern struct CoordTrans_t CoordTrans;
extern struct PosSensor_t PosSensor;
extern struct Driver_t Driver;
extern struct LoadObserver_t LoadOb;
/* USER CODE END EV */

/* USER CODE BEGIN */

void DriverInit(void)                 
{
	#if ROBOT_ID == R1
		#if CAN_ID_NUM == 1           
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
//	Driver.ControlMode = TORQUE_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 7233;                    
			CurrLoop.LimitCurrQ = 150.f;
			MainCtrl.RefMecAngle_Mode = ABSOLUTE_POS_MODE;
			MainCtrl.PosOffset = 0;
			CurrLoop.Kp_Q = 2500.f * INDUCTANCE_Q;
			CurrLoop.Ki_Q = 1000.f * PHASE_RES;
			CurrLoop.Kp_D = 2500.f * INDUCTANCE_D;
			CurrLoop.Ki_D = 1000.f * PHASE_RES;
			SpdLoop.ExptMecAngularSpeed_rad = 300.f * 2 * PI;	
			SpdLoop.Kp = SPEED_CONTROL_KP * 0.7f;
			SpdLoop.Ki = SPEED_CONTROL_KI * 0.5f;
			SpdLoop.Acceleration = 300.f;
			SpdLoop.Deceleration = 300.f;
			
			TorqueCtrl.ExptTorque_Nm = 0.01f;
			#elif CAN_ID_NUM == 2            
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 24509;                    
			CurrLoop.LimitCurrQ = 150.f;
			MainCtrl.RefMecAngle_Mode = ABSOLUTE_POS_MODE;
			MainCtrl.PosOffset = 0;
			CurrLoop.Kp_Q = 9000.f * INDUCTANCE_Q;
			CurrLoop.Ki_Q = 2000.f * PHASE_RES;
			CurrLoop.Kp_D = 2500.f * INDUCTANCE_D;
			CurrLoop.Ki_D = 1500.f * PHASE_RES;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;	
			SpdLoop.Kp = SPEED_CONTROL_KP * 0.7f;
			SpdLoop.Ki = SPEED_CONTROL_KI * 0.5f;
			SpdLoop.Acceleration = 300.f;
			SpdLoop.Deceleration = 300.f;
			
			#elif CAN_ID_NUM == 3            
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 15367;                  
			CurrLoop.LimitCurrQ = 150.f;
			MainCtrl.RefMecAngle_Mode = ABSOLUTE_POS_MODE;
			MainCtrl.PosOffset = 0;
			CurrLoop.Kp_Q = 9000.f * INDUCTANCE_Q;
			CurrLoop.Ki_Q = 2000.f * PHASE_RES;
			CurrLoop.Kp_D = 2500.f * INDUCTANCE_D;
			CurrLoop.Ki_D = 1500.f * PHASE_RES;	
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;	
			SpdLoop.Kp = SPEED_CONTROL_KP * 0.7f;
			SpdLoop.Ki = SPEED_CONTROL_KI * 0.4f;
			SpdLoop.Acceleration = 300.f;
			SpdLoop.Deceleration = 300.f;
			
		#elif CAN_ID_NUM == 4      
			Driver.ControlMode = POS_SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 14168; 
			CurrLoop.LimitCurrQ = 20.f;
			
			MainCtrl.RefMecAngle_Mode = RELATIVE_POS_MODE;
			MainCtrl.PosOffset = 27546;

			CurrLoop.Kp_Q = 16000.f * INDUCTANCE_Q;
			CurrLoop.Ki_Q = 7000.f * PHASE_RES;
			CurrLoop.Kp_D = 1000.f * INDUCTANCE_D;
			CurrLoop.Ki_D = 7000.f * PHASE_RES;
			SpdLoop.Kp = SPEED_CONTROL_KP * 2.f;
			SpdLoop.Ki = SPEED_CONTROL_KI * 10.f;
			SpdLoop.MaxExptMecAngularSpeed_rad = 2.f * 2.f * PI;
			PosLoop.Kp = POSITION_CONTROL_KP * 0.3f;
			PosLoop.Kd = POSITION_CONTROL_KD * 1.7f;
			SpdLoop.Acceleration = 200.f;
			SpdLoop.Deceleration = 200.f;			
			PosLoop.Acceleration = 50.f;
			PosLoop.Deceleration = 50.f;
			MainCtrl.ExptMecAngle_pulse = 32768 * 0;	
			
			#elif CAN_ID_NUM == 5            
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 4337;                    
			CurrLoop.LimitCurrQ = 150.f;
			MainCtrl.RefMecAngle_Mode = ABSOLUTE_POS_MODE;
			MainCtrl.PosOffset = 0;
			CurrLoop.Kp_Q = 2000.f * INDUCTANCE_Q;
			CurrLoop.Ki_Q = 1500.f * PHASE_RES;
			CurrLoop.Kp_D = 2000.f * INDUCTANCE_D;
			CurrLoop.Ki_D = 1500.f * PHASE_RES;
			SpdLoop.ExptMecAngularSpeed_rad = 5.f * 2 * PI;	
			SpdLoop.Kp = SPEED_CONTROL_KP * 0.5f;
			SpdLoop.Ki = SPEED_CONTROL_KI * 0.6f;

	    #endif

	#elif ROBOT_ID == R2
		#if CAN_ID_NUM == 2          //夹块
			Driver.ControlMode = POS_SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 17882;
			CurrLoop.LimitCurrQ = 80.f;
			MainCtrl.RefMecAngle_Mode = ABSOLUTE_POS_MODE;
			MainCtrl.PosOffset = 0;
			SpdLoop.MaxExptMecAngularSpeed_rad = 20.f * 2 * PI;	
			SpdLoop.Kp = SPEED_CONTROL_KP * 0.5f;
			SpdLoop.Ki = SPEED_CONTROL_KI * 1.5f;	
			MainCtrl.ExptMecAngle_pulse = (int32_t)(0);
			PosLoop.Kp = POSITION_CONTROL_KP * 2.f;
			PosLoop.Kd = POSITION_CONTROL_KD * 22.f;
			PosLoop.Acceleration = 100.f;
			PosLoop.Deceleration = 300.f;	
			
		#elif CAN_ID_NUM == 3          //左翻转
      Driver.ControlMode = POS_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 12251;
			CurrLoop.LimitCurrQ = 20.f;
			MainCtrl.RefMecAngle_Mode = RELATIVE_POS_MODE;
			CurrLoop.Kp_Q = 2500.f*INDUCTANCE_Q;
			CurrLoop.Ki_Q = 1000.f*PHASE_RES;
			CurrLoop.Kp_D = 2500.f*INDUCTANCE_D;
			CurrLoop.Ki_D = 1000.f*PHASE_RES;			
			MainCtrl.PosOffset = 3850;
			MainCtrl.ExptMecAngle_pulse = (int32_t)(0);
			PosLoop.Kp = POSITION_CONTROL_KP * 0.8f;
			PosLoop.Kd = POSITION_CONTROL_KD * 10.5f;
			PosLoop.Acceleration = 5;
			PosLoop.Deceleration = 5;	
			
		#elif CAN_ID_NUM == 4          //右翻转 滤波阶数5
			Driver.ControlMode = POS_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 1586;                    
			CurrLoop.LimitCurrQ = 20.f;
			MainCtrl.RefMecAngle_Mode = RELATIVE_POS_MODE;
			CurrLoop.Kp_Q = 1000.f*INDUCTANCE_Q;
			CurrLoop.Ki_Q = 600.f*PHASE_RES;
			CurrLoop.Kp_D = 1000.f*INDUCTANCE_D;
			CurrLoop.Ki_D = 600.f*PHASE_RES;
			MainCtrl.PosOffset = 11500;
			MainCtrl.ExptMecAngle_pulse = (int32_t)(0);
			PosLoop.Kp = POSITION_CONTROL_KP * 0.53f;
			PosLoop.Kd = POSITION_CONTROL_KD * 10.f;
			PosLoop.Acceleration = 5;
			PosLoop.Deceleration = 5;
	    #endif
			
	#elif ROBOT_ID == Test_Robot
			#if CAN_ID_NUM == 1           
			Driver.ControlMode = SPD_CURR_CTRL_MODE;
			DriverCtrlModeInit();
			PosSensor.PosOffset = 6355;                    
			CurrLoop.LimitCurrQ = 150.f;
			MainCtrl.RefMecAngle_Mode = ABSOLUTE_POS_MODE;
			MainCtrl.PosOffset = 0;
			CurrLoop.Kp_Q = 1200.f * INDUCTANCE_Q;
			CurrLoop.Ki_Q = 600.f * PHASE_RES;
			CurrLoop.Kp_D = 1200.f * INDUCTANCE_D;
			CurrLoop.Ki_D = 600.f * PHASE_RES;
			SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;	
			SpdLoop.Kp = SPEED_CONTROL_KP * 0.5f;
			SpdLoop.Ki = SPEED_CONTROL_KI * 12.f;
			SpdLoop.Acceleration = 250.f;
			SpdLoop.Deceleration = 250.f;
	    #endif
	#endif
	/*读取编码器, 计算速度, 防止上电电机跳动*/
	PutStr("INITOVER\r\n");
	PosSensor_Init();
	MainCtrl.CAN_FaultSign = 0;
	PWM_CMD(ENABLE);
}

 /**
   * @brief  
   */
void CurrentLoopInit(void)
{

	CurrLoop.Kp_D = CURRENT_CONTROL_KP_D ;
	CurrLoop.Ki_D = CURRENT_CONTROL_KI_D ;
	CurrLoop.Kp_Q = CURRENT_CONTROL_KP_Q ;	
	CurrLoop.Ki_Q = CURRENT_CONTROL_KI_Q ;
	
	PosSensor.CompRatio_forward = 3.0f;
	PosSensor.CompRatio_reverse = 3.0f;
	PosSensor.Generatrix_Vol = 24.f;
	
	CoordTrans.CurrA_Offset = 0.f;
	CoordTrans.CurrB_Offset = 0.f;
	CoordTrans.CurrC_Offset = 0.f;
	CoordTrans.ADCSelfCalibration_Status = 0;
	
	CurrLoop.LimitCurrQ = 20.f;
}

 /**
   * @brief  
   */
void SpeedLoopInit(void)
{
	if(Driver.ControlMode == SPD_CURR_CTRL_MODE)
	{
		SpdLoop.Kp = SPEED_CONTROL_KP;	
		SpdLoop.Ki = SPEED_CONTROL_KI;
		SpdLoop.ExptMecAngularSpeed_rad = 0.f * 2 * PI;	
		SpdLoop.MaxExptMecAngularSpeed_rad = MAX_SPD(24.f);	
		SpdLoop.Acceleration = 3000.f * 2 * PI;	
		SpdLoop.Deceleration = SpdLoop.Acceleration;
	}
	else if(Driver.ControlMode == POS_SPD_CURR_CTRL_MODE)
	{
		SpdLoop.Kp = SPEED_CONTROL_KP * 5.0f;
		SpdLoop.Ki = SPEED_CONTROL_KI * 5.0f;	
		SpdLoop.MaxExptMecAngularSpeed_rad = 100.f * 2 * PI;	
		SpdLoop.Acceleration = 3000.f * 2 * PI;	
		SpdLoop.Deceleration = SpdLoop.Acceleration;
	}
	if(Driver.ControlMode == TORQUE_CTRL_MODE)
	{
		SpdLoop.Kp = SPEED_CONTROL_KP;	
		SpdLoop.Ki = SPEED_CONTROL_KI;
		SpdLoop.MaxExptMecAngularSpeed_rad = TorqueCtrl.MaxMecSpd_rad;	
	}
}

 /**
   * @brief  
   */
void PositionLoopInit(void)
{					
	if(Driver.ControlMode == POS_SPD_CURR_CTRL_MODE)
	{
		PosLoop.Kp = POSITION_CONTROL_KP;
		PosLoop.Kd = POSITION_CONTROL_KD;		
		MainCtrl.ExptMecAngle_pulse = 0;
		PosLoop.MecAngleUpperLimit_rad = 1024 * 2.f * PI;
		PosLoop.MecAngleLowerLimit_rad = -1024 * 2.f * PI;
		PosLoop.Acceleration = 4000.f;
		PosLoop.Deceleration = 4000.f;
	}
	else if(Driver.ControlMode == POS_CURR_CTRL_MODE)
	{
		PosLoop.Kp = 5.0f;
		PosLoop.Kd = 0.001f;
		MainCtrl.ExptMecAngle_pulse = 0;
		PosLoop.MecAngleUpperLimit_rad = 1024 * 2.f * PI;
		PosLoop.MecAngleLowerLimit_rad = -1024 * 2.f * PI;
		PosLoop.Acceleration = 4000.f;
		PosLoop.Deceleration = 4000.f;
	}
}

 /**
   * @brief  
   */
void TorqueCtrlInit(void)
{					
	TorqueCtrl.ExptTorque_Nm = 1.0f;	
	TorqueCtrl.MaxMecSpd_rad = 200.f * 2.f * PI;	
	CurrLoop.LimitCurrQ = 200.f;		
}

 /**
   * @brief  
   */
void RefAngleInit(void)
{
	if(MainCtrl.RefMecAngle_Mode == ABSOLUTE_POS_MODE)
	{
		MainCtrl.LastMecAngle_pulse = PosSensor.MecAngle_15bit - MainCtrl.PosOffset;
	}
	if(MainCtrl.RefMecAngle_Mode == RELATIVE_POS_MODE)
	{
		MainCtrl.LastMecAngle_pulse = 0;
	}
	MainCtrl.PresentMecAngle_pulse = 0;
	MainCtrl.RefMecAngle_pulse = 0;
}

 /**
   * @brief  
   * @param[in]  exptCurrD     		
   * @param[in]  exptCurrQ      	
   * @param[in]  realCurrD     		
   * @param[in]  realCurrQ      	
   * @param[out] ctrlVolD 			
   * @param[out] ctrlVolQ 			
   */
void CurrentLoop(float exptCurrD, float exptCurrQ, float realCurrD, float realCurrQ, float *ctrlVolD, float *ctrlVolQ)
{
	/*Iq限幅*/
	Saturation_float(&exptCurrQ, CurrLoop.LimitCurrQ, -CurrLoop.LimitCurrQ);
	
	CurrLoop.ErrD = exptCurrD - realCurrD;
	CurrLoop.ErrQ = exptCurrQ - realCurrQ;
	
	CurrLoop.IntegralErrD += CurrLoop.ErrD * DEFAULT_CARRIER_PERIOD_S;
	CurrLoop.IntegralErrQ += CurrLoop.ErrQ * DEFAULT_CARRIER_PERIOD_S;
	
	Saturation_float(&CurrLoop.IntegralErrD, CURR_INTEGRAL_ERR_LIM_D, -CURR_INTEGRAL_ERR_LIM_D);
	Saturation_float(&CurrLoop.IntegralErrQ, CURR_INTEGRAL_ERR_LIM_Q, -CURR_INTEGRAL_ERR_LIM_Q);
	
	*ctrlVolD = CurrLoop.Kp_D * CurrLoop.ErrD + CurrLoop.Ki_D * CurrLoop.IntegralErrD;
//	*ctrlVolQ = CurrLoop.Kp_Q * CurrLoop.ErrQ + realCurrQ * PHASE_RES + PosSensor.EleAngularSpeed_rad * ROTATOR_FLUX_LINKAGE;
	*ctrlVolQ = CurrLoop.Kp_Q * CurrLoop.ErrQ + CurrLoop.Ki_Q * CurrLoop.IntegralErrQ;
	
	if(SQUARE(*ctrlVolD )+SQUARE(*ctrlVolQ )>SQUARE(PosSensor.Generatrix_Vol )/ 3)
	{
//		float sum = *ctrlVolD + *ctrlVolQ;
//		*ctrlVolD = *ctrlVolD / sum;
//		*ctrlVolQ = *ctrlVolQ / sum;
		
	}
//	
//	CurrLoop.LimitVolD = PosSensor.Generatrix_Vol / SQRT3;
//	arm_sqrt_f32(SQUARE(PosSensor.Generatrix_Vol) / 3.f - SQUARE(*ctrlVolD), &CurrLoop.LimitVolQ);
//	
//	Saturation_float(ctrlVolD, CurrLoop.LimitVolD, -CurrLoop.LimitVolD);
//	Saturation_float(ctrlVolQ, CurrLoop.LimitVolQ, -CurrLoop.LimitVolQ);
}

 /**
   * @brief  
   * @param[in]  exptMecAngularSpeed     	
   * @param[in]  realMecAngularSpeed      	
   * @param[out] ctrlCurrQ 					
   */
void SpeedLoop(float exptMecAngularSpeed, float realMecAngularSpeed, float *ctrlCurrQ)
{

	Saturation_float(&exptMecAngularSpeed, SpdLoop.MaxExptMecAngularSpeed_rad, -SpdLoop.MaxExptMecAngularSpeed_rad);
	
	SpdLoop.Err = exptMecAngularSpeed - realMecAngularSpeed;
	
	SpdLoop.IntegralErr += SpdLoop.Err * OUTER_LOOP_PERIOD;
	
	Saturation_float(&SpdLoop.IntegralErr, SPD_INTEGRAL_ERR_LIM, -SPD_INTEGRAL_ERR_LIM);
	
	*ctrlCurrQ = SpdLoop.Kp * SpdLoop.Err + SpdLoop.Ki * SpdLoop.IntegralErr;
}

 /**
   * @brief  
   * @param[in]  exptMecAngle     		
   * @param[in]  realMecAngle      		
   * @param[out] ctrlAngularSpeed 		
   */
void PositionLoop(float exptMecAngle, float realMecAngle, float *ctrlAngularSpeed)
{
	static float tempkp = 1.f;
	static float tempkd = 1.f;
	tempkp = PosLoop.Kp;
	tempkd = PosLoop.Kd;
	Saturation_float(&exptMecAngle, PosLoop.MecAngleUpperLimit_rad, PosLoop.MecAngleLowerLimit_rad);
	
	PosLoop.Err = exptMecAngle - realMecAngle;

	PosLoop.DiffErr = (PosLoop.Err - PosLoop.LastErr) / OUTER_LOOP_PERIOD;
	
	if(	-0.001 <= (PosLoop.Err - PosLoop.LastErr) && (PosLoop.Err - PosLoop.LastErr) <= 0.2)
	{
		tempkp = PosLoop.Kp * 1;
		tempkd = PosLoop.Kd * 1;       //kp=2;kd=0.5爪子
	}
	
	*ctrlAngularSpeed = tempkp * PosLoop.Err + tempkd * PosLoop.DiffErr;
		
	PosLoop.LastErr = PosLoop.Err;
}

 /**
   * @brief  
   */
void SpdCurrController(void)
{	
	static uint16_t Count = PERIOD_MULTIPLE - 1;	
	
	Count++;
	
	CurrLoop.ExptCurrD = 0.f;
	
	if(Count == PERIOD_MULTIPLE)
	{
		GetMecImformation();
		SpeedLoop(VelSlopeGenerator(SpdLoop.ExptMecAngularSpeed_rad), PosSensor.MecAngularSpeed_rad, &CurrLoop.ExptCurrQ);
		
		Count = 0;
	}
	
	ClarkeTransform(CoordTrans.CurrA, CoordTrans.CurrB, &CoordTrans.CurrAlpha, &CoordTrans.CurrBeta);

	if(PosSensor.EleAngularSpeed_degree >= 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_forward;
	}
	else if(PosSensor.EleAngularSpeed_degree < 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_reverse;
	}
	
	PosSensor.CompAngle = PosSensor.CompRatio * PosSensor.EleAngularSpeed_degree * DEFAULT_CARRIER_PERIOD_S;
	
	ParkTransform(CoordTrans.CurrAlpha, CoordTrans.CurrBeta, &CoordTrans.CurrD, &CoordTrans.CurrQ, PosSensor.EleAngle_degree + PosSensor.CompAngle);
	
	CurrentLoop(CurrLoop.ExptCurrD,CurrLoop.ExptCurrQ, CoordTrans.CurrD, CoordTrans.CurrQ, &CurrLoop.CtrlVolD, &CurrLoop.CtrlVolQ);
	
	InverseParkTransform(CurrLoop.CtrlVolD, CurrLoop.CtrlVolQ, &CoordTrans.VolAlpha, &CoordTrans.VolBeta, PosSensor.EleAngle_degree + PosSensor.CompAngle);
//	InverseParkTransform(0, 2, &CoordTrans.VolAlpha, &CoordTrans.VolBeta, PosSensor.EleAngle_degree + PosSensor.CompAngle);
	SpaceVectorModulation(CoordTrans.VolAlpha, CoordTrans.VolBeta);
	
	/* ✅ 每40us更新一次龙伯格+PLL 这个位置试验一下放前面还是放后面好*/
//	AngleEstimator_Step(CoordTrans.last_VolAlpha, CoordTrans.last_VolBeta, CoordTrans.CurrAlpha, CoordTrans.CurrBeta);
//	CoordTrans.last_VolAlpha = CoordTrans.true_VolAlpha;
//	CoordTrans.last_VolBeta = CoordTrans.true_VolBeta;
}

 /**
   * @brief  
   */
void PosSpdCurrController(void)
{	
	static uint16_t Count = PERIOD_MULTIPLE - 1;	
	
	Count++;
	
	CurrLoop.ExptCurrD = 0.f;
	
	if(Count == PERIOD_MULTIPLE)
	{
		GetMecImformation();
		
		PosLoop.ExptMecAngle_rad = DRV_PULSE_TO_RAD(MainCtrl.ExptMecAngle_pulse);
		
		PosLoop.RefMecAngle_rad = DRV_PULSE_TO_RAD(MainCtrl.RefMecAngle_pulse);
		
		PositionLoop(PosLoop.ExptMecAngle_rad, PosLoop.RefMecAngle_rad, &SpdLoop.ExptMecAngularSpeed_rad);
				
		SpeedLoop(VelSlopeGenerator(SpdLoop.ExptMecAngularSpeed_rad), PosSensor.MecAngularSpeed_rad, &CurrLoop.ExptCurrQ);
		
		Count = 0;
	}
	
	ClarkeTransform(CoordTrans.CurrA, CoordTrans.CurrB, &CoordTrans.CurrAlpha, &CoordTrans.CurrBeta);

	if(PosSensor.EleAngularSpeed_degree >= 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_forward;
	}
	else if(PosSensor.EleAngularSpeed_degree < 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_reverse;
	}
	
	PosSensor.CompAngle = PosSensor.CompRatio * PosSensor.EleAngularSpeed_degree * DEFAULT_CARRIER_PERIOD_S;
	
	ParkTransform(CoordTrans.CurrAlpha, CoordTrans.CurrBeta, &CoordTrans.CurrD, &CoordTrans.CurrQ, PosSensor.EleAngle_degree + PosSensor.CompAngle);
	
	CurrentLoop(CurrLoop.ExptCurrD,CurrLoop.ExptCurrQ, CoordTrans.CurrD, CoordTrans.CurrQ, &CurrLoop.CtrlVolD, &CurrLoop.CtrlVolQ);
	
	InverseParkTransform(CurrLoop.CtrlVolD, CurrLoop.CtrlVolQ, &CoordTrans.VolAlpha, &CoordTrans.VolBeta, PosSensor.EleAngle_degree + PosSensor.CompAngle);

//	InverseParkTransform(0, 1, &CoordTrans.VolAlpha, &CoordTrans.VolBeta, PosSensor.EleAngle_degree + PosSensor.CompAngle);
		
	SpaceVectorModulation(CoordTrans.VolAlpha, CoordTrans.VolBeta);
}

 /**
   * @brief  
   */
void PosCurrController(void)
{	
	static uint16_t Count = PERIOD_MULTIPLE - 1;	
	
	Count++;
	
	CurrLoop.ExptCurrD = 0.f;
	static float expcurrq = 0.f;
	if(Count == PERIOD_MULTIPLE)
	{
		GetMecImformation();
		
		PosLoop.ExptMecAngle_rad = DRV_PULSE_TO_RAD(MainCtrl.ExptMecAngle_pulse);
		
		PosLoop.RefMecAngle_rad = DRV_PULSE_TO_RAD(MainCtrl.RefMecAngle_pulse);
		
		PosLoop.real_ExptAngle_rad = PosSlopeGenerator(PosLoop.ExptMecAngle_rad, PosLoop.RefMecAngle_rad);
		
		PositionLoop(PosLoop.real_ExptAngle_rad, PosLoop.RefMecAngle_rad, &CurrLoop.ExptCurrQ);
		
		Count = 0;
	}
	
	ClarkeTransform(CoordTrans.CurrA, CoordTrans.CurrB, &CoordTrans.CurrAlpha, &CoordTrans.CurrBeta);

	if(PosSensor.EleAngularSpeed_degree >= 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_forward;
	}
	else if(PosSensor.EleAngularSpeed_degree < 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_reverse;
	}
	
	PosSensor.CompAngle = PosSensor.CompRatio * PosSensor.EleAngularSpeed_degree * DEFAULT_CARRIER_PERIOD_S;
	
	ParkTransform(CoordTrans.CurrAlpha, CoordTrans.CurrBeta, &CoordTrans.CurrD, &CoordTrans.CurrQ, PosSensor.EleAngle_degree + PosSensor.CompAngle);
	
	CurrentLoop(0.f, CurrLoop.ExptCurrQ, CoordTrans.CurrD, CoordTrans.CurrQ, &CurrLoop.CtrlVolD, &CurrLoop.CtrlVolQ);
	
	InverseParkTransform(CurrLoop.CtrlVolD, CurrLoop.CtrlVolQ, &CoordTrans.VolAlpha, &CoordTrans.VolBeta, PosSensor.EleAngle_degree + PosSensor.CompAngle);
	
	SpaceVectorModulation(CoordTrans.VolAlpha, CoordTrans.VolBeta);
}

 /**
   * @brief  
   */
void TorqueController(void)
{  
	static uint16_t Count = PERIOD_MULTIPLE - 1;
	
	Count++;
	
	CurrLoop.ExptCurrD = 0.f;
		
	if(Count == PERIOD_MULTIPLE)
	{
		GetMecImformation();
					
		if(PosSensor.MecAngularSpeed_rad < (TorqueCtrl.MaxMecSpd_rad - 3.f * 2.f * PI))
		{		
			CurrLoop.ExptCurrQ = TorqueCtrl.ExptTorque_Nm / (1.5f * MOTOR_POLE_PAIRS_NUM * ROTATOR_FLUX_LINKAGE);
		}
		else if(PosSensor.MecAngularSpeed_rad >= (TorqueCtrl.MaxMecSpd_rad - 3.f * 2.f * PI))
		{
			SpeedLoop(TorqueCtrl.MaxMecSpd_rad, PosSensor.MecAngularSpeed_rad, &CurrLoop.ExptCurrQ);
			
			Saturation_float(&CurrLoop.ExptCurrQ, TorqueCtrl.ExptTorque_Nm / (1.5f * MOTOR_POLE_PAIRS_NUM * ROTATOR_FLUX_LINKAGE), -TorqueCtrl.ExptTorque_Nm / (1.5f * MOTOR_POLE_PAIRS_NUM * ROTATOR_FLUX_LINKAGE));
		}
			
		Count = 0;
	}

	ClarkeTransform(CoordTrans.CurrA, CoordTrans.CurrB, &CoordTrans.CurrAlpha, &CoordTrans.CurrBeta);

	if(PosSensor.EleAngularSpeed_degree >= 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_forward;
	}
	else if(PosSensor.EleAngularSpeed_degree < 0)
	{
		PosSensor.CompRatio = PosSensor.CompRatio_reverse;
	}
	
	PosSensor.CompAngle = PosSensor.CompRatio * PosSensor.EleAngularSpeed_degree * DEFAULT_CARRIER_PERIOD_S;
	
	ParkTransform(CoordTrans.CurrAlpha, CoordTrans.CurrBeta, &CoordTrans.CurrD, &CoordTrans.CurrQ, PosSensor.EleAngle_degree + PosSensor.CompAngle);
	
	CurrentLoop(CurrLoop.ExptCurrD, CurrLoop.ExptCurrQ, CoordTrans.CurrD, CoordTrans.CurrQ, &CurrLoop.CtrlVolD, &CurrLoop.CtrlVolQ);
	
	InverseParkTransform(CurrLoop.CtrlVolD, CurrLoop.CtrlVolQ, &CoordTrans.VolAlpha, &CoordTrans.VolBeta, PosSensor.EleAngle_degree + PosSensor.CompAngle);
	
	SpaceVectorModulation(CoordTrans.VolAlpha, CoordTrans.VolBeta);
}

 /**加速度的作用，给一个速度的阶跃
   * @brief  
   * @param[in]  expectedVelocity      
   */
float VelSlopeGenerator(float exptVel)
{
	static float velProcessVolume = 0.0f;
	static float velStep_Acc = 0;
	static float velStep_Dec = 0;
	
	velStep_Acc = SpdLoop.Acceleration * OUTER_LOOP_PERIOD;
	velStep_Dec = SpdLoop.Deceleration * OUTER_LOOP_PERIOD;
	
	if(exptVel > 0 && velProcessVolume >= 0)
	{
		if(velProcessVolume < (exptVel - velStep_Acc))
		{
			velProcessVolume += velStep_Acc;
		}
		else if(velProcessVolume > (exptVel + velStep_Dec))
		{
			velProcessVolume -= velStep_Dec;
		}
		else
		{
			velProcessVolume = exptVel;
		}
	}
	else if(exptVel > 0 && velProcessVolume < 0)
	{
		velProcessVolume += velStep_Dec;
	}
	else if(exptVel < 0 && velProcessVolume >=  0)
	{
		velProcessVolume -= velStep_Dec;
	}
	else if(exptVel < 0 && velProcessVolume < 0)
	{
		if(velProcessVolume > (exptVel + velStep_Acc))
		{
			velProcessVolume -= velStep_Acc;
		}
		else if(velProcessVolume < (exptVel - velStep_Dec))
		{
			velProcessVolume += velStep_Dec;	
		}
		else
		{
			velProcessVolume = exptVel;
		}
	}
	else if(exptVel == 0 && velProcessVolume >= 0)
	{
		if(velProcessVolume > (exptVel + velStep_Dec))
		{
			velProcessVolume -= velStep_Dec;
		}
		else
		{
			velProcessVolume = exptVel;
		}
	}
	else if(exptVel == 0 && velProcessVolume < 0)
	{
		if(velProcessVolume < (exptVel - velStep_Dec))
		{
			velProcessVolume += velStep_Dec;
		}
		else
		{
			velProcessVolume = exptVel;
		}
	}
	
	return velProcessVolume;
}

 /**
   * @brief  
   */
void DriverCtrlModeInit(void)
{		
	switch(Driver.ControlMode)
	{
		case SPD_CURR_CTRL_MODE : 	
										CurrentLoopInit();

										SpeedLoopInit();

										break;
		
		case POS_SPD_CURR_CTRL_MODE :	
										CurrentLoopInit();
										
										SpeedLoopInit();
										
										PositionLoopInit();
			
										break;
		case POS_CURR_CTRL_MODE :	
										CurrentLoopInit();
										
										PositionLoopInit();
			
										break;
		case TORQUE_CTRL_MODE :	
										TorqueCtrlInit();	
		
										CurrentLoopInit();
										
										SpeedLoopInit();		
		
										break;
	}
}

/**加速度的作用，给一个位置的阶跃
   * @brief  
   * @param[in]  expectedVelocity      
   */
float PosSlopeGenerator(float exptpos, float realpos)
{
	static float posProcessVolume = 0.0f;
	static float posStep_Acc = 0;
	static float posStep_Dec = 0;
	
	posStep_Acc = PosLoop.Acceleration * OUTER_LOOP_PERIOD;
	posStep_Dec = PosLoop.Deceleration * OUTER_LOOP_PERIOD;
	
	if(exptpos - realpos > 0 && posProcessVolume >= realpos)
	{
		if(posProcessVolume < (exptpos - posStep_Acc))
		{
			posProcessVolume += posStep_Acc;
		}
		else if(posProcessVolume > (exptpos + posStep_Dec))
		{
			posProcessVolume -= posStep_Dec;
		}
		else                             
		{  
			posProcessVolume = exptpos;
		}
	}
	else if(exptpos - realpos > 0 && posProcessVolume < realpos)
	{      
		posProcessVolume += posStep_Dec;
	}
	else if(exptpos - realpos < 0 && posProcessVolume >=  realpos)
	{
		posProcessVolume -= posStep_Dec;
	}
	else if(exptpos - realpos < 0 && posProcessVolume < realpos)
	{
		if(posProcessVolume > (exptpos + posStep_Acc))
		{
			posProcessVolume -= posStep_Acc;
		}
		else if(posProcessVolume < (exptpos - posStep_Dec))
		{
			posProcessVolume += posStep_Dec;	
		}
		else
		{
			posProcessVolume = exptpos;
		}
	}
	else if(exptpos == 0 && posProcessVolume >= 0)
	{
		if(posProcessVolume > (exptpos + posStep_Dec))
		{
			posProcessVolume -= posStep_Dec;
		}
		else
		{
			posProcessVolume = exptpos;
		}
	}
	else if(exptpos == 0 && posProcessVolume < 0)
	{
		if(posProcessVolume < (exptpos - posStep_Dec))
		{
			posProcessVolume += posStep_Dec;
		}
		else
		{
			posProcessVolume = exptpos;
		}
	}
	
	return posProcessVolume;
}


/* USER CODE END */

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/
