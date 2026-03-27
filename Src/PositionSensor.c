/**
 ******************************************************************************
 * @file		PositionSensor.c
 * @author		WrathWings
 * @version 	V1.0
 * @date		2019.3.2
 * @brief		Set and read position sensor
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
 
 /* Private includes ----------------------------------------------------------*/
/* CODE BEGIN Includes */
#include "PositionSensor.h"
/* CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* CODE BEGIN PTD */
extern struct MainCtrl_t MainCtrl;
extern struct PosLoop_t PosLoop;
/* CODE END PTD */

/* Private macro -------------------------------------------------------------*/
/* CODE BEGIN PM */

/* CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* CODE BEGIN PV */
uint16_t ADC3Value[GENERATRIX_VOL_FILTER];
struct PosSensor_t PosSensor;
extern struct CAN_t CAN;

/* CODE END PV */

/* USER CODE BEGIN */
float lastESpeed;
	void GetEleImformation(void)
	{		
		GetMecAngle_AbsoluteMode_15bit();  //绝对式, 读取编码器角度值寄存器
		GetEleAngle(); //计算电角度
		GetEleAngularSpeed();  //计算电角速度
//		AngleEstimator_SetEncoder(PosSensor.EleAngle_rad, PosSensor.EleAngularSpeed_rad);
		EncodeErrorDetection();		//编码器异常检测
	}

	void GetMecImformation(void)
	{
		GetMecAngle(); //计算机械角度
		GetMecAngularSpeed(); //计算机械角速度
		GetRefMecAngle(); //计算参考机械角度（主控用）
	}
	
	void GetMecAngle_AbsoluteMode_15bit(void)
	{
		PosSensor.MecAngle_15bit = (TLE5012_ReadRegister(TLE5012_COMMAND_READ_CURRENT_VALUE_ANGLE, &PosSensor.SafetyWord)) & 0x7FFF;
	}
	
	void GetMecAngle(void)
	{		
		PosSensor.MecAngle_degree = 360.f * (float)PosSensor.MecAngle_15bit / TLE5012_ABS_MODE_RESOLUTION;
		
		PosSensor.MecAngle_rad = (float)PosSensor.MecAngle_degree / 360.f * 2.0f * PI;
	}
	
	void GetRefMecAngle(void)
	{
		
		int delta = 0;
		
		MainCtrl.PresentMecAngle_pulse = PosSensor.MecAngle_15bit - MainCtrl.PosOffset; //(PosSensor.MecAngle_15bit + TLE5012_ABS_MODE_RESOLUTION) % TLE5012_ABS_MODE_RESOLUTION类比队列求长
	
		if(MainCtrl.PresentMecAngle_pulse > (TLE5012_ABS_MODE_RESOLUTION / 2))
		{
			MainCtrl.PresentMecAngle_pulse -= TLE5012_ABS_MODE_RESOLUTION;
		}		
		if(MainCtrl.PresentMecAngle_pulse <= -(TLE5012_ABS_MODE_RESOLUTION / 2))
		{
			MainCtrl.PresentMecAngle_pulse += TLE5012_ABS_MODE_RESOLUTION;
		}
		
		delta = MainCtrl.PresentMecAngle_pulse - MainCtrl.LastMecAngle_pulse;

		while(delta <= -(TLE5012_ABS_MODE_RESOLUTION / 2) || delta > (TLE5012_ABS_MODE_RESOLUTION / 2))
		{				
			if(delta <= -(TLE5012_ABS_MODE_RESOLUTION / 2))
			{
				delta += TLE5012_ABS_MODE_RESOLUTION;
			}		
			else if(delta > (TLE5012_ABS_MODE_RESOLUTION / 2))
			{
				delta -= TLE5012_ABS_MODE_RESOLUTION;
			}
		}
		MainCtrl.LastMecAngle_pulse = MainCtrl.PresentMecAngle_pulse;
		MainCtrl.RefMecAngle_pulse += delta;
		
		/*ֻ限幅, 防止数据溢出*/
		if(MainCtrl.RefMecAngle_pulse > 4000 * 32768)
		{
			MainCtrl.RefMecAngle_pulse = 0;
		}
		else if(MainCtrl.RefMecAngle_pulse < -4000 * 32768)
		{
			MainCtrl.RefMecAngle_pulse = 0;
		}
	}
	
	void GetMecAngularSpeed(void)
	{
		const uint16_t FilterOrder = 10;
		
		float angleDifference = 0;
		float presentMecAngle = 0;
		static float lastMecAngle = 0;		
		static float array[FilterOrder] = {0};
		static float sum = 0.f;
		static float avg = 0.f;
		static float old = 0.f;
		static uint16_t pos = 0;
		
		presentMecAngle = PosSensor.MecAngle_rad;
		
		angleDifference = presentMecAngle - lastMecAngle;
		
		lastMecAngle = presentMecAngle;
		
		while(angleDifference > PI || angleDifference < -PI)
		{
			if(angleDifference > PI)
			{
				angleDifference -= 2.f * PI;
			}
			
			else if(angleDifference < -PI)
			{
				angleDifference += 2.f * PI;
			}
		}
		
		old = array[pos];
		
		array[pos] = angleDifference / (DEFAULT_CARRIER_PERIOD_S * PERIOD_MULTIPLE);
			
		sum = (sum - old) + array[pos];
			
		avg = sum / FilterOrder;

		pos = (pos+1) % FilterOrder;
		
		PosSensor.MecAngularSpeed_rad = avg;
	}

	void GetEleAngle(void)
	{
		float normPos = 0;
		
		#if ENCODER_MODE == ENCODER_ABSOLUTE_MODE
				
		PosSensor.EleAngle_degree = ((float)(PosSensor.MecAngle_15bit - PosSensor.PosOffset) / (float)(TLE5012_ABS_MODE_RESOLUTION / MOTOR_POLE_PAIRS_NUM)) * 360.f;
		
		PosSensor.EleAngle_rad = DEGREE_TO_RAD(PosSensor.EleAngle_degree);
		
		#elif ENCODER_MODE == ENCODER_INCREMENTAL_MODE
		
		#else
		#error "Encoder Mode Invalid"
		#endif
	}

	void GetEleAngularSpeed(void)
	{
		const uint16_t FilterOrder = 10;
		
		float angleDifference = 0;
		float presentEleAngle = 0;
		static float lastEleAngle = 0;
		static float array[FilterOrder] = {0};
		static float sum = 0.f;
		static float avg = 0.f;
		static float old = 0.f;
		static uint16_t pos = 0;
		
		presentEleAngle = PosSensor.EleAngle_rad;
		
		angleDifference = presentEleAngle - lastEleAngle;

		lastEleAngle = presentEleAngle;
		
		lastESpeed =  angleDifference;
		while(angleDifference > PI || angleDifference < -PI)
		{
			if(angleDifference > PI)
			{
				angleDifference -= 2.f * PI;
			}
			
			else if(angleDifference < -PI)
			{
				angleDifference += 2.f * PI;
			}
		}
		
		old = array[pos];
		
		array[pos] = angleDifference / DEFAULT_CARRIER_PERIOD_S;
			
		sum = (sum - old) + array[pos];
			
		avg = sum / FilterOrder;

		pos = (pos+1) % FilterOrder;
		
		PosSensor.EleAngularSpeed_rad = avg;
		
		PosSensor.EleAngularSpeed_degree = RAD_TO_DEGREE(PosSensor.EleAngularSpeed_rad);
	}

	void TLE5012_ReadFSYNC(void)
	{
		PosSensor.FSYNC = (TLE5012_ReadRegister(TLE5012_COMMAND_READ_CURRENT_VALUE_FSYNC, &PosSensor.SafetyWord)) >> 9;
	}
	
	void EncodeErrorDetection(void)
	{
		static uint8_t count = 0;
		
		TLE5012_ReadFSYNC();
		
		PosSensor.SafetyWord >>= 12;
		
		/*编码器错误检测*/
		if(PosSensor.SafetyWord != 0x07)
		{
//			count++;
//			
//			if(count > 10)
//			{
//				PWM_CMD(DISABLE);
//				
//				/*通过CAN总线发送编码器错误信息*/
//				CAN.Identifier = IDENTIFIER_ENCODER_ERROR;
//				CAN.TransmitData = PosSensor.SafetyWord;

//				while(1)
//				{
//					CAN_Transmit(CAN.Identifier, CAN.TransmitData, 2);
//				
//					UART_Transmit_DMA("ENCODER ERROR: %d\r\n", (uint8_t)PosSensor.SafetyWord);
//					SendBuf();
//					
//					LL_mDelay(10);
//				}
//			}
		}
		else
		{
			if(count > 0)	
			{
				count--;
			}
		}
	}
	
	uint16_t TLE5012_ReadRegister(uint16_t command, uint16_t *safetyWord)
	{
		uint16_t data = 0;
		
		TLE5012_SPI1_CHIP_SELECT;
		
		SPI_Transmit(SPI1, command, TimeOut);
		SPI_TX_OFF;
		
		SPI_Receive(SPI1, &data, TimeOut);
		SPI_Receive(SPI1, safetyWord, TimeOut);
		
		TLE5012_SPI1_CHIP_DISELECT;
		SPI_TX_ON;

		return data;
	}
	
	void PosSensor_Init(void)
	{
		RefAngleInit();
		for(uint16_t times = 0; times <= 50; times++)
		{
			GetEleImformation();
		
//			GetMecImformation();
		
			GetMecAngle(); //计算机械角度
		  GetMecAngularSpeed(); //计算机械角速度
			LL_mDelay(1);
		}
		HAL_ADC_Start_DMA(&hadc3,(uint32_t*)ADC3Value,GENERATRIX_VOL_FILTER);
		
	}
	
	void Get_Generatrix_Vol(void)
	{
		uint16_t GeneratrixVol = 0;
		for(uint8_t bytes = 0; bytes < GENERATRIX_VOL_FILTER; bytes++)
		{
			 GeneratrixVol += (uint16_t)(ADC3Value[bytes] / (GENERATRIX_VOL_FILTER));
		}
		#if ROBOT_ID == TRY_ROBOT && CAN_ID_NUM == 7 		 //射箭
			PosSensor.RealGeneratrix_Vol = (float)(GeneratrixVol / MC_CTRL_RESOLUTION * 3.3 / VOL_AMP_DIV_RES_2);
		    PosSensor.Generatrix_Vol = 24.f;
		#elif ROBOT_ID == DEFEND_ROBOT && CAN_ID_NUM == 7 		 //射箭
			PosSensor.RealGeneratrix_Vol = (float)(GeneratrixVol / MC_CTRL_RESOLUTION * 3.3 / VOL_AMP_DIV_RES_3);
		    PosSensor.Generatrix_Vol = 24.f;
		#else
		    PosSensor.Generatrix_Vol = 24.f;
//			PosSensor.Generatrix_Vol = (float)(GeneratrixVol / MC_CTRL_RESOLUTION * 3.3 / VOL_AMP_DIV_RES_1);
		#endif
		
	}
	
	void CorrectPosOffset_Encoder(float volD)
	{
		float volAlpha = 0;
		float volBeta = 0;
		uint16_t pos = 0;
		uint16_t lastPos = 0;
		uint16_t cnt = 0;
		uint16_t times = 0;
		static float slopeErr = 0;
		static float posSlope = 0;
		static float posIntercept = 0;
		static uint16_t arrayCnt = 0;
		static float indexArray[25] = {0};                   
		static float eleArray[25] = {0};
		static float posArray[25] = {0};
		static float tempArray[25] = {0};
		const float exptPosSlope = TLE5012_ABS_MODE_RESOLUTION / (360.f * MOTOR_POLE_PAIRS_NUM);
		
		PosSensor.Generatrix_Vol = 24.f;
		LL_mDelay(2);
		PWM_CMD(ENABLE);
		
		PutStr("Correct Begin...\r\n\r\n");SendBuf();

		/*旋转一个电角度周期, 通过编码器读数曲线斜率确认编码器是否正常*/
		for(uint16_t eleAngle = 0; eleAngle <= 360; eleAngle += 15)
		{
			
			InverseParkTransform(volD, 0.f, &volAlpha, &volBeta, (float)eleAngle);
			SpaceVectorModulation(volAlpha, volBeta);
			
			LL_mDelay(5);
			
			GetMecAngle_AbsoluteMode_15bit();
			
			/*存储校准数据*/
			eleArray[arrayCnt] = eleAngle;
			posArray[arrayCnt] = PosSensor.MecAngle_15bit;
			arrayCnt++;
						
			UART_Transmit_DMA("EleAngle	%d\tMecPosition %d\r\n", (int)eleAngle, (int)PosSensor.MecAngle_15bit);SendBuf();

			LL_mDelay(100);
		}
	
		/*检测编码器读数过零点, 将过零点前的读数减去分辨率, 从而将两段直线拼接成一段直线*/
		for(uint8_t i = 0; i < 25; i++)
		{
			if((posArray[i] - posArray[i + 1]) > 20000)
			{
				for(uint8_t j = 0; j <= i; j++)
				{
					posArray[j] -= TLE5012_ABS_MODE_RESOLUTION;
				}
				
				break;
			}
		}
		
		/*利用最小二乘法计算编码器读数曲线斜率判断编码器是否正常*/
		LeastSquare(eleArray, posArray, 25, &posSlope, &posIntercept);
		/*计算斜率误差*/
		slopeErr = fabs(exptPosSlope - posSlope) / exptPosSlope;
		UART_Transmit_DMA("\r\nPosition Line Slope Error:	%d%%\r\n", (int)(slopeErr * 100));SendBuf();
		
		LL_mDelay(100);
		
		/*若斜率误差大于10%则认为编码器异常*/
		if(slopeErr > 0.1)
		{
			PutStr("Encoder Error !!!\r\n");SendBuf();
		}
		else
		{
			PutStr("Encoder In Operating Condition\r\n");SendBuf();
			
			/*令定子坐标系零位与转子坐标系零位重合, 此时编码器读数即为编码器偏移量*/
			InverseParkTransform(volD, 0.f, &volAlpha, &volBeta, 0);
			SpaceVectorModulation(volAlpha, volBeta);
			
			GetMecAngle_AbsoluteMode_15bit();
			
			for(times = 0; times < 200; times++)
			{
				lastPos = pos;
				
				pos = PosSensor.MecAngle_15bit;
				
				if((times > 100) && (abs((int16_t)(pos - lastPos))<5))
				{
					cnt++;
					
					if(cnt > 30) 
					{
						PosSensor.PosOffset = pos;
						
						UART_Transmit_DMA("Position Offset:	%d\r\nCorrect Finished", (int)PosSensor.PosOffset);SendBuf();
						
						break;
					}
				}
				
				LL_mDelay(5);
			}
		}
				
		PWM_CMD(DISABLE);
	}
	

  
/* USER CODE END */

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/
