/**
 ******************************************************************************
 * @file		observer.c
 * @author		WrathWings
 * @version 	V1.0
 * @date		2019.8.20
 * @brief		State observer
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
 
 /* Private includes ----------------------------------------------------------*/
/* CODE BEGIN Includes */
#include "observer.h"
/* CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* CODE BEGIN PTD */

/* CODE END PTD */

/* Private variables ---------------------------------------------------------*/
/* CODE BEGIN PV */
AngleEstimator_t AngEst;
/* CODE END PV */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
/**
* @brief  负载观测器
* @param  *phase_var：观测到的速度
          dt:时间微分因子，在这里为运行周期      
					*x1：估算转速
					*x2：负载
					phase:位置观测器观测到的转子位置
*  @note  用PLL计算速度
 * @retval none
 */
static float wrap_pi(float x)
{
    while (x >  PI) x -= 2.0f * PI;
    while (x < -PI) x += 2.0f * PI;
    return x;
}

static float wrap_2pi(float x)
{
    while (x >= 2.0f * PI) x -= 2.0f * PI;
    while (x <  0.0f)      x += 2.0f * PI;
    return x;
}

static float clampf(float x, float hi, float lo)
{
    if (x > hi) return hi;
    if (x < lo) return lo;
    return x;
}

void AngleEstimator_Init(void)
{
    AngEst.mode = AE_MODE_ENCODER;

    AngEst.theta_e_enc_rad = 0.0f;
    AngEst.omega_e_enc_rad = 0.0f;

    AngEst.i_hat_alpha = 0.0f;
    AngEst.i_hat_beta  = 0.0f;
    AngEst.e_hat_alpha = 0.0f;
    AngEst.e_hat_beta  = 0.0f;

    AngEst.theta_pll_rad = 0.0f;
    AngEst.omega_pll_rad = 0.0f;
    AngEst.pll_int       = 0.0f;

    AngEst.blend = 0.0f;


    AngEst.theta_e_used_rad = 0.0f;
    AngEst.omega_e_used_rad = 0.0f;
}

void AngleEstimator_SetEncoder(float theta_e_rad, float omega_e_rad)
{
    AngEst.theta_e_enc_rad = wrap_2pi(theta_e_rad);
    AngEst.omega_e_enc_rad = omega_e_rad;
}

/* 核心：每 40us 调一次 */
void AngleEstimator_Step(float v_alpha, float v_beta, float i_alpha, float i_beta)
{
    float Ts = AE_TS;
    float L  = AE_LS;
    float R  = AE_R;


    /* ============ 1) 龙伯格观测器：更新 i_hat, e_hat ============ */

    /* omega_e 用谁？
       - 编码器模式：用编码器 omega_e（更准更稳）
       - 无感/融合：用 PLL omega_e（自洽） */
    float omega_e_for_obs = AngEst.omega_e_enc_rad;
//    if (AngEst.mode != AE_MODE_ENCODER)
//    {
//        omega_e_for_obs = AngEst.omega_pll_rad;
//    }

		
		/* === EMF + PLL 观测器主体 ===
			 e = v - R i - L di/dt
			 theta_from_e = atan2(e_beta,e_alpha) - pi/2
			 PLL: omega += Kp*err + Ki*∫err
		*/

		/*
			龙伯格观测器：
			i_hat_alpha(k+1) = (1 - R*T/L - K1*T)*i_hat_alpha(k) - (T/L)*e_hat_alpha(k) + (T/L)*u_alpha(k) + K1*T*i_alpha(k)
			i_hat_beta (k+1) = (1 - R*T/L - K1*T)*i_hat_beta (k) - (T/L)*e_hat_beta (k) + (T/L)*u_beta (k) + K1*T*i_beta (k)

			e_hat_alpha(k+1) = e_hat_alpha(k) - omega_e*T*e_hat_beta(k) + K2*T*( i_alpha(k) - i_hat_alpha(k) )
			e_hat_beta (k+1) = e_hat_beta (k) + omega_e*T*e_hat_alpha(k) + K2*T*( i_beta (k) - i_hat_beta (k) )
		*/   
		

    {
        float a = (1.0f - (R * Ts / L) - (AE_OBS_K1 * Ts));
        float b = (Ts / L);
        float iha = a * AngEst.i_hat_alpha - b * AngEst.e_hat_alpha + b * v_alpha + (AE_OBS_K1 * Ts) * i_alpha;
        float ihb = a * AngEst.i_hat_beta  - b * AngEst.e_hat_beta  + b * v_beta  + (AE_OBS_K1 * Ts) * i_beta; 

			  float s,c;
			  arm_sin_cos_f32(omega_e_for_obs * Ts, &s, &c);
        float ea = c * AngEst.e_hat_alpha + s * AngEst.e_hat_beta + AE_OBS_K2 * Ts * (i_alpha - AngEst.i_hat_alpha);
        float eb = - s * AngEst.e_hat_alpha + c * AngEst.e_hat_beta + AE_OBS_K2 * Ts * (i_beta - AngEst.i_hat_beta); 
			
//        float ea = AngEst.e_hat_alpha - omega_e_for_obs * Ts * AngEst.e_hat_beta
//                   + AE_OBS_K2 * Ts * (i_alpha - AngEst.i_hat_alpha);
//        float eb = AngEst.e_hat_beta  + omega_e_for_obs * Ts * AngEst.e_hat_alpha
//                   + AE_OBS_K2 * Ts * (i_beta - AngEst.i_hat_beta);
//        float ea = AngEst.e_hat_alpha + AE_OBS_K2 * Ts * (i_alpha - AngEst.i_hat_alpha);
//        float eb = AngEst.e_hat_beta + AE_OBS_K2 * Ts * (i_beta - AngEst.i_hat_beta);
			
			  AngEst.i_hat_alpha = iha;
        AngEst.i_hat_beta  = ihb; 
        AngEst.e_hat_alpha = ea;
        AngEst.e_hat_beta  = eb;
    }

//	测试哦	AngEst.theta_e_used_rad = (atan2f(AngEst.e_hat_beta, AngEst.e_hat_alpha) - 0.5f * PI); 
		
		
//    /* ============ 2) 用 e_hat 算瞬时电角，并用 PLL 跟踪 ============ */

//    {
        float emf2 = AngEst.e_hat_alpha * AngEst.e_hat_alpha + AngEst.e_hat_beta * AngEst.e_hat_beta;

        if (emf2 > (AE_EMF_MIN_V * AE_EMF_MIN_V))
        {
            /* SPMSM：e 向量相对转子磁链超前 90deg
               theta_e = atan2(eβ,eα) - 90deg */
            float theta_emf = atan2f(AngEst.e_hat_beta, AngEst.e_hat_alpha) - 0.5f * PI;
            theta_emf = wrap_2pi(theta_emf);

            float err = wrap_pi(theta_emf - AngEst.theta_pll_rad);

            AngEst.pll_int += err * Ts;
            AngEst.pll_int = clampf(AngEst.pll_int, 200.0f, -200.0f);

            AngEst.omega_pll_rad = AE_PLL_KP * err + AE_PLL_KI * AngEst.pll_int;
            AngEst.omega_pll_rad  = clampf(AngEst.omega_pll_rad, AE_OMEGA_E_LIMIT, -AE_OMEGA_E_LIMIT);

            AngEst.theta_pll_rad = wrap_2pi(AngEst.theta_pll_rad + AngEst.omega_pll_rad * Ts);
        }
        /* else：EMF太小，不更新PLL，防止低速乱跳 */
//    }

		
//    /* ============ 3) 融合/切换逻辑（低速编码器，高速无感） ============ */


//    {
//        float w_enc = fabsf(AngEst.omega_e_enc_rad);
//        float w_pll = fabsf(AngEst.omega_pll_rad);

//        /* 进入无感条件：编码器速度达到阈值，并且PLL也有输出（避免没收敛就切） */
//        if (AngEst.mode == AE_MODE_ENCODER)
//        {
//            if (w_enc > AE_SW_ON_OMEGA_E && w_pll > (0.5f * AE_SW_ON_OMEGA_E))
//            {
//                AngEst.mode  = AE_MODE_BLEND;
//                AngEst.blend = 0.0f;

//                /* 关键：切换瞬间“对齐PLL相位”，避免角度跳变 */
//                AngEst.theta_pll_rad = AngEst.theta_e_enc_rad;
//                AngEst.omega_pll_rad = AngEst.omega_e_enc_rad;
//                AngEst.pll_int       = 0.0f;
// 加上积分项清零
//            }
//        }
//        else if (AngEst.mode == AE_MODE_BLEND)
//        {
//            AngEst.blend += (Ts / AE_BLEND_TIME_S);
//            if (AngEst.blend >= 1.0f)
//            {
//                AngEst.blend = 1.0f;
//                AngEst.mode  = AE_MODE_SENSORLESS;
//            }

//            /* 若中途速度掉回去，直接回编码器（保护） */
//            if (w_enc < AE_SW_OFF_OMEGA_E)
//            {
//                AngEst.mode  = AE_MODE_ENCODER;
//                AngEst.blend = 0.0f;
//            }
//        }
//        else /* AE_MODE_SENSORLESS */
//        {
//            /* 滞回回退 */
//            if (w_enc < AE_SW_OFF_OMEGA_E)
//            {
//                AngEst.mode  = AE_MODE_ENCODER;
//                AngEst.blend = 0.0f;
//            }
//        }
//    }

//    /* ============ 4) 输出最终 used 的角度/速度 ============ */
//    if (AngEst.mode == AE_MODE_ENCODER)
//    {
//        AngEst.theta_e_used_rad = AngEst.theta_e_enc_rad;
//        AngEst.omega_e_used_rad = AngEst.omega_e_enc_rad;
//    }
//    else if (AngEst.mode == AE_MODE_SENSORLESS)
//    {
//        AngEst.theta_e_used_rad = AngEst.theta_pll_rad;
//        AngEst.omega_e_used_rad = AngEst.omega_pll_rad;
//    }
//    else /* BLEND */
//    {
//        /* 圆周插值：theta = theta_enc + blend * wrap(theta_pll - theta_enc) */
//        float d = wrap_pi(AngEst.theta_pll_rad - AngEst.theta_e_enc_rad);
//        AngEst.theta_e_used_rad = wrap_2pi(AngEst.theta_e_enc_rad + AngEst.blend * d);

//        /* 速度线性插值即可 */
//        AngEst.omega_e_used_rad = (1.0f - AngEst.blend) * AngEst.omega_e_enc_rad
//                                + (AngEst.blend)       * AngEst.omega_pll_rad;
//    }
}

float AngleEstimator_GetEleAngleDeg(void)
{
    return AngEst.theta_e_used_rad * (180.0f / PI);
}

float AngleEstimator_GetEleOmegaRad(void)
{
    return AngEst.omega_e_used_rad;
}

uint8_t AngleEstimator_IsSensorless(void)
{
    return (AngEst.mode == AE_MODE_SENSORLESS) ? 1U : 0U;
}




/* USER CODE END EV */

/* USER CODE BEGIN */


/* USER CODE END */

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/
