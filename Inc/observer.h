/**
 ******************************************************************************
 * @file		observer.h
 * @author		WrathWings
 * @version 	V1.0
 * @date		2019.8.20
 * @brief		The header file of observer.c
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
 /* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __OBSERVER_H
#define __OBSERVER_H

#include "stm32f4xx_hal.h"
#include "math.h"
#include "MotorConfig.h"
#include "foc.h"
#include "PositionSensor.h"

/* 观测器：用平均电感（SPMSM） */
#define AE_LS               (0.5f * (INDUCTANCE_D + INDUCTANCE_Q))
#define AE_R                (PHASE_RES)
#define AE_TS               (DEFAULT_CARRIER_PERIOD_S)


/* 龙伯格增益（需要实机调）
   K1 主要决定 i_hat 收敛速度（太大电流噪声会放大）
   K2 主要决定 e_hat 收敛速度（太大 PLL 抖、太小跟不上） */
#define AE_OBS_K1           (3953.58f)     /* r/l+k1=2*omega_n*sigama 3953.58*/
#define AE_OBS_K2           (-126.0f)    /* -k2/l = omega_n^2 omega_n=1/40*fre*2PI*/

/* EMF 太小时不更新 PLL（避免低速乱抖） */
#define AE_EMF_MIN_V        (1.0f)        /* 视电机/电压调：0.5~2V */

/* PLL 参数（需要实机调，先给保守值） */
#define AE_PLL_KP           (350.0f)
#define AE_PLL_KI           (80000.0f)

/* ωe 限幅（电角速度） */
#define AE_OMEGA_E_LIMIT    (300000.0f)    /* rad/s，按你电机最大转速估 */

/* 传感器→无感切换阈值（电角速度 rad/s）
   经验：先用 200~400Hz 电频率切入。
   ω = 2πf：200Hz≈1256rad/s；400Hz≈2513rad/s */
#define AE_SW_ON_OMEGA_E    (2000.0f)     /* 切到无感：约 318Hz 电频率 */
#define AE_SW_OFF_OMEGA_E   (1400.0f)     /* 回退编码器：做滞回 */

/* 融合时间（越长越平滑，越短越敏捷） */
#define AE_BLEND_TIME_S     (0.02f)       /* 20ms */


typedef enum
{
    AE_MODE_ENCODER = 0,
    AE_MODE_BLEND   = 1,
    AE_MODE_SENSORLESS = 2
} AE_Mode_e;

typedef struct
{
    AE_Mode_e mode;

    /* 来自编码器（外部喂给我） */
    float theta_e_enc_rad;
    float omega_e_enc_rad;

    /* 龙伯格观测器状态 */
    float i_hat_alpha;
    float i_hat_beta;
    float e_hat_alpha;
    float e_hat_beta;

    /* PLL 输出 */
    float theta_pll_rad;
    float omega_pll_rad;
    float pll_int;

    /* 融合参数 */
    float blend;      /* 0..1 */


    /* 输出给控制器用的最终值 */
    float theta_e_used_rad;
    float omega_e_used_rad;

} AngleEstimator_t;

extern AngleEstimator_t AngEst;

/* 初始化 */
void AngleEstimator_Init(void);

/* 编码器每次更新角度/速度后调用（建议放在 GetEleImformation() 末尾） */
void AngleEstimator_SetEncoder(float theta_e_rad, float omega_e_rad);

/* 每个 PWM 周期调用一次（25kHz）：输入 v_alpha/v_beta（电压指令）和 i_alpha/i_beta（测得电流） */
void AngleEstimator_Step(float v_alpha, float v_beta, float i_alpha, float i_beta);

/* 给 Park/InvPark 用：deg */
float AngleEstimator_GetEleAngleDeg(void);

/* 给解耦/观测用：rad/s */
float AngleEstimator_GetEleOmegaRad(void);

/* 当前是否已经进入无感主导 */
uint8_t AngleEstimator_IsSensorless(void);


/* USER CODE END PFP */


#endif

/************************ (C) COPYRIGHT ACTION *****END OF FILE****/
