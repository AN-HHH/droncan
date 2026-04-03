#include "droncan_esc_rpm.h"
#include "can.h"
#include "control.h"
#include "PositionSensor.h"

/* ===== 本地优先级定义（与 can.c 保持一致） ===== */
#ifndef DRONCAN_PRIORITY_NORMAL
#define DRONCAN_PRIORITY_NORMAL  2
#endif

/* ===== 定时参数（假设 ADC ISR 频率 1kHz） ===== */
#define STATUS_PUBLISH_INTERVAL  10U   /* 每 10 次 ISR 发送一次 Status (10ms, 100Hz) */
#define CAN_TIMEOUT_TICKS        120U  /* 120 * 10ms = 1200ms 无指令则超时停车 */
#define DRONCAN_ESC_STATUS_LENGTH  5U  /* 1 字节 identifier + 4 字节状态数据 */
#define PI_F                     3.14159265359f

/* ===== 外部变量 ===== */
extern struct SpdLoop_t SpdLoop;
extern struct PosSensor_t PosSensor;
extern struct CoordTrans_t CoordTrans;
extern struct CAN_t CAN;

/* ===== 全局管理器 ===== */
static DroneCAN_ESC_Manager_t esc_manager = {
    .status_interval_ms = 10,     /* 100Hz */
    .error_counter = 0,
    .can_timeout_counter = 0,
    .total_errors = 0,
    .armed = false
};

/**
 * @brief 初始化 DroneCAN ESC 管理器
 */
void DroneCAN_ESC_Init(void)
{
    esc_manager.status.esc_index = 0;
    esc_manager.status.current_a = 0;
    esc_manager.status.rpm = 0;
    esc_manager.status.error_flags = 0;
    
    esc_manager.last_status_time = 0;
    esc_manager.error_counter = 0;
    esc_manager.can_timeout_counter = 0;
    esc_manager.total_errors = 0;
    esc_manager.armed = false;
    
    for (int i = 0; i < 4; i++) {
        esc_manager.raw_cmd.cmd[i] = 0;
    }
    esc_manager.raw_cmd.num_cmds = 0;
}

/**
 * @brief 处理 RawCommand 消息 - 转速控制
 *
 * 转速指令映射：
 * - 0, 65535 = 停止
 * - 32768 = 中点 (0% 油门)
 * - < 32768 = 负向 (反转)
 * - > 32768 = 正向 (正转)
 */
void DroneCAN_ESC_RawCommandHandler(const uint8_t *cmd_data, uint8_t node_id)
{
    (void)node_id;

    if (cmd_data == NULL) {
        esc_manager.error_counter++;
        esc_manager.total_errors++;
        return;
    }
    
    /* 解析第一个电调指令（little-endian） */
    uint16_t raw_cmd = ((uint16_t)cmd_data[0]) | (((uint16_t)cmd_data[1]) << 8);
    
    esc_manager.raw_cmd.cmd[0] = raw_cmd;
    
    if (raw_cmd == 0 || raw_cmd == 65535) {
        /* 停止指令 */
        SpdLoop.ExptMecAngularSpeed_rad = 0.0f;
        esc_manager.armed = false;
    } else {
        /* 计算油门百分比 (-1.0 ~ +1.0) */
        int16_t throttle_raw = (int16_t)raw_cmd - 32768;
        float throttle_percent = (float)throttle_raw / 32768.0f;
        
        if (throttle_percent > 1.0f) throttle_percent = 1.0f;
        if (throttle_percent < -1.0f) throttle_percent = -1.0f;
        
        float expected_rpm = throttle_percent * MAX_MOTOR_RPM;
        SpdLoop.ExptMecAngularSpeed_rad = expected_rpm * RPM_TO_RAD_S;
        
        esc_manager.armed = true;
    }
    
    /* 重置 CAN 超时计数器 */
    esc_manager.can_timeout_counter = 0;
}

/**
 * @brief 发送 ESC Status 消息
 * @param esc_index: 电调编号 (0-3)
 */
void DroneCAN_ESC_PublishStatus(uint8_t esc_index)
{
    esc_manager.status.esc_index = esc_index;
    
    /* 电流转换：取绝对值，单位 0.5A/step */
    float current_a = CoordTrans.CurrQ;
    if (current_a < 0) current_a = -current_a;
    esc_manager.status.current_a = (uint8_t)((current_a / 0.5f) & 0xFF);
    
    /* 转速转换：rad/s → RPM */
    float rpm_float = PosSensor.MecAngularSpeed_rad * 30.0f / PI_F;
    if (rpm_float < 0) rpm_float = -rpm_float;
    esc_manager.status.rpm = (uint16_t)(rpm_float);
    
    /* 错误标志 */
    esc_manager.status.error_flags = (uint16_t)(esc_manager.total_errors & 0xFFFF);
    
    /* 打包 ESC 状态数据（esc_index, current, rpm_lo, rpm_hi）到 int32_t */
    int32_t status_value = 0;
    status_value |= ((int32_t)esc_manager.status.esc_index) << 0;
    status_value |= ((int32_t)esc_manager.status.current_a) << 8;
    status_value |= ((int32_t)(esc_manager.status.rpm & 0x00FFU)) << 16;
    status_value |= ((int32_t)((esc_manager.status.rpm >> 8) & 0x00FFU)) << 24;
    
    /* 发送 ESC Status: byte[0]=identifier, bytes[1-4]=esc_index,current,rpm_lo,rpm_hi */
    CAN_Transmit(DRONCAN_ESC_STATUS_ID, status_value, DRONCAN_ESC_STATUS_LENGTH,
                 DRIVER_CLIENT_CAN_ID, DRONCAN_PRIORITY_NORMAL);
}

/**
 * @brief 在 ADC ISR 中调用 - 处理定时任务
 * @param adc_sample_counter: ADC 采样计数
 */
void DroneCAN_ESC_ProcessInADCISR(uint16_t adc_sample_counter)
{
    /* 定时发送 Status 消息 (100Hz = STATUS_PUBLISH_INTERVAL ms 间隔) */
    if ((adc_sample_counter % STATUS_PUBLISH_INTERVAL) == 0)
    {
        DroneCAN_ESC_PublishStatus(0);
    }
    
    /* CAN 通信超时检测 (CAN_TIMEOUT_TICKS * STATUS_PUBLISH_INTERVAL ms) */
    esc_manager.can_timeout_counter++;
    if (esc_manager.can_timeout_counter > CAN_TIMEOUT_TICKS)
    {
        if (esc_manager.armed) {
            SpdLoop.ExptMecAngularSpeed_rad = 0.0f;
            esc_manager.armed = false;
            esc_manager.error_counter++;
            esc_manager.total_errors++;
        }
        esc_manager.can_timeout_counter = 0;
    }
}

/**
 * @brief 获取当前转速指令 (rad/s)
 */
float DroneCAN_ESC_GetExpectedSpeedRadS(void)
{
    return SpdLoop.ExptMecAngularSpeed_rad;
}

/**
 * @brief 获取 ESC 是否上电
 */
bool DroneCAN_ESC_IsArmed(void)
{
    return esc_manager.armed;
}