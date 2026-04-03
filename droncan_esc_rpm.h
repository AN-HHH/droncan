#ifndef __DRONCAN_ESC_RPM_H__
#define __DRONCAN_ESC_RPM_H__

#include <stdint.h>
#include <stdbool.h>

/* ===== DroneCAN ESC 消息 ID ===== */
#define DRONCAN_ESC_RAWCMD_ID    0x200   /* RawCommand */
#define DRONCAN_ESC_STATUS_ID    0x201   /* Status */

/* ===== 电机配置 ===== */
#define MAX_MOTOR_RPM            1000.0f  /* 电机最大 RPM，需根据实际修改 */
#define RPM_TO_RAD_S             0.10472f /* π/30 */

/* ===== ESC Status 消息结构 ===== */
typedef struct {
    uint8_t esc_index;           /* 电调编号 (0-3) */
    uint8_t current_a;           /* 电流 (0-255, 单位 0.5A/step) */
    uint16_t rpm;                /* 转速 RPM */
    uint16_t reserved;           /* 保留字段 */
    uint16_t error_flags;        /* 错误标志 + 错误计数 */
} DroneCAN_ESC_Status_t;

/* ===== ESC RawCommand 消息结构 ===== */
typedef struct {
    uint16_t cmd[4];             /* 电调指令 (0-65535) */
    uint8_t num_cmds;            /* 指令数 */
} DroneCAN_ESC_RawCmd_t;

/* ===== ESC 管理器 ===== */
typedef struct {
    DroneCAN_ESC_Status_t status;
    DroneCAN_ESC_RawCmd_t raw_cmd;
    uint32_t last_status_time;   /* 上次 Status 发送时间 (ms) */
    uint8_t status_interval_ms;  /* 发送间隔 (ms) */
    uint16_t error_counter;      /* 错误计数器 */
    uint16_t can_timeout_counter;/* CAN 超时计数 */
    uint16_t total_errors;       /* 累计错误数 */
    bool armed;                  /* 电调是否上电 */
} DroneCAN_ESC_Manager_t;

/* ===== 函数接口 ===== */

/**
 * @brief 初始化 DroneCAN ESC 管理器
 * @note  在 main() 中调用，CAN_Enable() 之后
 */
void DroneCAN_ESC_Init(void);

/**
 * @brief 处理 RawCommand 消息 - 转速控制
 * @param cmd_data: 8 字节指令数据
 * @param node_id: 源节点 ID
 */
void DroneCAN_ESC_RawCommandHandler(const uint8_t *cmd_data, uint8_t node_id);

/**
 * @brief 发送 ESC Status 消息 (100Hz)
 * @param esc_index: 电调编号 (0-3)
 */
void DroneCAN_ESC_PublishStatus(uint8_t esc_index);

/**
 * @brief 在 ADC ISR 中调用 - 处理定时任务
 * @param adc_sample_counter: ADC 采样计数 (用于定时触发)
 */
void DroneCAN_ESC_ProcessInADCISR(uint16_t adc_sample_counter);

/**
 * @brief 获取当前转速指令 (rad/s)
 * @return 期望角速度
 */
float DroneCAN_ESC_GetExpectedSpeedRadS(void);

/**
 * @brief 获取 ESC 是否上电
 * @return true: 已上电, false: 未上电
 */
bool DroneCAN_ESC_IsArmed(void);

#endif /* __DRONCAN_ESC_RPM_H__ */