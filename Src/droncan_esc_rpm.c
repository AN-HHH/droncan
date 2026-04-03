/**
 ******************************************************************************
 * @file        droncan_esc_rpm.c
 * @brief       DroneCAN ESC RPM control - core implementation
 *
 * Integrates into the ADC sampling ISR (ADC_IRQHandler).  At 20 kHz ISR rate
 * it publishes an ESC Status message at 100 Hz and detects a 1200 ms loss-of-
 * command timeout that automatically stops the motor.
 ******************************************************************************
 */

#include "droncan_esc_rpm.h"

#include "can.h"        /* CAN_Transmit_Raw, hcan1, DroneCAN helpers */
#include "control.h"    /* SpdLoop_t, SpdLoop */
#include "foc.h"        /* CoordTrans_t, CoordTrans */
#include "PositionSensor.h" /* PosSensor_t, PosSensor */
#include "stm32f4xx_hal.h"

/* External motor-controller state */
extern struct SpdLoop_t    SpdLoop;
extern struct CoordTrans_t CoordTrans;
extern struct PosSensor_t  PosSensor;

/* π / 30  (converts RPM → rad/s) and 30 / π (rad/s → RPM) */
#define RPM_TO_RAD_S    (3.14159265358979f / 30.0f)
#define RAD_S_TO_RPM    (30.0f / 3.14159265358979f)

/* ------------------------------------------------------------------ */
/* Module-private state                                                 */
/* ------------------------------------------------------------------ */

static uint16_t s_adc_tick_count   = 0U;   /* Counts ADC ISR calls       */
static uint16_t s_timeout_periods  = 0U;   /* Periods since last RawCmd  */
static uint8_t  s_error_count      = 0U;   /* Wrapping error counter     */
static uint8_t  s_rawcmd_received  = 0U;   /* Flag: at least one command */

/* ------------------------------------------------------------------ */
/* Public functions                                                     */
/* ------------------------------------------------------------------ */

/**
 * @brief  Initialise the DroneCAN ESC module.
 *         Call once during system start, after CAN_Enable().
 */
void DroneCAN_ESC_Init(void)
{
    s_adc_tick_count  = 0U;
    s_timeout_periods = 0U;
    s_error_count     = 0U;
    s_rawcmd_received = 0U;

    /* Start with motor stopped */
    SpdLoop.ExptMecAngularSpeed_rad = 0.0f;
}

/**
 * @brief  Parse a DroneCAN ESC RawCommand CAN frame and apply the speed
 *         setpoint for this ESC (ESC_NODE_INDEX).
 *
 * Called from HAL_CAN_RxFifo0MsgPendingCallback() when a RawCommand
 * message is received.  May be called from CAN RX ISR context.
 *
 * RawCommand encoding (little-endian uint16 per ESC):
 *   0 or 65535  → stop
 *   32768       → 0 % throttle
 *   1-32767     → negative throttle  ((cmd - 32768) / 32768)
 *   32769-65534 → positive throttle  ((cmd - 32768) / 32768)
 *
 * @param cmd_data  Pointer to the 8-byte CAN payload.
 */
void DroneCAN_ESC_RawCommandHandler(const uint8_t *cmd_data)
{
    if (cmd_data == NULL)
    {
        s_error_count++;
        return;
    }

    /* Extract the uint16 command word for ESC_NODE_INDEX */
    uint8_t offset = (uint8_t)(ESC_NODE_INDEX * 2U);
    if ((uint8_t)(offset + 1U) >= 8U)
    {
        s_error_count++;
        return;
    }
    uint16_t raw = (uint16_t)cmd_data[offset]
                 | ((uint16_t)cmd_data[offset + 1U] << 8U);

    /* Stop on sentinel values */
    if (raw == 0U || raw == 65535U)
    {
        SpdLoop.ExptMecAngularSpeed_rad = 0.0f;
    }
    else
    {
        /* Map [0, 65535] → [-1.0, +1.0] throttle fraction */
        float throttle = ((float)(int16_t)(raw - 32768U)) / 32768.0f;

        /* Convert to rad/s */
        float speed_rad_s = throttle * (MAX_MOTOR_RPM * RPM_TO_RAD_S);

        SpdLoop.ExptMecAngularSpeed_rad = speed_rad_s;
    }

    /* Reset the timeout watchdog */
    s_timeout_periods = 0U;
    s_rawcmd_received = 1U;
}

/**
 * @brief  Process DroneCAN ESC tasks from the ADC ISR.
 *
 * Must be called once per ADC_IRQHandler invocation.  Internally it
 * divides the 20 kHz ISR rate down to 100 Hz for status publishing and
 * timeout checking.
 */
void DroneCAN_ESC_ProcessInADCISR(void)
{
    s_adc_tick_count++;
    if (s_adc_tick_count < ESC_STATUS_PERIOD_TICKS)
    {
        return; /* Not yet time to act */
    }
    s_adc_tick_count = 0U;

    /* ---------- Timeout watchdog (1200 ms = 120 periods) ----------- */
    if (s_rawcmd_received)
    {
        s_timeout_periods++;
        if (s_timeout_periods > ESC_RAWCMD_TIMEOUT_PERIODS)
        {
            /* Loss of command - stop the motor */
            SpdLoop.ExptMecAngularSpeed_rad = 0.0f;
            s_error_count++;
            s_timeout_periods = ESC_RAWCMD_TIMEOUT_PERIODS; /* Hold at limit to avoid unnecessary increment */
        }
    }

    /* ---------- Build 8-byte Status payload ----------------------- */
    uint8_t status[8];

    /* [0] ESC index */
    status[0] = (uint8_t)ESC_NODE_INDEX;

    /* [1] Current magnitude (A × scale, clamped to 0-255) */
    float curr_a = CoordTrans.CurrQ;
    if (curr_a < 0.0f) curr_a = -curr_a;
    float curr_scaled = curr_a * ESC_CURRENT_SCALE;
    if (curr_scaled > 255.0f) curr_scaled = 255.0f;
    status[1] = (uint8_t)curr_scaled;

    /* [2-3] RPM, little-endian signed int16 (clamp to int16 range) */
    float rpm_f = PosSensor.MecAngularSpeed_rad * RAD_S_TO_RPM;
    if (rpm_f > 32767.0f)  rpm_f = 32767.0f;
    if (rpm_f < -32768.0f) rpm_f = -32768.0f;
    int16_t rpm_i16 = (int16_t)rpm_f;
    status[2] = (uint8_t)((uint16_t)rpm_i16 & 0xFFU);
    status[3] = (uint8_t)(((uint16_t)rpm_i16 >> 8U) & 0xFFU);

    /* [4-5] Reserved */
    status[4] = 0U;
    status[5] = 0U;

    /* [6] Error count */
    status[6] = s_error_count;

    /* [7] Flags (reserved) */
    status[7] = 0U;

    /* ---------- Transmit Status over CAN --------------------------- */
    CAN_Transmit_Raw(status, sizeof(status),
                     DRONCAN_ESC_STATUS_ID, DRONCAN_PRIORITY_NORMAL);
}
