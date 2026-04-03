/**
 ******************************************************************************
 * @file        droncan_esc_rpm.h
 * @brief       DroneCAN ESC RPM control - data structures and interface
 *
 * Lightweight DroneCAN ESC implementation:
 *   - Receives RawCommand (message type 0x200) from PX4 and converts to rad/s
 *   - Publishes Status (message type 0x201) at 100Hz from the ADC ISR
 *   - No voltage or temperature sensor required
 ******************************************************************************
 */

#ifndef __DRONCAN_ESC_RPM_H__
#define __DRONCAN_ESC_RPM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* DroneCAN ESC message type IDs (lightweight, non-standard) */
#define DRONCAN_ESC_RAWCMD_ID       0x200U  /* RawCommand: PX4 → ESC */
#define DRONCAN_ESC_STATUS_ID       0x201U  /* Status:     ESC → PX4 */

/* ESC identity - index of this ESC in the array (0-based) */
#define ESC_NODE_INDEX              0U

/* Motor maximum speed - MUST be tuned to match the actual motor specifications.
 * Default 1000 RPM is intentionally conservative; typical brushless motors for
 * UAVs may need values of 5000-20000 RPM. */
#define MAX_MOTOR_RPM               1000.0f /* RPM - adjust per motor */

/* ADC ISR tick counts for timing (ADC runs at DEFAULT_CARRIER_FREQ = 20 kHz) */
#define ADC_ISR_FREQ_HZ             20000U
#define ESC_STATUS_PERIOD_TICKS     200U    /* 20000 / 100 Hz = 200 ticks */

/* Timeout: 1200 ms expressed as a number of 100 Hz status periods */
#define ESC_RAWCMD_TIMEOUT_PERIODS  120U    /* 120 × 10 ms = 1200 ms */

/* Current scaling: CoordTrans.CurrQ [A] → uint8 (1 LSB = 0.1 A, 0-25.5 A) */
#define ESC_CURRENT_SCALE           10.0f

/*
 * Status message layout (8 bytes, sent over DroneCAN extended frame):
 *   [0]   ESC index (0-3)
 *   [1]   Current magnitude (A × ESC_CURRENT_SCALE, uint8, clamped 0-255)
 *   [2-3] RPM, little-endian int16 (signed, negative = reverse)
 *   [4-5] Reserved (zero)
 *   [6]   Error count (uint8, wraps at 255)
 *   [7]   Flags (reserved, zero)
 */

/* RawCommand layout per the DroneCAN ESC protocol:
 *   cmd value  meaning
 *   0          stop (same as 65535)
 *   1-32767    reverse   (throttle = (cmd - 32768) / 32768)
 *   32768      neutral   (0% throttle)
 *   32769-65534 forward  (throttle = (cmd - 32768) / 32768)
 *   65535      stop (same as 0)
 */

/* Public API */
void DroneCAN_ESC_Init(void);
void DroneCAN_ESC_RawCommandHandler(const uint8_t *cmd_data);
void DroneCAN_ESC_ProcessInADCISR(void);

#ifdef __cplusplus
}
#endif

#endif /* __DRONCAN_ESC_RPM_H__ */
