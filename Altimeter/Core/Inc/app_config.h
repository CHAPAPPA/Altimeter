/**
  ******************************************************************************
  * @file    app_config.h
  * @brief   Shared application-wide constants for the F5J altimeter.
  *
  * Values here are cross-referenced against the project's Milestone 1
  * requirements spec and the Throttle Fail-Safe Fault Analysis document.
  * Do not change these without re-checking that document.
  ******************************************************************************
  */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

/* ---- RC PWM pulse widths (microseconds, matches TIM2 1us tick) ---------- */
#define PULSE_ABS_MIN_US            900U   /* below this: not a real RC pulse       */
#define PULSE_ABS_MAX_US            2100U  /* above this: not a real RC pulse       */
#define PULSE_IDLE_US               1000U  /* safe "motor off" output pulse         */
#define PULSE_MOTOR_ON_THRESHOLD_US 1100U  /* >= this is treated as "motor on"      */
#define PULSE_FRAME_PERIOD_US       20000U /* 20 ms / 50 Hz, matches TIM2 ARR        */

/* Signal considered lost if no valid pulse captured within this window.
 * A few missed 20ms frames is a real dropout, not decoding jitter. */
#define THROTTLE_SIGNAL_TIMEOUT_MS  100U

/* ---- Flight timing ------------------------------------------------------
 * The MCU has no external crystal; SysTick/HAL_GetTick runs off HSI16,
 * datasheet accuracy ~1% over 0-85C. To GUARANTEE the motor is never
 * legally on past 30.0s of real time even at the slow-clock corner
 * (a slow MCU clock under-counts elapsed real time, so cutting at the
 * MCU's "30000ms" would let real time run past 30s), the enforced limit
 * is trimmed below 30000ms by more than the worst-case drift budget. */
#define MOTOR_MAX_RUN_MS            29400U   /* enforced cutoff, see note above */
#define RECORD_WINDOW_MS            10000U   /* post motor-off peak-tracking window */

/* ---- ALES cutoff modes ---------------------------------------------------*/
typedef enum
{
    ALES_UNLIMITED = 0,
    ALES_100M      = 1,
    ALES_150M      = 2,
    ALES_200M      = 3,
    ALES_MODE_COUNT
} ALES_Mode_t;

/* Metres, indexed by ALES_Mode_t. ALES_UNLIMITED has no height cap. */
static const float ALES_HeightLimitM[ALES_MODE_COUNT] = {
    0.0f,   /* unused for ALES_UNLIMITED */
    100.0f,
    150.0f,
    200.0f
};

#endif /* APP_CONFIG_H */
