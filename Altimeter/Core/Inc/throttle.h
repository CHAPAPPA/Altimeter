/**
  ******************************************************************************
  * @file    throttle.h
  * @brief   RC throttle capture (PA0/TIM2_CH1) and gated output (PA1/TIM2_CH2).
  *
  * SAFETY NOTE: the hardware fail-off path (TPS3851 supervisor + 74LVC1G08
  * AND gate, ESC_OUT = THROTTLE_OUT AND NRST/SYS_OK) is the actual backstop
  * against a locked-up or misbehaving MCU -- see
  * "Throttle Fail-Safe - Fault Analysis and Design Decision.docx".
  * This module is responsible for the residual firmware-domain risk that the
  * hardware gate cannot see: a HEALTHY MCU acting on bad/implausible input.
  * It must never emit a motor-on-looking pulse without first having observed
  * a legitimate idle frame (power-up / reconnect arming), and must default
  * to a safe idle output whenever the input signal is missing or implausible.
  ******************************************************************************
  */
#ifndef THROTTLE_H
#define THROTTLE_H

#include <stdint.h>
#include <stdbool.h>

/* Call once after MX_TIM2_Init(): starts PWM output at safe idle FIRST,
 * then starts input capture. */
void Throttle_Init(void);

/* Call every main-loop pass (non-blocking). Detects RX signal loss/timeout
 * and re-forces safe idle output if the input has gone stale. */
void Throttle_Process(void);

/* True if a plausible (900-2100us) pulse has been captured within the
 * signal-timeout window. */
bool Throttle_IsSignalValid(void);

/* True once a legitimate idle-range pulse has been observed since boot or
 * since the last signal-loss event (power-up arming gate). */
bool Throttle_IsArmed(void);

/* Convenience: signal valid AND armed AND pulse >= motor-on threshold. */
bool Throttle_IsMotorCommandedOn(void);

/* Last successfully captured, in-range pulse width in microseconds.
 * Only meaningful when Throttle_IsSignalValid() is true. */
uint16_t Throttle_GetPulseUs(void);

/* Pass a (clamped) pulse width straight through to the ESC output.
 * This is the normal-operation call: the FSM uses it to reproduce the
 * pilot's own commanded throttle. Values outside the plausible range are
 * clamped to PULSE_IDLE_US rather than passed through. */
void Throttle_SetOutput(uint16_t pulse_us);

/* Force the ESC output to the safe idle pulse immediately. Used for ALES/
 * 30s cutoff, and whenever the input is not valid/armed. */
void Throttle_ForceIdle(void);

#endif /* THROTTLE_H */
