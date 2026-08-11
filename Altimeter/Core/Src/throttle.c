/**
  ******************************************************************************
  * @file    throttle.c
  * @brief   RC throttle capture (PA0/TIM2_CH1) and gated output (PA1/TIM2_CH2).
  *          See throttle.h for the safety rationale.
  ******************************************************************************
  */
#include "throttle.h"
#include "app_config.h"
#include "main.h"

extern TIM_HandleTypeDef htim2;

/* ---- Capture-side state (written by ISR, read by main loop) ------------ */
typedef enum
{
    CAP_WAIT_RISING = 0,
    CAP_GOT_RISING
} CaptureState_t;

static volatile CaptureState_t s_captureState   = CAP_WAIT_RISING;
static volatile uint32_t       s_risingCapture   = 0U;

static volatile uint16_t s_lastPulseUs     = PULSE_IDLE_US;
static volatile uint32_t s_lastPulseTickMs = 0U;
static volatile bool     s_signalValid     = false;
static volatile bool     s_armed           = false;

/* ---- Output-side state --------------------------------------------------*/
static uint16_t s_currentOutputUs = PULSE_IDLE_US;

/* Wraparound-safe delta between two TIM2 CNT captures, given the counter
 * wraps every (PULSE_FRAME_PERIOD_US) counts (ARR = 20000, 1us tick). */
static uint32_t PulseWidthFromCaptures(uint32_t rising, uint32_t falling)
{
    if (falling >= rising)
    {
        return falling - rising;
    }
    else
    {
        return (PULSE_FRAME_PERIOD_US - rising) + falling;
    }
}

/* Called from the ISR context (via HAL_TIM_IC_CaptureCallback) for every
 * new plausible pulse. Applies the input-plausibility check and the
 * power-up/reconnect arming rule. Kept short: no I/O, no blocking. */
static void ProcessNewPulse(uint16_t pulse_us)
{
    if ((pulse_us < PULSE_ABS_MIN_US) || (pulse_us > PULSE_ABS_MAX_US))
    {
        /* Implausible frame: do not update last-known-good pulse, do not
         * touch the armed state. Signal-loss timeout (in Throttle_Process)
         * will catch a receiver stuck sending garbage. */
        return;
    }

    s_lastPulseUs     = pulse_us;
    s_lastPulseTickMs = HAL_GetTick();
    s_signalValid     = true;

    if (!s_armed && (pulse_us < PULSE_MOTOR_ON_THRESHOLD_US))
    {
        /* First legitimate idle-range frame seen since boot/reconnect:
         * only now is the device allowed to ever pass a motor-on pulse. */
        s_armed = true;
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM2)
    {
        return;
    }
    if (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_1)
    {
        return;
    }

    uint32_t captured = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

    /* Both-edge capture doesn't itself report which edge fired; by the time
     * this handler runs, the pin level reflects the transition that just
     * happened. Standard technique for single-channel RC PWM decode. */
    GPIO_PinState level = HAL_GPIO_ReadPin(THROTTLE_IN_GPIO_Port, THROTTLE_IN_Pin);

    if (level == GPIO_PIN_SET)
    {
        /* Rising edge: start of a new pulse. */
        s_risingCapture = captured;
        s_captureState  = CAP_GOT_RISING;
    }
    else
    {
        /* Falling edge: end of a pulse, only meaningful if we saw the
         * matching rising edge first. */
        if (s_captureState == CAP_GOT_RISING)
        {
            uint32_t width = PulseWidthFromCaptures(s_risingCapture, captured);
            if (width <= 0xFFFFU)
            {
                ProcessNewPulse((uint16_t)width);
            }
        }
        s_captureState = CAP_WAIT_RISING;
    }
}

void Throttle_Init(void)
{
    /* Output side first: force the compare register to safe idle BEFORE the
     * PWM channel is enabled, so no stray/default pulse width can ever be
     * emitted, even for one frame. */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PULSE_IDLE_US);
    s_currentOutputUs = PULSE_IDLE_US;
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    /* Input side. */
    s_captureState = CAP_WAIT_RISING;
    s_signalValid  = false;
    s_armed        = false;
    s_lastPulseUs  = PULSE_IDLE_US;
    HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
}

void Throttle_Process(void)
{
    /* Signal-loss watchdog: if nothing plausible has been captured recently,
     * the RX link (or the whole receiver) may be gone. Treat as motor-off
     * and require re-arming before trusting the input again. */
    uint32_t now = HAL_GetTick();
    bool     timedOut;

    if (!s_signalValid)
    {
        timedOut = true;
    }
    else
    {
        timedOut = (uint32_t)(now - s_lastPulseTickMs) > THROTTLE_SIGNAL_TIMEOUT_MS;
    }

    if (timedOut)
    {
        if (s_signalValid)
        {
            /* Transitioning from valid -> lost: drop armed state so a
             * reconnect must present a fresh idle frame before we trust it. */
            s_signalValid = false;
            s_armed       = false;
        }
        Throttle_ForceIdle();
    }
}

bool Throttle_IsSignalValid(void)
{
    return s_signalValid;
}

bool Throttle_IsArmed(void)
{
    return s_armed;
}

bool Throttle_IsMotorCommandedOn(void)
{
    return s_signalValid && s_armed && (s_lastPulseUs >= PULSE_MOTOR_ON_THRESHOLD_US);
}

uint16_t Throttle_GetPulseUs(void)
{
    return s_lastPulseUs;
}

void Throttle_SetOutput(uint16_t pulse_us)
{
    if ((pulse_us < PULSE_ABS_MIN_US) || (pulse_us > PULSE_ABS_MAX_US))
    {
        pulse_us = PULSE_IDLE_US;
    }
    s_currentOutputUs = pulse_us;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, pulse_us);
}

void Throttle_ForceIdle(void)
{
    s_currentOutputUs = PULSE_IDLE_US;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, PULSE_IDLE_US);
}
