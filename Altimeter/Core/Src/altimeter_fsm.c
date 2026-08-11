/**
  ******************************************************************************
  * @file    altimeter_fsm.c
  * @brief   Top-level flight state machine.
  ******************************************************************************
  */
#include "altimeter_fsm.h"
#include "app_config.h"
#include "throttle.h"
#include "button.h"
#include "settings.h"
#include "dps368.h"
#include "ssd1309.h"
#include "main.h"
#include <stdio.h>

extern I2C_HandleTypeDef hi2c1; /* DPS368 */
extern I2C_HandleTypeDef hi2c2; /* SSD1309 */

typedef enum
{
    STATE_POWER_UP = 0,
    STATE_ZEROING,
    STATE_ARMED_IDLE,
    STATE_MOTOR_RUN,
    STATE_CUTOFF,
    STATE_RECORD_WINDOW,
    STATE_RESULT_HOLD
} FsmState_t;

/* ---- Zeroing ------------------------------------------------------------*/
#define ZEROING_SAMPLE_COUNT     8U
#define ZEROING_SAMPLE_PERIOD_MS 50U

/* ---- Module state --------------------------------------------------------*/
static FsmState_t   s_state;
static DPS368_Handle_t s_baro;

static ALES_Mode_t s_alesMode;
static float        s_groundPressurePa;
static float        s_currentAltitudeM;
static uint32_t      s_lastSensorReadMs;
static uint32_t      s_lastDisplayMs;

static uint8_t   s_zeroingSamplesTaken;
static double     s_zeroingSumPa;
static uint32_t   s_zeroingLastSampleMs;

static uint32_t s_motorOnStartMs;
static uint32_t s_motorOffMs;
static float     s_peakAltitudeM;

static float     s_resultStartHeightM;
static uint32_t   s_resultFlightTimeMs;

/* ---- Small formatting helpers (no heap, no float printf dependency) -----*/
static int32_t RoundToInt(float value)
{
    return (int32_t)((value >= 0.0f) ? (value + 0.5f) : (value - 0.5f));
}

static void FormatInt(char *buf, int32_t value)
{
    /* Enough for -2147483648 + NUL; altitude/time values here are tiny. */
    sprintf(buf, "%ld", (long)value);
}

static void FormatMmSs(char *buf, uint32_t ms)
{
    uint32_t totalSec = ms / 1000U;
    uint32_t mm = totalSec / 60U;
    uint32_t ss = totalSec % 60U;
    sprintf(buf, "%02lu:%02lu", (unsigned long)mm, (unsigned long)ss);
}

static const char *ModeLabel(ALES_Mode_t m)
{
    switch (m)
    {
        case ALES_100M: return "100";
        case ALES_150M: return "150";
        case ALES_200M: return "200";
        case ALES_UNLIMITED:
        default:        return "UNL";
    }
}

/* ---- Sensor / altitude ----------------------------------------------------*/
#define SENSOR_READ_PERIOD_MS 20U   /* DPS368 configured at 16Hz; 20ms poll just re-reads latest sample */

static void UpdateAltitudeIfDue(void)
{
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - s_lastSensorReadMs) < SENSOR_READ_PERIOD_MS)
    {
        return;
    }
    s_lastSensorReadMs = now;

    float temperatureC, pressurePa;
    if (DPS368_ReadCompensated(&s_baro, &temperatureC, &pressurePa))
    {
        (void)temperatureC;
        s_currentAltitudeM = DPS368_PressureToRelativeAltitudeM(pressurePa, s_groundPressurePa);
    }
    /* On I2C failure: keep the last known altitude rather than glitching
     * to zero; this is a display/logic value, not the safety-critical
     * throttle path, so a stale reading for one cycle is acceptable. */
}

/* ---- Throttle pass-through helper -----------------------------------------*/
static void PassThroughOrIdle(void)
{
    if (Throttle_IsSignalValid())
    {
        Throttle_SetOutput(Throttle_GetPulseUs());
    }
    else
    {
        Throttle_ForceIdle();
    }
}

/* ---- Display ---------------------------------------------------------------*/
#define DISPLAY_PERIOD_MS 150U

static void RedrawArmedIdle(void)
{
    char line[16];
    SSD1309_Clear();
    SSD1309_DrawString(0, 0, "ALT", 2);
    FormatInt(line, RoundToInt(s_currentAltitudeM));
    SSD1309_DrawString(0, 20, line, 2);
    SSD1309_DrawString(0, 40, "CUT", 1);
    SSD1309_DrawString(24, 40, ModeLabel(s_alesMode), 1);
    SSD1309_DrawString(0, 52, "READY", 1);
    SSD1309_UpdateScreen();
}

static void RedrawMotorRun(bool recording)
{
    char line[16];
    SSD1309_Clear();
    SSD1309_DrawString(0, 0, "ALT", 2);
    FormatInt(line, RoundToInt(s_currentAltitudeM));
    SSD1309_DrawString(0, 20, line, 2);
    SSD1309_DrawString(0, 44, recording ? "REC" : "RUN", 1);
    FormatMmSs(line, HAL_GetTick() - s_motorOnStartMs);
    SSD1309_DrawString(28, 44, line, 1);
    SSD1309_UpdateScreen();
}

static void RedrawMotorRunNormal(void)
{
    RedrawMotorRun(false);
}

static void RedrawMotorRunRecording(void)
{
    RedrawMotorRun(true);
}

static void RedrawResultHold(void)
{
    char line[16];
    SSD1309_Clear();
    SSD1309_DrawString(0, 0, "START", 1);
    FormatInt(line, RoundToInt(s_resultStartHeightM));
    SSD1309_DrawString(0, 12, line, 2);
    SSD1309_DrawString(0, 40, "TIME", 1);
    FormatMmSs(line, s_resultFlightTimeMs);
    SSD1309_DrawString(0, 52, line, 1);
    SSD1309_UpdateScreen();
}

static void RedrawIfDue(void (*drawFn)(void))
{
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - s_lastDisplayMs) >= DISPLAY_PERIOD_MS)
    {
        s_lastDisplayMs = now;
        drawFn();
    }
}

/* ---- ALES cutoff evaluation -------------------------------------------------*/
static bool AlesLimitReached(void)
{
    if (s_alesMode == ALES_UNLIMITED)
    {
        return false;
    }
    return s_currentAltitudeM >= ALES_HeightLimitM[s_alesMode];
}

/* ---- State entry helpers ----------------------------------------------------*/
static void EnterArmedIdle(void)
{
    s_state = STATE_ARMED_IDLE;
    s_lastDisplayMs = 0U; /* force immediate redraw */
}

static void EnterMotorRun(void)
{
    s_state           = STATE_MOTOR_RUN;
    s_motorOnStartMs  = HAL_GetTick();
    s_peakAltitudeM   = s_currentAltitudeM;
    s_lastDisplayMs   = 0U;
}

static void EnterCutoff(void)
{
    s_state         = STATE_CUTOFF;
    Throttle_ForceIdle();
    s_motorOffMs    = HAL_GetTick();
    s_peakAltitudeM = s_currentAltitudeM; /* restart peak search for the 10s window */
}

static void EnterRecordWindow(void)
{
    s_state = STATE_RECORD_WINDOW;
    s_lastDisplayMs = 0U;
}

static void EnterResultHold(void)
{
    s_resultStartHeightM = s_peakAltitudeM;
    s_resultFlightTimeMs = s_motorOffMs - s_motorOnStartMs;
    s_state = STATE_RESULT_HOLD;
    s_lastDisplayMs = 0U;
}

/* ---- Public API ---------------------------------------------------------------*/
void FSM_Init(void)
{
    s_alesMode = Settings_Load();

    /* Sensor/display faults are not throttle-safety issues (the hardware
     * fail-off gate does not depend on either), but without them the
     * product can't do its job -- keep trying to init on the bench; a
     * persistently failed sensor will simply show a stale/zero altitude
     * rather than blocking flight. */
    (void)DPS368_Init(&s_baro, &hi2c1);
    (void)SSD1309_Init(&hi2c2);

    SSD1309_Clear();
    SSD1309_DrawString(0, 24, "F5J ALT", 2);
    SSD1309_UpdateScreen();

    s_currentAltitudeM   = 0.0f;
    s_lastSensorReadMs   = 0U;
    s_lastDisplayMs      = 0U;
    s_zeroingSamplesTaken = 0U;
    s_zeroingSumPa        = 0.0;
    s_zeroingLastSampleMs = HAL_GetTick();

    s_state = STATE_ZEROING;
}

void FSM_Tick(void)
{
    switch (s_state)
    {
        case STATE_POWER_UP:
            /* Handled synchronously in FSM_Init(); not re-entered. */
            break;

        case STATE_ZEROING:
        {
            /* Keep the motor forced off and non-blockingly gather a few
             * pressure samples spread across multiple ticks, so the main
             * loop keeps running (and WDI keeps getting kicked) the whole
             * time -- no blocking delay loop here. */
            Throttle_ForceIdle();
            uint32_t now = HAL_GetTick();
            if ((uint32_t)(now - s_zeroingLastSampleMs) >= ZEROING_SAMPLE_PERIOD_MS)
            {
                float t, p;
                if (DPS368_ReadCompensated(&s_baro, &t, &p))
                {
                    s_zeroingSumPa += (double)p;
                    s_zeroingSamplesTaken++;
                }
                s_zeroingLastSampleMs = now;

                if (s_zeroingSamplesTaken >= ZEROING_SAMPLE_COUNT)
                {
                    s_groundPressurePa = (float)(s_zeroingSumPa / (double)s_zeroingSamplesTaken);
                    EnterArmedIdle();
                }
            }
            break;
        }

        case STATE_ARMED_IDLE:
            UpdateAltitudeIfDue();
            PassThroughOrIdle();

            if (Button_ConsumePress())
            {
                s_alesMode = (ALES_Mode_t)((s_alesMode + 1U) % ALES_MODE_COUNT);
                Settings_Save(s_alesMode); /* safe here: not motor-run */
            }

            RedrawIfDue(RedrawArmedIdle);

            if (Throttle_IsMotorCommandedOn())
            {
                EnterMotorRun();
            }
            break;

        case STATE_MOTOR_RUN:
        {
            UpdateAltitudeIfDue();

            uint32_t elapsed = HAL_GetTick() - s_motorOnStartMs;
            bool     pilotCutOrLost = !Throttle_IsMotorCommandedOn();
            bool     timeLimit      = elapsed >= MOTOR_MAX_RUN_MS;
            bool     heightLimit    = AlesLimitReached();

            if (pilotCutOrLost || timeLimit || heightLimit)
            {
                EnterCutoff();
                EnterRecordWindow(); /* CUTOFF is a single-tick transitional state */
            }
            else
            {
                PassThroughOrIdle();
                RedrawIfDue(RedrawMotorRunNormal);
            }
            break;
        }

        case STATE_CUTOFF:
            /* Not normally reached as a standing state -- MOTOR_RUN folds
             * CUTOFF straight into RECORD_WINDOW in the same tick. Kept as
             * an enum value for traceability to the Milestone 1 diagram
             * and as a safe fallback if ever entered directly. */
            Throttle_ForceIdle();
            EnterRecordWindow();
            break;

        case STATE_RECORD_WINDOW:
        {
            UpdateAltitudeIfDue();
            Throttle_ForceIdle();

            if (s_currentAltitudeM > s_peakAltitudeM)
            {
                s_peakAltitudeM = s_currentAltitudeM;
            }

            RedrawIfDue(RedrawMotorRunRecording);

            uint32_t elapsed = HAL_GetTick() - s_motorOffMs;
            if (elapsed >= RECORD_WINDOW_MS)
            {
                EnterResultHold();
            }
            break;
        }

        case STATE_RESULT_HOLD:
            Throttle_ForceIdle(); /* motor stays off until a deliberate relaunch */
            RedrawIfDue(RedrawResultHold);

            if (Button_ConsumePress())
            {
                EnterArmedIdle();
                break;
            }
            if (Throttle_IsMotorCommandedOn())
            {
                EnterMotorRun(); /* re-launch, per US-version restart-after-cutoff option */
            }
            break;

        default:
            /* Unreachable, but a defined enum default keeps this safe
             * rather than undefined if ever hit. */
            Throttle_ForceIdle();
            s_state = STATE_ARMED_IDLE;
            break;
    }
}
