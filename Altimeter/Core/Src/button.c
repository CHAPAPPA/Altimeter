/**
  ******************************************************************************
  * @file    button.c
  * @brief   Debounced ALES mode-select button.
  ******************************************************************************
  */
#include "button.h"
#include "main.h"

static GPIO_PinState s_lastRaw       = GPIO_PIN_SET;   /* idle = high (pulled up) */
static GPIO_PinState s_confirmed     = GPIO_PIN_SET;
static uint32_t       s_lastChangeMs = 0U;
static bool            s_pressEvent  = false;

void Button_Process(void)
{
    GPIO_PinState raw = HAL_GPIO_ReadPin(Button_Input_GPIO_Port, Button_Input_Pin);
    uint32_t       now = HAL_GetTick();

    if (raw != s_lastRaw)
    {
        s_lastRaw      = raw;
        s_lastChangeMs = now;
    }
    else if ((raw != s_confirmed) &&
             ((uint32_t)(now - s_lastChangeMs) >= BUTTON_DEBOUNCE_MS))
    {
        /* Stable for the full debounce window: accept as the new state. */
        s_confirmed = raw;
        if (s_confirmed == GPIO_PIN_RESET)
        {
            /* Active-low: this is a press (release -> pressed transition). */
            s_pressEvent = true;
        }
    }
}

bool Button_ConsumePress(void)
{
    if (s_pressEvent)
    {
        s_pressEvent = false;
        return true;
    }
    return false;
}
