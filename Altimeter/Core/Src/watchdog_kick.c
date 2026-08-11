/**
  ******************************************************************************
  * @file    watchdog_kick.c
  * @brief   Service pulse for the external TPS3851 windowed supervisor (WDI).
  ******************************************************************************
  */
#include "watchdog_kick.h"
#include "main.h"

void WDI_Init(void)
{
    /* MX_GPIO_Init() already configures PB1 as push-pull output and drives
     * it low at reset; nothing further required here. Kept as an explicit
     * entry point for clarity/symmetry with the other modules. */
    HAL_GPIO_WritePin(WDI_GPIO_Port, WDI_Pin, GPIO_PIN_RESET);
}

void WDI_Kick(void)
{
    HAL_GPIO_TogglePin(WDI_GPIO_Port, WDI_Pin);
}
