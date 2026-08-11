/**
  ******************************************************************************
  * @file    settings.h
  * @brief   Flash-based persistence of the selected ALES cutoff mode.
  *
  * Uses the last 2KB page of the STM32G031G8's 64KB flash (page 31,
  * 0x0800F800). Blocking erase+program takes tens of ms; Settings_Save()
  * MUST ONLY be called from ARMED/IDLE, never during MOTOR_RUN/CUTOFF/
  * RECORD_WINDOW, so it can never delay the throttle path or starve the
  * watchdog kick. The FSM is the sole caller and is responsible for this.
  ******************************************************************************
  */
#ifndef SETTINGS_H
#define SETTINGS_H

#include "app_config.h"

/* Reads and validates the stored mode (magic + integrity byte). Returns
 * ALES_UNLIMITED if the page is erased/unwritten or fails validation --
 * i.e. an unconfigured or corrupted board always defaults to the simplest,
 * most permissive-but-safe (time-limit-only) behaviour. */
ALES_Mode_t Settings_Load(void);

/* Erases and reprograms the settings page with the given mode.
 * See the blocking-time warning above for when this may be called. */
void Settings_Save(ALES_Mode_t mode);

#endif /* SETTINGS_H */
