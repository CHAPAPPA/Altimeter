/**
  ******************************************************************************
  * @file    button.h
  * @brief   Debounced ALES mode-select button (PA3, active-low, external
  *          10k pull-up + RC filter on the board -- GPIO configured NOPULL).
  ******************************************************************************
  */
#ifndef BUTTON_H
#define BUTTON_H

#include <stdbool.h>

#define BUTTON_DEBOUNCE_MS 30U

/* Call every main-loop pass (non-blocking). */
void Button_Process(void);

/* One-shot: returns true exactly once per confirmed press, then clears.
 * The FSM decides whether/when to act on it (e.g. only in ARMED/IDLE). */
bool Button_ConsumePress(void);

#endif /* BUTTON_H */
