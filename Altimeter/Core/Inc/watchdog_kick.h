/**
  ******************************************************************************
  * @file    watchdog_kick.h
  * @brief   Service pulse for the external TPS3851 windowed supervisor (WDI).
  *
  * The TPS3851 (CWD=10k->VDD, SET1=high) has a ~200ms timeout: if WDI stops
  * toggling, RESET# is held and NRST forces the throttle gate to idle AND
  * resets the MCU (see "Throttle Fail-Safe - Fault Analysis and Design
  * Decision.docx"). WDI_Kick() must be called ONLY from the completed main
  * loop, never from an isolated timer ISR -- an ISR that keeps firing while
  * the main loop is stuck would otherwise defeat the whole point of having
  * an independent watchdog.
  ******************************************************************************
  */
#ifndef WATCHDOG_KICK_H
#define WATCHDOG_KICK_H

/* Call once at startup after MX_GPIO_Init(). */
void WDI_Init(void);

/* Call once per completed main-loop iteration. Toggles PB1 (WDI). Loop
 * cadence must stay well under the ~200ms supervisor timeout; target
 * calling this at least every ~20-50ms for comfortable margin. */
void WDI_Kick(void);

#endif /* WATCHDOG_KICK_H */
