/**
  ******************************************************************************
  * @file    altimeter_fsm.h
  * @brief   Top-level flight state machine (Milestone 1 Section 5).
  *
  * States: POWER_UP -> ZEROING -> ARMED_IDLE -> MOTOR_RUN -> CUTOFF ->
  *         RECORD_WINDOW -> RESULT_HOLD -> (re-launch) -> MOTOR_RUN
  *
  * Owns: DPS368 + SSD1309 init, ground-reference zeroing, ALES mode
  * selection/persistence, 30s/10s timing, throttle pass-through vs cutoff
  * decisions (via throttle.c), and display updates. Ties every other
  * module together; this is the file to read to understand overall
  * behaviour end to end.
  ******************************************************************************
  */
#ifndef ALTIMETER_FSM_H
#define ALTIMETER_FSM_H

/* Performs POWER_UP actions (load settings, init sensor + display) and
 * enters ZEROING. Call once after all MX_*_Init()/module Init() calls. */
void FSM_Init(void);

/* Call every main-loop pass (non-blocking; internally rate-gates sensor
 * reads and display redraws so it's cheap to call often). */
void FSM_Tick(void);

#endif /* ALTIMETER_FSM_H */
