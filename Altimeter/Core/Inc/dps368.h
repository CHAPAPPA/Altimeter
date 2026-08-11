/**
  ******************************************************************************
  * @file    dps368.h
  * @brief   Infineon DPS368 barometric pressure sensor driver (I2C1).
  *
  * Register map, calibration-coefficient bit-packing, scaling-factor table
  * and the compensation formulas are verified verbatim against Infineon's
  * official Arduino driver (Dps3xx family, shared DPS310/DPS368 core),
  * https://github.com/Infineon/DPS368-Library-Arduino (branch "dps368"):
  *   - src/util/dps368_config.h : PROD_ID location, coefficient block
  *   - src/DpsClass.cpp         : readcoeffs() bit-packing (quoted verbatim),
  *                                24-bit raw assembly + sign extension,
  *                                scaling_facts[] table (quoted verbatim)
  *   - src/Dps310.cpp           : calcTemp()/calcPressure() (quoted verbatim;
  *                                DPS368 shares this compensation math)
  *
  * NOT independently re-verified from source (standard, well-documented
  * DPS310/368 datasheet register map, cross-checked against the two
  * addresses the driver source DID confirm -- PROD_ID at 0x0D and the
  * coefficient block at 0x10/18 bytes both matched exactly): PRS_CFG/
  * TMP_CFG/MEAS_CFG/RESET register addresses and the TMP_EXT bit. Also
  * NOT confirmed: the exact DPS368__PROD_ID expected value (used 0x10
  * here per Infineon's datasheet; the fetched driver headers only showed
  * DPS310__PROD_ID=0x00 and DPS422__PROD_ID=0x0A, not a DPS368 constant).
  * CONFIRM DPS368_PROD_ID_VALUE below against your exact datasheet
  * revision before relying on the chip-ID gate -- a mismatch fails safe
  * (Init() returns false, FSM keeps running with a static/last-known
  * altitude; this is not a throttle-safety path).
  ******************************************************************************
  */
#ifndef DPS368_H
#define DPS368_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32g0xx_hal.h"

typedef struct
{
    I2C_HandleTypeDef *hi2c;
    int32_t c0Half, c1;
    int32_t c00, c10;
    int32_t c01, c11, c20, c21, c30;
    float    lastTempScaled;
    bool     ready;
} DPS368_Handle_t;

/* Soft-resets, verifies chip ID, reads + parses the 18-byte calibration
 * block, configures OSRx8/16Hz pressure and OSRx1/16Hz temperature
 * (TMP_EXT set per Infineon's app-note requirement), continuous
 * background mode (both). Returns false on any I2C/ID failure. */
bool DPS368_Init(DPS368_Handle_t *dev, I2C_HandleTypeDef *hi2c);

/* Reads raw pressure+temperature and returns compensated values.
 * Must be called at least once (temperature first) before pressure
 * compensation is meaningful -- this function always reads/compensates
 * temperature before pressure internally, matching the sensor's own
 * cross-dependency (pressure compensation uses the latest scaled temp). */
bool DPS368_ReadCompensated(DPS368_Handle_t *dev, float *temperatureC, float *pressurePa);

/* International barometric formula, valid for the short (<1km) relative
 * altitudes relevant to F5J. Returns metres above the reference pressure. */
float DPS368_PressureToRelativeAltitudeM(float pressurePa, float groundPressurePa);

#endif /* DPS368_H */
