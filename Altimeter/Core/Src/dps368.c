/**
  ******************************************************************************
  * @file    dps368.c
  * @brief   Infineon DPS368 barometric pressure sensor driver.
  *          See dps368.h for the source-verification notes.
  ******************************************************************************
  */
#include "dps368.h"
#include <math.h>
#include <string.h>

/* ---- Register map --------------------------------------------------------*/
#define DPS368_REG_PSR_B2      0x00U   /* pressure raw, 3 bytes MSB-first    */
#define DPS368_REG_TMP_B2      0x03U   /* temperature raw, 3 bytes MSB-first */
#define DPS368_REG_PRS_CFG     0x06U
#define DPS368_REG_TMP_CFG     0x07U
#define DPS368_REG_MEAS_CFG    0x08U
#define DPS368_REG_RESET       0x0CU
#define DPS368_REG_PROD_ID     0x0DU
#define DPS368_REG_COEF        0x10U   /* 18-byte calibration block, verified */

#define DPS368_CMD_SOFT_RESET  0x09U   /* RESET reg, bits[3:0] soft-reset     */
#define DPS368_MEAS_CONT_BOTH  0x07U   /* MEAS_CFG bits[2:0]: continuous P+T  */
#define DPS368_PROD_ID_MASK    0x0FU

/* CONFIRM against your exact DPS368 datasheet revision -- see dps368.h. */
#define DPS368_PROD_ID_VALUE   0x10U

#define DPS368_I2C_ADDR        (0x76U << 1)  /* SDO -> GND; confirm strapping */
#define DPS368_I2C_TIMEOUT_MS  100U
#define DPS368_COEF_LEN        18U

/* PRS_CFG: rate index 4 (16Hz) << 4 | osr index 3 (x8)   -> 0x43
 * TMP_CFG: TMP_EXT=1 (external/MEMS sensor -- REQUIRED per Infineon's
 *          DPS310/368 application note, matches how the factory
 *          calibration coefficients were derived) | rate index 4 (16Hz)
 *          << 4 | osr index 0 (x1)                       -> 0xC0
 * Neither OSR is >= 16x (index >= 4), so the CFG_REG P_SHIFT/T_SHIFT bits
 * are not needed and are left at their reset (disabled) state. */
#define DPS368_PRS_CFG_VALUE   0x43U
#define DPS368_TMP_CFG_VALUE   0xC0U

#define DPS368_PRS_OSR_INDEX   3U   /* x8  -> matches PRS_CFG_VALUE above */
#define DPS368_TMP_OSR_INDEX   0U   /* x1  -> matches TMP_CFG_VALUE above */

/* Verbatim from Infineon DpsClass.cpp (scaling_facts[]). */
static const int32_t s_scalingFacts[8] =
{
    524288, 1572864, 3670016, 7864320, 253952, 516096, 1040384, 2088960
};

static bool ReadRegs(DPS368_Handle_t *dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(dev->hi2c, DPS368_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                             buf, len, DPS368_I2C_TIMEOUT_MS) == HAL_OK;
}

static bool WriteReg(DPS368_Handle_t *dev, uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(dev->hi2c, DPS368_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                              &value, 1U, DPS368_I2C_TIMEOUT_MS) == HAL_OK;
}

/* Verbatim from Infineon DpsClass.cpp getTwosComplement(). */
static void TwosComplement(int32_t *raw, uint8_t length)
{
    if (*raw & ((int32_t)1 << (length - 1)))
    {
        *raw -= (int32_t)1 << length;
    }
}

/* Verbatim bit-packing from Infineon DpsClass.cpp readcoeffs(). */
static void ParseCoeffs(DPS368_Handle_t *dev, const uint8_t *b)
{
    dev->c0Half = ((int32_t)b[0] << 4) | (((int32_t)b[1] >> 4) & 0x0F);
    TwosComplement(&dev->c0Half, 12);
    dev->c0Half = dev->c0Half / 2;

    dev->c1 = (((int32_t)b[1] & 0x0F) << 8) | (int32_t)b[2];
    TwosComplement(&dev->c1, 12);

    dev->c00 = ((int32_t)b[3] << 12) | ((int32_t)b[4] << 4) | (((int32_t)b[5] >> 4) & 0x0F);
    TwosComplement(&dev->c00, 20);

    dev->c10 = (((int32_t)b[5] & 0x0F) << 16) | ((int32_t)b[6] << 8) | (int32_t)b[7];
    TwosComplement(&dev->c10, 20);

    dev->c01 = ((int32_t)b[8] << 8) | (int32_t)b[9];
    TwosComplement(&dev->c01, 16);

    dev->c11 = ((int32_t)b[10] << 8) | (int32_t)b[11];
    TwosComplement(&dev->c11, 16);

    dev->c20 = ((int32_t)b[12] << 8) | (int32_t)b[13];
    TwosComplement(&dev->c20, 16);

    dev->c21 = ((int32_t)b[14] << 8) | (int32_t)b[15];
    TwosComplement(&dev->c21, 16);

    dev->c30 = ((int32_t)b[16] << 8) | (int32_t)b[17];
    TwosComplement(&dev->c30, 16);
}

static bool ReadRaw24(DPS368_Handle_t *dev, uint8_t startReg, int32_t *out)
{
    uint8_t b[3];
    if (!ReadRegs(dev, startReg, b, 3U))
    {
        return false;
    }
    int32_t raw = ((int32_t)b[0] << 16) | ((int32_t)b[1] << 8) | (int32_t)b[2];
    TwosComplement(&raw, 24);
    *out = raw;
    return true;
}

bool DPS368_Init(DPS368_Handle_t *dev, I2C_HandleTypeDef *hi2c)
{
    uint8_t prodId = 0;
    uint8_t coefRaw[DPS368_COEF_LEN];

    memset(dev, 0, sizeof(*dev));
    dev->hi2c = hi2c;

    if (!WriteReg(dev, DPS368_REG_RESET, DPS368_CMD_SOFT_RESET))
    {
        return false;
    }
    HAL_Delay(40); /* power-up/reset settle */

    if (!ReadRegs(dev, DPS368_REG_PROD_ID, &prodId, 1U))
    {
        return false;
    }
    if ((prodId & DPS368_PROD_ID_MASK) != DPS368_PROD_ID_VALUE)
    {
        return false; /* fails safe: caller keeps last-known altitude */
    }

    if (!ReadRegs(dev, DPS368_REG_COEF, coefRaw, DPS368_COEF_LEN))
    {
        return false;
    }
    ParseCoeffs(dev, coefRaw);

    if (!WriteReg(dev, DPS368_REG_PRS_CFG, DPS368_PRS_CFG_VALUE))
    {
        return false;
    }
    if (!WriteReg(dev, DPS368_REG_TMP_CFG, DPS368_TMP_CFG_VALUE))
    {
        return false;
    }
    if (!WriteReg(dev, DPS368_REG_MEAS_CFG, DPS368_MEAS_CONT_BOTH))
    {
        return false;
    }
    HAL_Delay(10); /* allow first conversion to complete before first read */

    dev->lastTempScaled = 0.0f;
    dev->ready = true;
    return true;
}

bool DPS368_ReadCompensated(DPS368_Handle_t *dev, float *temperatureC, float *pressurePa)
{
    int32_t rawTemp, rawPress;

    if (!dev->ready)
    {
        return false;
    }

    /* Temperature must be read/compensated first: pressure compensation
     * depends on the latest scaled temperature (see calcPressure below),
     * matching Infineon's own reference driver ordering. */
    if (!ReadRaw24(dev, DPS368_REG_TMP_B2, &rawTemp))
    {
        return false;
    }
    if (!ReadRaw24(dev, DPS368_REG_PSR_B2, &rawPress))
    {
        return false;
    }

    /* --- calcTemp, verbatim from Infineon Dps310.cpp --- */
    float tempScaled = (float)rawTemp / (float)s_scalingFacts[DPS368_TMP_OSR_INDEX];
    dev->lastTempScaled = tempScaled;
    float temp = (float)dev->c0Half + (float)dev->c1 * tempScaled;

    /* --- calcPressure, verbatim from Infineon Dps310.cpp --- */
    float prsScaled = (float)rawPress / (float)s_scalingFacts[DPS368_PRS_OSR_INDEX];
    float prs = (float)dev->c00
              + prsScaled * ((float)dev->c10 + prsScaled * ((float)dev->c20 + prsScaled * (float)dev->c30))
              + dev->lastTempScaled * ((float)dev->c01 + prsScaled * ((float)dev->c11 + prsScaled * (float)dev->c21));

    *temperatureC = temp;
    *pressurePa   = prs;
    return true;
}

float DPS368_PressureToRelativeAltitudeM(float pressurePa, float groundPressurePa)
{
    if (groundPressurePa <= 0.0f)
    {
        return 0.0f;
    }
    float ratio = pressurePa / groundPressurePa;
    return 44330.0f * (1.0f - powf(ratio, 1.0f / 5.255f));
}
