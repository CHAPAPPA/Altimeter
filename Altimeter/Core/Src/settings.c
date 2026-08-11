/**
  ******************************************************************************
  * @file    settings.c
  * @brief   Flash-based persistence of the selected ALES cutoff mode.
  ******************************************************************************
  */
#include "settings.h"
#include "stm32g0xx_hal.h"
#include <string.h>

/* STM32G031G8: 64KB flash, 2KB pages -> pages 0..31. Reserve the last page
 * (31) for settings, well clear of application code. */
#define SETTINGS_PAGE       31U
#define SETTINGS_ADDRESS    (FLASH_BASE + (SETTINGS_PAGE * FLASH_PAGE_SIZE))
#define SETTINGS_MAGIC      0xA1E5C0DEUL

typedef struct
{
    uint32_t magic;
    uint8_t  mode;
    uint8_t  modeInverted; /* ~mode, cheap integrity check */
    uint16_t reserved;
} SettingsRecord_t; /* must be exactly 8 bytes: one flash doubleword */

ALES_Mode_t Settings_Load(void)
{
    const SettingsRecord_t *rec = (const SettingsRecord_t *)SETTINGS_ADDRESS;

    if ((rec->magic == SETTINGS_MAGIC) &&
        (rec->mode == (uint8_t)(~rec->modeInverted)) &&
        (rec->mode < (uint8_t)ALES_MODE_COUNT))
    {
        return (ALES_Mode_t)rec->mode;
    }
    return ALES_UNLIMITED;
}

void Settings_Save(ALES_Mode_t mode)
{
    SettingsRecord_t rec;
    rec.magic        = SETTINGS_MAGIC;
    rec.mode         = (uint8_t)mode;
    rec.modeInverted = (uint8_t)(~(uint8_t)mode);
    rec.reserved     = 0xFFFFU;

    FLASH_EraseInitTypeDef eraseInit;
    uint32_t                pageError = 0U;

    HAL_FLASH_Unlock();

    eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInit.Banks     = FLASH_BANK_1;
    eraseInit.Page       = SETTINGS_PAGE;
    eraseInit.NbPages    = 1U;

    if (HAL_FLASHEx_Erase(&eraseInit, &pageError) == HAL_OK)
    {
        uint64_t doubleword;
        memcpy(&doubleword, &rec, sizeof(doubleword));
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, SETTINGS_ADDRESS, doubleword);
    }

    HAL_FLASH_Lock();
}
