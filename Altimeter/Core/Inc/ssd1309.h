/**
  ******************************************************************************
  * @file    ssd1309.h
  * @brief   SSD1309 128x64 white monochrome OLED driver (I2C2, addr 0x3C).
  *
  * Assumes the selected display module has an on-board charge pump (locked
  * BOM requirement) so it runs from the single 3.3V logic rail. Uses the
  * standard SSD1306/SSD1309-compatible init sequence (0x8D charge-pump
  * command included: harmless no-op on modules with an external pump,
  * required on modules that use the controller's own pump).
  ******************************************************************************
  */
#ifndef SSD1309_H
#define SSD1309_H

#include <stdint.h>
#include <stdbool.h>
#include "stm32g0xx_hal.h"

#define SSD1309_WIDTH   128
#define SSD1309_HEIGHT  64

bool SSD1309_Init(I2C_HandleTypeDef *hi2c);

/* All drawing is into a local framebuffer; nothing reaches the panel until
 * SSD1309_UpdateScreen() is called. */
void SSD1309_Clear(void);
void SSD1309_UpdateScreen(void);

/* Draws text at pixel coordinate (x, y), top-left of the first glyph.
 * Font is 5x7 with a 1px inter-glyph gap. scale repeats each source pixel
 * scale x scale (1 = native 5x7, 2 = 10x14, ...). Unsupported characters
 * render as a blank space. */
void SSD1309_DrawString(uint8_t x, uint8_t y, const char *str, uint8_t scale);

#endif /* SSD1309_H */
