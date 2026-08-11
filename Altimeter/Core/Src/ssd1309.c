/**
  ******************************************************************************
  * @file    ssd1309.c
  * @brief   SSD1309 128x64 white monochrome OLED driver.
  ******************************************************************************
  */
#include "ssd1309.h"
#include <string.h>

#define SSD1309_I2C_ADDR       (0x3CU << 1)
#define SSD1309_I2C_TIMEOUT_MS 100U
#define CTRL_CMD               0x00U   /* Co=0, D/C#=0: command stream   */
#define CTRL_DATA              0x40U   /* Co=0, D/C#=1: data (GDDRAM)    */

static I2C_HandleTypeDef *s_hi2c;
static uint8_t s_fb[(SSD1309_WIDTH * SSD1309_HEIGHT) / 8];

/* ---- Font: authored row-major (bit4=leftmost col .. bit0=rightmost col),
 * 5 columns x 7 rows, transposed into the panel's page format at draw time
 * via per-pixel SetPixel() calls -- keeps the *authored* table trivial to
 * proofread/edit against the ASCII-art comments, independent of GDDRAM
 * layout. ------------------------------------------------------------- */
typedef struct { char ch; uint8_t rows[7]; } Glyph_t;

static const Glyph_t s_font[] = {
    {' ', {0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
    {'0', {0x0E,0x11,0x13,0x15,0x19,0x11,0x0E}},
    {'1', {0x04,0x0C,0x04,0x04,0x04,0x04,0x0E}},
    {'2', {0x0E,0x11,0x01,0x02,0x04,0x08,0x1F}},
    {'3', {0x1F,0x02,0x04,0x02,0x01,0x11,0x0E}},
    {'4', {0x02,0x06,0x0A,0x12,0x1F,0x02,0x02}},
    {'5', {0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E}},
    {'6', {0x06,0x08,0x10,0x1E,0x11,0x11,0x0E}},
    {'7', {0x1F,0x01,0x02,0x04,0x08,0x08,0x08}},
    {'8', {0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E}},
    {'9', {0x0E,0x11,0x11,0x0F,0x01,0x02,0x0C}},
    {':', {0x00,0x04,0x04,0x00,0x04,0x04,0x00}},
    {'A', {0x0E,0x11,0x11,0x1F,0x11,0x11,0x11}},
    {'C', {0x0F,0x10,0x10,0x10,0x10,0x10,0x0F}},
    {'D', {0x1E,0x11,0x11,0x11,0x11,0x11,0x1E}},
    {'E', {0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F}},
    {'H', {0x11,0x11,0x11,0x1F,0x11,0x11,0x11}},
    {'I', {0x0E,0x04,0x04,0x04,0x04,0x04,0x0E}},
    {'L', {0x10,0x10,0x10,0x10,0x10,0x10,0x1F}},
    {'M', {0x11,0x1B,0x15,0x15,0x11,0x11,0x11}},
    {'N', {0x11,0x19,0x15,0x15,0x13,0x11,0x11}},
    {'O', {0x0E,0x11,0x11,0x11,0x11,0x11,0x0E}},
    {'P', {0x1E,0x11,0x11,0x1E,0x10,0x10,0x10}},
    {'R', {0x1E,0x11,0x11,0x1E,0x14,0x12,0x11}},
    {'S', {0x0F,0x10,0x10,0x0E,0x01,0x01,0x1E}},
    {'T', {0x1F,0x04,0x04,0x04,0x04,0x04,0x04}},
    {'U', {0x11,0x11,0x11,0x11,0x11,0x11,0x0E}},
    {'W', {0x11,0x11,0x11,0x15,0x15,0x1B,0x11}},
    {'Y', {0x11,0x11,0x0A,0x04,0x04,0x04,0x04}},
    {'m', {0x00,0x00,0x1A,0x15,0x15,0x15,0x15}},
    {'-', {0x00,0x00,0x00,0x1F,0x00,0x00,0x00}},
};
#define FONT_COUNT (sizeof(s_font) / sizeof(s_font[0]))

static bool SendCmd(uint8_t cmd)
{
    return HAL_I2C_Mem_Write(s_hi2c, SSD1309_I2C_ADDR, CTRL_CMD, I2C_MEMADD_SIZE_8BIT,
                              &cmd, 1U, SSD1309_I2C_TIMEOUT_MS) == HAL_OK;
}

bool SSD1309_Init(I2C_HandleTypeDef *hi2c)
{
    s_hi2c = hi2c;
    HAL_Delay(50); /* panel power-up settle */

    static const uint8_t initCmds[] = {
        0xAE,             /* display off                       */
        0x20, 0x00,       /* memory addressing mode: horizontal */
        0xC8,             /* COM scan direction, remapped       */
        0x40,             /* display start line = 0             */
        0x81, 0xCF,       /* contrast: bright, for daylight legibility */
        0xA1,             /* segment remap                      */
        0xA6,             /* normal (non-inverted) display      */
        0xA8, 0x3F,       /* multiplex ratio = 63 (64 rows)     */
        0xA4,             /* resume to RAM content display      */
        0xD3, 0x00,       /* display offset = 0                 */
        0xD5, 0x80,       /* clock divide / osc freq             */
        0xD9, 0xF1,       /* pre-charge period                  */
        0xDA, 0x12,       /* COM pins hardware config           */
        0xDB, 0x40,       /* VCOMH deselect level                */
        0x8D, 0x14,       /* charge pump enable (no-op if module has external pump) */
        0xAF              /* display on                          */
    };

    for (uint32_t i = 0; i < sizeof(initCmds); i++)
    {
        if (!SendCmd(initCmds[i]))
        {
            return false;
        }
    }

    SSD1309_Clear();
    SSD1309_UpdateScreen();
    return true;
}

void SSD1309_Clear(void)
{
    memset(s_fb, 0, sizeof(s_fb));
}

void SSD1309_UpdateScreen(void)
{
    /* Column addressing already wraps page-to-page in horizontal mode
     * (set at init), so the whole framebuffer can be streamed as one
     * contiguous write. */
    HAL_I2C_Mem_Write(s_hi2c, SSD1309_I2C_ADDR, CTRL_DATA, I2C_MEMADD_SIZE_8BIT,
                       s_fb, sizeof(s_fb), SSD1309_I2C_TIMEOUT_MS);
}

static void SetPixel(int16_t x, int16_t y, bool on)
{
    if ((x < 0) || (x >= SSD1309_WIDTH) || (y < 0) || (y >= SSD1309_HEIGHT))
    {
        return;
    }
    uint16_t idx = (uint16_t)((y / 8) * SSD1309_WIDTH + x);
    uint8_t  bit = (uint8_t)(y % 8);
    if (on)
    {
        s_fb[idx] |= (uint8_t)(1U << bit);
    }
    else
    {
        s_fb[idx] &= (uint8_t)~(1U << bit);
    }
}

static const Glyph_t *FindGlyph(char c)
{
    for (uint32_t i = 0; i < FONT_COUNT; i++)
    {
        if (s_font[i].ch == c)
        {
            return &s_font[i];
        }
    }
    return &s_font[0]; /* space */
}

static uint8_t DrawChar(uint8_t x, uint8_t y, char c, uint8_t scale)
{
    const Glyph_t *g = FindGlyph(c);
    for (uint8_t row = 0; row < 7; row++)
    {
        uint8_t bits = g->rows[row];
        for (uint8_t col = 0; col < 5; col++)
        {
            bool on = (bits & (uint8_t)(0x10U >> col)) != 0U;
            for (uint8_t sy = 0; sy < scale; sy++)
            {
                for (uint8_t sx = 0; sx < scale; sx++)
                {
                    SetPixel((int16_t)(x + col * scale + sx),
                             (int16_t)(y + row * scale + sy), on);
                }
            }
        }
    }
    return (uint8_t)((5U * scale) + scale); /* glyph width + 1px gap, scaled */
}

void SSD1309_DrawString(uint8_t x, uint8_t y, const char *str, uint8_t scale)
{
    if (scale == 0U)
    {
        scale = 1U;
    }
    uint8_t cursor = x;
    while (*str != '\0')
    {
        cursor = (uint8_t)(cursor + DrawChar(cursor, y, *str, scale));
        str++;
    }
}
