/*
 * gfx.c
 *
 *  Created on: 15 avr. 2026
 *      Author: Jonathan Marois
 *
 *      Cette couche dessine dans un framebuffer RAM.
 *		Elle ne parle jamais directement au LCD.
 */


#include "gfx.h"
#include <string.h>
#include "font5x7.h"

void GFX_Init(gfx_t *gfx, uint16_t width, uint16_t height, uint8_t *buffer)
{
    gfx->width = width;
    gfx->height = height;
    gfx->buffer = buffer;
}

void GFX_Clear(gfx_t *gfx, uint8_t color)
{
    memset(gfx->buffer, color ? 0xFF : 0x00, (gfx->width * gfx->height) / 8);
}

void GFX_DrawPixel(gfx_t *gfx, uint16_t x, uint16_t y, uint8_t color)
{
    if (x >= gfx->width || y >= gfx->height)
        return;

    uint32_t index = y * (gfx->width / 8u) + (x / 8u);
    uint8_t mask = (uint8_t)(0x80u >> (x % 8u));

    if (color)
        gfx->buffer[index] |= mask;
    else
        gfx->buffer[index] &= (uint8_t)(~mask);
}

void GFX_DrawHLine(gfx_t *gfx, uint16_t x, uint16_t y, uint16_t w, uint8_t color)
{
    for (uint16_t i = 0; i < w; i++)
    {
        GFX_DrawPixel(gfx, x + i, y, color);
    }
}

void GFX_DrawVLine(gfx_t *gfx, uint16_t x, uint16_t y, uint16_t h, uint8_t color)
{
    for (uint16_t i = 0; i < h; i++)
    {
        GFX_DrawPixel(gfx, x, y + i, color);
    }
}

void GFX_DrawRect(gfx_t *gfx, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t color)
{
    if (w == 0 || h == 0)
        return;

    GFX_DrawHLine(gfx, x, y, w, color);
    GFX_DrawHLine(gfx, x, y + h - 1, w, color);
    GFX_DrawVLine(gfx, x, y, h, color);
    GFX_DrawVLine(gfx, x + w - 1, y, h, color);
}

void GFX_DrawChar5x7(gfx_t *gfx, uint16_t x, uint16_t y, char c, uint8_t color)
{
    if ((uint8_t)c < FONT5X7_FIRST || (uint8_t)c >= FONT5X7_LAST)
        return;

    const uint8_t *glyph = font5x7[(uint8_t)c - FONT5X7_FIRST];

    for (uint8_t col = 0; col < 5; col++)
    {
        uint8_t bits = glyph[col];

        for (uint8_t row = 0; row < 7; row++)
        {
            if (bits & (1u << row))
            {
                GFX_DrawPixel(gfx, x + col, y + row, color);
            }
        }
    }
}

void GFX_DrawString5x7(gfx_t *gfx, uint16_t x, uint16_t y, const char *s, uint8_t color)
{
    while (*s)
    {
        GFX_DrawChar5x7(gfx, x, y, *s++, color);
        x += 6;
    }
}
