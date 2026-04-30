/*
 * gfx.h
 *
 *  Created on: 15 avr. 2026
 *      Author: Jonathan Marois
 *
 *      Cette couche dessine dans un framebuffer RAM.
 *		Elle ne parle jamais directement au LCD.
 */

#ifndef MODULES_INC_GFX_H_
#define MODULES_INC_GFX_H_

#include <stdint.h>

typedef struct
{
    uint16_t width;
    uint16_t height;
    uint8_t *buffer;
} gfx_t;

void GFX_Init(gfx_t *gfx, uint16_t width, uint16_t height, uint8_t *buffer);
void GFX_Clear(gfx_t *gfx, uint8_t color);
void GFX_DrawPixel(gfx_t *gfx, uint16_t x, uint16_t y, uint8_t color);
void GFX_DrawHLine(gfx_t *gfx, uint16_t x, uint16_t y, uint16_t w, uint8_t color);
void GFX_DrawVLine(gfx_t *gfx, uint16_t x, uint16_t y, uint16_t h, uint8_t color);
void GFX_DrawRect(gfx_t *gfx, uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint8_t color);
void GFX_DrawChar5x7(gfx_t *gfx, uint16_t x, uint16_t y, char c, uint8_t color);
void GFX_DrawString5x7(gfx_t *gfx, uint16_t x, uint16_t y, const char *s, uint8_t color);


#endif /* MODULES_INC_GFX_H_ */
