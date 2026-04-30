/*
 * st7920.c
 *
 *  Created on: 15 avr. 2026
 *      Author: Jonathan Marois
 *
 *      Cette couche connaît les commandes du ST7920, mais pas les dessins avancés.
 */


#include "st7920.h"
#include "st7920_port.h"

static void ST7920_PulseEnable(void)
{
    ST7920_Port_SetE(1);
    ST7920_Port_DelayShort(100);
    ST7920_Port_SetE(0);
    ST7920_Port_DelayShort(100);
}

static void ST7920_Send(uint8_t rs, uint8_t data)
{
    ST7920_Port_SetRS(rs);
    ST7920_Port_SetRW(0);
    ST7920_Port_WriteBus(data);
    ST7920_PulseEnable();
    ST7920_Port_DelayShort(800);
}

static void ST7920_SendCmd(uint8_t cmd)
{
    ST7920_Send(0, cmd);

    if ((cmd == 0x01u) || (cmd == 0x02u))
    {
        ST7920_Port_DelayMs(2);
    }
    else
    {
        ST7920_Port_DelayShort(2000);
    }
}

static void ST7920_SendData(uint8_t data)
{
    ST7920_Send(1, data);
    ST7920_Port_DelayShort(2000);
}

void ST7920_Init(void)
{
    ST7920_Port_Init();

    ST7920_Port_DelayMs(50);

    ST7920_SendCmd(0x30);
    ST7920_Port_DelayMs(1);

    ST7920_SendCmd(0x30);
    ST7920_Port_DelayMs(1);

    ST7920_SendCmd(0x0C);
    ST7920_Port_DelayMs(1);

    ST7920_SendCmd(0x01);
    ST7920_Port_DelayMs(2);

    ST7920_SendCmd(0x06);
    ST7920_Port_DelayMs(1);
}

void ST7920_ClearText(void)
{
    ST7920_SendCmd(0x01);
}

void ST7920_Home(void)
{
    ST7920_SendCmd(0x02);
}

void ST7920_SetCursor(uint8_t row, uint8_t col)
{
    uint8_t addr;

    switch (row)
    {
        case 0: addr = 0x80u + col; break;
        case 1: addr = 0x90u + col; break;
        case 2: addr = 0x88u + col; break;
        case 3: addr = 0x98u + col; break;
        default: return;
    }

    ST7920_SendCmd(addr);
}

void ST7920_WriteChar(char c)
{
    ST7920_SendData((uint8_t)c);
}

void ST7920_WriteString(const char *s)
{
    while (*s)
    {
        ST7920_WriteChar(*s++);
    }
}

void ST7920_GraphicMode(uint8_t enable)
{
    ST7920_SendCmd(0x34);

    if (enable)
    {
        ST7920_SendCmd(0x36);
    }
    else
    {
        ST7920_SendCmd(0x30);
    }
}

void ST7920_ClearGraphic(void)
{
    ST7920_GraphicMode(1);

    for (uint8_t y = 0; y < 64; y++)
    {
        uint8_t lcd_y = (y < 32) ? y : (y - 32);
        uint8_t lcd_x = (y < 32) ? 0 : 8;

        ST7920_SendCmd(0x80 | lcd_y);
        ST7920_SendCmd(0x80 | lcd_x);

        for (uint8_t i = 0; i < 16; i++)
        {
            ST7920_SendData(0x00);
        }
    }
}

void ST7920_Update(const uint8_t *fb)
{
    ST7920_GraphicMode(1);

    for (uint8_t y = 0; y < 64; y++)
    {
        uint8_t lcd_y = (y < 32) ? y : (y - 32);
        uint8_t lcd_x = (y < 32) ? 0 : 8;

        ST7920_SendCmd(0x80 | lcd_y);
        ST7920_SendCmd(0x80 | lcd_x);

        for (uint8_t i = 0; i < 16; i++)
        {
            ST7920_SendData(fb[y * 16 + i]);
        }
    }
}
