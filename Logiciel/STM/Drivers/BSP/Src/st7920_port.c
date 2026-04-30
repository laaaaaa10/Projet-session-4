/*
 * st7920_port.c
 *
 *  Created on: 15 avr. 2026
 *      Author: Jonathan Marois
 *
 *      Contient uniquement ce qui dépend du STM32 et de ces broches.
 */


#include "st7920_port.h"

#define LCD_RS_GPIO_Port     GPIOD
#define LCD_RS_Pin           GPIO_PIN_11

#define LCD_RW_GPIO_Port     GPIOD
#define LCD_RW_Pin           GPIO_PIN_10

#define LCD_PSB_GPIO_Port    GPIOD
#define LCD_PSB_Pin          GPIO_PIN_9

#define LCD_E_GPIO_Port      GPIOB
#define LCD_E_Pin            GPIO_PIN_8

#define LCD_DATA_GPIO_Port   GPIOE

void ST7920_Port_Init(void)
{
    ST7920_Port_SetPSB(1);
    ST7920_Port_SetE(0);
    ST7920_Port_SetRS(0);
    ST7920_Port_SetRW(0);
    ST7920_Port_WriteBus(0x00);
}

void ST7920_Port_SetRS(uint8_t level)
{
    HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin,
                      level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void ST7920_Port_SetRW(uint8_t level)
{
    HAL_GPIO_WritePin(LCD_RW_GPIO_Port, LCD_RW_Pin,
                      level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void ST7920_Port_SetE(uint8_t level)
{
    HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin,
                      level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void ST7920_Port_SetPSB(uint8_t level)
{
    HAL_GPIO_WritePin(LCD_PSB_GPIO_Port, LCD_PSB_Pin,
                      level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void ST7920_Port_WriteBus(uint8_t value)
{
    uint32_t odr = LCD_DATA_GPIO_Port->ODR;
    odr &= ~(0xFFu << 8);
    odr |= ((uint32_t)value << 8);
    LCD_DATA_GPIO_Port->ODR = odr;
}

void ST7920_Port_DelayShort(volatile uint32_t count)
{
    while (count--)
    {
        __NOP();
    }
}

void ST7920_Port_DelayMs(uint32_t ms)
{
    HAL_Delay(ms);
}
