/*
 * st7920_port.h
 *
 *  Created on: 15 avr. 2026
 *      Author: Jonathan Marois
 *
 *      Contient uniquement ce qui dépend du STM32 et de ces broches.
 */

#ifndef BSP_INC_ST7920_PORT_H_
#define BSP_INC_ST7920_PORT_H_

#include "main.h"
#include <stdint.h>

void ST7920_Port_Init(void);
void ST7920_Port_SetRS(uint8_t level);
void ST7920_Port_SetRW(uint8_t level);
void ST7920_Port_SetE(uint8_t level);
void ST7920_Port_SetPSB(uint8_t level);
void ST7920_Port_WriteBus(uint8_t value);
void ST7920_Port_DelayShort(volatile uint32_t count);
void ST7920_Port_DelayMs(uint32_t ms);


#endif /* BSP_INC_ST7920_PORT_H_ */
