/*
 * font5x7.h
 *
 *  Created on: 15 avr. 2026
 *      Author: Jonathan Marois
 */

#ifndef MODULES_INC_FONT5X7_H_
#define MODULES_INC_FONT5X7_H_

#include <stdint.h>

#define FONT5X7_WIDTH   5
#define FONT5X7_HEIGHT  7
#define FONT5X7_FIRST   32
#define FONT5X7_LAST    127

extern const uint8_t font5x7[][5];

#endif /* MODULES_INC_FONT5X7_H_ */
