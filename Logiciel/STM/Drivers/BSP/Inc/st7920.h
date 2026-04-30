/*
 * st7920.h
 *
 *  Created on: 15 avr. 2026
 *      Author: Jonathan Marois
 *
 *      Cette couche connaît les commandes du ST7920, mais pas les dessins avancés.
 */

#ifndef BSP_INC_ST7920_H_
#define BSP_INC_ST7920_H_

#include <stdint.h>

#define ST7920_WIDTH    128
#define ST7920_HEIGHT   64
#define ST7920_FB_SIZE  (ST7920_WIDTH * ST7920_HEIGHT / 8)

void ST7920_Init(void);

/* texte */
void ST7920_ClearText(void);
void ST7920_Home(void);
void ST7920_SetCursor(uint8_t row, uint8_t col);
void ST7920_WriteChar(char c);
void ST7920_WriteString(const char *s);

/* graphique */
void ST7920_GraphicMode(uint8_t enable);
void ST7920_ClearGraphic(void);
void ST7920_Update(const uint8_t *fb);


#endif /* BSP_INC_ST7920_H_ */
