/*
 * pcf8574.h
 *
 *  Created on: 12 févr. 2026
 *      Author: Jonathan Marois
 */

#ifndef BSP_INC_PCF8574_H_
#define BSP_INC_PCF8574_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint8_t addr_7bit;     // ex: 0x20..0x27
    uint8_t out_latch;     // dernière valeur écrite
} PCF8574_t;

/**
 * @brief Initialise le driver PCF8574.
 * @param dev       Structure device
 * @param hi2c      Handle I2C (ex: &hi2c1)
 * @param addr_7bit Adresse 7-bit (0x20..0x27)
 */
void PCF8574_Init(PCF8574_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr_7bit);

/**
 * @brief Écrit un octet (8 bits) sur les ports P0..P7.
 *        Attention: sur PCF8574, écrire '1' = relâcher la ligne (entrée / pull-up faible)
 *                   écrire '0' = forcer à 0 (sink).
 */
HAL_StatusTypeDef PCF8574_WriteByte(PCF8574_t *dev, uint8_t value, uint32_t timeout_ms);

/**
 * @brief Lit l'état des ports P0..P7 (1 octet).
 */
HAL_StatusTypeDef PCF8574_ReadByte(PCF8574_t *dev, uint8_t *value, uint32_t timeout_ms);

/**
 * @brief Met un pin en "entrée" (relâché) -> écrit 1 sur ce bit.
 */
HAL_StatusTypeDef PCF8574_SetPinInput(PCF8574_t *dev, uint8_t pin, uint32_t timeout_ms);

/**
 * @brief Force un pin à 0 -> écrit 0 sur ce bit.
 */
HAL_StatusTypeDef PCF8574_SetPinLow(PCF8574_t *dev, uint8_t pin, uint32_t timeout_ms);

/**
 * @brief Écrit un bit (0/1) dans le latch puis envoie au PCF8574.
 *        value=1 -> relâché (entrée), value=0 -> forcé bas.
 */
HAL_StatusTypeDef PCF8574_WritePin(PCF8574_t *dev, uint8_t pin, uint8_t value, uint32_t timeout_ms);


#endif /* BSP_INC_PCF8574_H_ */
