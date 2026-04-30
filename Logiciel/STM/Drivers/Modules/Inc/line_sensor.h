/*
 * line_sensor.h
 *
 *  Created on: 12 févr. 2026
 *      Author: Jonathan Marois
 */

#ifndef MODULES_INC_LINE_SENSOR_H_
#define MODULES_INC_LINE_SENSOR_H_

#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

/**
 * @brief Initialise le module suiveur de ligne.
 * @param hi2c       Handle I2C (ex: &hi2c1)
 * @param addr_7bit  Adresse 7-bit du PCF8574 (ex: 0x22)
 */
void LineSensor_Init(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit);

/**
 * @brief Active ou désactive l'alimentation des capteurs.
 */
void LineSensor_SetPower(bool enable);

/**
 * @brief Lit les 7 capteurs (bits 0..6).
 * @return Octet avec bits 0..6 = capteurs.
 */
uint8_t LineSensor_ReadRaw(void);


#endif /* MODULES_INC_LINE_SENSOR_H_ */
