/*
 * line_sensor.c
 *
 *  Created on: 12 févr. 2026
 *      Author: Jonathan Marois
 */

#include "line_sensor.h"
#include "pcf8574.h"

#define PIN_CTRL_ALIM   7     // P7 = CTRL-ALIM
#define SENSOR_MASK     0x7F  // bits 0..6

static PCF8574_t pcf;

void LineSensor_Init(I2C_HandleTypeDef *hi2c, uint8_t addr_7bit)
{
    PCF8574_Init(&pcf, hi2c, addr_7bit);

    // Relâcher tous les pins (mode entrée)
    PCF8574_WriteByte(&pcf, 0xFF, 50);

    // Activer l'alimentation des capteurs
    LineSensor_SetPower(true);

    // Petit délai de stabilisation optique
    HAL_Delay(2);
}

void LineSensor_SetPower(bool enable)
{
	if (enable)
	    {
	        // ON = gate LOW
	        PCF8574_WritePin(&pcf, PIN_CTRL_ALIM, 0, 50);
	    }
	    else
	    {
	        // OFF = gate HIGH (relâché)
	        PCF8574_WritePin(&pcf, PIN_CTRL_ALIM, 1, 50);
	    }
}

uint8_t LineSensor_ReadRaw(void)
{
    uint8_t value = 0xFF;

    if (PCF8574_ReadByte(&pcf, &value, 50) != HAL_OK)
        return 0;

    // garder seulement les 7 capteurs
    return (uint8_t)(value & SENSOR_MASK);
}
