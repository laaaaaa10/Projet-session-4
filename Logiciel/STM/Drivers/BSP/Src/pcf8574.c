/*
 * pcf8574.c
 *
 *  Created on: 12 févr. 2026
 *      Author: Jonathan Marois
 */


#include "pcf8574.h"

static uint16_t addr8(const PCF8574_t *dev)
{
    return (uint16_t)(dev->addr_7bit << 1);
}

void PCF8574_Init(PCF8574_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr_7bit)
{
    dev->hi2c = hi2c;
    dev->addr_7bit = addr_7bit;
    dev->out_latch = 0xFF; // par défaut: tout relâché (entrée)
}

HAL_StatusTypeDef PCF8574_WriteByte(PCF8574_t *dev, uint8_t value, uint32_t timeout_ms)
{
    dev->out_latch = value;
    return HAL_I2C_Master_Transmit(dev->hi2c, addr8(dev), &value, 1, timeout_ms);
}

HAL_StatusTypeDef PCF8574_ReadByte(PCF8574_t *dev, uint8_t *value, uint32_t timeout_ms)
{
    return HAL_I2C_Master_Receive(dev->hi2c, addr8(dev), value, 1, timeout_ms);
}

HAL_StatusTypeDef PCF8574_WritePin(PCF8574_t *dev, uint8_t pin, uint8_t value, uint32_t timeout_ms)
{
    if (pin > 7) return HAL_ERROR;

    if (value) dev->out_latch |=  (1u << pin);  // relâché (entrée)
    else       dev->out_latch &= ~(1u << pin);  // forcé bas

    return PCF8574_WriteByte(dev, dev->out_latch, timeout_ms);
}

HAL_StatusTypeDef PCF8574_SetPinInput(PCF8574_t *dev, uint8_t pin, uint32_t timeout_ms)
{
    return PCF8574_WritePin(dev, pin, 1u, timeout_ms);
}

HAL_StatusTypeDef PCF8574_SetPinLow(PCF8574_t *dev, uint8_t pin, uint32_t timeout_ms)
{
    return PCF8574_WritePin(dev, pin, 0u, timeout_ms);
}
