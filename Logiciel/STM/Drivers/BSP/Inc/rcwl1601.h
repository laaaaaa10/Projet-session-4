/*
 * rcwl1601.h
 *
 *  Created on: 16 avr. 2026
 *      Author: jonny
 */

#ifndef BSP_INC_RCWL1601_H_
#define BSP_INC_RCWL1601_H_

#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef enum
{
    RCWL1601_OK = 0,
    RCWL1601_BUSY,
    RCWL1601_NO_DATA,
    RCWL1601_TIMEOUT,
    RCWL1601_ERROR
} RCWL1601_Status_t;

typedef struct
{
    GPIO_TypeDef *trig_port;
    uint16_t      trig_pin;

    GPIO_TypeDef *echo_port;
    uint16_t      echo_pin;

    TIM_HandleTypeDef *htim;

    volatile uint32_t echo_start_us;
    volatile uint32_t echo_end_us;
    volatile uint32_t echo_width_us;

    volatile uint8_t  measuring;
    volatile uint8_t  data_ready;
    volatile uint8_t  overflow;
    volatile uint8_t  timeout;

    uint32_t trigger_tick;
    uint32_t timeout_ms;
} RCWL1601_Handle_t;

void RCWL1601_Init(RCWL1601_Handle_t *h,
                   GPIO_TypeDef *trig_port, uint16_t trig_pin,
                   GPIO_TypeDef *echo_port, uint16_t echo_pin,
                   TIM_HandleTypeDef *htim,
                   uint32_t timeout_ms);

void RCWL1601_Trigger(RCWL1601_Handle_t *h);
void RCWL1601_EXTI_Callback(RCWL1601_Handle_t *h, uint16_t gpio_pin);
void RCWL1601_Process(RCWL1601_Handle_t *h);

uint8_t RCWL1601_IsReady(const RCWL1601_Handle_t *h);
RCWL1601_Status_t RCWL1601_GetEchoTimeUs(RCWL1601_Handle_t *h, uint32_t *time_us);
RCWL1601_Status_t RCWL1601_GetDistanceMm(RCWL1601_Handle_t *h, uint32_t *distance_mm);
void RCWL1601_ClearData(RCWL1601_Handle_t *h);

#endif /* BSP_INC_RCWL1601_H_ */
