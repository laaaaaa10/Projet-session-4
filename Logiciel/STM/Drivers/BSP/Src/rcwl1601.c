/*
 * rcwl1601.c
 *
 *  Created on: 16 avr. 2026
 *      Author: Jonathan Marois
 */

#include "rcwl1601.h"

static void RCWL1601_Delay10us(void)
{
    for (volatile uint32_t i = 0; i < 100; i++)
    {
        __NOP();
    }
}

void RCWL1601_Init(RCWL1601_Handle_t *h,
                   GPIO_TypeDef *trig_port, uint16_t trig_pin,
                   GPIO_TypeDef *echo_port, uint16_t echo_pin,
                   TIM_HandleTypeDef *htim,
                   uint32_t timeout_ms)
{
    if (h == NULL)
    {
        return;
    }

    h->trig_port = trig_port;
    h->trig_pin  = trig_pin;
    h->echo_port = echo_port;
    h->echo_pin  = echo_pin;
    h->htim      = htim;

    h->echo_start_us = 0;
    h->echo_end_us   = 0;
    h->echo_width_us = 0;

    h->measuring   = 0;
    h->data_ready  = 0;
    h->overflow    = 0;
    h->timeout     = 0;

    h->trigger_tick = 0;
    h->timeout_ms   = timeout_ms;

    HAL_GPIO_WritePin(h->trig_port, h->trig_pin, GPIO_PIN_RESET);
}

void RCWL1601_Trigger(RCWL1601_Handle_t *h)
{
    if (h == NULL)
    {
        return;
    }

    if (h->measuring)
    {
        return;
    }

    h->echo_start_us = 0;
    h->echo_end_us   = 0;
    h->echo_width_us = 0;
    h->data_ready    = 0;
    h->overflow      = 0;
    h->timeout       = 0;
    h->measuring     = 1;
    h->trigger_tick  = HAL_GetTick();

    HAL_GPIO_WritePin(h->trig_port, h->trig_pin, GPIO_PIN_SET);
    RCWL1601_Delay10us();
    HAL_GPIO_WritePin(h->trig_port, h->trig_pin, GPIO_PIN_RESET);
}

void RCWL1601_EXTI_Callback(RCWL1601_Handle_t *h, uint16_t gpio_pin)
{
    uint32_t now_us;

    if ((h == NULL) || (gpio_pin != h->echo_pin))
    {
        return;
    }

    now_us = __HAL_TIM_GET_COUNTER(h->htim);

    if (HAL_GPIO_ReadPin(h->echo_port, h->echo_pin) == GPIO_PIN_SET)
    {
        if (h->measuring)
        {
            h->echo_start_us = now_us;
        }
    }
    else
    {
        if (h->measuring)
        {
            h->echo_end_us = now_us;

            if (h->echo_end_us >= h->echo_start_us)
            {
                h->echo_width_us = h->echo_end_us - h->echo_start_us;
            }
            else
            {
                h->echo_width_us = (0xFFFFFFFFu - h->echo_start_us) + h->echo_end_us + 1u;
                h->overflow = 1;
            }

            h->data_ready = 1;
            h->measuring  = 0;
        }
    }
}

void RCWL1601_Process(RCWL1601_Handle_t *h)
{
    if (h == NULL)
    {
        return;
    }

    if (h->measuring)
    {
        if ((HAL_GetTick() - h->trigger_tick) > h->timeout_ms)
        {
            h->timeout   = 1;
            h->measuring = 0;
        }
    }
}

uint8_t RCWL1601_IsReady(const RCWL1601_Handle_t *h)
{
    if (h == NULL)
    {
        return 0;
    }

    return h->data_ready;
}

RCWL1601_Status_t RCWL1601_GetEchoTimeUs(RCWL1601_Handle_t *h, uint32_t *time_us)
{
    if ((h == NULL) || (time_us == NULL))
    {
        return RCWL1601_ERROR;
    }

    if (h->timeout)
    {
        h->timeout = 0;
        return RCWL1601_TIMEOUT;
    }

    if (!h->data_ready)
    {
        return h->measuring ? RCWL1601_BUSY : RCWL1601_NO_DATA;
    }

    *time_us = h->echo_width_us;
    h->data_ready = 0;

    return RCWL1601_OK;
}

RCWL1601_Status_t RCWL1601_GetDistanceMm(RCWL1601_Handle_t *h, uint32_t *distance_mm)
{
    uint32_t time_us;
    RCWL1601_Status_t status;

    if ((h == NULL) || (distance_mm == NULL))
    {
        return RCWL1601_ERROR;
    }

    status = RCWL1601_GetEchoTimeUs(h, &time_us);
    if (status != RCWL1601_OK)
    {
        return status;
    }

    /* distance_mm = time_us * 343 / 2000
       v = 343 m/s, aller-retour inclus */
    *distance_mm = (time_us * 343u) / 2000u;

    return RCWL1601_OK;
}

void RCWL1601_ClearData(RCWL1601_Handle_t *h)
{
    if (h == NULL)
    {
        return;
    }

    h->echo_start_us = 0;
    h->echo_end_us   = 0;
    h->echo_width_us = 0;
    h->data_ready    = 0;
    h->overflow      = 0;
    h->timeout       = 0;
    h->measuring     = 0;
}
