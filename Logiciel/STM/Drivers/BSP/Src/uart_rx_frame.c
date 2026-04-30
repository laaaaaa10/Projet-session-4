/*
 * uart_frame_rx.c
 *
 *  Created on: 10 avr. 2026
 *      Author: Jonathan Marois
 */


#include "uart_frame_rx.h"

#include "main.h"
#include <stdbool.h>
#include <stddef.h>

extern UART_HandleTypeDef huart2;

static uint8_t rx_char;
static volatile char rx_buf[128];
static volatile uint8_t rx_idx = 0;
static volatile bool frame_ready = false;

void UartFrameRx_Start(void)
{
    HAL_UART_Receive_IT(&huart2, &rx_char, 1);
}

bool UartFrameRx_Fetch(char *dst, size_t len)
{
    bool ok = false;

    if ((dst == NULL) || (len == 0))
        return false;

    __disable_irq();
    if (frame_ready)
    {
        size_t i;

        for (i = 0; i < len - 1; i++)
        {
            dst[i] = rx_buf[i];
            if (rx_buf[i] == '\0')
                break;
        }

        dst[len - 1] = '\0';
        frame_ready = false;
        ok = true;
    }
    __enable_irq();

    return ok;
}

void UartFrameRx_RxCpltCallback(void)
{
    char c = (char)rx_char;

    if (c == '\r' || c == '\n')
    {
        HAL_UART_Receive_IT(&huart2, &rx_char, 1);
        return;
    }

    if (!frame_ready)
    {
        if (rx_idx == 0)
        {
            if (c == '<')
            {
                rx_buf[rx_idx++] = c;
            }
        }
        else
        {
            if (c == '<')
            {
                rx_idx = 0;
                rx_buf[rx_idx++] = c;
            }
            else if (rx_idx < sizeof(rx_buf) - 1)
            {
                rx_buf[rx_idx++] = c;

                if (c == '>')
                {
                    rx_buf[rx_idx] = '\0';
                    rx_idx = 0;
                    frame_ready = true;
                }
            }
            else
            {
                rx_idx = 0;
                rx_buf[0] = '\0';
            }
        }
    }

    HAL_UART_Receive_IT(&huart2, &rx_char, 1);
}

void UartFrameRx_ErrorCallback(void)
{
    rx_idx = 0;
    frame_ready = false;
    rx_buf[0] = '\0';

    HAL_UART_AbortReceive(&huart2);
    HAL_UART_Receive_IT(&huart2, &rx_char, 1);
}
