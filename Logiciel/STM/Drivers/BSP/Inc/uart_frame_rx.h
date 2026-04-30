/*
 * uart_frame_rx.h
 *
 *  Created on: 10 avr. 2026
 *      Author: Jonathan Marois
 */

#ifndef BSP_INC_UART_RX_FRAME_H_
#define BSP_INC_UART_RX_FRAME_H_

#include <stdbool.h>
#include <stddef.h>

void UartFrameRx_Start(void);
bool UartFrameRx_Fetch(char *dst, size_t len);

void UartFrameRx_RxCpltCallback(void);
void UartFrameRx_ErrorCallback(void);

#endif /* BSP_INC_UART_RX_FRAME_H_ */
