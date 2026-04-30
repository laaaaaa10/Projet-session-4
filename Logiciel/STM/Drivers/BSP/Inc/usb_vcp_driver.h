/*
 * usb_vcp_driver.h
 *
 *  Created on: 1 févr. 2026
 *      Author: Jonathan Marois
 */

#ifndef BSP_INC_USB_VCP_DRIVER_H_
#define BSP_INC_USB_VCP_DRIVER_H_

#include <stdint.h>

uint8_t usb_vcp_is_ready(void); // 1 = USB configuré, 0 sinon
uint8_t usb_vcp_write_blocking(uint8_t *pData, uint16_t len, uint32_t timeout_ms);
void    usb_vcp_rx_callback(uint8_t *pData, uint16_t len);

#endif /* BSP_INC_USB_VCP_DRIVER_H_ */
