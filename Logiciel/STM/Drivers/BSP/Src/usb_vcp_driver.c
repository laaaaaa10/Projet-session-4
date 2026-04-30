/*
 * usb_vcp_driver.c
 *
 *  Created on: 1 févr. 2026
 *      Author: Jonathan Marois
 */

#include "../Inc/usb_vcp_driver.h"

#include "usbd_cdc_if.h"   // CDC_Transmit_FS
#include "usbd_cdc.h"
#include "usbd_def.h"      // USBD_STATE_CONFIGURED, USBD_OK, USBD_BUSY
#include "main.h"          // HAL_GetTick

extern USBD_HandleTypeDef hUsbDeviceFS;


uint8_t usb_vcp_is_ready(void)				// Vérifie si le USB est configuré
{
    return (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) ? 1U : 0U;
}

static uint8_t tx_free(void)
{
    USBD_CDC_HandleTypeDef *hcdc =
        (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;

    return (hcdc != NULL) && (hcdc->TxState == 0);
}

/*
 * Retour:
 *  0 = OK
 *  1 = timeout (USB pas prêt ou TX occupé trop longtemps)
 *  2 = invalid / fail
 */

uint8_t usb_vcp_write_blocking(uint8_t *pData, uint16_t len, uint32_t timeout_ms)
{
    if (pData == NULL || len == 0)
        return 2;

    uint32_t t0 = HAL_GetTick();

    while ((HAL_GetTick() - t0) <= timeout_ms)
    {
        if (!usb_vcp_is_ready())
        {
            HAL_Delay(1); 	// pour faire respirer le CPU, IRQ etc...
            continue;		// pas encore énuméré
        }

        if (!tx_free())
        {
           HAL_Delay(1); 	// pour faire respirer le CPU, IRQ etc...
           continue;		// TX encore occupé
        }


        uint8_t ret = CDC_Transmit_FS(pData, len);

        if (ret == USBD_OK)
            return 0;

        if (ret != USBD_BUSY)
            return 2; // FAIL ou autre
        // si BUSY -> on retente jusqu'au timeout
    }

    return 1;
}

void usb_vcp_rx_callback(uint8_t *pData, uint16_t len)
{
    (void)pData;
    (void)len;
}

