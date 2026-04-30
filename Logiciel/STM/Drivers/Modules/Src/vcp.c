/*
 * VCP.c
 *
 *  Created on: 1 févr. 2026
 *      Author: Jonathan Marois
 */


#include "vcp.h"
#include "usb_vcp_driver.h"
#include "main.h"      // HAL_GetTick
#include <string.h>

#define VCP_READY_TIMEOUT_MS  2000U		// attente énumération USB
#define VCP_TX_TIMEOUT_MS      100U



uint8_t VCP_IsReady(void)
{
    return usb_vcp_is_ready();
}

int8_t VCP_SendStringBlocking(const char *str)
{
    if (str == NULL)
        return 3;

    uint16_t len = (uint16_t)strlen(str);
    if (len == 0)
        return 0;

    // 1) Attendre que l'USB soit prêt (énumération) avec timeout interne
    uint32_t t0 = HAL_GetTick();
    while (!VCP_IsReady())
    {
        if ((HAL_GetTick() - t0) > VCP_READY_TIMEOUT_MS)
            return 1; // pas prêt
        HAL_Delay(1);	// pour faire respirer le CPU, IRQ etc...
    }

    // 2) Envoyer (driver gère TX free + BUSY) avec timeout interne
    uint8_t r = usb_vcp_write_blocking((uint8_t*)str, len, VCP_TX_TIMEOUT_MS);

    return (r == 0) ? 0 : 2;
}



