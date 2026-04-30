/*
 * vehicle_display_data.c
 *
 *  Created on: 26 avr. 2026
 *      Author: Jonathan Marois
 */

/*
 * Module de données d'affichage/debug.
 *
 * Ce fichier centralise les informations partagées entre les tâches
 * et les modules d'affichage : état Bluetooth, dernière trame reçue,
 * données du capteur de ligne et distances des capteurs de proximité.
 *
 * Ces données servent à l'affichage et au diagnostic, pas à la logique
 * critique de contrôle.
 */

#include "vehicle_display_data.h"
#include <string.h>

static volatile vehicle_display_data_t g_display_data = {0};

void VehicleDisplayData_SetBluetoothConnected(bool connected)
{
    g_display_data.bt_connected = connected;
}

void VehicleDisplayData_SetLastRxFrame(const char *frame)
{
    if (frame == NULL)
        return;

    strncpy((char *)g_display_data.last_rx_frame,
            frame,
            sizeof(g_display_data.last_rx_frame) - 1);

    g_display_data.last_rx_frame[sizeof(g_display_data.last_rx_frame) - 1] = '\0';
}

void VehicleDisplayData_SetLineData(uint8_t raw, int error)
{
    g_display_data.line_raw = raw;
    g_display_data.line_error = error;
}

void VehicleDisplayData_SetProximityData(const proximity_sensor_data_t *prox,
                                         uint32_t left_mv,
                                         uint32_t right_mv)
{
    if (prox == NULL)
        return;

    g_display_data.prox = *prox;
    g_display_data.prox_left_mv = left_mv;
    g_display_data.prox_right_mv = right_mv;
}

void VehicleDisplayData_Get(vehicle_display_data_t *data)
{
    if (data == NULL)
        return;

    *data = g_display_data;
}
