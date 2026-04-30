/*
 * vehicle_display_data.h
 *
 *  Created on: 26 avr. 2026
 *      Author: Jonathan Marois
 */

#ifndef INC_VEHICLE_DISPLAY_DATA_H_
#define INC_VEHICLE_DISPLAY_DATA_H_

#include <stdint.h>
#include <stdbool.h>
#include "app_types.h"

typedef struct
{
    bool bt_connected;

    char last_rx_frame[128];

    uint8_t line_raw;
    int line_error;

    uint32_t prox_left_mv;
    uint32_t prox_right_mv;

    proximity_sensor_data_t prox;

} vehicle_display_data_t;

void VehicleDisplayData_SetBluetoothConnected(bool connected);
void VehicleDisplayData_SetLastRxFrame(const char *frame);
void VehicleDisplayData_SetLineData(uint8_t raw, int error);
void VehicleDisplayData_SetProximityData(const proximity_sensor_data_t *prox,
                                         uint32_t left_mv,
                                         uint32_t right_mv);

void VehicleDisplayData_Get(vehicle_display_data_t *data);

#endif /* INC_VEHICLE_DISPLAY_DATA_H_ */
