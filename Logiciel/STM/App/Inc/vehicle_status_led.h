/*
 * vehicle_status_led.h
 *
 *  Created on: 19 avr. 2026
 *      Author: Jonathan Marois
 */

#ifndef INC_VEHICLE_STATUS_LED_H_
#define INC_VEHICLE_STATUS_LED_H_

#include "vehicle_control.h"

typedef enum
{
    BT_STATE_DISCONNECTED = 0,
    BT_STATE_CONNECTED
} bt_state_t;

void VehicleStatusLed_Init(void);
void VehicleStatusLed_SetBtState(bt_state_t state);
void VehicleStatusLed_SetVehicleState(vehicle_state_t state);
void VehicleStatusLed_SetGreen(bool on);
void VehicleStatusLed_SetBlue(bool on);
void VehicleStatusLed_SetRed(bool on);
void VehicleStatusLed_SetOrange(bool on);

void Task_StatusLED(void *argument);


#endif /* INC_VEHICLE_STATUS_LED_H_ */
