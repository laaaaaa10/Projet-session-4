/*
 * vehicle_diagnostic.h
 *
 *  Created on: 23 avr. 2026
 *      Author: Jonathan Marois
 */

#ifndef INC_VEHICLE_DIAGNOSTIC_H_
#define INC_VEHICLE_DIAGNOSTIC_H_

#include <stdbool.h>
#include "app_types.h"

typedef enum
{
    DIAG_MENU_ROOT = 0,
    DIAG_MENU_SENSORS,
    DIAG_MENU_SENSOR_LINE,
    DIAG_MENU_SENSOR_PROX,
    DIAG_MENU_COMM,

    DIAG_MENU_MOTOR_LIST,
    DIAG_MENU_MOTOR_AVG,
    DIAG_MENU_MOTOR_AVD,
    DIAG_MENU_MOTOR_ARG,
    DIAG_MENU_MOTOR_ARD,

    DIAG_MENU_LED_LIST,
    DIAG_MENU_LED_GREEN,
	DIAG_MENU_LED_ORANGE,
    DIAG_MENU_LED_BLUE,
    DIAG_MENU_LED_RED,
    DIAG_MENU_DISPLAY
} diag_menu_t;
typedef enum
{
    DIAG_DISPLAY_MENU = 0,
    DIAG_DISPLAY_ALL_OFF,
    DIAG_DISPLAY_ALL_ON
} diag_display_mode_t;

typedef struct
{
    diag_menu_t menu;

    uint8_t cursor;
    uint8_t scroll;

    bool led_green_enabled;
    bool led_orange_enabled;
    bool led_blue_enabled;
    bool led_red_enabled;

    diag_display_mode_t display_mode;

    uint8_t comm_scroll;

    control_cmd_t prev_cmd;
} diagnostic_ctx_t;

void VehicleDiagnostic_Init(void);
void VehicleDiagnostic_ProcessCommand(const control_cmd_t *cmd);
void VehicleDiagnostic_GetContext(diagnostic_ctx_t *ctx);
bool VehicleDiagnostic_GetSelectedMotor(motor_target_t *motor);

#endif /* INC_VEHICLE_DIAGNOSTIC_H_ */
