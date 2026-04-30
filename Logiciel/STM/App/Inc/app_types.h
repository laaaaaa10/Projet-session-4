/*
 * app_types.h
 *
 *  Created on: 10 avr. 2026
 *      Author: Jonathan Marois
 */

#ifndef INC_APP_TYPES_H_
#define INC_APP_TYPES_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    int mode;
    int speed;
    int turn;
    int trim;
    int start;
    int stop;
    int up;
    int down;
    int left;
    int right;
    int chk;
} control_cmd_t;

typedef enum
{
    CTRL_EVT_FRAME_RX = 0
} ctrl_event_type_t;

typedef struct
{
    ctrl_event_type_t type;
    control_cmd_t cmd;
} ctrl_event_t;

typedef enum
{
    MOTOR_TARGET_NONE = 0,
    MOTOR_TARGET_AVG,
    MOTOR_TARGET_AVD,
    MOTOR_TARGET_ARG,
    MOTOR_TARGET_ARD
} motor_target_t;

typedef struct
{
    int16_t left_cmd;
    int16_t right_cmd;
    bool coast;

    bool single_motor_mode;
    motor_target_t target_motor;
    int16_t single_motor_cmd;

} motor_cmd_t;


typedef enum
{
    LINE_STATE_UNKNOWN = 0,
    LINE_STATE_CENTER,
    LINE_STATE_LEFT,
    LINE_STATE_RIGHT,
    LINE_STATE_LOST
} line_state_t;

typedef struct
{
    uint32_t left_mm;
    uint32_t center_mm;
    uint32_t right_mm;

    bool left_valid;
    bool center_valid;
    bool right_valid;

} proximity_sensor_data_t;

#endif /* INC_APP_TYPES_H_ */
