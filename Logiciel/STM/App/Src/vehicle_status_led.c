/*
 * vehicle_status_led.c
 *
 *  Created on: 19 avr. 2026
 *      Author: Jonathan Marois
 */


#include "vehicle_status_led.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "vehicle_diagnostic.h"

#define STATUS_LED_PERIOD_MS 250

static bt_state_t g_bt_state = BT_STATE_DISCONNECTED;
static vehicle_state_t g_vehicle_state = VEHICLE_STATE_IDLE;

static void Led_AllOff(void)
{
    HAL_GPIO_WritePin(GPIOD, LD3_Pin | LD4_Pin | LD5_Pin | LD6_Pin, GPIO_PIN_RESET);
}

static void Led_GreenOn(void)
{
    HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_SET);
}

static void Led_GreenOff(void)
{
    HAL_GPIO_WritePin(GPIOD, LD4_Pin, GPIO_PIN_RESET);
}

static void Led_RedOn(void)
{
    HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_SET);
}

static void Led_RedOff(void)
{
    HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_RESET);
}

static void Led_BlueOn(void)
{
    HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_SET);
}

static void Led_BlueOff(void)
{
    HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_RESET);
}

static void Led_OrangeOn(void)
{
    HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_SET);
}

static void Led_OrangeOff(void)
{
    HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_RESET);
}

void VehicleStatusLed_Init(void)
{
    g_bt_state = BT_STATE_DISCONNECTED;
    g_vehicle_state = VEHICLE_STATE_IDLE;
}

void VehicleStatusLed_SetBtState(bt_state_t state)
{
    g_bt_state = state;
}

void VehicleStatusLed_SetVehicleState(vehicle_state_t state)
{
    g_vehicle_state = state;
}

void VehicleStatusLed_SetGreen(bool on)
{
    HAL_GPIO_WritePin(GPIOD, LD4_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void VehicleStatusLed_SetBlue(bool on)
{
    HAL_GPIO_WritePin(GPIOD, LD6_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void VehicleStatusLed_SetRed(bool on)
{
    HAL_GPIO_WritePin(GPIOD, LD5_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void VehicleStatusLed_SetOrange(bool on)
{
    HAL_GPIO_WritePin(GPIOD, LD3_Pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}


void Task_StatusLED(void *argument)
{
    TickType_t lastWakeTime = xTaskGetTickCount();
    uint32_t slow_blink = 0;
    uint32_t fast_blink = 0;
    diagnostic_ctx_t diag;

    (void)argument;

    for (;;)
    {
    	if (g_vehicle_state == VEHICLE_STATE_DIAGNOSTIC)
		{
			VehicleDiagnostic_GetContext(&diag);

			Led_AllOff();

			if (diag.led_green_enabled)
				Led_GreenOn();

			if (diag.led_blue_enabled)
				Led_BlueOn();

			if (diag.led_red_enabled)
				Led_RedOn();

			if (diag.led_orange_enabled)
				Led_OrangeOn();

			vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(250));
			continue;
		}


    	slow_blink ^= 1U;
        fast_blink ^= 1U;

        /* Vert = Bluetooth */
        if (g_bt_state == BT_STATE_CONNECTED)
        {
            Led_GreenOn();
        }
        else
        {
            if (fast_blink)
                Led_GreenOn();
            else
                Led_GreenOff();
        }

        /* Éteint les DEL de mode avant d'appliquer l'état courant */
        Led_OrangeOff();
        Led_BlueOff();
        Led_RedOff();

        /* Modes du véhicule */
        switch (g_vehicle_state)
        {
            case VEHICLE_STATE_IDLE:
                /* Aucune DEL de mode */
                break;

            case VEHICLE_STATE_MANUAL:
                Led_OrangeOn();
                break;

            case VEHICLE_STATE_LINE_FOLLOW:
                Led_BlueOn();
                break;

            case VEHICLE_STATE_OBSTACLE_AVOID:
                Led_OrangeOn();
                Led_BlueOn();
                break;

            case VEHICLE_STATE_FAILSAFE:
            default:
                Led_RedOn();
                break;
        }

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(STATUS_LED_PERIOD_MS));
    }
}
