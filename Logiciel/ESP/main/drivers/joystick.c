/*********joystick.c **********/
/*Jonathan Marois 20/03/2026*/

#include "joystick.h"

#include "driver/adc.h"

#define JOY_X ADC1_CHANNEL_6
#define JOY_Y ADC1_CHANNEL_7

void joystick_init(void)
{
    adc1_config_width(ADC_WIDTH_BIT_12);

    adc1_config_channel_atten(JOY_X, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(JOY_Y, ADC_ATTEN_DB_11);
}

int joystick_read_x(void)
{
    return adc1_get_raw(JOY_X);
}

int joystick_read_y(void)
{
    return adc1_get_raw(JOY_Y);
}