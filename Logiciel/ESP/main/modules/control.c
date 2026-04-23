/*********control.c **********/
/*Jonathan Marois 20/03/2026*/
/******************************* 
Ce module sert à :
- prendre les valeurs brutes du joystick (raw_x, raw_y)
- tenir compte du centre du joystick
- convertir ces valeurs en une commande normalisée :
    - turn entre -100 et 100
    - speed entre -100 et 100
    - btn à 0 ou 1
*************************************/

#include "control.h"
#include <stddef.h>

#define CONTROL_DEFAULT_CENTER     2048
#define CONTROL_DEFAULT_MIN        0
#define CONTROL_DEFAULT_MAX        4095
#define CONTROL_DEADZONE_PERCENT   12

static int s_center_x = CONTROL_DEFAULT_CENTER;
static int s_center_y = CONTROL_DEFAULT_CENTER;

static int s_min_x = CONTROL_DEFAULT_MIN;
static int s_max_x = CONTROL_DEFAULT_MAX;
static int s_min_y = CONTROL_DEFAULT_MIN;
static int s_max_y = CONTROL_DEFAULT_MAX;


static int clamp_int(int value, int min, int max)
{
    if (value < min)
    {
        return min;
    }

    if (value > max)
    {
        return max;
    }

    return value;
}


static int map_axis(int raw, int center, int min, int max)
{
    int value;

    if (raw >= center)
    {
        int span_pos = max - center;

        if (span_pos <= 0)
        {
            return 0;
        }

        value = ((raw - center) * 100) / span_pos;
    }
    else
    {
        int span_neg = center - min;

        if (span_neg <= 0)
        {
            return 0;
        }

        value = ((raw - center) * 100) / span_neg;
    }

    /* Zone morte autour du centre */
    if ((value > -CONTROL_DEADZONE_PERCENT) &&
        (value <  CONTROL_DEADZONE_PERCENT))
    {
        value = 0;
    }

    return clamp_int(value, -100, 100);
}

void control_init(void)
{
    s_center_x = CONTROL_DEFAULT_CENTER;
    s_center_y = CONTROL_DEFAULT_CENTER;

    s_min_x = CONTROL_DEFAULT_MIN;
    s_max_x = CONTROL_DEFAULT_MAX;
    s_min_y = CONTROL_DEFAULT_MIN;
    s_max_y = CONTROL_DEFAULT_MAX;
}

void control_calibrate_center(int raw_x, int raw_y)
{
    s_center_x = raw_x;
    s_center_y = raw_y;
}

void control_set_min_max(int min_x_val, int max_x_val,
                         int min_y_val, int max_y_val)
{
    s_min_x = min_x_val;
    s_max_x = max_x_val;
    s_min_y = min_y_val;
    s_max_y = max_y_val;
}

void control_compute(control_cmd_t *cmd, int raw_x, int raw_y, bool btn_pressed)
{
    if (cmd == NULL)
    {
        return;
    }

    cmd->turn  = (int16_t)map_axis(raw_x, s_center_x, s_min_x, s_max_x);

    /* Axe Y inversé : pousser vers l'avant donne une vitesse positive */
    cmd->speed = (int16_t)(-map_axis(raw_y, s_center_y, s_min_y, s_max_y));

    cmd->btn = btn_pressed ? 1U : 0U;
}