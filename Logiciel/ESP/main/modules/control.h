/*********control.h **********/
/*Jonathan Marois 20/03/2026*/

#ifndef CONTROL_H
#define CONTROL_H

#include <stdint.h>
#include <stdbool.h>

/* Commande applicative produite à partir du joystick et du bouton */
typedef struct
{
    int16_t speed;   /* -100 à +100 */
    int16_t turn;    /* -100 à +100 */
    uint8_t btn;     /* 0 ou 1 */
} control_cmd_t;

/* Initialise le module avec des valeurs par défaut */
void control_init(void);

/* Calibre le centre du joystick */
void control_calibrate_center(int raw_x, int raw_y);

/* Définit les bornes min/max mesurées du joystick */
void control_set_min_max(int min_x_val, int max_x_val,
                         int min_y_val, int max_y_val);

/* Transforme les entrées brutes en commande applicative */
void control_compute(control_cmd_t *cmd, int raw_x, int raw_y, bool btn_pressed);

#endif 



