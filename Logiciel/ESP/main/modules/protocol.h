/*********protocol.h **********/
/*Jonathan Marois 20/03/2026*/

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stddef.h>
#include "control.h"

void button_init(void);
bool button_is_pressed(void);

/* Formate une trame texte à partir d'une commande applicative */
void protocol_format_ctrl(char *buffer, size_t len, const control_cmd_t *cmd);

#endif /* PROTOCOL_H */