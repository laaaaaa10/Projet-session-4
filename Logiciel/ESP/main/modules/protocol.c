/*********protocol.c **********/
/*Jonathan Marois 20/03/2026*/

#include "protocol.h"
#include <stdio.h>

void protocol_format_ctrl(char *buffer, size_t len, const control_cmd_t *cmd)
{
    if ((buffer == NULL) || (cmd == NULL) || (len == 0U))
    {
        return;
    }

    //TODO : Formatez pour obtenir des trames semblables à "<type=ctrl;speed=0;turn=0;btn=0>\r\n"
    // Vos entrées sont les membres de votre structure control_cmd_t
    // Votre sortie sera : buffer
    snprintf(buffer, len, "<type=ctrl;speed=%d;turn=%d;btn=%u>\r\n", cmd->speed, cmd->turn, cmd->btn);
}
















