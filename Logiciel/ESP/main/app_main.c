/*********app_main.c **********/
/*Jonathan Marois 20/03/2026*/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//  includes 
#include "control.h"
#include "joystick.h"
#include "button.h"
#include "protocol.h"
#include "app_tasks.h"
#include "bt_spp_client.h"


// Ici le scheduler est déjà démarré
// Le main
void app_main(void)
{
    static const esp_bd_addr_t hc06_addr = {0x00, 0x21, 0x13, 0x03, 0xCA, 0x17};
    static const char hc06_pin[] = "1234";

    control_init();
    joystick_init();
    button_init();
    bt_spp_client_init(hc06_addr, hc06_pin);
    app_tasks_init();
}