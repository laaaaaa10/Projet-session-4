/*********bt_spp_client.c **********/
/*Jonathan Marois 20/03/2026*/

#ifndef BT_SPP_CLIENT_H
#define BT_SPP_CLIENT_H

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#include "esp_err.h"
#include "esp_bt_defs.h"

#define BT_CONNECTED_BIT BIT0
#define BT_PIN_MAX_LEN   16

void bt_spp_client_init(const esp_bd_addr_t peer_addr, const char *pin_code);
esp_err_t bt_spp_client_send_text(const char *text);
bool bt_spp_client_is_connected(void);
EventGroupHandle_t bt_spp_client_get_event_group(void);

#endif