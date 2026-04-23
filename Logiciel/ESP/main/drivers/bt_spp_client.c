/********* bt_spp_client.c **********/
/* Jonathan Marois 20/03/2026 */
/********* bt_spp_client.c **********/
/* Jonathan Marois 20/03/2026 */

#include "bt_spp_client.h"

#include <string.h>

#include "esp_log.h"
#include "esp_err.h"

#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_spp_api.h"

static const char *TAG = "BT_SPP_CLIENT";

static esp_bd_addr_t s_peer_bd_addr = {0};

static char s_pin_code[BT_PIN_MAX_LEN + 1] = "1234";
static uint8_t s_pin_len = 4;

static volatile uint32_t s_spp_handle = 0;
static volatile bool s_spp_congested = false;
static EventGroupHandle_t s_bt_event_group = NULL;

static void bt_spp_client_connect(void)
{
    esp_err_t err = esp_spp_connect(ESP_SPP_SEC_NONE,
                                    ESP_SPP_ROLE_MASTER,
                                    1,
                                    s_peer_bd_addr);

    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "esp_spp_connect failed: %s", esp_err_to_name(err));
    }
}

static void bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event)
    {
        case ESP_BT_GAP_AUTH_CMPL_EVT:
            ESP_LOGI(TAG, "Authentification terminee");
            break;

        case ESP_BT_GAP_PIN_REQ_EVT:
        {
            esp_bt_pin_code_t pin_reply = {0};

            memcpy(pin_reply, s_pin_code, s_pin_len);
            esp_bt_gap_pin_reply(param->pin_req.bda, true, s_pin_len, pin_reply);

            ESP_LOGI(TAG, "PIN envoye");
            break;
        }

        default:
            break;
    }
}

static void spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event)
    {
        case ESP_SPP_INIT_EVT:
            ESP_LOGI(TAG, "SPP init OK");
            esp_bt_dev_set_device_name("LAB7_TX");
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            bt_spp_client_connect();
            break;

        case ESP_SPP_OPEN_EVT:
            s_spp_handle = param->open.handle;
            s_spp_congested = false;
            xEventGroupSetBits(s_bt_event_group, BT_CONNECTED_BIT);
            ESP_LOGI(TAG, "Connecte au module Bluetooth");
            break;

        case ESP_SPP_CLOSE_EVT:
            s_spp_handle = 0;
            s_spp_congested = false;
            xEventGroupClearBits(s_bt_event_group, BT_CONNECTED_BIT);
            ESP_LOGW(TAG, "Connexion fermee");
            bt_spp_client_connect();
            break;

        case ESP_SPP_CONG_EVT:
            s_spp_congested = param->cong.cong;
            ESP_LOGI(TAG, "Congestion = %d", s_spp_congested);
            break;

        case ESP_SPP_WRITE_EVT:
            ESP_LOGD(TAG, "Write termine");
            break;

        default:
            break;
    }
}

void bt_spp_client_init(const esp_bd_addr_t peer_addr, const char *pin_code)
{
    esp_err_t ret;
    size_t len;

    if (peer_addr == NULL)
    {
        ESP_LOGE(TAG, "peer_addr NULL");
        return;
    }

    memcpy(s_peer_bd_addr, peer_addr, ESP_BD_ADDR_LEN);

    if (pin_code != NULL)
    {
        len = strlen(pin_code);

        if (len == 0 || len > BT_PIN_MAX_LEN)
        {
            ESP_LOGE(TAG, "PIN invalide");
            return;
        }

        memcpy(s_pin_code, pin_code, len);
        s_pin_code[len] = '\0';
        s_pin_len = (uint8_t)len;
    }

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    else
    {
        ESP_ERROR_CHECK(ret);
    }

    s_bt_event_group = xEventGroupCreate();
    configASSERT(s_bt_event_group != NULL);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT));

    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_bt_gap_register_callback(bt_gap_cb));
    ESP_ERROR_CHECK(esp_spp_register_callback(spp_cb));

    esp_spp_cfg_t spp_cfg = {
        .mode = ESP_SPP_MODE_CB,
        .enable_l2cap_ertm = false,
        .tx_buffer_size = 0,
    };
    ESP_ERROR_CHECK(esp_spp_enhanced_init(&spp_cfg));
}

esp_err_t bt_spp_client_send_text(const char *text)
{
    EventBits_t bits;

    if (text == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_bt_event_group == NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    bits = xEventGroupGetBits(s_bt_event_group);
    if ((bits & BT_CONNECTED_BIT) == 0)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_spp_handle == 0)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_spp_congested)
    {
        return ESP_ERR_INVALID_STATE;
    }

    return esp_spp_write((uint32_t)s_spp_handle,
                         strlen(text),
                         (uint8_t *)text);
}

bool bt_spp_client_is_connected(void)
{
    if (s_bt_event_group == NULL)
    {
        return false;
    }

    return (xEventGroupGetBits(s_bt_event_group) & BT_CONNECTED_BIT) != 0;
}

EventGroupHandle_t bt_spp_client_get_event_group(void)
{
    return s_bt_event_group;
}