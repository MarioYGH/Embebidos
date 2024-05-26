#include <stdio.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"

#define ESP_CHANNEL 1

static uint8_t peer_mac_1[ESP_NOW_ETH_ALEN] = {0xb0, 0xa7, 0x32, 0x34, 0x54, 0x5c}; // MAC del ESP 2
static uint8_t peer_mac_2[ESP_NOW_ETH_ALEN] = {0xd8, 0xbc, 0x38, 0xf8, 0xb5, 0x94}; // MAC del ESP 3

static const char *TAG = "esp_now_init";

static esp_err_t init_wifi(void)
{
    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    esp_netif_init();
    esp_event_loop_create_default();
    nvs_flash_init();
    esp_wifi_init(&wifi_init_config);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_storage(WIFI_STORAGE_FLASH);
    esp_wifi_start();
    ESP_LOGI(TAG, "wifi init complete");
    return ESP_OK;
}

void recv_cb(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len)
{
    ESP_LOGI(TAG, "Data received from: " MACSTR ", Data: %s", MAC2STR(esp_now_info->src_addr), data);
}

void send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS)
    {
        ESP_LOGI(TAG, "ESP_NOW_SEND_SUCCESS");
    }
    else
    {
        ESP_LOGW(TAG, "ESP_NOW_SEND_FAIL");
    }
}

static esp_err_t init_esp_now(void)
{
    esp_now_init();
    esp_now_register_recv_cb(recv_cb);
    esp_now_register_send_cb(send_cb);
    ESP_LOGI(TAG, "esp now init complete");
    return ESP_OK;
}

static esp_err_t register_peer(uint8_t *peer_addr)
{
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, peer_addr, ESP_NOW_ETH_ALEN);
    peer_info.channel = ESP_CHANNEL;
    peer_info.ifidx = ESP_IF_WIFI_STA;

    esp_now_add_peer(&peer_info);
    return ESP_OK;
}

static esp_err_t esp_now_send_data(const uint8_t *peer_addr, const uint8_t *data, size_t len)
{
    esp_now_send(peer_addr, data, len);
    return ESP_OK;
}

void app_main(void)
{
    ESP_ERROR_CHECK(init_wifi());
    ESP_ERROR_CHECK(init_esp_now());
    ESP_ERROR_CHECK(register_peer(peer_mac_1));
    ESP_ERROR_CHECK(register_peer(peer_mac_2));

    uint8_t dataR[] = "R";
    uint8_t dataG[] = "G";
    uint8_t dataB[] = "B";

    uint8_t count = 0;

    while (true)
    {
        count++;
        if (count > 2)
        {
            count = 0;
        }

        switch (count)
        {
        case 0:
            esp_now_send_data(peer_mac_1, dataR, sizeof(dataR));
            esp_now_send_data(peer_mac_2, dataR, sizeof(dataR));
            break;

        case 1:
            esp_now_send_data(peer_mac_1, dataG, sizeof(dataG));
            esp_now_send_data(peer_mac_2, dataG, sizeof(dataG));
            break;

        case 2:
            esp_now_send_data(peer_mac_1, dataB, sizeof(dataB));
            esp_now_send_data(peer_mac_2, dataB, sizeof(dataB));
            break;

        default:
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
