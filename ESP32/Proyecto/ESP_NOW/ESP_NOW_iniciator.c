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

static uint8_t peer_mac[ESP_NOW_ETH_ALEN] = {0xb0, 0xa7, 0x32, 0x34, 0x54, 0x5c}; // add del otro ESP
// b0:a7:32:34:54:5c add ESP 2
// c8:f0:9e:ec:0d:18 add ESP 1

static const char *TAG = "esp_now_init";
static esp_err_t init_wifi(void) // Inicializa el wifi
{
    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT(); // En la documentacion Se recomienda utilizar esta macro para configurar el wifi por defecto

    esp_netif_init();
    esp_event_loop_create_default();
    nvs_flash_init();
    esp_wifi_init(&wifi_init_config);
    esp_wifi_set_mode(WIFI_MODE_STA); // EL modo de operacion usamos station, pero también funciona con AP
    esp_wifi_set_storage(WIFI_STORAGE_FLASH);
    esp_wifi_start(); // inicializa wifi

    ESP_LOGI(TAG, "wifi init complete");
    return ESP_OK;
}

void recv_cb(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len) // funcion para recibir
{
    ESP_LOGI(TAG, "Data recived: " MACSTR "%s", MAC2STR(esp_now_info->src_addr), data);
}

void send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) // funcion para enviar
{
    if (status == ESP_NOW_SEND_SUCCESS)
    {
        ESP_LOGI(TAG, "ESP_NOW_SEND_SUCCESS");
    }
    else
    {
        {
            ESP_LOGW(TAG, "ESP_NOW_SEND_FAIL");
        }
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
    esp_now_peer_info_t esp_now_peer_info = {};
    memcpy(esp_now_peer_info.peer_addr, peer_mac, ESP_NOW_ETH_ALEN);
    esp_now_peer_info.channel = ESP_CHANNEL;
    esp_now_peer_info.ifidx = ESP_IF_WIFI_STA;

    esp_now_add_peer(&esp_now_peer_info);
    return ESP_OK;
}

static esp_err_t esp_now_send_data(const uint8_t *peer_addr, const uint8_t *data, size_t len)
{
    esp_now_send(peer_addr, data, len);
    return ESP_OK;
}

void app_main(void)
{
    ESP_ERROR_CHECK(init_wifi());             // funcion init wifi
    ESP_ERROR_CHECK(init_esp_now());          // funcion init esp_now
    ESP_ERROR_CHECK(register_peer(peer_mac)); // funcion register peer

    uint8_t dataR [] = "R";
    uint8_t dataG [] = "G";
    uint8_t dataB [] = "B";
    
    uint8_t count = 0;


    while(true)
    {
        count ++;
        if(count > 2)
        {
            count = 0;
        }

        switch (count)
        {
        case 0:
            esp_now_send_data(peer_mac, dataR, 32);
            break;
        
        case 1:
            esp_now_send_data(peer_mac, dataG, 32);
            break;

        case 2:
            esp_now_send_data(peer_mac, dataB, 32);
            break;

        default:
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));

    }

}

