//ESP_now resp con casos
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
#define ESP_NOW_PMK "pmk1234567890123" //Cualquier texto de 16 caracteres Master primary key
#define ESP_NOW_LMK "lmk1234567890123" //local master key

static uint8_t peer_mac[ESP_NOW_ETH_ALEN] = {0xc8, 0xf0, 0x9e, 0xec, 0x0d, 0x18}; // add del otro ESP

static const char *TAG = "esp_now_resp";

static esp_err_t esp_now_send_data(const uint8_t *peer_addr, const uint8_t *data, size_t len);

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
    ESP_LOGI(TAG, "Data received: " MACSTR ", %s", MAC2STR(esp_now_info->src_addr), data);

    if (data_len >= 2) {
        switch (data[1]) {
            case '1':
                ESP_LOGI(TAG, "Received T1");
                // Lógica para T1
                break;
            case '2':
                ESP_LOGI(TAG, "Received T2");
                // Lógica para T2
                break;
            case '3':
                ESP_LOGI(TAG, "Received T3");
                // Lógica para T3
                break;
            default:
                ESP_LOGW(TAG, "Unknown data received");
                break;
        }
    }

    uint8_t *mac_addr = esp_now_info->src_addr;
    ESP_ERROR_CHECK(esp_now_send_data(mac_addr, data, data_len));
}

void send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) // funcion para enviar
{
    if (status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGI(TAG, "ESP_NOW_SEND_SUCCESS");
    } else {
        ESP_LOGW(TAG, "ESP_NOW_SEND_FAIL");
    }
}

static esp_err_t init_esp_now(void)
{
    esp_now_init();
    esp_now_register_recv_cb(recv_cb);
    esp_now_register_send_cb(send_cb);
    esp_now_set_pmk((const uint8_t*)ESP_NOW_PMK); //Para cifrar data con ESP_now no es suficiente con poner una clave

    ESP_LOGI(TAG, "esp now init complete");
    return ESP_OK;
}

static esp_err_t register_peer(uint8_t *peer_addr)
{
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, peer_mac, ESP_NOW_ETH_ALEN);
    peer_info.channel = ESP_CHANNEL;
    peer_info.ifidx = ESP_IF_WIFI_STA;
    memcpy(peer_info.lmk, ESP_NOW_LMK, ESP_NOW_KEY_LEN);
    peer_info.encrypt = true; //el true/false indican si se encripta o no la data

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
    ESP_ERROR_CHECK(init_wifi());             // funcion init wifi
    ESP_ERROR_CHECK(init_esp_now());          // funcion init esp_now
    ESP_ERROR_CHECK(register_peer(peer_mac)); // funcion register peer
}
