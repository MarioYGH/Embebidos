#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_system.h"

#include "SHT1x.h"
#include "SHT1x_platform.h"

#include "esp_http_client.h"
#include "connect_wifi.h"

static const char *TAG = "HTTP_CLIENT";
char *api_key = "0LCIV8KQ22AXZNR6";

void send_data_to_thingspeak(void *pvParameters) {
    SHT1x_Handler_t Handler = {0};
    SHT1x_Sample_t  Sample = {0};
    SHT1x_Platform_Init(&Handler);
    SHT1x_Init(&Handler);

    char *thingspeak_url = "https://api.thingspeak.com";
    char data[] = "/update?api_key=%s&field1=%.2f&field2=%.2f&field3=%.2f";
    char post_data[200];
    esp_err_t err;

    esp_http_client_config_t config = {
        .url = thingspeak_url,
        .method = HTTP_METHOD_GET,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_header(client, "Content-Type", "application/x-www-form-urlencoded");

    while (true) {
        SHT1x_ReadSample(&Handler, &Sample);
        ESP_LOGI(TAG, "Temperature: %.2f°C, Humidity: %.2f%%", Sample.TempCelsius, Sample.HumidityPercent);

        snprintf(post_data, sizeof(post_data), data, api_key, Sample.TempCelsius, Sample.HumidityPercent);
        ESP_LOGI(TAG, "post = %s", post_data);
        esp_http_client_set_url(client, post_data);

        err = esp_http_client_perform(client);

        if (err == ESP_OK) {
            int status_code = esp_http_client_get_status_code(client);
            if (status_code == 200) {
                ESP_LOGI(TAG, "Message sent Successfully");
            } else {
                ESP_LOGI(TAG, "Message sent Failed");
            }
        } else {
            ESP_LOGI(TAG, "Message sent Failed");
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    esp_http_client_cleanup(client);
    vTaskDelete(NULL);
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);

    connect_wifi();

    if (wifi_connect_status) {
        xTaskCreate(&send_data_to_thingspeak, "send_data_to_thingspeak", 8192, NULL, 6, NULL);
    }
}
