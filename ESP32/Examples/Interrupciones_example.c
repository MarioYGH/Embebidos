/*
Autor: Mario García
Programa ESP32 Interrupciones
date created: 26/02/24
last modified: 26/02/24
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

const char *TAG = "Test log"; //Apuntador a caracter

void app_main(void)
{
    while(true){
        ESP_LOGE(TAG, "Este es un error");
        ESP_LOGI(TAG, "Este es un informativo");
        ESP_LOGW(TAG, "Este es un warning");

        vTaskDelay(pdMS_TO_TICKS(1000));
    }


}
