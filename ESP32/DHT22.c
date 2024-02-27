#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "dht.h"

#define DHTPIN GPIO_NUM_26

const char *TAG = "DHT22 SENSOR";

gpio_num_t dht_gpio = DHTPIN; //Digital pin connected to the DHT
dht_sensor_type_t sensor_type = DHT_TYPE_AM2301; //Para DHT11 -> DHT_TYPE_DH11

esp_err_t temperature_task();

void app_main(void)
{
    while(true){
        ESP_ERROR_CHECK(temperature_task());

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

esp_err_t temperature_task(){
    float humidity = 0;
    float temperature = 0;
    if (dht_read_float_data(sensor_type, dht_gpio, &humidity, &temperature) == ESP_OK)
        ESP_LOGI(TAG, "humidity: %f%% Temp: %fC", humidity, temperature);
    else 
        ESP_LOGE(TAG, "Could not read data from sensor");

    return ESP_OK;
}
