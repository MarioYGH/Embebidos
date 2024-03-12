#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"


#define M1_A 32
#define M1_B 33
#define L 0
#define H 1

esp_err_t pinout_initialize();
esp_err_t forward();
esp_err_t reverse();
esp_err_t stop();


void app_main(){

    ESP_ERROR_CHECK(pinout_initialize());

    while(1){
        ESP_ERROR_CHECK(forward());
        vTaskDelay(pdMS_TO_TICKS(3000)); //delay de 3000
        ESP_ERROR_CHECK(reverse()); // usamos errorcheck para ver si esta bien
        vTaskDelay(pdMS_TO_TICKS(3000));//delay de 3000
        ESP_ERROR_CHECK(stop());
        vTaskDelay(pdMS_TO_TICKS(3000));//delay de 3000
    }
}

esp_err_t pinout_initialize(){
    gpio_reset_pin(M1_A);
    gpio_set_direction(M1_A, GPIO_MODE_OUTPUT);

    gpio_reset_pin(M1_B);
    gpio_set_direction(M1_B, GPIO_MODE_OUTPUT);

    return ESP_OK;
}

esp_err_t forward(){ 
    gpio_set_level(M1_A, H);
    gpio_set_level(M1_B, L);

    return ESP_OK;
}

esp_err_t reverse(){
    gpio_set_level(M1_A, L);
    gpio_set_level(M1_B, H);

    return ESP_OK;
}

esp_err_t stop(){
    gpio_set_level(M1_A, L);
    gpio_set_level(M1_B, L);

    return ESP_OK;
}
