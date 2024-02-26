/*
Autor: Mario García
Programa ESP32 Interrupciones
date created: 26/02/24
last modified: 26/02/24
*/

#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define BUTTON_ISR 27

const char *TAG = "Interrupt example"; //Apuntador a caracter
uint8_t counter = 0;

esp_err_t init_iris();
void isr_handler(void *args);

void app_main(void)
{
    init_iris();

    while(true){
        ESP_LOGI(TAG, "Counter value: %d", counter);   
    }
}

esp_err_t init_iris(){
    gpio_config_t GPIO_config;
    GPIO_config.pin_bit_mask = (1ULL << BUTTON_ISR); //Inicializa el botón desde aquí
    GPIO_config.mode = GPIO_MODE_DEF_INPUT;
    GPIO_config.pull_up_en = GPIO_PULLUP_DISABLE; //Se pueden activar, esto va a depender de si se quiere poner el pull-up en fisico o no
    GPIO_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    GPIO_config.intr_type = GPIO_INTR_NEGEDGE; //interrupt by falling edge //flanco de bajada 


    //configure GPIO´s mode, pullup, pulldown, intrType
    gpio_config(&GPIO_config);

    gpio_install_isr_service(0); //habilita global interrupts
    gpio_isr_handler_add(BUTTON_ISR, isr_handler, NULL); //pin y funcion a ejecutar


    return ESP_OK;

}

void isr_handler(void *args){

counter ++;
    
}
