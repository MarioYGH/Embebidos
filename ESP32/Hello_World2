//Programa para el ESP32 que enciende un LED en el pin 2 a modo de "Hola mundo"
/*Autor: Mario García
Fecha de creación: 13/02/24
Fecha de modificación
correo: marioygh15@gmail.com
*/
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LED 2

//Variables globales

// Funciones

void app_main(void){
    gpio_reset_pin(LED);
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);
    int state = 0;
    while(1){
        state = !state;
        gpio_set_level(LED, state);
        vTaskDelay(pdMS_TO_TICKS(500));
   
     
    }
}
