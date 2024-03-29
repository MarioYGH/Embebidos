//Programa para el ESP32 que enciende un LED en el pin 2 a modo de "Hola mundo"
/*Autor: Mario García
Fecha de creación: 12/02/24
Fecha de modificación
correo: marioygh15@gmail.com
*/
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LED 2
#define ON 1
#define OFF 0

//Variables globales

// Funciones

void app_main(void){
    gpio_reset_pin(LED); //Inicializa pin
    gpio_set_direction(LED, GPIO_MODE_OUTPUT); //Declara pin como entrada o salida, salida en este caso

    while(1){
        gpio_set_level(LED, ON);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(LED,OFF);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
//Para subir el programa al ESP32, usamos el rayito, y seleccionamos 
