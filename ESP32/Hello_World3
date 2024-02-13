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

esp_err_t pin_initialize(); //Inicializamos funcion

void app_main(void){ 

    int state = 0;
    pin_initialize();  //Llamamos y ejecutamos funcion 

    while(1){  //Cambia los estados entre 0 y 1 cada 500ms de forma indefinida 
        state = !state;
        gpio_set_level(LED, state);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

esp_err_t pin_initialize(){ //Función para inicializar pin de forma automática 
    gpio_reset_pin(LED); //Inicializa pin
    gpio_set_direction(LED, GPIO_MODE_OUTPUT); //Salida

    return ESP_OK; //Retorna al codigo
}
//Para subir el programa al ESP32, usamos el rayito, y seleccionamos 
