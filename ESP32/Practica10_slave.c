/*
Author: Mario García - marioygh15@gmail.com
Programa ESP32, Practica 10, comunicacion serial entre ESPs, este se encarga del puente H
date created: 27/03/24
last modified: 27/03/24
*/

#include <stdio.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#define UART_PORT UART_NUM_1
#define BUF_SIZE 1024 
#define TASK_MEMORY 2028
#define TXD_PIN GPIO_NUM_1
#define RXD_PIN GPIO_NUM_3

static const char *TAG = "UART";

///////////////////Puente H
#define M1_A 32
#define M1_B 33
#define L 0
#define H 1

esp_err_t pinout_initialize();
esp_err_t forward();
esp_err_t reverse();
esp_err_t stop();

esp_err_t uart_initialize(); //Uart

/////////////////// Funciones Puente H
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

/////////////////////////////////Uart
static void uart_task(void *pvParameters)
{
uint8_t *data = (uint8_t *) malloc(BUF_SIZE); //puntero para almacenar la data que llega

    while (true) {
        /* Clear space memory */
        bzero(data, BUF_SIZE); //Borramos el espacio de memoria que este en data

        /* Read data from the UART */
        int len = uart_read_bytes(UART_PORT, data, BUF_SIZE, pdMS_TO_TICKS(100));
        if (len==0) //si el tamaño de lo que llego es cero, continua
        {
            continue;
        }
        // si es diferente de cero lo volvemos a leer
        /* Write data back to the UART */

        uart_write_bytes(UART_PORT, (const char *) data, len);
        uart_flush(UART_PORT);
 
        ESP_LOGI(TAG, "Data recived: %s",data); //imprimimos en consola lo que acaba de llegar
    

        for (size_t i=0; i < len -2; i++) //for por cada elemento que llegue en Data y quitamos salto de /r/n
        {
            char value = data[i];

            switch (value)
            {
            case 'R': //Rojo
                ESP_ERROR_CHECK(stop());
                break;

            case 'G': //Verde
                ESP_ERROR_CHECK(forward());
                break;

            case 'B': //Azul
                ESP_ERROR_CHECK(reverse());
                break;
            
            default:
                ESP_ERROR_CHECK(stop());
                break;
            }

        }
    }
}



esp_err_t uart_initialize(){
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    // We won't use a buffer for sending data.
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, BUF_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    return ESP_OK;
}

void app_main(){
    
    uart_initialize(); 
}
