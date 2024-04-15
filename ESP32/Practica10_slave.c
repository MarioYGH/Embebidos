/*
Author: Mario García - marioygh15@gmail.com
Programa ESP32, Practica 10, comunicacion serial entre ESPs, este se encarga del puente H
date created: 27/03/24
last modified: 01/04/24
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
long key;

///////////////////Puente H
#define M1_A 32
#define M1_B 33
#define L 0
#define H 1
//char* R_str = "2961";
//char* B_str = "2800";
//char* G_str = "2920";

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
static void uart_task(void *pvParameters){
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

        uart_write_bytes(UART_PORT, (const char *) data, len);
        uart_flush(UART_PORT);
 
        ESP_LOGI(TAG, "Data recived: %s",data); //imprimimos en consola lo que acaba de llegar
    

        char data_str[len + 1]; // Buffer para almacenar los datos como una cadena terminada en null
        memcpy(data_str, data, len); // Copiar los datos recibidos al buffer de cadena
        //data_str[len] = '\0'; // Asegurarse de que la cadena esté terminada en null
        data_str[len - 2] = '\0';
        char *endptr;
        key = strtol(data_str, &endptr, 10);

        if (endptr == data_str) {
        printf("Error: No se pudo convertir la cadena a un entero.\n");
        }

            if(key == 2961){
            ESP_ERROR_CHECK(stop());
            vTaskDelay(pdMS_TO_TICKS(5000));
            }

            if(key == 2920){
            ESP_ERROR_CHECK(forward());
            vTaskDelay(pdMS_TO_TICKS(5000));
            }

            if(key == 4095){
            ESP_ERROR_CHECK(reverse());
            vTaskDelay(pdMS_TO_TICKS(5000));
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
    ESP_ERROR_CHECK(pinout_initialize());
    uart_initialize(); 
}
