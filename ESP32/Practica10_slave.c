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
#include "freertos/queue.h"

#define UART_PORT UART_NUM_1
#define BUF_SIZE 1024 
#define TASK_MEMORY 2028
#define TXD_PIN GPIO_NUM_1
#define RXD_PIN GPIO_NUM_3
static QueueHandle_t uart_queue;

static const char *TAG = "UART";
int key;

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

    uart_event_t event;
    char *data = (char *) malloc(BUF_SIZE); //puntero para almacenar la data que llega

    while (true) {

        if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY)){
        bzero(data, BUF_SIZE); //Borramos el espacio de memoria que este en data

        switch (event.type)
        {
        case UART_DATA:
            uart_read_bytes(UART_PORT, data, event.size, pdMS_TO_TICKS(100));
            uart_write_bytes(UART_PORT, (const char *) data, event.size);
            uart_flush(UART_PORT);

            ESP_LOGI(TAG, "Data recived: %s",data); //imprimimos en consola lo que acaba de llegar

            for (size_t i = 0; i < event.size - 2; i++)
            {
                key = atoi(data);

                switch (key)
                {
                case 1660: //Rojo
                    ESP_ERROR_CHECK(stop());
                    vTaskDelay(pdMS_TO_TICKS(5000));
                    break;

                case 1500: //VErde 1500
                    ESP_ERROR_CHECK(forward());
                    vTaskDelay(pdMS_TO_TICKS(5000));
                    ESP_ERROR_CHECK(stop());
                    break;
                
                case 1590: //Azul
                    ESP_ERROR_CHECK(reverse());
                    vTaskDelay(pdMS_TO_TICKS(5000));
                    ESP_ERROR_CHECK(stop());
                    break;
                
                default:
                    ESP_ERROR_CHECK(stop());
                    break;
                }
            }

            break;
        
        default:
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
        .source_clk = UART_SCLK_APB,
    };

    uart_param_config(UART_PORT, &uart_config);

    uart_set_pin(UART_PORT, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    uart_driver_install(UART_PORT, BUF_SIZE, BUF_SIZE, 5, &uart_queue, 0);
    xTaskCreate(uart_task, "uart_task", TASK_MEMORY, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "init uart complete");

    return ESP_OK;
}

void app_main(){
    ESP_ERROR_CHECK(pinout_initialize());
    uart_initialize(); 
}
