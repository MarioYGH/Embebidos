/*
Author: Mario García - marioygh15@gmail.com
Programa ESP32, Practica 10, comunicacion serial entre ESPs, se encarga de escribir el valor del sensor RGB
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
#include "driver/mcpwm_prelude.h"
#include "esp_adc/adc_oneshot.h"

#define UART_PORT UART_NUM_1
#define BUF_SIZE 1024 
#define TASK_MEMORY 2028
#define TXD_PIN GPIO_NUM_1
#define RXD_PIN GPIO_NUM_3

static const char *TAG = "UART";

///////////////////ADC
#define ADC1_CHAN1 ADC_CHANNEL_4 // Pin 32
#define ADC_ATTEN ADC_ATTEN_DB_11
int adc_raw1;  

int voltage1;

esp_err_t config_ADC();
esp_err_t get_ADC_value();

adc_oneshot_unit_handle_t adc1_handle;

esp_err_t uart_initialize(); //Uart

/////////////////// Funciones ADC
esp_err_t config_ADC() {
    
     //ADC init
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };

    adc_oneshot_new_unit(&init_config1, &adc1_handle);

    //ADC config
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT, //ancho de banda
        .atten = ADC_ATTEN, //Atenuacion
    };


    adc_oneshot_config_channel(adc1_handle, ADC1_CHAN1, &config);
   

    return ESP_OK; 
}

esp_err_t get_ADC_value(){
    
    adc_oneshot_read(adc1_handle, ADC1_CHAN1, &adc_raw1);
    
    
    voltage1 = adc_raw1;   

    return ESP_OK;
}

/////////////////////////////////Uart
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
    config_ADC(); 
    uart_initialize(); 
    char str[50];

    while (1) {

        get_ADC_value();

        const int len = strlen(str); //cuenta tamaño de la cadena //len tiene el tamaño de la cadena
        uart_write_bytes(UART_PORT, str, len); //longitud de la cadena, se puede poner len-1, etc para cortar un caracter
        sprintf(str, "/*%d", voltage1);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
