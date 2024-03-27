#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"

#define UART_PORT UART_NUM_1
#define BUF_SIZE 1024
#define TASK_MEMORY 2028
#define TXD_PIN GPIO_NUM_1
#define RXD_PIN GPIO_NUM_3

esp_err_t uart_initialize();
static void rx_task(void *arg);

const char *TAG = "RX UART";

void app_main(){
    ESP_ERROR_CHECK(uart_initialize());

    xTaskCreate(rx_task, "uart_rx_task", TASK_MEMORY, NULL, configMAX_PRIORITIES-1, NULL);
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

static void rx_task(void *arg){
    // Configure a temporary buffer for the incoming data
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE);

    while (true) {
        /* Clear space memory */
        bzero(data, BUF_SIZE);

        /* Read data from the UART */
        int len = uart_read_bytes(UART_PORT, data, BUF_SIZE, pdMS_TO_TICKS(100));
        
        /* Write data back to the UART */
        uart_write_bytes(UART_PORT, (const char *) data, len);
    }
}
