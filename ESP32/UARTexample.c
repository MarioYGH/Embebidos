#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "string.h"
//#include "soc/uart_struct.h"

#define UART_PORT_NUM UART_NUM_1
#define TX_BUF_SIZE 1024
#define TXD_PIN GPIO_NUM_1
#define RXD_PIN GPIO_NUM_3

esp_err_t uart_initialize();
static void tx_task(void *arg);

void app_main(){
    uart_initialize();

    xTaskCreate(tx_task, "uart_tx_task", TX_BUF_SIZE*2, NULL, configMAX_PRIORITIES-1, NULL);
}

esp_err_t uart_initialize(){
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    // We won't use a buffer for sending data.
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, TX_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    return ESP_OK;
}

static void tx_task(void *arg){
    const char *data = "Hello world";
    const int len = strlen(data);
    
    while (1) {
        uart_write_bytes(UART_PORT_NUM, data, len);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
