#include <stdio.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#define ESP_CHANNEL 1
#define ESP_NOW_PMK "pmk1234567890123"
#define ESP_NOW_LMK "lmk1234567890123"
#define BUTTON GPIO_NUM_18

#define UART_PORT_NUM UART_NUM_1
#define TX_BUF_SIZE 1024
#define TXD_PIN GPIO_NUM_1
#define RXD_PIN GPIO_NUM_3

static uint8_t peer_mac_1[ESP_NOW_ETH_ALEN] = {0x08, 0xd1, 0xf9, 0xe7, 0x9f, 0xd8};
static uint8_t peer_mac_2[ESP_NOW_ETH_ALEN] = {0x08, 0xd1, 0xf9, 0xe7, 0x96, 0xb8};

static const char *TAG = "esp_now_init";
static uint8_t count = 0;
static float temperature = 0.0;
static float humidity = 0.0;
static float voltage = 0.0;

void connect_wifi(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    if (esp_event_loop_create_default() != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(esp_event_loop_create_default());
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi initialized successfully");
}

static esp_err_t init_wifi(void) {
    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_init_config));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "wifi init complete");
    return ESP_OK;
}

esp_err_t uart_initialize() {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, TX_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    return ESP_OK;
}

void recv_cb(const uint8_t *mac_addr, const uint8_t *data, int data_len)
{
    float received_temp = 0.0;
    float received_hum = 0.0;
    float received_voltage = 0.0;
    int matches;

    char temp_buffer[data_len + 1];
    strncpy(temp_buffer, (char *)data, data_len);
    temp_buffer[data_len] = '\0';

    matches = sscanf(temp_buffer, "Temperature: %f, Humidity: %f, Voltage: %f", &received_temp, &received_hum, &received_voltage);

    if (matches == 3) {
        ESP_LOGI(TAG, "Data received from: " MACSTR ", Temperature: %.2f, Humidity: %.2f, Voltage: %.2f", MAC2STR(mac_addr), received_temp, received_hum, received_voltage);

        temperature = received_temp;
        humidity = received_hum;
        voltage = received_voltage;

        char uart_data[100];
        snprintf(uart_data, sizeof(uart_data), "/*%2.2f, %2.2f, %2.2f*/", temperature, humidity, voltage);
        uart_write_bytes(UART_PORT_NUM, uart_data, strlen(uart_data));
    } else {
        ESP_LOGW(TAG, "Invalid format: sscanf could not parse Temperature, Humidity, and Voltage");
    }
}

void send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    if (status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGI(TAG, "ESP_NOW_SEND_SUCCESS");
    } else {
        ESP_LOGW(TAG, "ESP_NOW_SEND_FAIL");
    }
}

static esp_err_t init_esp_now(void)
{
    esp_now_init();
    esp_now_register_recv_cb(recv_cb);
    esp_now_register_send_cb(send_cb);
    esp_now_set_pmk((const uint8_t*)ESP_NOW_PMK);

    ESP_LOGI(TAG, "esp now init complete");
    return ESP_OK;
}

static esp_err_t register_peer(uint8_t *peer_addr)
{
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, peer_addr, ESP_NOW_ETH_ALEN);
    peer_info.channel = ESP_CHANNEL;
    peer_info.ifidx = ESP_IF_WIFI_STA;
    memcpy(peer_info.lmk, ESP_NOW_LMK, ESP_NOW_KEY_LEN);
    peer_info.encrypt = true;

    esp_now_add_peer(&peer_info);
    return ESP_OK;
}

static esp_err_t esp_now_send_data(const uint8_t *peer_addr, const uint8_t *data, size_t len)
{
    esp_now_send(peer_addr, data, len);
    return ESP_OK;
}

static void gpio_task_example(void* arg)
{
    uint8_t data1[] = "1";
    uint8_t data2[] = "2";
    uint8_t data3[] = "3";

    while (true)
    {
        if (count > 3)
        {
            count = 1;
        }
        switch (count)
        {
            case 1:
                esp_now_send_data(peer_mac_1, data1, sizeof(data1));
                esp_now_send_data(peer_mac_2, data1, sizeof(data1));
                break;

            case 2:
                esp_now_send_data(peer_mac_1, data2, sizeof(data2));
                esp_now_send_data(peer_mac_2, data2, sizeof(data2));
                break;

            case 3:
                esp_now_send_data(peer_mac_1, data3, sizeof(data3));
                esp_now_send_data(peer_mac_2, data3, sizeof(data3));
                break;

            default:
                break;
        }

        ESP_LOGI(TAG, "Button pressed, count: %d", count);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void isr_handler(void *args)
{
    count++;
}

esp_err_t init_iris()
{
    gpio_config_t io_conf;
    io_conf.pin_bit_mask = (1ULL << BUTTON);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON, isr_handler, NULL);

    return ESP_OK;
}

void app_main(void)
{
    init_wifi();
    connect_wifi();
    uart_initialize();
    init_iris();
    init_esp_now();
    register_peer(peer_mac_1);
    register_peer(peer_mac_2);

    xTaskCreate(gpio_task_example, "gpio_task_example", 4096, NULL, 10, NULL);
}
