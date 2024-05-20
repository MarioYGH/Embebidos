#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <esp_log.h>
#include <driver/gpio.h>

#define SHT1x_DATA_PIN  21  // GPIO pin for data
#define SHT1x_CLK_PIN   22  // GPIO pin for clock

static const char *TAG = "SHT1x";

void sht1x_delay()
{
    esp_rom_delay_us(1); // Small delay for timing
}

void sht1x_set_data_output()
{
    gpio_set_direction(SHT1x_DATA_PIN, GPIO_MODE_OUTPUT);
}

void sht1x_set_data_input()
{
    gpio_set_direction(SHT1x_DATA_PIN, GPIO_MODE_INPUT);
}

void sht1x_write_bit(uint8_t bit)
{
    gpio_set_level(SHT1x_DATA_PIN, bit);
    sht1x_delay();
    gpio_set_level(SHT1x_CLK_PIN, 1);
    sht1x_delay();
    gpio_set_level(SHT1x_CLK_PIN, 0);
    sht1x_delay();
}

uint8_t sht1x_read_bit()
{
    gpio_set_level(SHT1x_CLK_PIN, 1);
    sht1x_delay();
    uint8_t bit = gpio_get_level(SHT1x_DATA_PIN);
    gpio_set_level(SHT1x_CLK_PIN, 0);
    sht1x_delay();
    return bit;
}

void sht1x_send_command(uint8_t command)
{
    sht1x_set_data_output();
    gpio_set_level(SHT1x_DATA_PIN, 0);
    gpio_set_level(SHT1x_CLK_PIN, 1);
    sht1x_delay();
    gpio_set_level(SHT1x_CLK_PIN, 0);
    sht1x_delay();
    gpio_set_level(SHT1x_CLK_PIN, 1);
    sht1x_delay();
    gpio_set_level(SHT1x_DATA_PIN, 1);
    sht1x_delay();
    gpio_set_level(SHT1x_CLK_PIN, 0);
    sht1x_delay();

    for (int i = 0; i < 8; i++) {
        sht1x_write_bit((command >> (7 - i)) & 0x01);
    }

    sht1x_set_data_input();
    sht1x_read_bit(); // ACK
    sht1x_set_data_output();
    sht1x_write_bit(0); // Additional clock pulse for ACK
    sht1x_set_data_input();
}

uint16_t sht1x_read_data()
{
    uint16_t data = 0;
    sht1x_set_data_input();
    for (int i = 0; i < 16; i++) {
        data = (data << 1) | sht1x_read_bit();
    }
    sht1x_set_data_output();
    sht1x_write_bit(0); // ACK
    sht1x_set_data_input();
    sht1x_read_bit(); // Additional clock pulse for ACK
    return data;
}

static void sht1x_task(void *pvParameter)
{
    while (1)
    {
        sht1x_send_command(0x03); // Measure temperature command
        vTaskDelay(pdMS_TO_TICKS(200));  // Wait for measurement to complete
        uint16_t temp_raw = sht1x_read_data();
        float temp_c = -40.1 + 0.01 * temp_raw;
        ESP_LOGI(TAG, "Temperature: %.2f °C", temp_c);

        sht1x_send_command(0x05); // Measure humidity command
        vTaskDelay(pdMS_TO_TICKS(200));  // Wait for measurement to complete
        uint16_t humid_raw = sht1x_read_data();
        float humid_rh = -4 + 0.0405 * humid_raw - 0.0000028 * (humid_raw * humid_raw);
        ESP_LOGI(TAG, "Humidity: %.2f %%RH", humid_rh);

        vTaskDelay(pdMS_TO_TICKS(2000));  // Delay 2 seconds between measurements
    }
}

void app_main(void)
{
    gpio_set_direction(SHT1x_CLK_PIN, GPIO_MODE_OUTPUT);
    sht1x_set_data_output();
    xTaskCreate(sht1x_task, "sht1x_task", 2048, NULL, 5, NULL);
}
