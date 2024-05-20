#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"

static const char *TAG = "i2c-sht1x-example";

#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define SHT1x_SENSOR_ADDR           0x40        /*!< Slave address of the SHT1x sensor */

// Commands for SHT1x
#define SHT1x_MEASURE_TEMP          0xF3        /*!< Command to measure temperature */
#define SHT1x_MEASURE_HUMID         0xF5        /*!< Command to measure humidity */

/*
 * @brief Write a command to the SHT1x sensor
 */
static esp_err_t sht1x_command_write(uint8_t command)
{
    return i2c_master_write_to_device(I2C_MASTER_NUM, SHT1x_SENSOR_ADDR, &command, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/*
 * @brief Read a sequence of bytes from the SHT1x sensor
 */
static esp_err_t sht1x_data_read(uint8_t *data, size_t len)
{
    return i2c_master_read_from_device(I2C_MASTER_NUM, SHT1x_SENSOR_ADDR, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/*
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void app_main(void)
{
    uint8_t data[2];
    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    // Measure temperature
    ESP_ERROR_CHECK(sht1x_command_write(SHT1x_MEASURE_TEMP));
    vTaskDelay(50 / portTICK_PERIOD_MS);  // Wait for measurement to complete
    ESP_ERROR_CHECK(sht1x_data_read(data, 2));
    uint16_t temp_raw = (data[0] << 8) | data[1];
    ESP_LOGI(TAG, "Raw Temperature = %d", temp_raw);

    // Measure humidity
    ESP_ERROR_CHECK(sht1x_command_write(SHT1x_MEASURE_HUMID));
    vTaskDelay(50 / portTICK_PERIOD_MS);  // Wait for measurement to complete
    ESP_ERROR_CHECK(sht1x_data_read(data, 2));
    uint16_t humid_raw = (data[0] << 8) | data[1];
    ESP_LOGI(TAG, "Raw Humidity = %d", humid_raw);

    ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}
