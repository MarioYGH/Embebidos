#include <stdio.h>
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "dht.h"

#define DHTPIN GPIO_NUM_26
#define ADC1_CHAN0 ADC_CHANNEL_7 //7 Pin 32
#define ADC_ATTEN ADC_ATTEN_DB_11
#define BUZZER 22

adc_oneshot_unit_handle_t adc1_handle;

static int adc_raw; 
static float voltage;
float OFFSET = 0.5780859231; //desviación estandar 

esp_err_t config_ADC();
esp_err_t get_ADC_value();
esp_err_t pin_initialize();

const char *TAG = "DHT22 SENSOR";

gpio_num_t dht_gpio = DHTPIN; //Digital pin connected to the DHT
dht_sensor_type_t sensor_type = DHT_TYPE_AM2301; //Para DHT11 -> DHT_TYPE_DH11

esp_err_t temperature_task();

void app_main(void)
{
    pin_initialize();
    config_ADC(); 

    while(true){
        get_ADC_value();

        //vTaskDelay(500/ portTICK_PERIOD_MS);

        ESP_ERROR_CHECK(temperature_task());

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

esp_err_t temperature_task(){
    float humidity = 0;
    float temperature = 0;
    if (dht_read_float_data(sensor_type, dht_gpio, &humidity, &temperature) == ESP_OK)
        ESP_LOGI(TAG, "humidity: %f%% Temp: %fC", humidity, temperature);
    else 
        ESP_LOGE(TAG, "Could not read data from sensor");

    return ESP_OK;
}

/////////////////////////////////////////////////////////////////

esp_err_t pin_initialize() {

    gpio_reset_pin(BUZZER);
    gpio_set_direction(BUZZER, GPIO_MODE_OUTPUT);

    return ESP_OK;
}

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

    adc_oneshot_config_channel(adc1_handle, ADC1_CHAN0, &config);
    
    return ESP_OK; 
}

esp_err_t get_ADC_value(){

    adc_oneshot_read(adc1_handle, ADC1_CHAN0, &adc_raw);
    //printf("Raw data: %d\n", adc_raw);

    voltage = (((adc_raw * 5.0 / 4095.0)*100.0)-OFFSET); //Aca le ponemos *100 y lo pasa a grados xd
    printf("Grados LM35: %2.2f C\n", voltage);
    //printf("%2.2f \n", voltage);

    return ESP_OK;

}
