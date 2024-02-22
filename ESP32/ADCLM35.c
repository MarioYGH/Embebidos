/*
Programa ESP32 ADC
date created: 21/02/24
last modified: 21/02/24
*/

#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h" //Para hacer lecturas Oneshot o continuas
//#include "esp_adc/adc_cali.h"  //Solo poner si se va a calibrar el ADC

#define ADC1_CHAN0 ADC_CHANNEL_4 //7 Pin 32
#define ADC_ATTEN ADC_ATTEN_DB_11

adc_oneshot_unit_handle_t adc1_handle;

static int adc_raw; 
static float voltage;

esp_err_t config_ADC();
esp_err_t get_ADC_value();


void app_main(void)
{
    config_ADC(); 

    while (true) {
        get_ADC_value();

        vTaskDelay(500/ portTICK_PERIOD_MS); //Aumentamos el tiempo para darle más chance de convertir
    } 

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
    printf("Raw data: %d\n", adc_raw);

    voltage = ((adc_raw * 5.0 / 4095.0)*100.0); //Aca le ponemos *100 y lo pasa a grados xd
    printf("Grados: %2.2f C\n", voltage);

    return ESP_OK;

}
