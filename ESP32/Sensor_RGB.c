#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
#include "esp_adc/adc_oneshot.h" //Para hacer lecturas Oneshot o continuas
#include "driver/gpio.h"
#include "dht.h"
#include "esp_err.h"
#include "driver/uart.h"
#include "string.h"
//#include "soc/uart_struct.h"

#define ADC1_CHAN1 ADC_CHANNEL_4 // Pin 32
#define ADC1_CHAN2 ADC_CHANNEL_6
#define ADC_ATTEN ADC_ATTEN_DB_11


int adc_raw1; 
int adc_raw2; 

int voltage1;
int voltage2;

/*
#define ADC1_CHAN0 ADC_CHANNEL_4 //7 Pin 32
#
*/
esp_err_t config_ADC();
esp_err_t get_ADC_value();
esp_err_t pin_initialize();

adc_oneshot_unit_handle_t adc1_handle;

//motordc
#define M1_A 4
#define M1_B 18
#define L 0
#define H 1

esp_err_t pinout_initialize();
esp_err_t forward();
esp_err_t reverse();
esp_err_t stop();

#define UART_PORT_NUM UART_NUM_1
#define TX_BUF_SIZE 1024
#define TXD_PIN GPIO_NUM_1
#define RXD_PIN GPIO_NUM_3

esp_err_t uart_initialize();
static void tx_task(void *arg);

void app_main(void){

    config_ADC(); 
    ESP_ERROR_CHECK(pinout_initialize());
    uart_initialize();
    xTaskCreate(tx_task, "uart_tx_task", TX_BUF_SIZE*2, NULL, configMAX_PRIORITIES-1, NULL);

}

//motor  dc
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
/*
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

    adc_oneshot_config_channel(adc1_handle, ADC1_CHAN0, &config); //configura la conversion a oneshot
    adc_oneshot_config_channel(adc1_handle, ADC1_CHAN1, &config);
    adc_oneshot_config_channel(adc1_handle, ADC1_CHAN2, &config);
    
    return ESP_OK; 
}
}
*/
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
    adc_oneshot_config_channel(adc1_handle, ADC1_CHAN2, &config);

    return ESP_OK; 
}

esp_err_t get_ADC_value(){
    
    adc_oneshot_read(adc1_handle, ADC1_CHAN1, &adc_raw1);
    adc_oneshot_read(adc1_handle, ADC1_CHAN2, &adc_raw2);
    
    voltage1 = adc_raw1; 
    voltage2 = adc_raw2;  

    return ESP_OK;
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
    char str[50];

    while (1) {

        get_ADC_value();
        if( voltage2<45) { //paro

            ESP_ERROR_CHECK(stop());

        }

        if( 85>voltage2 && voltage2>75) { //rojo

            ESP_ERROR_CHECK(stop());

        }
        else if( 45<voltage2 && voltage2<50 ){ //verde

            ESP_ERROR_CHECK(forward());

        }
        else if( 55<voltage2 && voltage2<65){ //Azul

            ESP_ERROR_CHECK(reverse());

        }
       
        sprintf(str, "/*%d, %d*/", voltage1, voltage2);
        const int len = strlen(str);

        uart_write_bytes(UART_PORT_NUM, str, strlen(str) + 1);
        vTaskDelay(pdMS_TO_TICKS(500));
    
    }
}

