/*
Autor: Mario García
Programa ESP32 Sacando el promedio de las temperaturas de LM35 y DHT22, encendemos diferentes LEDs,
con la ultima condición encendemos un buzzer y led y con un botón interrupcion, apagamos solo el buzzer pero no el LED
date created: 29/02/24
last modified: 29/02/24
*/


#include <stdio.h>
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "dht.h"

#define DHTPIN GPIO_NUM_26
#define ADC1_CHAN0 ADC_CHANNEL_4 //7 Pin 32
#define ADC_ATTEN ADC_ATTEN_DB_11
#define BUZZER 18
#define LEDR 21
#define LEDG 22
#define LEDY 23
//#define OFF 19
#define BUTTON_ISR 34

adc_oneshot_unit_handle_t adc1_handle;

static int adc_raw; 
static float voltage;
static float temperature = 0;
static float tempfin;
uint8_t buzzerON = 0;
float OFFSET = 0.5780859231; //desviación estandar LM35

esp_err_t config_ADC();
esp_err_t get_ADC_value();
esp_err_t pin_initialize();


esp_err_t condicionAlarma();

const char *TAG = "DHT22 SENSOR";
const char *TAG2 = "Alarma";

gpio_num_t dht_gpio = DHTPIN; //Digital pin connected to the DHT
dht_sensor_type_t sensor_type = DHT_TYPE_AM2301; //Para DHT11 -> DHT_TYPE_DH11

esp_err_t temperature_task();
esp_err_t init_iris();
esp_err_t buzzer();
void isr_handler(void *args); 

bool boton_presionado = false;

void app_main(void)
{
    pin_initialize();
    config_ADC(); 
    init_iris();
   

    while(true){

        tempfin = ((voltage + temperature)/2);
        get_ADC_value();
        ESP_ERROR_CHECK(temperature_task());
        vTaskDelay(pdMS_TO_TICKS(500));

        condicionAlarma();
        
        //printf("Grados LM35: %2.2f C  DHT22: %2.2f C Prom: %2.2f", voltage, temperature, tempfin);
    }
}
///////////////////////////////////////////////////////////////////

esp_err_t temperature_task(){
    float humidity = 0;
    //float temperature = 0;
    if (dht_read_float_data(sensor_type, dht_gpio, &humidity, &temperature) == ESP_OK)
        ESP_LOGI(TAG, "Grados LM35: %2.2f C  DHT22: %2.2f C Prom: %2.2f", voltage, temperature, tempfin);
    else 
        ESP_LOGE(TAG, "Could not read data from sensor");

    return ESP_OK;
}

/////////////////////////////////////////////////////////////////

esp_err_t pin_initialize() {

    gpio_reset_pin(BUZZER);
    gpio_set_direction(BUZZER, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LEDR);
    gpio_set_direction(LEDR, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LEDG);
    gpio_set_direction(LEDG, GPIO_MODE_OUTPUT);
    gpio_reset_pin(LEDY);
    gpio_set_direction(LEDY, GPIO_MODE_OUTPUT);

    return ESP_OK;
}

///////////////////////////////////////////////////////////////////

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

///////////////////////////////////////////////////////////////////

esp_err_t get_ADC_value(){

    adc_oneshot_read(adc1_handle, ADC1_CHAN0, &adc_raw);
    //printf("Raw data: %d\n", adc_raw);

    voltage = (((adc_raw * 5.0 / 4095.0)*100.0)-OFFSET); //Aca le ponemos *100 y lo pasa a grados xd
    //printf("Grados LM35: %2.2f C\n", voltage);
    //printf("%2.2f \n", voltage);


    return ESP_OK;

}

///////////////////////////////////////////////////////////////////

esp_err_t condicionAlarma() {
    //condiciones para sonido de alarma 

    if(tempfin<29){  // si la temp cumple voltage>29&&voltage<35 suena lento
        //printf("Aumento de temperatura - Activando alarma\n");
        gpio_set_level(LEDG, 1);
        gpio_set_level(LEDY, 0);
        gpio_set_level(LEDR, 0);


        }

       if(tempfin>29&&tempfin<35){ // Se detecta incremento en temperatura 
        //vTaskDelay(pdMS_TO_TICKS(300));
        gpio_set_level(LEDY, 1); // Encender alarma
        gpio_set_level(LEDG, 0);
        gpio_set_level(LEDR, 0);

        
 
    } 
        if(/* voltage*/tempfin>35){ // Se detecta superacion de 40 grados 
        //printf("Aumento de temperatura - Activando alarma\n");
        gpio_set_level(LEDY, 0); // Encender alarma
        gpio_set_level(LEDG, 0);
        gpio_set_level(LEDR, 1);
        buzzer();
        }

    return ESP_OK; 
}

///////////////////////////////////////////////////////////////////

esp_err_t init_iris(){
    gpio_config_t GPIO_config;
    GPIO_config.pin_bit_mask = (1ULL << BUTTON_ISR); //Inicializa el botón desde aquí
    GPIO_config.mode = GPIO_MODE_DEF_INPUT;
    GPIO_config.pull_up_en = GPIO_PULLUP_DISABLE; //Se pueden activar, esto va a depender de si se quiere poner el pull-up en fisico o no
    GPIO_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    GPIO_config.intr_type = GPIO_INTR_NEGEDGE; //interrupt by falling edge //flanco de bajada 


    //configure GPIO´s mode, pullup, pulldown, intrType
    gpio_config(&GPIO_config);

    gpio_install_isr_service(0); //habilita global interrupts
    gpio_isr_handler_add(BUTTON_ISR, isr_handler, NULL); //pin y funcion a ejecutar


    return ESP_OK;

}

///////////////////////////////////////////////////////////////////

void isr_handler(void *args){
ESP_LOGI(TAG2, "Se apaga buzzer");
    
buzzerON ++;
}

/////////////////////////////////////////////////////////

esp_err_t buzzer(){

    if(buzzerON==0){
    gpio_set_level(BUZZER, 1); // Encender alarma
    vTaskDelay(pdMS_TO_TICKS(1000));
    gpio_set_level(BUZZER, 0);
    }
    if(buzzerON==1){
    gpio_set_level(BUZZER,0);
    }
    if(buzzerON>1){
    buzzerON = 0;
    }


    return ESP_OK;

}
