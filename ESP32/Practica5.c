/*
Autor: Mario García
Programa ESP32. EN un tren, Con ayuda del DHT22 medimos constante mente la temperatura y cuando supera los 35 g encendemos led y buzzer, 
apagamos con una intrrupcion accionada con un boton el buzzer pero no el boton
simultaneamente con un paro optico accionamos una interrupcion que llevara la cuenta de los pasajeros que ingresan, verde menos de 10, entre 10 y 18 amarillo
y mas de 18 rojo, imprime mensjes con esp_log
date created: 03/02/24
last modified: 04/02/24
*/
#include <stdio.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "dht.h"

//sensores Temperatura
#define DHTPIN GPIO_NUM_26
//salidas
#define BUZZER 18
#define LEDR 21
#define LEDG 23
#define LEDY 22
//Entradas
#define BUTTON_ISR 34
#define Pop 33//Paro optico

///funciones
esp_err_t pin_initialize();
esp_err_t PasajerosCuenta();
esp_err_t buzzer();
esp_err_t temperature_task();
esp_err_t init_iris();
void isr_handler(void *args);

esp_err_t init_iris2();
void isr_handler2(void *args); 

const char *TAG = "DHT22 SENSOR";
uint8_t pasajeros = 0;
uint8_t buzzerON = 0;
float temperature = 0;

gpio_num_t dht_gpio = DHTPIN; //Digital pin connected to the DHT
dht_sensor_type_t sensor_type = DHT_TYPE_AM2301; //Para DHT11 -> DHT_TYPE_DH11
 
void app_main(void)
{
    pin_initialize();
    init_iris();
    init_iris2(); 
    

    while(true){
    PasajerosCuenta();
    
    ESP_ERROR_CHECK(temperature_task());
    }
}

///////////////////////////////////////////////////////////////////

esp_err_t temperature_task(){
    float humidity = 0;
    if (dht_read_float_data(sensor_type, dht_gpio, &humidity, &temperature) == ESP_OK)
        ESP_LOGI(TAG, " ");
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

esp_err_t PasajerosCuenta() {
    //condiciones para sonido de alarma 

    if(pasajeros<29){  // LED verde, todo OK
        //printf("Aumento de temperatura - Activando alarma\n");
        gpio_set_level(LEDG, 1);
        gpio_set_level(LEDY, 0);
        gpio_set_level(LEDR, 0);
        ESP_LOGI(TAG, "Pasajeros: %d C Temp: %f", pasajeros, temperature);
        }
        if(pasajeros>29&&pasajeros<35){ // Se detecta incremento en temperatura, LED amarillo
        //vTaskDelay(pdMS_TO_TICKS(300));
        gpio_set_level(LEDY, 1); // Encender alarma
        gpio_set_level(LEDG, 0);
        gpio_set_level(LEDR, 0);
        buzzerON = 0; // vuelve a activar la posibilidad de encende el buzzer por si aca
        ESP_LOGW(TAG, "Pasajeros: %d C Temp: %f", pasajeros, temperature);
        } 
        if(pasajeros>35){ // Se detecta superacion de 35 grados, enciende buzzer y LED rojo
        //printf("Aumento de temperatura - Activando alarma\n");
        gpio_set_level(LEDY, 0); // Encender alarma
        gpio_set_level(LEDG, 0);
        gpio_set_level(LEDR, 1);
        ESP_LOGE(TAG, "Pasajeros: %d C Temp: %f", pasajeros, temperature);
        }
        
        if(temperature>35)
            buzzer();
        else
           buzzerON = 0;

        

    return ESP_OK; 
}

///////////////////////////////////////////////////////////////////

esp_err_t init_iris(){ //interrupcion boton
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
//////////////////////////////////////////////////////////////////////
esp_err_t init_iris2(){ //interrupcion paro optico
    gpio_config_t GPIO_config;
    GPIO_config.pin_bit_mask = (1ULL << Pop); //Inicializa el botón desde aquí
    GPIO_config.mode = GPIO_MODE_DEF_INPUT;
    GPIO_config.pull_up_en = GPIO_PULLUP_DISABLE; //Se pueden activar, esto va a depender de si se quiere poner el pull-up en fisico o no
    GPIO_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    GPIO_config.intr_type = GPIO_INTR_NEGEDGE; //interrupt by falling edge //flanco de bajada 

    //configure GPIO´s mode, pullup, pulldown, intrType
    gpio_config(&GPIO_config);

    gpio_install_isr_service(0); //habilita global interrupts
    gpio_isr_handler_add(Pop, isr_handler2, NULL); //pin y funcion a ejecutar

    return ESP_OK;
}
///////////////////////////////////////////////////////////////////

void isr_handler(void *args){   
buzzerON ++; //Ojo aqui, para esta funcion NO usar el ESP_LOGI(TAG // no se muy bien pq pero reinicia el codigo en lugar de hacer una interrupcion
}

void isr_handler2(void *args){   
pasajeros ++; 
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
