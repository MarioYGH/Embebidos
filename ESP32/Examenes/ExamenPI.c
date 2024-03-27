/*
Autor: Mario García
Examen: Se diseñó un programa para el ESP32 que permita realizar la lectura de tres LM35 conectados en los canales
6, 7 y 4 del ADC1, respectivamente. El programa muestra las lecturas con un Info LOG además del
promedio de las tres lecturas realizadas.
Si supone que los LM35 se encuentran calibrados, entonces se realizó una función que emité un
Warning LOG cuando alguna de las mediciones están ± 0.25o C con respecto a los demás. Adicional a esto,
por medio de un bontón conectado y configurado para trabajar con interrupción externa muestra las lecturas
en °C, °K, o °F seg ́un el valor del contador de acuerdo con la siguiente tabla:
Contador Unidades
0 °C
1 °K
2 °F
Programa ESP32 
date created: 04/02/24
last modified: 04/02/24
*/

#include <stdio.h>
#include <math.h>
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"


//sensores Temperatura
#define ADC1_CHAN0 ADC_CHANNEL_6 // Pin 34
#define ADC1_CHAN1 ADC_CHANNEL_7 // Pin 35
#define ADC1_CHAN2 ADC_CHANNEL_4 // Pin 32
//adc
#define ADC_ATTEN ADC_ATTEN_DB_11
//ENTRADAS
#define BUTTON_ISR 33

adc_oneshot_unit_handle_t adc1_handle;
//VARIABLES
static float tempfin;
uint8_t contador = 0;

const char *TAG = "LM35 SENSOR";
//FUNCIONES
esp_err_t config_ADC();
esp_err_t get_ADC_value();
esp_err_t init_iris();
esp_err_t cambioUnidades();
esp_err_t diferencia(float temp1, float temp2, float temp3);
void isr_handler(void *args);

///////////////////////////////////////// FUNCION MAIN
void app_main(void)
{
    config_ADC(); 
    init_iris();
    

    while (true) {
        get_ADC_value();
        cambioUnidades();

        vTaskDelay(500/ portTICK_PERIOD_MS); //Aumentamos el tiempo para darle más chance de convertir
    } 

}
/////////////////////////////////////////////////////77
//CONFIGURACION ADC
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
////////////////////////////////////////////////////////////
//ADQUIERE LOS DATOS DEL LM35, Y CON AYUDA DEL ADC LOS CONVIERTE
esp_err_t get_ADC_value(){
    static int adc_raw1, adc_raw2, adc_raw3;
    static float temp1, temp2, temp3;
    
    // Realizar lecturas del ADC
    adc_oneshot_read(adc1_handle, ADC1_CHAN0, &adc_raw1);
    adc_oneshot_read(adc1_handle, ADC1_CHAN1, &adc_raw2);
    adc_oneshot_read(adc1_handle, ADC1_CHAN2, &adc_raw3);

    // Verificar si las lecturas del ADC son exitosas
    if (adc_raw1 < 0 || adc_raw2 < 0 || adc_raw3 < 0) {
        ESP_LOGE(TAG, "Error al leer el ADC");
        return ESP_FAIL;
    }

    // Calcular temperaturas
    temp1 = ((adc_raw1 * 5.0 / 4095.0)*100.0);
    temp2 = ((adc_raw2 * 5.0 / 4095.0)*100.0);
    temp3 = ((adc_raw3 * 5.0 / 4095.0)*100.0);
    tempfin = (temp1 + temp2 + temp3) / 3;

    // Llamar a la función para verificar diferencias entre temperaturas
    diferencia(temp1, temp2, temp3);

    return ESP_OK;

}

///////////////////////////////////////////////////////////////////
//FUNCION PARA CONFIGURAR LA INTERRUPCION
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
//FUNCIÓN QUE SE EJECUTA CUANDO SE PRESIONA EL BOTÓN
void isr_handler(void *args){    
contador ++; //INCREMENTA CONTADOR
}
//////////////////////////////////////////////////////////
//FUNCIÓN CAMBIO UNIDADES
esp_err_t cambioUnidades() {
    //
        // Variable local para almacenar la temperatura convertida
    float tempfin_convertida;

    // Cambiar la unidad según el valor del contador
    switch (contador) {
        case 0:
            // No se realiza conversión, ya está en Celsius
            tempfin_convertida = tempfin;
            break;
        case 1:
            // Conversión de Celsius a Kelvin
            tempfin_convertida = tempfin + 273.15;
            break;
        case 2:
            // Conversión de Celsius a Fahrenheit
            tempfin_convertida = tempfin * 1.8 + 32;
            break;
        default:
            // Resetear contador si es mayor que 2
            contador = 0;
            return ESP_FAIL;
    }

    // Emitir Info LOG con la temperatura convertida
    ESP_LOGI(TAG, "Temperatura Convertida: %2.2f", tempfin_convertida);

    return ESP_OK; 
} 
///////////////////////////////////////////////////////////////////
//FUNCIÓN QUE BUSCA SI HAY UNA DIFERENCIA ENTRE LAS DISTINTAS TEMPERATURAS
esp_err_t diferencia(float temp1, float temp2, float temp3){
    // Calcular diferencias entre temperaturas
    float diff1 = fabs(temp1 - temp2);
    float diff2 = fabs(temp1 - temp3);
    float diff3 = fabs(temp2 - temp3);

    // Compara las diferencias con ±0.25°C y emite un "Warning LOG" si alguna excede este límite
    if (diff1 > 0.25 || diff2 > 0.25 || diff3 > 0.25) {
        ESP_LOGW(TAG, "Warning: Las mediciones están fuera del rango de ±0.25°C");
    }

    return ESP_OK;
}

