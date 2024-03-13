/*
Programa ESP32 ADC
date created: 13/03/24
last modified: 13/03/24
*/

#include <stdio.h>
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "dht.h"
#include "driver/mcpwm_prelude.h"//PWM
#include "esp_adc/adc_oneshot.h" //Para hacer lecturas Oneshot o continuas

//DHT22
#define DHTPIN GPIO_NUM_26
//Parametros ADC
#define ADC1_CHAN0 ADC_CHANNEL_6 //pin 34
#define ADC_ATTEN ADC_ATTEN_DB_11

/* Set the parameters according to your servo */
#define SERVO_MIN_PULSEWIDTH_US 500 /* Minimum pulse width in microsecond */
#define SERVO_MAX_PULSEWIDTH_US 2400 /* Maximum pulse width in microsecond */
#define SERVO_MIN_DEGREE 0 /* Minimum angle */
#define SERVO_MAX_DEGREE 180 /* Maximum angle */
#define SERVO_PULSE_GPIO 21 /* GPIO connects to the PWM signal line */
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000 /* 1MHz, 1us per tick */
#define SERVO_TIMEBASE_PERIOD 20000 /* 20000 ticks, 20ms */

//PuenteH
#define M1_A 32
#define M1_B 33
#define L 0
#define H 1

const char *TAG1 = "DHT22 SENSOR";
static const char *TAG = "PWM servo"; //SERVO
static int adc_raw;  //ADC
//static float voltage; //ADC

adc_oneshot_unit_handle_t adc1_handle; //ADC

//SERVO
mcpwm_timer_handle_t timer = NULL;
mcpwm_oper_handle_t oper = NULL;
mcpwm_cmpr_handle_t comparator = NULL;
mcpwm_gen_handle_t generator = NULL;

esp_err_t config_ADC();
esp_err_t get_ADC_value();
esp_err_t mcpwm_config();
static inline uint32_t angle_to_compare(int angle);
int angle = 0;
int ok;

//DHT22
float temperature = 0;
gpio_num_t dht_gpio = DHTPIN; //Digital pin connected to the DHT
dht_sensor_type_t sensor_type = DHT_TYPE_AM2301; //Para DHT11 -> DHT_TYPE_DH11

esp_err_t temperature_task();

//Puente H
esp_err_t pinout_initialize();
esp_err_t condicionAlarma();
esp_err_t forward();
esp_err_t reverse();
esp_err_t stop();

void app_main(void)
{
    config_ADC(); 
    mcpwm_config();
    ESP_ERROR_CHECK(pinout_initialize());

    while(true){

         get_ADC_value();

        condicionAlarma();
        //ESP_LOGI(TAG, "Angle of rotation: %d", angle);
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, angle_to_compare(angle)));
        ESP_ERROR_CHECK(temperature_task());

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/////////////////////////////////////////DHT22

esp_err_t temperature_task(){
    float humidity = 0;
    if (dht_read_float_data(sensor_type, dht_gpio, &humidity, &temperature) == ESP_OK)
    ok++;

    return ESP_OK;
}

///////////////////////////////////////SERVO

esp_err_t mcpwm_config(){
    ESP_LOGI(TAG, "Create timer and operator");
    
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    
    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // operator must be in the same group to the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    ESP_LOGI(TAG, "Create comparator and generator from the operator");
    
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = SERVO_PULSE_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    // set the initial compare value, so that the servo will spin to the center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, angle_to_compare(0)));

    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    return ESP_OK;
}

static inline uint32_t angle_to_compare(int angle){
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
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
    /*
    printf("Raw data: %d\n", adc_raw);

    voltage = (adc_raw * 3.3 / 4095.0); //importante poner el cero, para que lo detecte como flotante pq sino pone ceros
    printf("Voltage: %2.2f V\n", voltage);
    */
    angle = ((adc_raw * 3.3 / 4095.0) * (180.0 / 3.3)); //importante poner el cero, para que lo detecte como flotante pq sino pone ceros
    //printf("Voltage: %2.2d V\n", angle);

    return ESP_OK;
}

/////////////////////////////////PuenteH

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

////////////////////////////////////Funcion principal
esp_err_t condicionAlarma() {
    //condiciones para sonido de alarma 

    if(temperature<25){ 
        ESP_ERROR_CHECK(forward());
        ESP_LOGI(TAG1, "Modo Inverso Temp: %fC Angle of rotation: %d", temperature, angle);
        }
       if(temperature>25&&temperature<30){ 
        ESP_ERROR_CHECK(stop());
        ESP_LOGI(TAG1, "Ventilador Apagado Temp: %fC Angle of rotation: %d", temperature, angle);
        
    } 
        if(temperature>30){ 
        ESP_ERROR_CHECK(reverse());
        ESP_LOGI(TAG1, "Modo extractor Temp: %fC Angle of rotation: %d", temperature, angle);
        
        }

    return ESP_OK; 
}
