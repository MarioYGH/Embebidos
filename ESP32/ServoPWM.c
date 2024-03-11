/*
Programa ESP32, Con ayuda de un potenciometro vamos a hacer una conversión ADC, para pasar el voltaje a Grados, 0V-0° 3.3V-180°
date created: 11/03/24
last modified: 11/03/24
*/

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"//PWM
#include "esp_adc/adc_oneshot.h" //Para hacer lecturas Oneshot o continuas

//Parametros ADC
#define ADC1_CHAN0 ADC_CHANNEL_4 //7 32
#define ADC_ATTEN ADC_ATTEN_DB_11

/* Set the parameters according to your servo */
#define SERVO_MIN_PULSEWIDTH_US 500 /* Minimum pulse width in microsecond */
#define SERVO_MAX_PULSEWIDTH_US 2400 /* Maximum pulse width in microsecond */
#define SERVO_MIN_DEGREE 0 /* Minimum angle */
#define SERVO_MAX_DEGREE 180 /* Maximum angle */
#define SERVO_PULSE_GPIO 21 /* GPIO connects to the PWM signal line */
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000 /* 1MHz, 1us per tick */
#define SERVO_TIMEBASE_PERIOD 20000 /* 20000 ticks, 20ms */

static const char *TAG = "PWM servo"; //SERVO
static int adc_raw;  //ADC
static float voltage; //ADC

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

void app_main(void){
    config_ADC(); 
    mcpwm_config();

    int step = 0;

    while (1) {
        get_ADC_value();

        vTaskDelay(250/ portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Angle of rotation: %d", angle);
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, angle_to_compare(angle)));

        /* Add delay, since it takes time for servo to rotate, usually 200ms/60degree rotation under 5V power supply */
        vTaskDelay(pdMS_TO_TICKS(500));
    }       
}

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
    //printf("Raw data: %d\n", adc_raw);

    voltage = (adc_raw * 3.3 / 4095.0); //importante poner el cero, para que lo detecte como flotante pq sino pone ceros
    //printf("Voltage: %2.2f V\n", voltage);

    angle = voltage * (180.0 / 3.3); // Convertir directamente de voltaje a grados
    printf("Degrees: %d\n", angle);

    return ESP_OK;

}
