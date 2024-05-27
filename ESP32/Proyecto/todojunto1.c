//sin ESP_NOW
#include <stdio.h>
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/pulse_cnt.h"
#include "bdc_motor.h"
#include "pid_ctrl.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/mcpwm_prelude.h"
#include "SHT1x.h"
#include "SHT1x_platform.h"

static const char *TAG = "Salidas"; //tag general
////SHT1x/////

/////FIN SHT1x/////

/////SENSOR LUZ///////
#define ADC1_CHAN0 ADC_CHANNEL_6 // Pin 34
#define ADC_ATTEN ADC_ATTEN_DB_11

adc_oneshot_unit_handle_t adc1_handle;

static int adc_raw; 
static float voltage;

esp_err_t config_ADC();
esp_err_t get_ADC_value();
//// FIN SENSOR LUZ/////


///////BOMBA Y VENT///////////////
#define SERIAL_STUDIO_DEBUG           CONFIG_SERIAL_STUDIO_DEBUG

#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ             25000    // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX       (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks
///BOMBA
#define BDC_MCPWM_GPIO_A 2
#define BDC_MCPWM_GPIO_B 15
///VENT
#define BDC_MCPWM_GPIO_C 4
#define BDC_MCPWM_GPIO_D 13            

bdc_motor_handle_t motor = NULL;
bdc_motor_handle_t motor1 = NULL;
///////FIN BOMBA Y VENT///////

////SERVO//////
#define SERVO_MIN_PULSEWIDTH_US 500 /* Minimum pulse width in microsecond */
#define SERVO_MAX_PULSEWIDTH_US 2400 /* Maximum pulse width in microsecond */
#define SERVO_MIN_DEGREE 0 /* Minimum angle */
#define SERVO_MAX_DEGREE 180 /* Maximum angle */
#define SERVO_PULSE_GPIO 21 /* GPIO connects to the PWM signal line */
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000 /* 1MHz, 1us per tick */
#define SERVO_TIMEBASE_PERIOD 20000 /* 20000 ticks, 20ms */

mcpwm_timer_handle_t timer = NULL;
mcpwm_oper_handle_t oper = NULL;
mcpwm_cmpr_handle_t comparator = NULL;
mcpwm_gen_handle_t generator = NULL;

esp_err_t mcpwm_config();
static inline uint32_t angle_to_compare(int angle);
///FIN SERVO///////

void app_main(void)
{
    /////////BOMBA Y VENT///////
    ESP_LOGI(TAG, "Create BOMBA");
    bdc_motor_config_t motor_I_config = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = BDC_MCPWM_GPIO_A,
        .pwmb_gpio_num = BDC_MCPWM_GPIO_B,
    };
    ESP_LOGI(TAG, "Create FAN");
    bdc_motor_config_t motor_D_config = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = BDC_MCPWM_GPIO_C,
        .pwmb_gpio_num = BDC_MCPWM_GPIO_D,
    };
    bdc_motor_mcpwm_config_t mcpwm_1_config = {
        .group_id = 0,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };
    bdc_motor_mcpwm_config_t mcpwm_2_config = {
        .group_id = 1,
        .resolution_hz = BDC_MCPWM_TIMER_RESOLUTION_HZ,
    };
  
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_I_config, &mcpwm_1_config, &motor));
    ESP_ERROR_CHECK(bdc_motor_new_mcpwm_device(&motor_D_config, &mcpwm_2_config, &motor1));

    ESP_LOGI(TAG, "Enable");
    ESP_ERROR_CHECK(bdc_motor_enable(motor));
    ESP_ERROR_CHECK(bdc_motor_enable(motor1));
    bdc_motor_set_speed(motor,300);
    bdc_motor_set_speed(motor1,300);
    /////FIN BOMBA Y VENT//////

    //////SENSOR LUZ///////
    esp_err_t ret = config_ADC(); 
    if (ret != ESP_OK) {
        printf("Failed to initialize ADC: %s\n", esp_err_to_name(ret));
        return;
    }

    while (true) {
        ret = get_ADC_value();
        if (ret != ESP_OK) {
            printf("Failed to get ADC value: %s\n", esp_err_to_name(ret));
        } else {
            printf("Raw data: %d\n", adc_raw);
            voltage =((adc_raw * 5.0 )/ 4095.0); // Convertir a voltaje basado en referencia de 5V
            printf("Voltage: %2.2f V\n", voltage);
        }

        vTaskDelay(500 / portTICK_PERIOD_MS); // Aumentamos el tiempo para darle mas chance de convertir
    } 
    /////FIN SENSOR LUZ/////

    /////SERVO/////
     mcpwm_config();

    int angle = 0;

    while (1) {
        // Move to 120Â°
        angle = 120;
        ESP_LOGI(TAG, "Angle of rotation: %d", angle);
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, angle_to_compare(angle)));
        vTaskDelay(pdMS_TO_TICKS(10000)); // Delay for 10 seconds

        // Return to 0Â°
        angle = 0;
        ESP_LOGI(TAG, "Angle of rotation: %d", angle);
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, angle_to_compare(angle)));
        vTaskDelay(pdMS_TO_TICKS(10000)); // Delay for 10 seconds
    }
    ////FIN SERVO//////

   ////////SHT1x//////
  SHT1x_Handler_t Handler = {0};
  SHT1x_Sample_t  Sample = {0};

  ESP_LOGI(TAG, "SHT1x Driver");
  esp_log_level_set("gpio",ESP_LOG_NONE); //Eliminar GPIOS

  SHT1x_Platform_Init(&Handler);
  SHT1x_Init(&Handler);

  while (1)
  {
    SHT1x_ReadSample(&Handler, &Sample);
    ESP_LOGI(TAG, "Temperature: %.2fÂ°C, "
                  "Humidity: %.2f%%",
             Sample.TempCelsius,
             Sample.HumidityPercent);

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

  SHT1x_DeInit(&Handler);
  //////FIN SHT1x//////

}

///TASK SENSOR LUZ//////
esp_err_t config_ADC() {
    // ADC init
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };

    esp_err_t ret = adc_oneshot_new_unit(&init_config1, &adc1_handle);
    if (ret != ESP_OK) {
        printf("Error inicializando ADC unit: %s\n", esp_err_to_name(ret));
        return ret;
    }

    // ADC config
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT, // ancho de banda
        .atten = ADC_ATTEN, // Atenuacion
    };

    ret = adc_oneshot_config_channel(adc1_handle, ADC1_CHAN0, &config);
    if (ret != ESP_OK) {
        printf("Error configurando ADC channel: %s\n", esp_err_to_name(ret));
    }

    return ret; 
}

esp_err_t get_ADC_value() {
    esp_err_t ret = adc_oneshot_read(adc1_handle, ADC1_CHAN0, &adc_raw);
    if (ret != ESP_OK) {
        printf("Error ADC: %s\n", esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}
/////FIN SENSOR LUZ//////

////TASK SERVO/////
esp_err_t mcpwm_config() {
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

static inline uint32_t angle_to_compare(int angle) {
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}
////FIN SERVO///////
