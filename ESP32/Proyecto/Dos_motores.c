#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/pulse_cnt.h"
#include "bdc_motor.h"
#include "pid_ctrl.h"

//////////////MOTOR
// Enable this config,  we will print debug formated string, which in return can be captured and parsed by Serial-Studio
#define SERIAL_STUDIO_DEBUG           CONFIG_SERIAL_STUDIO_DEBUG

#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define BDC_MCPWM_FREQ_HZ             25000    // 25KHz PWM
#define BDC_MCPWM_DUTY_TICK_MAX       (BDC_MCPWM_TIMER_RESOLUTION_HZ / BDC_MCPWM_FREQ_HZ) // maximum value we can set for the duty cycle, in ticks
///Motor uno
#define BDC_MCPWM_GPIO_A 2
#define BDC_MCPWM_GPIO_B 15
///Motor dos
#define BDC_MCPWM_GPIO_C 4
#define BDC_MCPWM_GPIO_D 13            

static const char *TAG = "example";
bdc_motor_handle_t motor = NULL;
bdc_motor_handle_t motor1 = NULL;


void app_main(void)
{
    /////////MOTOR
    ESP_LOGI(TAG, "Create DC motor 1");
    bdc_motor_config_t motor_I_config = {
        .pwm_freq_hz = BDC_MCPWM_FREQ_HZ,
        .pwma_gpio_num = BDC_MCPWM_GPIO_A,
        .pwmb_gpio_num = BDC_MCPWM_GPIO_B,
    };
    ESP_LOGI(TAG, "Create DC motor 2");
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

    ESP_LOGI(TAG, "Enable motors");
    ESP_ERROR_CHECK(bdc_motor_enable(motor));
    ESP_ERROR_CHECK(bdc_motor_enable(motor1));
    // what does 300 or 400 mean?
    bdc_motor_set_speed(motor,300);
    bdc_motor_set_speed(motor1,300);
}

   
