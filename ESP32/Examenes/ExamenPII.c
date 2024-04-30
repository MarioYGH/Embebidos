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
//Ultrasonic
#include <stdbool.h>
#include <ultrasonic.h>
#include <esp_err.h>

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

//Ultrasonic
#define MAX_DISTANCE_CM 500 // 5m max

#if defined(CONFIG_IDF_TARGET_ESP8266)
#define TRIGGER_GPIO 4
#define ECHO_GPIO 5
#else
#define TRIGGER_GPIO 17
#define ECHO_GPIO 16
#endif

void ultrasonic_test(void *pvParameters)
{
    ultrasonic_sensor_t sensor = {
        .trigger_pin = TRIGGER_GPIO,
        .echo_pin = ECHO_GPIO
    };

    ultrasonic_init(&sensor);

    while (true)
    {
        float distance;
        esp_err_t res = ultrasonic_measure(&sensor, MAX_DISTANCE_CM, &distance);
        if (res != ESP_OK)
        {
            printf("Error %d: ", res);
            switch (res)
            {
                case ESP_ERR_ULTRASONIC_PING:
                    printf("Cannot ping (device is in invalid state)\n");
                    break;
                case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
                    printf("Ping timeout (no device found)\n");
                    break;
                case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
                    printf("Echo timeout (i.e. distance too big)\n");
                    break;
                default:
                    printf("%s\n", esp_err_to_name(res));
            }
        }
        else
            printf("Distance: %0.04f cm\n", distance*100);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

//fin Ultrasonic

esp_err_t motor_init() {
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

    return ESP_OK;
}

void app_main()
{
    motor_init();
    xTaskCreate(ultrasonic_test, "ultrasonic_test", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
}
