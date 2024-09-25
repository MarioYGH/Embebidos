#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"

// Parámetros del servo
#define SERVO_MIN_PULSEWIDTH_US 500  // Ancho de pulso mínimo en microsegundos
#define SERVO_MAX_PULSEWIDTH_US 2400 // Ancho de pulso máximo en microsegundos
#define SERVO_MIN_DEGREE 0           // Ángulo mínimo
#define SERVO_MAX_DEGREE 180         // Ángulo máximo
#define SERVO_PULSE_GPIO 21          // GPIO conectado a la señal PWM del servo
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000 // 1 MHz, 1us por tick
#define SERVO_TIMEBASE_PERIOD 20000  // 20 ms por ciclo

#define ADC1_CHAN0 ADC_CHANNEL_6 // Pin 34
#define ADC_ATTEN ADC_ATTEN_DB_11

// Variables globales para MCPWM y ADC
adc_oneshot_unit_handle_t adc1_handle;
static int adc_raw;
static float voltage;

mcpwm_timer_handle_t timer = NULL;
mcpwm_oper_handle_t oper = NULL;
mcpwm_cmpr_handle_t comparator = NULL;
mcpwm_gen_handle_t generator = NULL;

static const char *TAG = "PWM Servo + ADC";

// Prototipos de funciones
esp_err_t config_ADC();
esp_err_t get_ADC_value();
esp_err_t mcpwm_config();
static inline uint32_t angle_to_compare(int angle);
void update_servo_angle(int angle);

void app_main(void) {
    // Configuración de ADC y MCPWM
    esp_err_t ret = config_ADC();
    if (ret != ESP_OK) {
        printf("Failed to initialize ADC: %s\n", esp_err_to_name(ret));
        return;
    }

    mcpwm_config();

    // Bucle principal
    while (true) {
        // Leer el valor del ADC
        ret = get_ADC_value();
        if (ret != ESP_OK) {
            printf("Failed to get ADC value: %s\n", esp_err_to_name(ret));
        } else {
            printf("Raw ADC data: %d\n", adc_raw);
            voltage = ((adc_raw * 5.0) / 4095.0); // Convertir a voltaje basado en referencia de 5V
            printf("Voltage: %2.2f V\n", voltage);
        }

        // Controlar el ángulo del servo
        update_servo_angle(120); // Cambiar a 120 grados
        vTaskDelay(pdMS_TO_TICKS(1000)); // Esperar 1 segundo

        update_servo_angle(90);  // Cambiar a 90 grados
        vTaskDelay(pdMS_TO_TICKS(1000)); // Esperar 1 segundo

        update_servo_angle(180); // Cambiar a 180 grados
        vTaskDelay(pdMS_TO_TICKS(1000)); // Esperar 1 segundo
    }
}

void update_servo_angle(int angle) {
    ESP_LOGI(TAG, "Setting angle of rotation: %d", angle);
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, angle_to_compare(angle)));
}

esp_err_t config_ADC() {
    // Configuración de ADC
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };

    esp_err_t ret = adc_oneshot_new_unit(&init_config1, &adc1_handle);
    if (ret != ESP_OK) {
        printf("Error inicializando ADC unit: %s\n", esp_err_to_name(ret));
        return ret;
    }

    // Configuración del canal ADC
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT, // ancho de banda
        .atten = ADC_ATTEN, // Atenuación
    };

    ret = adc_oneshot_config_channel(adc1_handle, ADC1_CHAN0, &config);
    if (ret != ESP_OK) {
        printf("Error configurando ADC channel: %s\n", esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t get_ADC_value() {
    // Leer el valor del ADC
    esp_err_t ret = adc_oneshot_read(adc1_handle, ADC1_CHAN0, &adc_raw);
    if (ret != ESP_OK) {
        printf("Error ADC: %s\n", esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}

esp_err_t mcpwm_config() {
    ESP_LOGI(TAG, "Create timer and operator");

    // Configuración del temporizador MCPWM
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    // Configuración del operador MCPWM
    mcpwm_operator_config_t operator_config = {
        .group_id = 0, // El operador debe estar en el mismo grupo que el temporizador
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    ESP_LOGI(TAG, "Create comparator and generator from the operator");

    // Configuración del comparador MCPWM
    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    // Configuración del generador MCPWM
    mcpwm_generator_config_t generator_config = {
        .gen_gpio_num = SERVO_PULSE_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

    // Establecer el valor de comparación inicial para centrar el servo
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, angle_to_compare(0)));

    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    // Actuar alto en el evento de temporizador vacío
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // Actuar bajo en el evento de comparación
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    return ESP_OK;
}

static inline uint32_t angle_to_compare(int angle) {
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}
