#include <stdio.h>
#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_mac.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "driver/pulse_cnt.h"
#include "bdc_motor.h"
#include "pid_ctrl.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/mcpwm_prelude.h"
#include "SHT1x.h"
#include "SHT1x_platform.h"

#define ESP_CHANNEL 1
#define ESP_NOW_PMK "pmk1234567890123"
#define ESP_NOW_LMK "lmk1234567890123"
#define ADC1_CHAN0 ADC_CHANNEL_6
#define ADC_ATTEN ADC_ATTEN_DB_11
#define BDC_MCPWM_TIMER_RESOLUTION_HZ 10000000
#define BDC_MCPWM_FREQ_HZ 25000
#define BDC_MCPWM_GPIO_A 2
#define BDC_MCPWM_GPIO_B 15
#define BDC_MCPWM_GPIO_C 4
#define BDC_MCPWM_GPIO_D 13
#define SERVO_MIN_PULSEWIDTH_US 500
#define SERVO_MAX_PULSEWIDTH_US 2400
#define SERVO_MIN_DEGREE 0
#define SERVO_MAX_DEGREE 180
#define SERVO_PULSE_GPIO 21
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000
#define SERVO_TIMEBASE_PERIOD 20000

int position_flag = 0; // Variable para determinar la posición del servo

static const char *TAG = "Main";
static uint8_t peer_mac[ESP_NOW_ETH_ALEN] = {0xc8, 0xf0, 0x9e, 0xec, 0x0d, 0x18};

adc_oneshot_unit_handle_t adc1_handle;
static int adc_raw;
static float voltage;

bdc_motor_handle_t motor = NULL;
bdc_motor_handle_t motor1 = NULL;
mcpwm_timer_handle_t timer = NULL;
mcpwm_oper_handle_t oper = NULL;
mcpwm_cmpr_handle_t comparator = NULL;
mcpwm_gen_handle_t generator = NULL;

esp_err_t config_ADC();
esp_err_t get_ADC_value();
esp_err_t mcpwm_config();
static inline uint32_t angle_to_compare(int angle);

void sensor_task(void *pvParameter);
void motor_task(void *pvParameter);
void servo_task(void *pvParameter);
void sht1x_task(void *pvParameter);
void esp_now_task(void *pvParameter);

static esp_err_t esp_now_send_data(const uint8_t *peer_addr, const uint8_t *data, size_t len);

static esp_err_t init_wifi(void) {
    wifi_init_config_t wifi_init_config = WIFI_INIT_CONFIG_DEFAULT();
    esp_netif_init();
    esp_event_loop_create_default();
    nvs_flash_init();
    esp_wifi_init(&wifi_init_config);
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_storage(WIFI_STORAGE_FLASH);
    esp_wifi_start();

    ESP_LOGI(TAG, "WiFi init complete");
    return ESP_OK;
}

void recv_cb(const esp_now_recv_info_t *esp_now_info, const uint8_t *data, int data_len) {
    ESP_LOGI(TAG, "Data received: " MACSTR ", %s", MAC2STR(esp_now_info->src_addr), data);

    if (data_len >= 2) {
        switch (data[1]) {
            case '1':
                ESP_LOGI(TAG, "Received T1");
                // Lógica para T1
                break;
            case '2':
                ESP_LOGI(TAG, "Received T2");
                // Lógica para T2
                break;
            case '3':
                ESP_LOGI(TAG, "Received T3");
                // Lógica para T3
                break;
            default:
                ESP_LOGW(TAG, "Unknown data received");
                break;
        }
    }

    uint8_t *mac_addr = esp_now_info->src_addr;
    ESP_ERROR_CHECK(esp_now_send_data(mac_addr, data, data_len));
}

void send_cb(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGI(TAG, "ESP_NOW_SEND_SUCCESS");
    } else {
        ESP_LOGW(TAG, "ESP_NOW_SEND_FAIL");
    }
}

static esp_err_t init_esp_now(void) {
    esp_now_init();
    esp_now_register_recv_cb(recv_cb);
    esp_now_register_send_cb(send_cb);
    esp_now_set_pmk((const uint8_t*)ESP_NOW_PMK);

    ESP_LOGI(TAG, "ESP-NOW init complete");
    return ESP_OK;
}

static esp_err_t register_peer(uint8_t *peer_addr) {
    esp_now_peer_info_t peer_info = {};
    memcpy(peer_info.peer_addr, peer_mac, ESP_NOW_ETH_ALEN);
    peer_info.channel = ESP_CHANNEL;
    peer_info.ifidx = ESP_IF_WIFI_STA;
    memcpy(peer_info.lmk, ESP_NOW_LMK, ESP_NOW_KEY_LEN);
    peer_info.encrypt = true;

    esp_now_add_peer(&peer_info);
    return ESP_OK;
}

static esp_err_t esp_now_send_data(const uint8_t *peer_addr, const uint8_t *data, size_t len) {
    esp_now_send(peer_addr, data, len);
    return ESP_OK;
}

void app_main(void) {
    ESP_ERROR_CHECK(init_wifi());
    ESP_ERROR_CHECK(init_esp_now());
    ESP_ERROR_CHECK(register_peer(peer_mac));

    // Tareas en núcleo 0
    xTaskCreatePinnedToCore(sensor_task, "sensor_task", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(motor_task, "motor_task", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(servo_task, "servo_task", 4096, NULL, 1, NULL, 0);

    // Tareas en núcleo 1
    xTaskCreatePinnedToCore(sht1x_task, "sht1x_task", 4096, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(esp_now_task, "esp_now_task", 4096, NULL, 1, NULL, 1);
}

void sensor_task(void *pvParameter) {
    esp_err_t ret = config_ADC();
    if (ret != ESP_OK) {
        printf("Failed to initialize ADC: %s\n", esp_err_to_name(ret));
        vTaskDelete(NULL);
        return;
    }

    while (true) {
        ret = get_ADC_value();
        if (ret != ESP_OK) {
            printf("Failed to get ADC value: %s\n", esp_err_to_name(ret));
        } else {
            printf("Raw data: %d\n", adc_raw);
            voltage = ((adc_raw * 5.0) / 4095.0);
            printf("Voltage: %2.2f V\n", voltage);
        }

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void motor_task(void *pvParameter) {
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
    bdc_motor_set_speed(motor, 300);
    bdc_motor_set_speed(motor1, 300);

    while (true) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void servo_task(void *pvParameter) {
    ESP_ERROR_CHECK(mcpwm_config());

    while (true) {
        switch (position_flag) {
            case 0:
                ESP_LOGI(TAG, "Angle of rotation: 120");
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, angle_to_compare(120)));
                break;
            case 1:
                ESP_LOGI(TAG, "Angle of rotation: 90");
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, angle_to_compare(90)));
                break;
            default:
                ESP_LOGE(TAG, "Invalid position flag");
                break;
        }
    }
}


void sht1x_task(void *pvParameter) {
    SHT1x_Handler_t Handler = {0};
    SHT1x_Sample_t Sample = {0};

    ESP_LOGI(TAG, "SHT1x Driver");
    esp_log_level_set("gpio", ESP_LOG_NONE);

    SHT1x_Platform_Init(&Handler);
    SHT1x_Init(&Handler);

    while (true) {
        SHT1x_ReadSample(&Handler, &Sample);
        ESP_LOGI(TAG, "Temperature: %.2f°C, Humidity: %.2f%%", Sample.TempCelsius, Sample.HumidityPercent);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    SHT1x_DeInit(&Handler);
}

void esp_now_task(void *pvParameter) {
    while (true) {
        // Lógica de espera de ESP-NOW
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

esp_err_t config_ADC() {
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };

    esp_err_t ret = adc_oneshot_new_unit(&init_config1, &adc1_handle);
    if (ret != ESP_OK) {
        printf("Error inicializando ADC unit: %s\n", esp_err_to_name(ret));
        return ret;
    }

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN,
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
        .group_id = 0,
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

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, angle_to_compare(0)));

    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    return ESP_OK;
}

static inline uint32_t angle_to_compare(int angle) {
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}
