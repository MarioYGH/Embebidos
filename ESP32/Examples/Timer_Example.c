#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "freertos/timers.h"

#define LED_PIN GPIO_NUM_2

uint8_t led_level = 0;
static const char *TAG = "Timer example";
TimerHandle_t xTimer;
int timerId = 1;
int interval = 500; /* interval in ms */

esp_err_t init_LED();
esp_err_t blink_LED();
esp_err_t set_timer();
void vTimerCallback(TimerHandle_t);

void app_main(void){
    ESP_ERROR_CHECK(init_LED());
    set_timer();
}

esp_err_t init_LED(){
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    return ESP_OK;
}

esp_err_t blink_LED(){
    led_level = !led_level;
    gpio_set_level(LED_PIN, led_level);

    return ESP_OK;
}

void vTimerCallback(TimerHandle_t pxTimer){
    ESP_LOGI(TAG, "Event was called from timer");

    blink_LED();
}

esp_err_t set_timer(){
    ESP_LOGI(TAG, "Timer init configuration");

    xTimer = xTimerCreate("Timer", /* Just a text name */
                            (pdMS_TO_TICKS(interval)), /* The timer period in ticks */
                            pdTRUE, /* The timer will auto-reload themselves when they expire */
                            (void *)timerId, /* Assign each timer a unique id equal to its array index */
                            vTimerCallback /* Each timer calls the same callback when it expires */
    );

    if (xTimer == NULL){
        /* The timer was not created */
        ESP_LOGE(TAG, "The timer was not created");
    }
    else{
        if(xTimerStart(xTimer, 0) != pdPASS){
            /* The timer could not be set into the active state */
            ESP_LOGE(TAG, "The timer could not be set into the active state");
        }
    }

    return ESP_OK;
}
