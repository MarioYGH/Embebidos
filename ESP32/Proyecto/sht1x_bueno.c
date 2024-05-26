#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "SHT1x.h"
#include "SHT1x_platform.h"

static const char *TAG = "example";

void app_main(void)
{
  SHT1x_Handler_t Handler = {0};
  SHT1x_Sample_t  Sample = {0};

  ESP_LOGI(TAG, "SHT1x Driver Example");
  esp_log_level_set("gpio",ESP_LOG_NONE); //Eliminar GPIOS

  SHT1x_Platform_Init(&Handler);
  SHT1x_Init(&Handler);

  while (1)
  {
    SHT1x_ReadSample(&Handler, &Sample);
    ESP_LOGI(TAG, "Temperature: %.2f°C, "
                  "Humidity: %.2f%%",
             Sample.TempCelsius,
             Sample.HumidityPercent);

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

  SHT1x_DeInit(&Handler);
}
