/**
 **********************************************************************************
 * @file   main.c
 * @author Hossein.M (https://github.com/Hossein-M98)
 * @brief  example code for SHT1x Driver (for ESP32-IDF)
 **********************************************************************************
 *
 * Copyright (c) 2023 Mahda Embedded System (MIT License)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 **********************************************************************************
 */

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
  esp_log_level_set("gpio",ESP_LOG_NONE);

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