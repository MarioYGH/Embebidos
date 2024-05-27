#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "dht.h"

#include "esp_http_client.h"
#include "connect_wifi.h"

#define DHTPIN GPIO_NUM_26

static const char *TAG = "HTTP_CLIENT";
char *api_key = "0SDHEYLFIOWCVOIR";
gpio_num_t dht_gpio = DHTPIN; /* Digital pin connected to the DHT sensor */
dht_sensor_type_t sensor_type = DHT_TYPE_AM2301;

float temperature;
float press = 0.0;
float humidity;

void send_data_to_thingspeak(void *pvParameters){
   char *thingspeak_url = "https://api.thingspeak.com";
   char data[] = "/update?api_key=%s&field1=%2.3f&field2=%2.3f&field3=%2.3f";
   char post_data[200];
   esp_err_t err;

   esp_http_client_config_t config = {
      .url = thingspeak_url,
      .method = HTTP_METHOD_GET,
   };

   esp_http_client_handle_t client = esp_http_client_init(&config);
   esp_http_client_set_header(client, "Content-Type", "application/x-www-form-urlencoded");

   while (true){
      if (dht_read_float_data(sensor_type, dht_gpio, &humidity, &temperature) == ESP_OK)
            ESP_LOGI(TAG, "Humidity: %2.3f%% Temp: %2.3fC", humidity, temperature);
        else
            ESP_LOGE(TAG, "Could not read data from sensor");


      vTaskDelay(250 / portTICK_PERIOD_MS);
      strcpy(post_data, "");
      snprintf(post_data, sizeof(post_data), data, api_key, temperature, press, humidity);
      ESP_LOGI(TAG, "post = %s", post_data);
      //esp_http_client_set_post_field(client, post_data, strlen(post_data));
      esp_http_client_set_url(client, post_data);

      err = esp_http_client_perform(client);

      if (err == ESP_OK){
         int status_code = esp_http_client_get_status_code(client);
         if (status_code == 200){
            ESP_LOGI(TAG, "Message sent Successfully");
         }
         else{
            ESP_LOGI(TAG, "Message sent Failed");            
            goto exit;
         }
      }
      else{
         ESP_LOGI(TAG, "Message sent Failed");
         goto exit;
      }
   }
exit:
   esp_http_client_cleanup(client);
   vTaskDelete(NULL);
}

void app_main(void){
   esp_err_t ret = nvs_flash_init();

   if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND){
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
   }

   ESP_ERROR_CHECK(ret);
   
   connect_wifi();

   if (wifi_connect_status){
      xTaskCreate(&send_data_to_thingspeak, "send_data_to_thingspeak", 8192, NULL, 6, NULL);
   }
}
