#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h" //Librería UART
#include "string.h"
#include "dht.h"
//#include "soc/uart_struct.h"

//UART
#define UART_PORT_NUM UART_NUM_1 //Seleccionas el uart 1 que trae por default el ESP
#define TX_BUF_SIZE 1024 //Tamaño de buf de 8bits
#define TXD_PIN GPIO_NUM_1 //pin que manda 
#define RXD_PIN GPIO_NUM_3 //pin que recibe

////DHT22
#define DHTPIN GPIO_NUM_26
float temperature = 0;

const char *TAG = "DHT22 SENSOR";

gpio_num_t dht_gpio = DHTPIN; //Digital pin connected to the DHT
dht_sensor_type_t sensor_type = DHT_TYPE_AM2301; //Para DHT11 -> DHT_TYPE_DH11

esp_err_t temperature_task();

///////////////////////////UART
esp_err_t uart_initialize();
static void tx_task(void *arg);

void app_main(){
    uart_initialize();
    //creamos tarea tx_task, lleva 
    xTaskCreate(tx_task, "uart_tx_task", TX_BUF_SIZE*2, NULL, configMAX_PRIORITIES-1, NULL); //El buf es una pila de datos, si se llena se tiene que vaciar para recibir datos
}

esp_err_t uart_initialize(){  
    const uart_config_t uart_config = {
        .baud_rate = 115200,  //baudios
        .data_bits = UART_DATA_8_BITS,  //1byte
        .parity = UART_PARITY_DISABLE, //Paritybit, checa si se envian correctamente los datos por bit, al final cuenta los datos que se enviaron correctamente, par correcto, impar incorrecto
        .stop_bits = UART_STOP_BITS_1, //bit de paro
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    // We won't use a buffer for sending data.
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT_NUM, TX_BUF_SIZE * 2, 0, 0, NULL, 0)); //2 datos, uno null y otro dato
    ESP_ERROR_CHECK(uart_param_config(UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT_NUM, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    return ESP_OK;
}

static void tx_task(void *arg){
    char str[80];

   
   //puts(str);
   
   
    
    while (1) {
        const int len = strlen(str); //cuenta tamaño de la cadena //len tiene el tamaño de la cadena
        ESP_ERROR_CHECK(temperature_task());
        uart_write_bytes(UART_PORT_NUM, str, len+1); //longitud de la cadena, se puede poner len-1, etc para cortar un caracter
        sprintf(str, "\nTemperature = %2.2f", temperature);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

esp_err_t temperature_task(){
    float humidity = 0;
    if (dht_read_float_data(sensor_type, dht_gpio, &humidity, &temperature) == ESP_OK)
        vTaskDelay(pdMS_TO_TICKS(100));
    else 
        ESP_LOGE(TAG, "Could not read data from sensor");

    return ESP_OK;
}
