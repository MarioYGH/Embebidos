#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define BUZZER 27
#define SENSOR_LUZ 23
#define SENSOR_OS 25
#define BOTON 33

int state;
int bot_state = 1;


esp_err_t pin_initialiize();
esp_err_t init_irs();
void isr_handler(void *args);

void app_main(void)
{


    pin_initialiize(); //reinicia el pin 
    init_irs();

    while(true){


        
            while(bot_state==1){
                
                vTaskDelay(pdMS_TO_TICKS(500));
                
                state=gpio_get_level(SENSOR_LUZ);
                gpio_set_level(BUZZER,state);

                printf("estoy en sensor de luz %d\n", state);

            }
            
            while(bot_state==0){

                vTaskDelay(pdMS_TO_TICKS(500));
                
                state=gpio_get_level(SENSOR_OS);
                gpio_set_level(BUZZER,state);

                printf("estoy en sensor de obscuridad %d\n", state);

            }


        

    }


}


//incializa el pin y retorna un error o no error de tip esp_err_t
esp_err_t pin_initialiize(){

    gpio_reset_pin(BUZZER);
    gpio_set_direction(BUZZER, GPIO_MODE_OUTPUT);

    gpio_reset_pin(SENSOR_LUZ);
    gpio_set_direction(SENSOR_LUZ, GPIO_MODE_INPUT);

    gpio_reset_pin(SENSOR_OS);
    gpio_set_direction(SENSOR_OS, GPIO_MODE_INPUT);

    gpio_reset_pin(BOTON);
    gpio_set_direction(BOTON, GPIO_MODE_INPUT);

    return ESP_OK;
}

esp_err_t init_irs(void) {

    gpio_config_t pGPIOConfig;
    pGPIOConfig.pin_bit_mask = (1ULL << BOTON);
    pGPIOConfig.mode = GPIO_MODE_DEF_INPUT;
    pGPIOConfig.pull_up_en = GPIO_PULLUP_DISABLE;
    pGPIOConfig.pull_down_en = GPIO_PULLUP_DISABLE;
    pGPIOConfig.intr_type = GPIO_INTR_NEGEDGE;

    gpio_config(&pGPIOConfig);

    gpio_install_isr_service(0); //habilita global interrupts
    gpio_isr_handler_add(BOTON, isr_handler, NULL); //pin y funcion a ejecutar

    return ESP_OK;

}

void isr_handler(void *args){

    if(bot_state == 1){
        bot_state = 0;
    }
    else {
        bot_state = 1;
    }
    
}
