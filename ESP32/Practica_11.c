
/*
Sistema de acceso 
Programa para el ESP32 detecte presencia con un sensor IR encienda un led amarillo y mande info al ESP
se pide un usuario y contrasenia se mueve un servo 0-180 grads y otro 60-120 grads en modo de "escaneo facial"
en caso de usuario y contra corectos led verde (azul en nuestro caso) enciende y buzzer beepea una vez 
en caso de error se enciende led rojo y buzzer  beepea intermitente como alarma 
el usuario y contra se ingresan por medio de bluetooth 
cuando se pide usuario y log es por medio de LOGI: 
AUTOR: Tecnoteam alphabuenamaravillaondaescuadronlobo siu 
Fecha de creacion: 3/04/2024
Fecha de modificacion: 4/04/2024

*/

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <inttypes.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"
#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"

#include "time.h"
#include "sys/time.h"

#define SPP_TAG "SPP CLASS"
#define SPP_SERVER_NAME "SPP_SERVER"
#define EXAMPLE_DEVICE_NAME "COMPANIERITO"
#define SPP_SHOW_DATA 0
#define SPP_SHOW_SPEED 1
#define SPP_SHOW_MODE SPP_SHOW_DATA    //*Choose show mode: show data or speed*//

#define LEDR 2
#define LEDV 5
#define BUZZER 4
#define SIR 18  // Sensor IR de presencia
#define ON 1
#define OFF 0
int dato_ingresado =0;


/* Set the parameters according to your servo */
#define SERVO_MIN_PULSEWIDTH_US 500   // Minimum pulse width in microseconds
#define SERVO_MAX_PULSEWIDTH_US 2400  // Maximum pulse width in microseconds
#define SERVO_MIN_DEGREE 0            // Minimum angle
#define SERVO_MAX_DEGREE 180          // Maximum angle
#define SERVO_PULSE_GPIO_1 21         // servo 1
#define SERVO_PULSE_GPIO_2 22         //  servo 2
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD 20000            // 20000 ticks, 20ms
;

static const char *TAG = "PWM servo";

mcpwm_timer_handle_t timer = NULL;
mcpwm_oper_handle_t oper = NULL;
mcpwm_cmpr_handle_t comparator1 = NULL;
mcpwm_cmpr_handle_t comparator2 = NULL;
mcpwm_gen_handle_t generator1 = NULL;
mcpwm_gen_handle_t generator2 = NULL;

esp_err_t mcpwm_config();
static inline uint32_t angle_to_compare(int angle, int min_degree, int max_degree);


static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
static const bool esp_spp_enable_l2cap_ertm = true;

static struct timeval time_new, time_old;
static long data_num = 0;

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

static char *bda2str(uint8_t * bda, char *str, size_t size)
{
    if (bda == NULL || str == NULL || size < 18) {
        return NULL;
    }

    uint8_t *p = bda;
    sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x",
            p[0], p[1], p[2], p[3], p[4], p[5]);
    return str;
}

static void print_speed(void)
{
    float time_old_s = time_old.tv_sec + time_old.tv_usec / 1000000.0;
    float time_new_s = time_new.tv_sec + time_new.tv_usec / 1000000.0;
    float time_interval = time_new_s - time_old_s;
    float speed = data_num * 8 / time_interval / 1000.0;
    ESP_LOGI(SPP_TAG, "speed(%fs ~ %fs): %f kbit/s" , time_old_s, time_new_s, speed);
    data_num = 0;
    time_old.tv_sec = time_new.tv_sec;
    time_old.tv_usec = time_new.tv_usec;
}

esp_err_t turn_on_BUZZERR();
esp_err_t turn_on_LED();
esp_err_t turn_off_LED();
esp_err_t turn_on_LED_Error();
esp_err_t servos_start();


////////////////////////////////////////////////////////////////////////////////////

//funcion de inicialisacion de leds y buzzer
esp_err_t initialize_LED(){
    gpio_reset_pin(LEDV);
    gpio_set_direction(LEDV, GPIO_MODE_OUTPUT);

    gpio_reset_pin(LEDR);
    gpio_set_direction(LEDR, GPIO_MODE_OUTPUT);

    gpio_reset_pin(BUZZER);
    gpio_set_direction(BUZZER, GPIO_MODE_OUTPUT);

    gpio_reset_pin(SIR);
    gpio_set_direction(SIR, GPIO_MODE_INPUT);

    return ESP_OK;
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//funcion de alarmado 

esp_err_t turn_on_BUZZERR(){
    
    while (gpio_get_level(LEDR)){
    gpio_set_level(BUZZER, ON);
     vTaskDelay(250 / portTICK_PERIOD_MS); // Enciende el buzzer durante 250 ms
      gpio_set_level(BUZZER, OFF);
      vTaskDelay(250 / portTICK_PERIOD_MS); // Apaga el buzzer durante 250 ms
        // segun yo deberia funcionar si no solo poner if LEDR ON{turn_on_BUZZERR()}
    } 
    return ESP_OK; 
}

esp_err_t turn_on_LED(){
    // esta funcion sra para cuando es correcto encendemos led verde 
    gpio_set_level(LEDV, ON);
    gpio_set_level(LEDR, OFF);
    // beepea una vez el buzzer 
    gpio_set_level(BUZZER, ON);
     vTaskDelay(500 / portTICK_PERIOD_MS); // Enciende el buzzer durante 250 ms
      gpio_set_level(BUZZER, OFF);

    return ESP_OK;
}

esp_err_t turn_off_LED(){
    // esta funcion es pra cuando queremos apagar el led verde porque ya termino el tiempo de acceso 
    gpio_set_level(LEDV, OFF);
    // tambien verificamos que esten apagado el led rojo 
    gpio_set_level(LEDR, OFF);
    // tambien verificamos que esten apagado el Buzzer
    gpio_set_level(BUZZER, OFF);

    return ESP_OK;
}

esp_err_t turn_on_LED_Error(){
    // esta funcion es para para cuando es incorrecto encendemos led rojo
    gpio_set_level(LEDR, ON);
    gpio_set_level(LEDV, OFF);
   // se llama a la funcion de la alarma tambien pa que alarme vea
    turn_on_BUZZERR(); 

    return ESP_OK;
}

//////////////////////////////////////////////////////////////////////////////////////
//funcion servos 

esp_err_t servos_start(){
    int step1 = 179;  // Step servo 1
    int angle1 = 1;  // angulo for servo 1

    int step2 = 59;   // Step servo 2
    int angle2 = 61; // angle for servo 2

    while (1) {
        ESP_LOGI(TAG, "Angle of rotation for servo 1: %d", angle1);
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator1, angle_to_compare(angle1, SERVO_MIN_DEGREE, SERVO_MAX_DEGREE)));

        ESP_LOGI(TAG, "Angle of rotation for servo 2: %d", angle2);
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, angle_to_compare(angle2, 60, 120)));

        
        vTaskDelay(pdMS_TO_TICKS(500));

        // paso servo 1
        if ((angle1 + step1) > SERVO_MAX_DEGREE) {
            angle1 = 0;
        }
        angle1 += step1;

        // paso servo 2
        if ((angle2 + step2) > 120) {
            angle2 = 60;
        }
        angle2 += step2;
    }
}
    ////////////////////////////////////////////////////////////////////////////////////////


static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    char bda_str[18] = {0};

    switch (event) {
    case ESP_SPP_INIT_EVT:
        if (param->init.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
            esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
        } else {
            ESP_LOGE(SPP_TAG, "ESP_SPP_INIT_EVT status:%d", param->init.status);
        }
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
        break;
    case ESP_SPP_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
        break;
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT status:%d handle:%"PRIu32" close_by_remote:%d", param->close.status,
                 param->close.handle, param->close.async);
        break;
    case ESP_SPP_START_EVT:
        if (param->start.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT handle:%"PRIu32" sec_id:%d scn:%d", param->start.handle, param->start.sec_id,
                     param->start.scn);
            esp_bt_dev_set_device_name(EXAMPLE_DEVICE_NAME);
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
        } else {
            ESP_LOGE(SPP_TAG, "ESP_SPP_START_EVT status:%d", param->start.status);
        }
        break;
    case ESP_SPP_CL_INIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
        break;
    case ESP_SPP_DATA_IND_EVT:
#if (SPP_SHOW_MODE == SPP_SHOW_DATA)
    /*
     * We only show the data in which the data length is less than 128 here. If you want to print the data and
     * the data rate is high, it is strongly recommended to process them in other lower priority application task
     * rather than in this callback directly. Since the printing takes too much time, it may stuck the Bluetooth
     * stack and also have a effect on the throughput!
     */
    /*ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT len:%d handle:%"PRIu32,
             param->data_ind.len, param->data_ind.handle);
    if (param->data_ind.len < 128) {
        esp_log_buffer_hex("", param->data_ind.data, param->data_ind.len);
    } */
    // Asignar memoria dinámicamente para el buffer de datos data_str 
    char *data_str = (char *)malloc(param->data_ind.len + 1);
    if (data_str == NULL) {
        ESP_LOGE(SPP_TAG, "Error al asignar memoria para data_str");
        return;
    }

    // Copiar los datos recibidos al buffer de cadena
    memcpy(data_str, param->data_ind.data, param->data_ind.len);
    data_str[param->data_ind.len] = '\0'; // Asegurarse de que la cadena esté terminada en null

    for (size_t i = 0; i < param->data_ind.len; i++){
            if (data_str[i] == '\r' || data_str[i] == '\n') {
            data_str[i] = '\0'; // Reemplazar '\r' o '\n' con el carácter nulo
            break; // Salir del bucle una vez que se encuentre el primer '\r' o '\n'
             }
        }

    // Imprimir la cadena de caracteres recibida
    ESP_LOGI(SPP_TAG, "Data received: %s", data_str);

       switch(dato_ingresado){
            case 0:
                    turn_off_LED();
                    break;

            case 1:
                if(!strcmp(data_str, "ON") ){
                    dato_ingresado=2;
                    ESP_LOGI(SPP_TAG, "Usuario correcto. Por favor ingrese la contraseña.");
                    turn_on_LED();
                    vTaskDelay(2000 / portTICK_PERIOD_MS);
                    turn_off_LED();
                } else {
                ESP_LOGI(SPP_TAG, "Usuario incorrecto");
                turn_on_LED_Error();
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                 dato_ingresado = 0 ; 
                }
                break;
            case 2:
                if(!strcmp(data_str, "OFF")){
                    ESP_LOGI(SPP_TAG, "Contraseña correcta");
                    turn_on_LED(); // Realizar alguna acción si el usuario y la contraseña son correctos
                    vTaskDelay(2000 / portTICK_PERIOD_MS);
                    dato_ingresado=0; 

                } else {
                ESP_LOGI(SPP_TAG, "Contraseña incorrecta");
                turn_on_LED_Error(); 
                vTaskDelay(2000 / portTICK_PERIOD_MS);
                dato_ingresado=0;
                }    
               break;    
                    
       }

    free(data_str); // Liberar la memoria asignada dinámicamente
#else
        gettimeofday(&time_new, NULL);
        data_num += param->data_ind.len;
        if (time_new.tv_sec - time_old.tv_sec >= 3) {
            print_speed();
        }
#endif
        break;
    case ESP_SPP_CONG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT");
        break;
    case ESP_SPP_WRITE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT");
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT status:%d handle:%"PRIu32", rem_bda:[%s]", param->srv_open.status,
                 param->srv_open.handle, bda2str(param->srv_open.rem_bda, bda_str, sizeof(bda_str)));
        gettimeofday(&time_old, NULL);
        break;
    case ESP_SPP_SRV_STOP_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_STOP_EVT");
        break;
    case ESP_SPP_UNINIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_UNINIT_EVT");
        break;
    default:
        break;
    }
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    char bda_str[18] = {0};

    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT:{
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(SPP_TAG, "authentication success: %s bda:[%s]", param->auth_cmpl.device_name,
                     bda2str(param->auth_cmpl.bda, bda_str, sizeof(bda_str)));
        } else {
            ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
        }
        break;
    }
    case ESP_BT_GAP_PIN_REQ_EVT:{
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
        if (param->pin_req.min_16_digit) {
            ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
            esp_bt_pin_code_t pin_code = {0};
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
        } else {
            ESP_LOGI(SPP_TAG, "Input pin code: 1234");
            esp_bt_pin_code_t pin_code;
            pin_code[0] = '1';
            pin_code[1] = '2';
            pin_code[2] = '3';
            pin_code[3] = '4';
            esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
        }
        break;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %"PRIu32, param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%"PRIu32, param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        break;
#endif

    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_MODE_CHG_EVT mode:%d bda:[%s]", param->mode_chg.mode,
                 bda2str(param->mode_chg.bda, bda_str, sizeof(bda_str)));
        break;

    default: {
        ESP_LOGI(SPP_TAG, "event: %d", event);
        break;
    }
    }
    return;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////

void app_main(void)
{
    mcpwm_config();
    initialize_LED();
    if(gpio_get_level(SIR) == ON ){
        servos_start();
        dato_ingresado = 1 ;
    }
    char bda_str[18] = {0};
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));
    
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    esp_spp_cfg_t bt_spp_cfg = {
        .mode = esp_spp_mode,
        .enable_l2cap_ertm = esp_spp_enable_l2cap_ertm,
        .tx_buffer_size = 0, /* Only used for ESP_SPP_MODE_VFS mode */
    };
    if ((ret = esp_spp_enhanced_init(&bt_spp_cfg)) != ESP_OK) {
        ESP_LOGE(SPP_TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    /* Set default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

    /*
     * Set default parameters for Legacy Pairing
     * Use variable pin, input pin code when pairing
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);

    ESP_LOGI(SPP_TAG, "Own address:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));
}

/////////////////////////////////////////////////////////////////////////////////////////////

// configuracion servos 
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
        .group_id = 0,  // operator must be in the same group to the timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    ESP_LOGI(TAG, "Connect timer and operator");
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    ESP_LOGI(TAG, "Create comparators and generators for servo 1 and servo 2");

    // servo 1
    mcpwm_comparator_config_t comparator1_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator1_config, &comparator1));

    mcpwm_generator_config_t generator1_config = {
        .gen_gpio_num = SERVO_PULSE_GPIO_1,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator1_config, &generator1));

    // servo 2
    mcpwm_comparator_config_t comparator2_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator2_config, &comparator2));

    mcpwm_generator_config_t generator2_config = {
        .gen_gpio_num = SERVO_PULSE_GPIO_2,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator2_config, &generator2));

    // Set the initial compare value, so that the servos will spin to the center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator1, angle_to_compare(0, SERVO_MIN_DEGREE, SERVO_MAX_DEGREE)));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, angle_to_compare(60, 60, 120)));

    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    // For servo 1
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator1, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator1, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator1, MCPWM_GEN_ACTION_LOW)));

    // For servo 2
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator2, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator2, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator2, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    return ESP_OK;
}

static inline uint32_t angle_to_compare(int angle, int min_degree, int max_degree) {
    return (angle - min_degree) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (max_degree - min_degree) + SERVO_MIN_PULSEWIDTH_US;
}

////////////////////////////////////////////////////////////////////////////////////////////////
