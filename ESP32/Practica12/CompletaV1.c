/**
 * DESCRIPCION: Programa para controlar un carrito mediante bluetooth. Utiliza red neuronal con 3 entradas (valores de acelaracion X,Y,Z), 
 * 2 capas ocultas y 5 salidas (5 posiciones: A - adelante, D - derecha, I - izquierda, R - reversa, S - alto). 
 * Sensor: MPU6050
 */

// LIBRERIAS
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
#include "console_uart.h"
#include "time.h"
#include "sys/time.h"
#include "mpu6050.h"
#include "driver/gpio.h"
#include <math.h>

// MACROS, CTES, VAR
/*BLUETOOTH*/
#define SPP_TAG "SPP_INITIATOR_DEMO"
#define EXAMPLE_DEVICE_NAME "ESP_SPC_INITIATOR"
#define SPP_SHOW_DATA 0
#define SPP_SHOW_SPEED 1
#define SPP_SHOW_MODE SPP_SHOW_DATA    /*Choose show mode: show data or speed*/
static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;
static const bool esp_spp_enable_l2cap_ertm = true;
static struct timeval time_new, time_old;
static long data_num = 0;
static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_NONE;     //ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_master = ESP_SPP_ROLE_MASTER;
esp_bd_addr_t peer_bd_addr = {0};
static uint8_t peer_bdname_len;
static char peer_bdname[ESP_BT_GAP_MAX_BDNAME_LEN + 1];
static const char remote_device_name[] = "ESP_SPC_ACCEPTOR"; //"ESP_SPP_ACCEPTOR";
static const esp_bt_inq_mode_t inq_mode = ESP_BT_INQ_MODE_GENERAL_INQUIRY;
static const uint8_t inq_len = 30;
static const uint8_t inq_num_rsps = 0;
#if (SPP_SHOW_MODE == SPP_SHOW_DATA)
#define SPP_DATA_LEN 20
#else
#define SPP_DATA_LEN ESP_SPP_MAX_MTU
#endif
static uint8_t spp_data[SPP_DATA_LEN];
static uint8_t *s_p_data = NULL; /* data pointer of spp_data */
char strX[80];
char strY[80];
char strZ[80];

/*MPU6050*/
#ifdef CONFIG_EXAMPLE_I2C_ADDRESS_LOW
#define ADDR MPU6050_I2C_ADDRESS_LOW
#else
#define ADDR MPU6050_I2C_ADDRESS_HIGH
#endif
#define SCL GPIO_NUM_22
#define SDA GPIO_NUM_21
static const char *TAG = "mpu6050_test";
#define GRAV 9.80665
float x;
float y;
float z;

/*Red Neuronal*/
#define INPUT_SIZE 3        // tres elementos de entrada = coordenadas x,y,z
#define HIDDEN_SIZE 2       // 2 neuronas en la capa interna
#define OUTPUT_SIZE 5       // 5 salidas (1 por categoria)
float mean[INPUT_SIZE]={0.83237043,-0.35830095,5.48635219};
float std[INPUT_SIZE]={6.18558559,3.31498169, 4.44293329};
float IW[INPUT_SIZE][HIDDEN_SIZE] = {                                                   // Pesos de la capa oculta
    {1.3321757,  5.328084},
    { 7.3920493,  -1.4556104},
    { 0.50585186, -0.6176774}
    };
float LW[HIDDEN_SIZE][OUTPUT_SIZE] = {                                                  // Pesos de la capa de salida
    {-10.016404,    -5.396266,    -0.74626297,   3.851712,     2.0110877},
    {0.08528665,  -3.9198296,    0.07594351,  -6.8958106,   -2.0529678}
    };
float b_1[HIDDEN_SIZE] = {0.45303446,2.19868};
float b_2[OUTPUT_SIZE] = {-0.5343409,   6.162336,   -9.768317,  -10.820537,   -2.6967258};
int max_index = 4;
char classes[OUTPUT_SIZE] = {'A', 'D', 'I', 'R', 'S'};

// FUNCIONES

/* Red NN*/
void scaler(float *X){                  // Funcion de StandardScaler
    for(int i=0; i<INPUT_SIZE;i++){
        X[i]-=mean[i];
        X[i]/=std[i];
    }
}

float relu(float x) {                   // Funcion de activacion ReLU -->  max(0,x)
    //return fmax(0, x);
    return x*(x>=0) + 0*(x<0);
}

void softmax(float *input, int size) {  // Funcion de activacion softmax --> exp(x) / sum_j exp(x_j)
    float max_val = input[0];          
    for (int i = 1; i < size; i++) {
        if (input[i] > max_val)
            max_val = input[i];
    }

    float sum = 0;
    for (int i = 0; i < size; i++) {
        // Restamos el valor maximo para evitar desbordamiento
        input[i] = exp(input[i] - max_val); 
        sum += input[i];
    }

    for (int i = 0; i < size; i++) {
        input[i] /= sum;
    }
}

void forward_pass(float *X, float *y_hat) {             // Propagacion hacia adelante de la red neuronal (entrada --> capas ocultas --> capa salida)
    float hidden[HIDDEN_SIZE];
    float output[OUTPUT_SIZE];

    // Calculamos las activaciones de la capa oculta (fn ReLU)
    scaler(X);
    for (int i = 0; i < HIDDEN_SIZE; i++) {
        hidden[i] = 0;
        for (int j = 0; j < INPUT_SIZE; j++) {
            hidden[i] += X[j] * IW[j][i];           // IW * X
        }
        
        hidden[i] += b_1[i];                        // IW * X + b1
        hidden[i] = relu(hidden[i]);                // p1(IW * X + b1)
    } 


    // Calculamos las activaciones de la capa de salida (fn softmax)
    for (int i = 0; i < OUTPUT_SIZE; i++) {
        output[i] = 0;
        for (int j = 0; j < HIDDEN_SIZE; j++) {
            output[i] += hidden[j] * LW[j][i];      // LW * p1(IW * X + b1)
        }

        output[i] += b_2[i];                        // LW * p1(IW * X + b1) + b2
    }

    // Aplicamos p2 (softmax)
    softmax(output, OUTPUT_SIZE);                   // p2(LW * p1(IW * X + b1) + b2)

    // Guardamos los resultados en arreglo y_gorrito (aqui se guarda la probabilidad a pertenecer a cada clase)
    for (int i = 0; i < OUTPUT_SIZE; i++) {         
        y_hat[i] = output[i];
    }
}

int argmax(float *arr, int size) {          // Recuperamos el indice de mayor probabilidad 
    int max_index = 0;
    float max_value = arr[0];
    for (int i = 1; i < size; i++) {
        if (arr[i] > max_value) {
            max_value = arr[i];
            max_index = i;
        }
    }
    return max_index;
}

void classify(){
    float X[INPUT_SIZE] = {x, y, z};        // INPUT: Datos de prueba
    float y_hat[OUTPUT_SIZE];               // Realizamos Foward-pass
    forward_pass(X, y_hat);
    max_index = argmax(y_hat, OUTPUT_SIZE);   // Recuperamos el indice de yn_hat con mayor probabilidad}
}


/*MPU6050*/
void read_mpu6050(void *args){
    mpu6050_dev_t dev = { 0 };

    ESP_ERROR_CHECK(mpu6050_init_desc(&dev, ADDR, 0, SDA, SCL));

    while (1)
    {
        esp_err_t res = i2c_dev_probe(&dev.i2c_dev, I2C_DEV_WRITE);
        if (res == ESP_OK)
        {
            ESP_LOGI(TAG, "Found MPU60x0 device");
            break;
        }
        ESP_LOGE(TAG, "MPU60x0 not found");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_ERROR_CHECK(mpu6050_init(&dev));

    ESP_LOGI(TAG, "Accel range: %d", dev.ranges.accel);
    ESP_LOGI(TAG, "Gyro range:  %d", dev.ranges.gyro);
    while (1)
    {
        mpu6050_acceleration_t accel = { 0 };
        mpu6050_rotation_t rotation = { 0 };

        ESP_ERROR_CHECK(mpu6050_get_motion(&dev, &accel, &rotation));

        ESP_LOGI(TAG, "********");
        ESP_LOGI(TAG, "Acceleracion: x=%.4f   y=%.4f   z=%.4f", accel.x, accel.y, accel.z);

        x =  accel.x * GRAV;
        y = accel.y * GRAV;
        z = accel.z * GRAV;

        classify();
        vTaskDelay(pdMS_TO_TICKS(200));    
    }
}

/*Bluetooth*/

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

static bool get_name_from_eir(uint8_t *eir, char *bdname, uint8_t *bdname_len)
{
    uint8_t *rmt_bdname = NULL;
    uint8_t rmt_bdname_len = 0;

    if (!eir) {
        return false;
    }

    rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_CMPL_LOCAL_NAME, &rmt_bdname_len);
    if (!rmt_bdname) {
        rmt_bdname = esp_bt_gap_resolve_eir_data(eir, ESP_BT_EIR_TYPE_SHORT_LOCAL_NAME, &rmt_bdname_len);
    }

    if (rmt_bdname) {
        if (rmt_bdname_len > ESP_BT_GAP_MAX_BDNAME_LEN) {
            rmt_bdname_len = ESP_BT_GAP_MAX_BDNAME_LEN;
        }

        if (bdname) {
            memcpy(bdname, rmt_bdname, rmt_bdname_len);
            bdname[rmt_bdname_len] = '\0';
        }
        if (bdname_len) {
            *bdname_len = rmt_bdname_len;
        }
        return true;
    }

    return false;
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    uint8_t i = 0;
    char bda_str[18] = {0};

    switch (event) {
    case ESP_SPP_INIT_EVT:
        if (param->init.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
            esp_bt_dev_set_device_name(EXAMPLE_DEVICE_NAME);
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            esp_bt_gap_start_discovery(inq_mode, inq_len, inq_num_rsps);
        } else {
            ESP_LOGE(SPP_TAG, "ESP_SPP_INIT_EVT status:%d", param->init.status);
        }
        break;
    case ESP_SPP_DISCOVERY_COMP_EVT:
        if (param->disc_comp.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT scn_num:%d", param->disc_comp.scn_num);
            for (i = 0; i < param->disc_comp.scn_num; i++) {
                ESP_LOGI(SPP_TAG, "-- [%d] scn:%d service_name:%s", i, param->disc_comp.scn[i],
                         param->disc_comp.service_name[i]);
            }
            /* We only connect to the first found server on the remote SPP acceptor here */
            esp_spp_connect(sec_mask, role_master, param->disc_comp.scn[0], peer_bd_addr);
        } else {
            ESP_LOGE(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT status=%d", param->disc_comp.status);
        }
        break;
    case ESP_SPP_OPEN_EVT:
        if (param->open.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT handle:%"PRIu32" rem_bda:[%s]", param->open.handle,
                     bda2str(param->open.rem_bda, bda_str, sizeof(bda_str)));
            /* Start to write the first data packet */
            esp_spp_write(param->open.handle, SPP_DATA_LEN, spp_data);
            s_p_data = spp_data;
            gettimeofday(&time_old, NULL);
        } else {
            ESP_LOGE(SPP_TAG, "ESP_SPP_OPEN_EVT status:%d", param->open.status);
        }
        break;
    case ESP_SPP_CLOSE_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT status:%d handle:%"PRIu32" close_by_remote:%d", param->close.status,
                 param->close.handle, param->close.async);
        break;
    case ESP_SPP_START_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT");
        break;
    case ESP_SPP_CL_INIT_EVT:
        if (param->cl_init.status == ESP_SPP_SUCCESS) {
            ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT handle:%"PRIu32" sec_id:%d", param->cl_init.handle, param->cl_init.sec_id);
        } else {
            ESP_LOGE(SPP_TAG, "ESP_SPP_CL_INIT_EVT status:%d", param->cl_init.status);
        }
        break;
    case ESP_SPP_DATA_IND_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT");
        break;
    case ESP_SPP_WRITE_EVT:
        if (param->write.status == ESP_SPP_SUCCESS) {
            if (s_p_data + param->write.len == spp_data + SPP_DATA_LEN) {
                /* Means the previous data packet be sent completely, send a new data packet */
                s_p_data = spp_data;
            } else {
                /*
                 * Means the previous data packet only be sent partially due to the lower layer congestion, resend the
                 * remainning data.
                 */
                s_p_data += param->write.len;
            }
#if (SPP_SHOW_MODE == SPP_SHOW_DATA)
            /*
             * We only show the data in which the data length is less than 128 here. If you want to print the data and
             * the data rate is high, it is strongly recommended to process them in other lower priority application task
             * rather than in this callback directly. Since the printing takes too much time, it may stuck the Bluetooth
             * stack and also have a effect on the throughput!
             */
            ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT len:%d handle:%"PRIu32" cong:%d", param->write.len, param->write.handle,
                     param->write.cong);
            if (param->write.len < 128 && !param->write.cong) {
                switch(max_index){
                    case 0:
                        //esp_log_buffer_hex("A", spp_data, param->write.len);
                        esp_spp_write(param->write.handle, 2,(unsigned char *)"A");
                        break;
                    case 1:
                        //esp_log_buffer_hex("D", spp_data, param->write.len);
                        esp_spp_write(param->write.handle, 2,(unsigned char *)"D"); 
                        break;
                    case 2:
                        //esp_log_buffer_hex("I", spp_data, param->write.len);
                        esp_spp_write(param->write.handle, 2, (unsigned char *)"I"); 
                        break;
                    case 3:
                        //esp_log_buffer_hex("R", spp_data, param->write.len);
                        esp_spp_write(param->write.handle,2,(unsigned char *) "R");  
                        break;
                    case 4:
                        //esp_log_buffer_hex("S", spp_data, param->write.len);
                        esp_spp_write(param->write.handle,2,(unsigned char *)"S");
                        break;
                }                
                
                /* Delay a little to avoid the task watch dog */
                vTaskDelay(pdMS_TO_TICKS(10));
            }
#else
            gettimeofday(&time_new, NULL);
            data_num += param->write.len;
            if (time_new.tv_sec - time_old.tv_sec >= 3) {
                print_speed();
            }
#endif
        } else {
            /* Means the prevous data packet is not sent at all, need to send the whole data packet again. */
            ESP_LOGE(SPP_TAG, "ESP_SPP_WRITE_EVT status:%d", param->write.status);
        }

        //if (!param->write.cong) {
        //    /* The lower layer is not congested, you can send the next data packet now. */
        //    esp_spp_write(param->write.handle, spp_data + SPP_DATA_LEN - s_p_data, s_p_data);
        //} else {
            /*
             * The lower layer is congested now, don't send the next data packet until receiving the
             * ESP_SPP_CONG_EVT with param->cong.cong == 0.
             */
        //    ;
        //}*/

        /*
         * If you don't want to manage this complicated process, we also provide the SPP VFS mode that hides the
         * implementation details. However, it is less efficient and will block the caller until all data has been sent.
         */
        break;
    case ESP_SPP_CONG_EVT:
#if (SPP_SHOW_MODE == SPP_SHOW_DATA)
        ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT cong:%d", param->cong.cong);
#endif
        if (param->cong.cong == 0) {
            /* Send the privous (partial) data packet or the next data packet. */
            esp_spp_write(param->write.handle, spp_data + SPP_DATA_LEN - s_p_data, s_p_data);
        }
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT");
        break;
    case ESP_SPP_UNINIT_EVT:
        ESP_LOGI(SPP_TAG, "ESP_SPP_UNINIT_EVT");
        break;
    default:
        break;
    }
}

static void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch(event){
    case ESP_BT_GAP_DISC_RES_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_DISC_RES_EVT");
        esp_log_buffer_hex(SPP_TAG, param->disc_res.bda, ESP_BD_ADDR_LEN);
        /* Find the target peer device name in the EIR data */
        for (int i = 0; i < param->disc_res.num_prop; i++){
            if (param->disc_res.prop[i].type == ESP_BT_GAP_DEV_PROP_EIR
                && get_name_from_eir(param->disc_res.prop[i].val, peer_bdname, &peer_bdname_len)){
                esp_log_buffer_char(SPP_TAG, peer_bdname, peer_bdname_len);
                if (strlen(remote_device_name) == peer_bdname_len
                    && strncmp(peer_bdname, remote_device_name, peer_bdname_len) == 0) {
                    memcpy(peer_bd_addr, param->disc_res.bda, ESP_BD_ADDR_LEN);
                    /* Have found the target peer device, cancel the previous GAP discover procedure. And go on
                     * dsicovering the SPP service on the peer device */
                    esp_bt_gap_cancel_discovery();
                    esp_spp_start_discovery(peer_bd_addr);
                }
            }
        }
        break;
    case ESP_BT_GAP_DISC_STATE_CHANGED_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_DISC_STATE_CHANGED_EVT");
        break;
    case ESP_BT_GAP_RMT_SRVCS_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_RMT_SRVCS_EVT");
        break;
    case ESP_BT_GAP_RMT_SRVC_REC_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_RMT_SRVC_REC_EVT");
        break;
    case ESP_BT_GAP_AUTH_CMPL_EVT:{
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(SPP_TAG, "authentication success: %s", param->auth_cmpl.device_name);
            esp_log_buffer_hex(SPP_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
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
        ESP_LOGW(SPP_TAG, "To confirm the value, type `spp ok;`");
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%"PRIu32, param->key_notif.passkey);
        ESP_LOGW(SPP_TAG, "Waiting responce...");
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        ESP_LOGW(SPP_TAG, "To input the key, type `spp key xxxxxx;`");
        break;
#endif

    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(SPP_TAG, "ESP_BT_GAP_MODE_CHG_EVT mode:%d", param->mode_chg.mode);
        break;

    default:
        break;
    }
}

void app_main(void)
{   
    ESP_ERROR_CHECK(i2cdev_init()); 
    xTaskCreate(read_mpu6050, "mpu6050", configMINIMAL_STACK_SIZE * 6, NULL, 5, NULL);
    esp_err_t ret = ESP_OK;
    char bda_str[18] = {0};

    for (int i = 0; i < SPP_DATA_LEN; ++i) {
        spp_data[i] = i;
    }

    ret = nvs_flash_init();
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

#if (CONFIG_BT_SSP_ENABLED == true)
    /* Set default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IN;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
    if (iocap == ESP_BT_IO_CAP_IN || iocap == ESP_BT_IO_CAP_IO) {
        console_uart_init();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
#endif

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

    /*
     * Set default parameters for Legacy Pairing
     * Use variable pin, input pin code when pairing
     */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
    esp_bt_pin_code_t pin_code;
    esp_bt_gap_set_pin(pin_type, 0, pin_code);

    ESP_LOGI(SPP_TAG, "Own address:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));
}
