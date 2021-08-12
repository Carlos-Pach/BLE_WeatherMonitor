#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include <driver/i2c.h>
#include <driver/uart.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

#include "gatts_table_weather.h"

// I2C parameters
#define I2C_SLAVE_NUM               (I2C_NUM_0)
#define I2C_SLAVE_SDA_IO            (4UL)
#define I2C_SLAVE_SCL_IO            (5UL)
#define SLAVE_ADDR                  (0x07)

// UART parameters
#define BAUD_RATE                   (115200UL)
#define UART_BUF_SIZE               (1024UL)
#define UART_NUM                    (UART_NUM_0)

// BLE parameters
#define PROFILE_NUM                 (1UL)
#define PROFILE_APP_IDX             (PROFILE_NUM - 1)
#define ESP_APP_ID                  (0x55)
#define SAMPLE_DEVICE_NAME          "ESP_WTHR_DEMO"
#define GATTS_TABLE_TAG             "GATTS_TABLE_DEMO"
#define SVC_INST_ID                 0  

#define BATTERY_UUID                (0x180F)        // battery level data UUID (GATT service)
#define BATTERY_LEVEL_UUID          (0x2A19)        // battery level measure UUID (GATT characteristic object type)

#define ENV_UUID                    (0x181A)        // environment sensor data UUID (GATT service)
#define PRESSURE_UUID               (0x2A6D)        // pressure measurement (GATT characteristic object type)
#define TEMP_UUID                   (0x2A6E)        // temperature measurement (GATT characteristic object type)
#define HUMID_UUID                  (0x2A6F)        // humidity measurement (GATT characteristic object type)

#define HRS_UUID                    (0x180D)        // heart rate sensor data UUID

#define ENABLE_GAP                  (1UL)           // used for testing GAP/GATT BLE

#define GATTS_DEMO_CHAR_VAL_LEN_MAX (500UL)
#define PREPARE_BUF_MAX_SIZE        (1024UL)
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)

static uint8_t adv_config_done       = 0 ;

uint16_t weather_handle_table[WEATHER_IDX_NB] ;

typedef struct {
    uint8_t                 *prepare_buf ;
    int                     prepare_len ;
} prepare_type_env_t ;

static prepare_type_env_t prepare_write_env ;

#define CONFIG_SET_RAW_ADV_DATA
#ifdef CONFIG_SET_RAW_ADV_DATA
static uint8_t raw_adv_data[] = {
        /* flags */
        0x02, 0x01, 0x06,
        /* tx power*/
        0x02, 0x0a, 0xeb,
        /* service uuid */
        0x03, 0x03, 0xFF, 0x00,
        /* device name */
        //0x0f, 0x09, 'E', 'S', 'P', '_', 'G', 'A', 'T', 'T', 'S', '_', 'D','E', 'M', 'O'
        0x0F, 0x09, 'E', 'S', 'P', '_', 'W', 'T', 'H', 'R', '_', 'D', 'E', 'M', 'O'
};
static uint8_t raw_scan_rsp_data[] = {
        /* flags */
        0x02, 0x01, 0x06,
        /* tx power */
        0x02, 0x0a, 0xeb,
        /* service uuid */
        0x03, 0x03, 0xFF,0x00
};

#else
static uint8_t service_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

/* The length of adv data must be less than 31 bytes */
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval        = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance          = 0x00,
    .manufacturer_len    = 0,    //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //test_manufacturer,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(service_uuid),
    .p_service_uuid      = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp        = true,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x0006,
    .max_interval        = 0x0010,
    .appearance          = 0x00,
    .manufacturer_len    = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(service_uuid),
    .p_service_uuid      = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
#endif /* CONFIG_SET_RAW_ADV_DATA */

static esp_ble_adv_params_t adv_params = {
    .adv_int_min         = 0x20,
    .adv_int_max         = 0x40,
    .adv_type            = ADV_TYPE_IND,
    .own_addr_type       = BLE_ADDR_TYPE_PUBLIC,
    .channel_map         = ADV_CHNL_ALL,
    .adv_filter_policy   = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
					esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst weather_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/* Service */
static const uint16_t GATTS_SERVICE_UUID_TEST      = ENV_UUID;        // 0x00FF --> ENV_UUID
static const uint16_t GATTS_CHAR_UUID_TEST_A       = TEMP_UUID;       // 0xFF01 --> TEMP_UUID
static const uint16_t GATTS_CHAR_UUID_TEST_B       = HUMID_UUID ;

#if 0
static const uint16_t GATTS_SERVICE_UUID_TEST_BATT = BATTERY_UUID ;
static const uint16_t GATTS_CHAR_UUID_TEST_C       = BATTERY_LEVEL_UUID ;
#endif

static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

#if 0
static const uint8_t char_prop_read                =  ESP_GATT_CHAR_PROP_BIT_READ;    // error: defined but not used
static const uint8_t char_prop_write               = ESP_GATT_CHAR_PROP_BIT_WRITE;    // error: defined but not used
static const uint8_t char_prop_read_write_notify   = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_notify              = ESP_GATT_CHAR_PROP_BIT_NOTIFY ;  // error: defines but not used
#endif 

static const uint8_t char_prop_read_notify         = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY ;

#if 0
//static const uint8_t heart_measurement_ccc[2]      = {0x00, 0x00};
//static const uint8_t char_value[4]                 = {0x11, 0x22, 0x33, 0x44};
#endif

static const uint8_t weather_ccc[2]                = {0x00, 0x00} ;   // error: defined but not used
//static const uint8_t battery_ccc[2]                 = {0x00, 0x00} ;


// misc parameters
#define I2C_PERIOD_MS               (1000UL)
#define DATA_LEN                    (512UL)
#define I2C_BUF_SIZE                (2UL)
#define I2C_DATA_LEN                (DATA_LEN << 1)
#define RTOS_STACK_SIZE             (1024UL)


// I2C global vars
volatile uint8_t buf[I2C_BUF_SIZE] = { [0 ... (I2C_BUF_SIZE - 1) ] = '\n' },     // removed 'static' keyword, added 'volatile'
                 temperatureVal,
                 humidityVal ;
static const uint8_t n = sizeof(buf)/sizeof(buf[0]) ;
volatile int32_t temperatureValBLE ;


// RTOS vars
SemaphoreHandle_t xSemaphore ;  // binary semaphore used to "lock" i2c buf



// function prototypes
esp_err_t slaveInitI2C(void) ;



/* Full Database Description - Used to add attributes into the database */
static const esp_gatts_attr_db_t gatt_db[WEATHER_IDX_NB] =
{
    // Service Declaration
    [IDX_SVC]        =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
      sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID_TEST), (uint8_t *)&GATTS_SERVICE_UUID_TEST}},

    
    /* Temperature */
    /* Characteristic Declaration */
    [IDX_CHAR_A]     =      // char_prop_read_write_notify --> char_prop_notify --> char_prop_read_notify
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    /* Characteristic Value */
    [CHAR_A_VAL]    =      // char_value --> buf --> temperatureVal ,   PERM_READ | PERM_WRITE --> PERM_READ
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_A, ESP_GATT_PERM_READ,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(temperatureVal), (uint8_t*)&temperatureVal}},

    /* Client Characteristic Configuration */
    [IDX_CHAR_CFG_A] =  // weather_ccc --> buf
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(weather_ccc), (uint8_t *)weather_ccc}},  

    
    /* Humidity */
    /* Characteristic Declaration */
    [IDX_CHAR_B]    =       // char_prop_notify --> char_prop_read_write_notify --> char_prop_read_notify
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    /* Characteristic Value */
    [CHAR_B_VAL]    =       // &buf[1] --> buf --> humidityVal  ,   PERM_READ | PERM_WRITE --> PERM_READ
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_B, ESP_GATT_PERM_READ,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(humidityVal), (uint8_t*)&humidityVal}},

    /* Client Characteristic Configuration */
    [IDX_CHAR_CFG_B] =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(weather_ccc), (uint8_t *)weather_ccc}},
};


/* TODO: Service Declaration (battery) */


#if ENABLE_GAP
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    #ifdef CONFIG_SET_RAW_ADV_DATA
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
    #else
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
    #endif
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            /* advertising start complete event to indicate advertising start successfully or failed */
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "advertising start failed");
            }else{
                ESP_LOGI(GATTS_TABLE_TAG, "advertising start successfully");
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "Advertising stop failed");
            }
            else {
                ESP_LOGI(GATTS_TABLE_TAG, "Stop adv successfully\n");
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
            break;
        default:
            break;
    }
}
#endif



void example_prepare_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(GATTS_TABLE_TAG, "prepare write, handle = %d, value len = %d", param->write.handle, param->write.len);
    esp_gatt_status_t status = ESP_GATT_OK;
    if (prepare_write_env->prepare_buf == NULL) {
        prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
        prepare_write_env->prepare_len = 0;
        if (prepare_write_env->prepare_buf == NULL) {
            ESP_LOGE(GATTS_TABLE_TAG, "%s, Gatt_server prep no mem", __func__);
            status = ESP_GATT_NO_RESOURCES;
        }
    } else {
        if(param->write.offset > PREPARE_BUF_MAX_SIZE) {
            status = ESP_GATT_INVALID_OFFSET;
        } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
            status = ESP_GATT_INVALID_ATTR_LEN;
        }
    }
    
    /*send response when param->write.need_rsp is true */
    if (param->write.need_rsp){
        esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
        if (gatt_rsp != NULL){
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK){
               ESP_LOGE(GATTS_TABLE_TAG, "Send response error");
            }
            free(gatt_rsp);
        }else{
            ESP_LOGE(GATTS_TABLE_TAG, "%s, malloc failed", __func__);
        }
    }
    if (status != ESP_GATT_OK){
        return;
    }
    memcpy(prepare_write_env->prepare_buf + param->write.offset,
           param->write.value,
           param->write.len);
    prepare_write_env->prepare_len += param->write.len;

}



void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && prepare_write_env->prepare_buf){
        esp_log_buffer_hex(GATTS_TABLE_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }else{
        ESP_LOGI(GATTS_TABLE_TAG,"ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}



static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    static int16_t tempInC, humidityBLE ;

    switch (event) {
        case ESP_GATTS_REG_EVT:{
            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);
            if (set_dev_name_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "set device name failed, error code = %x", set_dev_name_ret);
            }
    #ifdef CONFIG_SET_RAW_ADV_DATA
            esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
            if (raw_adv_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
            if (raw_scan_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
    #else
            //config adv data
            esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
            if (ret){
                ESP_LOGE(GATTS_TABLE_TAG, "config adv data failed, error code = %x", ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            //config scan response data
            ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
            if (ret){
                ESP_LOGE(GATTS_TABLE_TAG, "config scan response data failed, error code = %x", ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
    #endif
            esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, WEATHER_IDX_NB, SVC_INST_ID);
            if (create_attr_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "create attr table failed, error code = %x", create_attr_ret);
            }
        }
       	    break;
        case ESP_GATTS_READ_EVT:{   // curly bracket needed to create a variable inside of a case statement
            // create signed 16 bit variable for temperature
            #if 0
            int16_t tempInC, humidityBLE ;
            tempInC = ((temperatureVal - 32) / 1.8) * 100 ;
            humidityBLE = humidityVal * 100 ;
            #endif

            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_READ_EVT");
            ESP_LOGI(GATTS_TABLE_TAG, "Humidity: %d ,temp: %d", buf[0], buf[1]) ;
            
            // update attribute inside of table
            // check if read value does not exceed max value
            if(!param->read.is_long){
                // using param->set_attr_val.attr_handle does NOT update temperatureVal and humidityVal to client
                // instead use param->read.handle
                if(weather_handle_table[CHAR_A_VAL] == param->read.handle){
                    // update temperature value
                    tempInC = ((temperatureVal - 32) / 1.8) * 100 ;
                    esp_ble_gatts_set_attr_value(param->read.handle, sizeof(tempInC), &tempInC) ;
                } else if(weather_handle_table[CHAR_B_VAL] == param->read.handle){
                    // update humidity value
                    humidityBLE = humidityVal * 100 ;
                    esp_ble_gatts_set_attr_value(param->read.handle, sizeof(humidityBLE), &humidityBLE) ;
                }
                
                #if 0   // these lines update both characteristic values with the most recent function call (e.g. humidityVal)
                esp_ble_gatts_set_attr_value(param->read.handle, sizeof(temperatureVal), &temperatureVal) ;    // buf --> temperatureVal
                esp_ble_gatts_set_attr_value(param->read.handle, sizeof(humidityVal), &humidityVal) ;
                #endif
            } else{
                ESP_LOGI(GATTS_TABLE_TAG, "Long read: %d", param->read.is_long) ;
            }   
        }
            break;
        case ESP_GATTS_WRITE_EVT:
            if (!param->write.is_prep){
                // the data length of gattc write  must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
                ESP_LOGI(GATTS_TABLE_TAG, "GATT_WRITE_EVT, handle = %d, value len = %d, value :", param->write.handle, param->write.len);
                esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
                
                if (weather_handle_table[IDX_CHAR_CFG_A] == param->write.handle && param->write.len == 2){
                    uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                    if (descr_value == 0x0001){
                        ESP_LOGI(GATTS_TABLE_TAG, "notify enable");
                        uint8_t notify_data[15];
                        for (int i = 0; i < sizeof(notify_data); ++i)
                        {
                            notify_data[i] = i % 0xff;
                        }
                        //the size of notify_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, weather_handle_table[CHAR_A_VAL],
                                                sizeof(notify_data), notify_data, false);
                    } else if (descr_value == 0x0002){
                        ESP_LOGI(GATTS_TABLE_TAG, "indicate enable");
                        uint8_t indicate_data[15];
                        for (int i = 0; i < sizeof(indicate_data); ++i)
                        {
                            indicate_data[i] = i % 0xff;
                        }
                        //the size of indicate_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, weather_handle_table[CHAR_A_VAL],
                                            sizeof(indicate_data), indicate_data, true);
                    }
                    else if (descr_value == 0x0000){
                        ESP_LOGI(GATTS_TABLE_TAG, "notify/indicate disable ");
                    }else{
                        ESP_LOGE(GATTS_TABLE_TAG, "unknown descr value");
                        esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
                    }

                } 

                /* send response when param->write.need_rsp is true*/
                if (param->write.need_rsp){
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }else{
                /* handle prepare write */
                example_prepare_write_event_env(gatts_if, &prepare_write_env, param);
            }
      	    break;
        case ESP_GATTS_EXEC_WRITE_EVT:
            // the length of gattc prepare write data must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
            example_exec_write_event_env(&prepare_write_env, param);
            break;
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            break;
        case ESP_GATTS_CONF_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);

            if(weather_handle_table[IDX_CHAR_A] == param->read.handle){
                // update temperature value
                tempInC = ((temperatureVal - 32) / 1.8) * 100 ;
                esp_ble_gatts_set_attr_value(param->read.handle, sizeof(tempInC), &tempInC) ;
            } else if(weather_handle_table[IDX_CHAR_B] == param->read.handle){
                // update humidity value
                humidityBLE = humidityVal * 100 ;
                esp_ble_gatts_set_attr_value(param->read.handle, sizeof(humidityBLE), &humidityBLE) ;
            }
            
            break;
        case ESP_GATTS_START_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
            break;
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
            esp_log_buffer_hex(GATTS_TABLE_TAG, param->connect.remote_bda, 6);
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            /* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
            //start sent the update connection parameters to the peer device.
            esp_ble_gap_update_conn_params(&conn_params);
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:{
            if (param->add_attr_tab.status != ESP_GATT_OK){
                ESP_LOGE(GATTS_TABLE_TAG, "create attribute table failed, error code=0x%x", param->add_attr_tab.status);
            }
            else if (param->add_attr_tab.num_handle != WEATHER_IDX_NB){
                ESP_LOGE(GATTS_TABLE_TAG, "create attribute table abnormally, num_handle (%d) \
                        doesn't equal to WEATHER_IDX_NB(%d)", param->add_attr_tab.num_handle, WEATHER_IDX_NB);
            }
            else {
                ESP_LOGI(GATTS_TABLE_TAG, "create attribute table successfully, the number handle = %d\n",param->add_attr_tab.num_handle);
                memcpy(weather_handle_table, param->add_attr_tab.handles, sizeof(weather_handle_table));
                esp_ble_gatts_start_service(weather_handle_table[IDX_SVC]);
            }
            break;
        }
        case ESP_GATTS_STOP_EVT:
            break ;
        case ESP_GATTS_OPEN_EVT:
            break ;
        case ESP_GATTS_CANCEL_OPEN_EVT:
            break ;
        case ESP_GATTS_CLOSE_EVT:
            break ;
        case ESP_GATTS_LISTEN_EVT:
            break ;
        case ESP_GATTS_CONGEST_EVT:
            break ;
        case ESP_GATTS_UNREG_EVT:
            break ;
        case ESP_GATTS_DELETE_EVT:
            break ;
        default:
            break;
    }
}



static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            weather_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGE(GATTS_TABLE_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == weather_profile_tab[idx].gatts_if) {
                if (weather_profile_tab[idx].gatts_cb) {
                    weather_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}



/* 
    =======================================
    I2C Pins and Params

    SDA: GPIO4 (PUR disabled)
    SCL: GPIO5 (PUR disabled)

    interrupts: enabled
    =======================================
*/



static void StartTaskI2C(void *arg){
    // init local params
    static int8_t sz = 0 ;
    static uint8_t cnt = 0 ;
    esp_err_t stat = ESP_OK ;

    int8_t str[10] ;
    uint8_t m = sizeof(str)/sizeof(str[0]) ;

    // init I2C slave
    stat = slaveInitI2C() ;

    // check if I2C initialization was successful
    if(stat != ESP_OK){
        uart_write_bytes(UART_NUM, "ESP ERR\n", 8) ;
        return ;
    }

    // enter while loop
    while(1){
        // do stuff here
        //uart_write_bytes(UART_NUM, "Hello\n", 6) ;

        // lock i2c buf
        //xSemaphoreTake(xSemaphore, (TickType_t)10 / portTICK_PERIOD_MS) ;
            
        sz = i2c_slave_read_buffer(I2C_SLAVE_NUM, buf, n, I2C_PERIOD_MS / portTICK_RATE_MS ) ;
        
        if(sz > 0){
            
            // assign bytes from buffer to vars
            humidityVal = buf[0] ;
            temperatureVal = buf[1] ;
            
            #if 0
            // test to see if i2c data was received
            m = sprintf((char*)str, "%d\n", sz) ;
            uart_write_bytes(UART_NUM, (const char*)str, m) ;

            m = sprintf((char *)str, "Hum: %d\n", humidityVal) ;
            uart_write_bytes(UART_NUM, (const char*)str, m) ;    // const char --> char

            m = sprintf((char *)str, "Tmp: %d\n", temperatureVal) ;
            uart_write_bytes(UART_NUM, (const char*)str, m) ;    // const char --> char
            
            uart_write_bytes(UART_NUM, "\n", 1) ;
            cnt = 0 ;
            #endif
        } else if(sz == 0){
            uart_write_bytes(UART_NUM, "sz 0\n", 5) ;
        } else{
            //uart_write_bytes(UART_NUM, "sz err\n", 7) ;
            __asm("nop") ;
        }

        // release binary semaphore
        //xSemaphoreGive(xSemaphore) ;

        // reset fifo
        i2c_reset_tx_fifo(I2C_SLAVE_NUM) ;

        cnt++ ;

        vTaskDelay(I2C_PERIOD_MS / portTICK_PERIOD_MS) ;
    }
    vTaskDelete(NULL) ;
}



void app_main() {
    // init UART
    const uart_port_t uartNum = UART_NUM ;

    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    } ;
    uart_set_pin(uartNum, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) ;

    uart_param_config(uartNum, &uart_config) ;

    QueueHandle_t uart_queue ;
    uart_driver_install(uartNum, UART_BUF_SIZE, UART_BUF_SIZE, 10, &uart_queue, 0) ;

    // create binary semaphore
    xSemaphore = xSemaphoreCreateBinary() ;
    if(xSemaphore == NULL){
        uart_write_bytes(UART_NUM, "OK\n", 3) ;
    }

    // begin BLE init
    esp_err_t ret;

    /* Initialize NVS. */
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TABLE_TAG, "gatts register error, error code = %x", ret);
        return;
    }

    #if ENABLE_GAP
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TABLE_TAG, "gap register error, error code = %x", ret);
        return;
    }
    #endif

    ret = esp_ble_gatts_app_register(ESP_APP_ID);
    if (ret){
        ESP_LOGE(GATTS_TABLE_TAG, "gatts app register error, error code = %x", ret);
        return;
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret){
        ESP_LOGE(GATTS_TABLE_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    // create RTOS task/thread
    xTaskCreate(StartTaskI2C, "TaskI2C", RTOS_STACK_SIZE << 1, (void *)0, 10, NULL) ;
}



/* 
    ============================================
    init i2c struct in slave mode
    ============================================
*/
esp_err_t slaveInitI2C(void){
    // create i2c_port_t variable for i2c init
    i2c_port_t i2c_num = I2C_SLAVE_NUM ;

    // create i2c struct
    i2c_config_t stmSlave = {
        .sda_io_num = I2C_SLAVE_SDA_IO ,    // defined above
        .sda_pullup_en = GPIO_PULLUP_DISABLE ,
        .scl_io_num = I2C_SLAVE_SCL_IO ,    // defined above
        .scl_pullup_en = GPIO_PULLUP_DISABLE ,
        .mode = I2C_MODE_SLAVE ,            // slave mode
        .slave.addr_10bit_en = 0 ,          // 7 bit only
        .slave.slave_addr = SLAVE_ADDR      // defined above
    } ;

    // param config
    esp_err_t stat = ESP_OK ;
    
    stat = i2c_param_config(i2c_num, &stmSlave) ;
    if(stat != ESP_OK){
        uart_write_bytes(UART_NUM, "Param error\n", 12) ;
        return stat ;
    }
    
    // install driver
    stat = i2c_driver_install(i2c_num, stmSlave.mode, I2C_DATA_LEN, I2C_DATA_LEN, 0) ;
    if(stat != ESP_OK){
        uart_write_bytes(UART_NUM, "ESP ERR\n", 8) ;
        return stat ;
    }

    return stat ;
}