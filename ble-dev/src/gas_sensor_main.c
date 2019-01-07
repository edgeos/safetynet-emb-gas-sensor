/**\mainpage
 * Copyright (C) 2018 GE Global Research
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of the
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 *
 * File		gas_sensor_main.c
 * Date		18 Dec 2018
 * Version	1
 *
 */

/*! @file gas_sensor_main.c
    @brief Main file for SafetyNet Gas Sensor BLE Application. This file initializes and instantiate the BLE SoftDevice,
     as well as a number of FreeRTOS Tasks and Timers for measuring and managing multiple external sensors, like the BME280 
     environmental sensor and the ADuCM355 impedance measurement sensor used for estimating gas TVOC.
*/

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_dis.h"
#include "ble_gas_srv.h"
#include "ble_conn_params.h"
#include "sensorsim.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_freertos.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "fds.h"
#include "ble_conn_state.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_rtc.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_drv_gpiote.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// internal peripheral contollers
#include "i2c_device_controller.h"

// gas sensor controller
#include "aducm355_controller.h"

// external devices (eeprom and temp humidity sensor)
#include "m24m02.h"
#include "bme280.h"

// nv flash
#include "nvflash_controller.h"

/**\name Macro Definitions */
#define BME280_MEASURE_INTERVAL                   1000                              /**< BME280 measurement interval (ms). */
#define ADUCM355_MEASURE_INTERVAL_CONNECTED       1700                              /**< ADUCM355 measurement interval (ms). */
#define ADUCM355_MEASURE_INTERVAL_DISCONNECTED    1700                              /**< ADUCM355 measurement interval (ms). */
#define ADUCM355_MEASURE_CHECK_INTERVAL           20                                 /**< ADUCM355 check rx buffer interval (ms). */
#define DEVICE_NAME                         "GE Gas Sensor"                         /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                   "GE"                                    /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_BLE_OBSERVER_PRIO               3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG                1                                       /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_ADV_INTERVAL                    300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */
#define APP_ADV_DURATION                    18000                                   /**< The advertising duration (180 seconds) in units of 10 milliseconds. */
#define BATTERY_LEVEL_MEAS_INTERVAL         2000                                    /**< Battery level measurement interval (ms). */
#define MIN_BATTERY_LEVEL                   81                                      /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL                   100                                     /**< Maximum simulated battery level. */
#define BATTERY_LEVEL_INCREMENT             1                                       /**< Increment between each simulated battery level measurement. */
#define MIN_CONN_INTERVAL                   MSEC_TO_UNITS(200, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL                   MSEC_TO_UNITS(400, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY                       0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                    MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory time-out (4 seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY      5000                                    /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY       30000                                   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT        3                                       /**< Number of attempts before giving up the connection parameter negotiation. */
#define SEC_PARAM_BOND                      0                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                      0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                      0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS                  0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES           BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                       0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE              7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE              16                                      /**< Maximum encryption key size. */
#define DEAD_BEEF                           0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define OSTIMER_WAIT_FOR_QUEUE              2                                       /**< Number of ticks to wait for the timer queue to be ready */

BLE_GAS_SRV_DEF(m_gas);                                             /**< Gas Sensor service instance. */
BLE_BAS_DEF(m_bas);                                                 /**< Battery service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                 /**< Advertising module instance. */

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;            /**< Handle of the current connection. */
static sensorsim_cfg_t   m_battery_sim_cfg;                         /**< Battery Level sensor simulator configuration. */
static sensorsim_state_t m_battery_sim_state;                       /**< Battery Level sensor simulator state. */
static bool m_ble_connected_bool = false;
static bool simulated_gas_sensor_data = true;
static char m_ble_advertising_name[MAX_BLE_NAME_LENGTH] = {0};

// global bools for handling phone-to-device ble commands (gas service config characteristic)
static bool read_eeprom_cmd = false;
static bool clear_eeprom_cmd = false;
static bool update_utc_time = false;
static bool update_aducm355_config = false;

// globals to store new config data
static uint32_t new_utc_time_ref = 0;

static ble_uuid_t m_adv_uuids[] =                                   /**< Universally unique service identifiers. */
{
    {BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE},
    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE},
    {BLE_UUID_GAS_SENSOR_SERVICE, BLE_UUID_TYPE_BLE}
};

static TimerHandle_t m_battery_timer;                               /**< Definition of battery timer. */
static TaskHandle_t bme280_measure_task_handle;                     /**< Definition of BME280 measurement task. */
static TaskHandle_t aducm355_measure_task_handle;                   /**< Definition of ADuCM355 measurement task. */
static TaskHandle_t aducm355_command_task_handle;                   /**< Definition of ADuCM355 command handler task. */
xSemaphoreHandle i2c_semaphore = 0;
xSemaphoreHandle use_aducm355_semaphore = 0;

static m24m02_storage_block_t latest_measurement_data = {0};
static bool m_clear_eeprom = false;
static bool m_send_eeprom_data = false;

#if NRF_LOG_ENABLED
static TaskHandle_t m_logger_thread;                                /**< Definition of Logger thread. */
#endif

/**\name Function definitions */
/**@brief Function to start BLE advertising.
 *
 * @details This function is called by FreeRTOS when the Idle Task is initially started.
 *
 * @param[in]   p_erase_bonds   A boolean to delete any prior bonding material.
 */
static void advertising_start(void * p_erase_bonds);


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    bool delete_bonds = false;

    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start(&delete_bonds);
            break;

        default:
            break;
    }
}


/**@brief Function for performing battery measurement and updating the Battery Level characteristic
 *        in Battery Service.
 */
static void battery_level_update(void)
{
    ret_code_t err_code;
    uint8_t  battery_level;

    battery_level = (uint8_t)sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);

    err_code = ble_bas_battery_level_update(&m_bas, battery_level, BLE_CONN_HANDLE_ALL);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
}


/**@brief Function for handling the Battery measurement timer time-out.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] xTimer Handler to the timer that called this function.
 *                   You may get identifier given to the function xTimerCreate using pvTimerGetTimerID.
 */
static void battery_level_meas_timeout_handler(TimerHandle_t xTimer)
{
    UNUSED_PARAMETER(xTimer);
    battery_level_update();
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.
    m_battery_timer = xTimerCreate("BATT",
                                   BATTERY_LEVEL_MEAS_INTERVAL,
                                   pdTRUE,
                                   NULL,
                                   battery_level_meas_timeout_handler);

    /* Error checking */
    if ( (NULL == m_battery_timer) )
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)m_ble_advertising_name,
                                          strlen(m_ble_advertising_name));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    uint8_t max_length = 0;
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        //max_length = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", max_length, max_length);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}


/**@brief Function for initializing the GATT module. */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the data from the Gas Sensor Service.
 *
 * @details This function will process the data received from the Gas Sensor BLE Service
 *
 * @param[in] p_evt       Gas Sensor Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void gas_data_handler(ble_gas_srv_evt_t * p_evt)
{
    ret_code_t err_code;
    
    uint8_t buf[BLE_GAS_NORMAL_DATA_LEN];
    if (p_evt->type == BLE_GAS_SRV_EVT_CONFIG_UPDATED)
    {
        uint32_t err_code;

        NRF_LOG_DEBUG("Received config update from BLE Gas Sensor Service.");

        // working buffer copy, parse
        memcpy(&buf[0], p_evt->params.config_data.config_data_buffer, BLE_GAS_NORMAL_DATA_LEN);
        switch(buf[0])
        {
            case BLE_NAME_CHANGE_CMD:
                // write to flash
                memcpy(&m_ble_advertising_name[0], &buf[1], MAX_BLE_NAME_LENGTH);
                write_flash_ble_advertisement_name(&m_ble_advertising_name[0], MAX_BLE_NAME_LENGTH);
                break;
            case READ_EEPROM_CMD:
            case CLEAR_EEPROM_CMD:
            case WRITE_UTC_TIMESTAMP_CMD:
            case WRITE_GAS_CONFIG_CMD:
            default:
                NRF_LOG_DEBUG("Invalid Config command written, ignoring");
                break;
        }
    }
}


/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Battery and Device Information services.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    ble_bas_init_t     bas_init;
    ble_dis_init_t     dis_init;
    ble_gas_srv_init_t gas_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Gas Sensor Service.
    memset(&gas_init, 0, sizeof(gas_init));

    gas_init.evt_handler = gas_data_handler;

    err_code = ble_gas_srv_init(&m_gas, &gas_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.
    bas_init.bl_rd_sec        = SEC_OPEN;
    bas_init.bl_cccd_wr_sec   = SEC_OPEN;
    bas_init.bl_report_rd_sec = SEC_OPEN;

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);

    dis_init.dis_char_rd_sec = SEC_OPEN;

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the sensor simulators. */
static void sensor_simulator_init(void)
{
    m_battery_sim_cfg.min          = MIN_BATTERY_LEVEL;
    m_battery_sim_cfg.max          = MAX_BATTERY_LEVEL;
    m_battery_sim_cfg.incr         = BATTERY_LEVEL_INCREMENT;
    m_battery_sim_cfg.start_at_max = true;

    sensorsim_init(&m_battery_sim_state, &m_battery_sim_cfg);
}


/**@brief   Function for starting application timers.
 * @details Timers are run after the scheduler has started.
 */
static void application_timers_start(void)
{
    ret_code_t         err_code;

    // Start application timers.
    if (pdPASS != xTimerStart(m_battery_timer, OSTIMER_WAIT_FOR_QUEUE))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }   
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module. */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            m_ble_connected_bool = true;
            NRF_LOG_INFO("Connected");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING_SLOW);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            m_ble_connected_bool = false;
            NRF_LOG_INFO("Disconnected");
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}


/**@brief Function for the Peer Manager initialization. */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Clear bond information from persistent storage. */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality. */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = 2;
    init.advdata.uuids_complete.p_uuids  = &m_adv_uuids[0];

    //init.srdata.uuids_complete.uuid_cnt = 1;
    //init.srdata.uuids_complete.p_uuids  = &m_adv_uuids[2];

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for starting advertising. */
static void advertising_start(void * p_erase_bonds)
{
    bool erase_bonds = *(bool*)p_erase_bonds;

    if (erase_bonds)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);
    }
}


#if NRF_LOG_ENABLED
/**@brief Thread for handling the logger.
 *
 * @details This thread is responsible for processing log entries if logs are deferred.
 *          Thread flushes all log entries and suspends. It is resumed by idle task hook.
 *
 * @param[in]   arg   Pointer used for passing some arbitrary information (context) from the
 *                    osThreadCreate() call to the thread.
 */
static void logger_thread(void * arg)
{
    UNUSED_PARAMETER(arg);

    while (1)
    {
        NRF_LOG_FLUSH();

        vTaskSuspend(NULL); // Suspend myself
    }
}
#endif //NRF_LOG_ENABLED

/**@brief Function for sending Gas Sensor data over BLE
 */
static void gas_characteristic_update(ble_gas_char_update_t char_update, uint8_t * p_data, uint16_t * p_length)
{
    ret_code_t err_code;

    err_code = char_update(&m_gas, p_data, p_length, m_conn_handle); 
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
}


/**@brief A function which is hooked to idle task.
 * @note Idle hook must be enabled in FreeRTOS configuration (configUSE_IDLE_HOOK).
 */
void vApplicationIdleHook( void )
{
#if NRF_LOG_ENABLED
     vTaskResume(m_logger_thread);
#endif
}

/**@brief Function for initializing the clock.
 */
static void clock_init(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL);
}


/**@brief Function for pinging the BME280 for a measurement.
 */
static void bme280_measure_task (void * pvParameter)
{
    static uint8_t p_data[BLE_GAS_NORMAL_DATA_LEN];
    static uint16_t p_data_length;

    UNUSED_PARAMETER(pvParameter);
    while(1)
    {
        if(xSemaphoreTake(i2c_semaphore,BME280_MEASURE_INTERVAL))
        {
            get_sensor_data(BME280, &p_data[0], &p_data_length);
            memcpy(&latest_measurement_data.temperature, &p_data[0], sizeof(latest_measurement_data.temperature));
            memcpy(&latest_measurement_data.humidity, &p_data[4], sizeof(latest_measurement_data.humidity));
            memcpy(&latest_measurement_data.pressure, &p_data[8], sizeof(latest_measurement_data.pressure));
            xSemaphoreGive(i2c_semaphore);
        }
        if(m_ble_connected_bool)
        {
            gas_characteristic_update(ble_gas_srv_temp_humid_pressure_update, &p_data[0], &p_data_length);
        }
        vTaskDelay(BME280_MEASURE_INTERVAL);
    }
}


/**@brief Function for generating simulated ADuCM355 measurement data.
 */
static void generate_simulated_aducm355_data(gas_sensor_results_t *gas_results)
{
    static uint8_t sim_ct = 0;
    static float tmp = 0;

    // generate simulated sensor data
    if (sim_ct == 0)
    {
        gas_results->gas_ppm = gas_results->Z_im = gas_results->Z_re = 0;
    }

    for(uint8_t lcv = 0; lcv <= 16; lcv++)
    {
        gas_results[lcv].gas_sensor = lcv % 4 + 1;
        gas_results[lcv].gas_ppm = 1000 + sim_ct;
        gas_results[lcv].freq  = 10+lcv;
    
        // swap to add some variation
        tmp = gas_results[lcv].Z_im;
        gas_results[lcv].Z_im = gas_results->Z_re;
        gas_results[lcv].Z_re = tmp;

        // add counter var
        gas_results[lcv].Z_im += sim_ct;
        gas_results[lcv].Z_re -= sim_ct;
        sim_ct++;
    }
}

/**@brief Function for pinging the ADuCM355 for a measurement.
 */
static void aducm355_measure_task (void * pvParameter)
{
    ret_code_t err_code;
    static bool measurement_in_progress = false;
    static bool measurement_done = false;
    static gas_sensor_results_t gas_results[16] = {0};
    static gas_sensing_state_t gas_sensing_state;
    static uint32_t ms_count = 0;
    static bool aducm355_no_iface = true;
    static uint32_t num_measurements = 0;
    static uint32_t current_thread_delay = ADUCM355_MEASURE_INTERVAL_DISCONNECTED;

    uint16_t p_data_length = sizeof(gas_sensor_results_t)*16 + 1;
    uint8_t  p_data[sizeof(gas_sensor_results_t)*16 + 1] = {0};
    p_data[0] = 16;
    
    UNUSED_PARAMETER(pvParameter);
    while(1)
    {
        // try to take it immediately
        if(!xSemaphoreTake(use_aducm355_semaphore,1))
        {
            continue;
        }

        ms_count += current_thread_delay;

        // simulate ON
        if (simulated_gas_sensor_data)
        { 
            // simulated data
            generate_simulated_aducm355_data(&gas_results[0]);

            // update
            measurement_done = true;
        }
        else // simulate OFF
        {
            // start new measurement if not already in progress
            if(measurement_in_progress == false)
            {
                err_code = start_aducm355_measurement_seq(ALL);
                APP_ERROR_CHECK(err_code);
                measurement_in_progress = true;

                // check more often once a measurement has started
                current_thread_delay = ADUCM355_MEASURE_CHECK_INTERVAL;
            }
            else // else wait for it to finish / continue on
            {
                err_code = continue_aducm355_measurement_seq(&gas_results[0], &measurement_done);
                APP_ERROR_CHECK(err_code);       
            }
        }

        // if measurement is done, upload the measurement and increase thread interval back up
        if (measurement_done)
        {
            latest_measurement_data.gas_ppm = gas_results[0].gas_ppm;
            latest_measurement_data.gas_sensor_type = gas_results[0].gas_sensor;

            // update characteristic is connected
            if (m_ble_connected_bool)
            {   
                memcpy(&p_data[1], (uint8_t*) &gas_results, sizeof(gas_sensor_results_t)*16);
                //gas_characteristic_update(ble_gas_srv_gas_sensor_update, (uint8_t*) &gas_results, &p_data_length);
                gas_characteristic_update(ble_gas_srv_gas_sensor_update, &p_data[0], &p_data_length);
                current_thread_delay = ADUCM355_MEASURE_INTERVAL_CONNECTED;
            }
            else // store in EEPROM otherwise
            {
                // using i2c, make sure BME280 thread doesn't contest
                if(xSemaphoreTake(i2c_semaphore,ADUCM355_MEASURE_INTERVAL_CONNECTED))
                {
#ifndef BOARD_PCA10056 
                    store_sensor_data_eeprom(&latest_measurement_data, &ms_count);
#endif
                    xSemaphoreGive(i2c_semaphore);
                }
                current_thread_delay = ADUCM355_MEASURE_INTERVAL_DISCONNECTED;
            }

            // clear new measurement
            measurement_in_progress = measurement_done = false;
            num_measurements++;

            // reset counter
            ms_count = 0;
        }
        xSemaphoreGive(use_aducm355_semaphore);
        vTaskDelay(current_thread_delay);
    }
}

/**@brief Function for handling commands from the BLE app.
 */
static void aducm355_command_task (void * pvParameter)
{
    static bool restart = true;
    static bool finished = false;
    static m24m02_storage_block_t record = {0};

    UNUSED_PARAMETER(pvParameter);
    while(1)
    {
        // try to take it immediately
        if(xSemaphoreTake(use_aducm355_semaphore,5))
        {
            if(read_eeprom_cmd)
            {   
                get_sensor_data_eeprom(&restart, &finished, &record);
                restart = false;
         
                // clear command
                if (finished)
                {
                    restart = true;
                    read_eeprom_cmd = false;
                }
            }
            if(clear_eeprom_cmd)
            {
                // clear command
                clear_eeprom_cmd = false;
            }
            if(update_utc_time)
            {
                // clear command
                update_utc_time = false;
            }
            if(update_aducm355_config)
            {
            }
            xSemaphoreGive(use_aducm355_semaphore);
        }
        vTaskDelay(ADUCM355_MEASURE_CHECK_INTERVAL);
    }
}

/**@brief Function for initializeing external sensors.
 */
static void external_sensor_init(void)
{
    BaseType_t xReturned;

    // initialize i2c + spi sensors
#ifndef BOARD_PCA10056
    ext_device_init(M24M02);
#endif
    ext_device_init(BME280 | SIMULATE);

  /* //test reading stored measurements
    bool restart = true;
    bool finished = false;
    m24m02_storage_block_t record = {0};
    get_sensor_data_eeprom(&restart, &finished, &record);
    restart = false;
    while(!finished)
    {
        get_sensor_data_eeprom(&restart, &finished, &record);
    }
*/

    xReturned = xTaskCreate(bme280_measure_task, "BME280", configMINIMAL_STACK_SIZE + 200, NULL, 1, &bme280_measure_task_handle);
    if (xReturned != pdPASS)
    {
        NRF_LOG_ERROR("BME280 task not created.");
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
}


/**@brief Function for initializeing external sensors.
 */
static void gas_sensor_init(void)
{
    BaseType_t xReturned;

    ret_code_t ret_code;

    // LED2 on = error
    if(!nrf_drv_gpiote_is_init())
    {
        APP_ERROR_CHECK(nrf_drv_gpiote_init());
    }
#ifdef BOARD_MATCHBOX_V1
    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(true); // false = init low
#else
    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false); // false = init low
#endif
    APP_ERROR_CHECK(nrf_drv_gpiote_out_init(LED_2, &out_config));

    if (!simulated_gas_sensor_data)
    {
        while(init_aducm355_iface())
        {
            nrf_delay_ms(50);
        }
        //ret_code = init_aducm355_iface();
        //APP_ERROR_CHECK(ret_code);
    }
    nrf_drv_gpiote_out_toggle(LED_2);

    xReturned = xTaskCreate(aducm355_measure_task, "ADUCM355_Meas", configMINIMAL_STACK_SIZE + 200, NULL, 1, &aducm355_measure_task_handle);
    if (xReturned != pdPASS)
    {
        NRF_LOG_ERROR("ADUCM355 measure task not created.");
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }

    xReturned = xTaskCreate(aducm355_command_task, "ADUCM355_Command", configMINIMAL_STACK_SIZE + 200, NULL, 1, &aducm355_command_task_handle);
    if (xReturned != pdPASS)
    {
        NRF_LOG_ERROR("ADUCM355 command task not created.");
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
}

/**@brief Function for application main entry.
 */
int main(void)
{
    bool erase_bonds = true;

    // Initialize semaphores
    i2c_semaphore = xSemaphoreCreateMutex();
    use_aducm355_semaphore = xSemaphoreCreateMutex();

    // Initialize modules.
    log_init();
    clock_init();

    // enable DCDCs to reduce power consumption
    NRF_POWER->DCDCEN = 1;
#ifdef NRF52840_XXAA
    NRF_POWER->DCDCEN0 = 1;
#endif

    // Do not start any interrupt that uses system functions before system initialisation.
    // The best solution is to start the OS before any other initalisation.

#if NRF_LOG_ENABLED
    // Start execution.
    if (pdPASS != xTaskCreate(logger_thread, "LOGGER", 256, NULL, 1, &m_logger_thread))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
#endif

    // Activate deep sleep mode.
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    NRF_LOG_INFO("Reboot.");

     // Initialize modules.
    initialize_flash();

    // Set advertising name from flash
    if (!read_flash_ble_advertisement_name(&m_ble_advertising_name[0]))
    {
        memcpy(&m_ble_advertising_name[0], (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    }

    // Configure and initialize the BLE stack.
    ble_stack_init();

    // Initialize modules.
    timers_init();
    buttons_leds_init(&erase_bonds);
    gap_params_init();
    gatt_init();
    services_init();
    advertising_init();
    sensor_simulator_init();
    conn_params_init();
    //peer_manager_init();
    application_timers_start(); // this includes the timer for the SAADC sampling, should only do this while connected (power saving)
    
    // added for gas sensor functions
    external_sensor_init();

    // iface to adcucm355
    gas_sensor_init();

    // Create a FreeRTOS task for the BLE stack.
    // The task will run advertising_start() before entering its loop.
    //pm_peers_delete();
    nrf_sdh_freertos_init(advertising_start, &erase_bonds);

    NRF_LOG_INFO("Voltage Band FreeRTOS Scheduler started.");
    // Start FreeRTOS scheduler.
    vTaskStartScheduler();

    for (;;)
    {
        APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
    }
}


