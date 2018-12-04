#ifndef BLE_GAS_SRV_H__
#define BLE_GAS_SRV_H__

#include <stdint.h>
#include <stdbool.h>
#include "sdk_config.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"
#include "ble_link_ctx_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

#define OPCODE_LENGTH 1                                                             
#define HANDLE_LENGTH 2                                                         

/**@brief Macro for defining a ble_bas instance.
 *
 * @param   _name  Name of the instance.
 * @hideinitializer
 */
#define BLE_GAS_SRV_DEF(_name)                          \
    static ble_gas_srv_t _name;                         \
    NRF_SDH_BLE_OBSERVER(_name ## _obs,                   \
                         BLE_GAS_SRV_BLE_OBSERVER_PRIO, \
                         ble_gas_srv_on_ble_evt,        \
                         &_name)

// 58daad0b-87f2-464b-8e75-9e6dcfa567c1 from https://www.uuidgenerator.net/version4
#define GAS_SERVICE_UUID_BASE          {0xc1, 0x67, 0xa5, 0xcf, 0x6d, 0x9e, 0x75, 0x8e, \
                                        0x46, 0x4b, 0x87, 0xf2, 0x0b, 0xad, 0xda, 0x58}

#define BLE_UUID_GAS_SENSOR_SERVICE                  0x0001
#define GAS_SENSOR_DATA_CHAR_UUID                    0x0002
#define TEMP_HUMID_PRESSURE_DATA_CHAR_UUID           0x0003
#define CONFIG_DATA_WRITE_CHAR_UUID                  0x0004
#define CONFIG_DATA_GAS1_CHAR_UUID                   0x0005
#define CONFIG_DATA_GAS2_CHAR_UUID                   0x0006
#define CONFIG_DATA_GAS3_CHAR_UUID                   0x0007
#define CONFIG_DATA_GAS4_CHAR_UUID                   0x0008

#define BLE_GAS_NORMAL_DATA_LEN                      20

/**@brief Gas Sensor event type. */
typedef enum
{
    BLE_GAS_SRV_MODE_NORMAL = 1,
    BLE_GAS_SRV_MODE_ENGINEERING = 2
} ble_gas_srv_config_mode_t;

/**@brief Gas Sensor event type. */
typedef enum
{
    BLE_GAS_SRV_EVT_CONFIG_UPDATED
} ble_gas_srv_evt_type_t;

/**@brief Gas Sensor configuration data. */
typedef struct
{
    uint8_t config_data_buffer[BLE_GAS_NORMAL_DATA_LEN];
} ble_gas_srv_config_t;

/**@brief Gas Sensor Service client context structure.
 *
 * @details This structure contains state context related to hosts.
 */
typedef struct
{
    bool is_notification_enabled; /**< Variable to indicate if the peer has enabled notification of the characteristic.*/
} ble_gas_srv_client_context_t;

// Forward declaration of the ble_gas_srv_t type.
typedef struct ble_gas_srv_s ble_gas_srv_t;

/**@brief Gas Sensor event. */
typedef struct
{
    ble_gas_srv_evt_type_t          type;        /**< Type of event. */
    ble_gas_srv_t                 * p_gas_srv; /**< A pointer to the instance. */
    uint16_t                        conn_handle; /**< Connection handle. */
    ble_gas_srv_client_context_t  * p_link_ctx;  /**< A pointer to the link context. */
    union
    {
        ble_gas_srv_config_t config_data;        /**< @ref BLE_gas_SRV_EVT_CONFIG_UPDATED event data. */
    } params;
} ble_gas_srv_evt_t;

/**@brief Gas Sensor Service event handler type. */
typedef void (* ble_gas_srv_handler_t) (ble_gas_srv_evt_t * p_evt);

/**@brief Gas Sensor Service update characteristic type. */
typedef uint32_t (* ble_gas_char_update_t) (ble_gas_srv_t * p_gas_srv, uint8_t * p_data, uint16_t * p_length, uint16_t conn_handle);

/**@brief Gas Sensor Service init structure. This contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_gas_srv_handler_t  evt_handler;                    /**< Event handler to be called for handling events in the Battery Service. */
} ble_gas_srv_init_t;

/**@brief Gas Sensor Service structure. This contains various status information for the service. */
struct ble_gas_srv_s
{
    ble_gas_srv_handler_t           evt_handler;                        /**< Event handler to be called for handling events in the Gas Sensor Service. */
    uint16_t                        service_handle;                     /**< Handle of Gas Sensor Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t        gas_sensor_handles;                 /**< Handles related to the Gas Sensor Gas Sensor Data characteristic (as provided by the SoftDevice). */
    ble_gatts_char_handles_t        temp_humid_pressure_handles;        /**< Handles related to the Gas Sensor Temp/Humid/Pressure characteristic (as provided by the SoftDevice). */
    ble_gatts_char_handles_t        config_write_handles;               /**< Handles related to the Gas Sensor Configuration characteristic (as provided by the SoftDevice). */
    ble_gatts_char_handles_t        config_gas1_handles;                /**< Handles related to the Gas Sensor 1 Configuration characteristic (as provided by the SoftDevice). */
    ble_gatts_char_handles_t        config_gas2_handles;                /**< Handles related to the Gas Sensor 2 Configuration characteristic (as provided by the SoftDevice). */
    ble_gatts_char_handles_t        config_gas3_handles;                /**< Handles related to the Gas Sensor 3 Configuration characteristic (as provided by the SoftDevice). */
    ble_gatts_char_handles_t        config_gas4_handles;                /**< Handles related to the Gas Sensor 4 Configuration characteristic (as provided by the SoftDevice). */
    blcm_link_ctx_storage_t * const p_link_ctx_storage;                 /**< Pointer to link context storage with handles of all current connections and its context. */
    bool                            is_notification_supported_gas_d;    /**< TRUE if notification of Gas Sensor Data is supported. */
    bool                            is_notification_supported_thp_d;    /**< TRUE if notification of Temp/Humid/Pressure Data is supported. */
    bool                            is_notification_supported_config_w; /**< TRUE if notification of Config State is supported. */
    bool                            is_notification_supported_config_1; /**< TRUE if notification of Config State Gas 1 is supported. */
    bool                            is_notification_supported_config_2; /**< TRUE if notification of Config State Gas 2 is supported. */
    bool                            is_notification_supported_config_3; /**< TRUE if notification of Config State Gas 3 is supported. */
    bool                            is_notification_supported_config_4; /**< TRUE if notification of Config State Gas 4 is supported. */
    uint8_t                         uuid_type;                     /**< UUID type for Gas Sensor Service Base UUID. */
};

/**@brief   Function for initializing the Gas Sensor Service.
 *
 * @param[out] p_gas_srv      Gas Sensor Service structure. This structure must be supplied
 *                              by the application. It is initialized by this function and will
 *                              later be used to identify this particular service instance.
 * @param[in] p_gas_srv_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was successfully initialized. Otherwise, an error code is returned.
 * @retval NRF_ERROR_NULL If either of the pointers p_gas_srv or p_gas_srv_init is NULL.
 */
uint32_t ble_gas_srv_init(ble_gas_srv_t * p_gas_srv, ble_gas_srv_init_t const * p_gas_srv_init);


/**@brief   Function for handling the Gas Sensor Service's BLE events.
 *
 * @details The Gas Sensor Service expects the application to call this function each time an
 * event is received from the SoftDevice. This function processes the event if it
 * is relevant and calls the Gas Sensor Service event handler of the
 * application if necessary.
 *
 * @param[in] p_ble_evt     Event received from the SoftDevice.
 * @param[in] p_context     Gas Sensor Service structure.
 */
void ble_gas_srv_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief   Function for updating the gas sensor characteristic.
 *
 * @details This function sends the gas sensor data as characteristic notification to the
 *          peer.
 *
 * @param[in]     p_gas_srv       Pointer to the Gas Sensor Service structure.
 * @param[in]     p_data            Pointer to data to be sent.
 * @param[in,out] p_length          Pointer Length of the string. Amount of sent bytes.
 * @param[in]     conn_handle       Connection Handle of the destination client.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_gas_srv_gas_sensor_update(ble_gas_srv_t * p_gas_srv,
                                         uint8_t     * p_data,
                                         uint16_t    * p_length,
                                         uint16_t      conn_handle);


/**@brief   Function for updating the temp/humid/pressure characteristic.
 *
 * @details This function sends the temp/humid/pressure data as characteristic notification to the
 *          peer.
 *
 * @param[in]     p_gas_srv       Pointer to the Gas Sensor Service structure.
 * @param[in]     p_data            Pointer to data to be sent.
 * @param[in,out] p_length          Pointer Length of the string. Amount of sent bytes.
 * @param[in]     conn_handle       Connection Handle of the destination client.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_gas_srv_temp_humid_pressure_update(ble_gas_srv_t * p_gas_srv,
                                                  uint8_t     * p_data,
                                                  uint16_t    * p_length,
                                                  uint16_t      conn_handle);


/**@brief   Function for updating the Gas Sensor 1 Config characteristic.
 *
 * @details This function sends the Gas Sensor 1 Config as characteristic notification to the
 *          peer.
 *
 * @param[in]     p_gas_srv       Pointer to the Gas Sensor Service structure.
 * @param[in]     p_data            Pointer to data to be sent.
 * @param[in,out] p_length          Pointer Length of the string. Amount of sent bytes.
 * @param[in]     conn_handle       Connection Handle of the destination client.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_gas_srv_gas1_config_update(ble_gas_srv_t * p_gas_srv,
                                          uint8_t     * p_data,
                                          uint16_t    * p_length,
                                          uint16_t      conn_handle);


/**@brief   Function for updating the Gas Sensor 2 Config characteristic.
 *
 * @details This function sends the Gas Sensor 2 Config as characteristic notification to the
 *          peer.
 *
 * @param[in]     p_gas_srv       Pointer to the Gas Sensor Service structure.
 * @param[in]     p_data            Pointer to data to be sent.
 * @param[in,out] p_length          Pointer Length of the string. Amount of sent bytes.
 * @param[in]     conn_handle       Connection Handle of the destination client.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_gas_srv_gas2_config_update(ble_gas_srv_t * p_gas_srv,
                                          uint8_t     * p_data,
                                          uint16_t    * p_length,
                                          uint16_t      conn_handle);


/**@brief   Function for updating the Gas Sensor 3 Config characteristic.
 *
 * @details This function sends the Gas Sensor 3 Config as characteristic notification to the
 *          peer.
 *
 * @param[in]     p_gas_srv       Pointer to the Gas Sensor Service structure.
 * @param[in]     p_data            Pointer to data to be sent.
 * @param[in,out] p_length          Pointer Length of the string. Amount of sent bytes.
 * @param[in]     conn_handle       Connection Handle of the destination client.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_gas_srv_gas3_config_update(ble_gas_srv_t * p_gas_srv,
                                          uint8_t     * p_data,
                                          uint16_t    * p_length,
                                          uint16_t      conn_handle);


/**@brief   Function for updating the Gas Sensor 4 Config characteristic.
 *
 * @details This function sends the Gas Sensor 4 Config as characteristic notification to the
 *          peer.
 *
 * @param[in]     p_gas_srv       Pointer to the Gas Sensor Service structure.
 * @param[in]     p_data            Pointer to data to be sent.
 * @param[in,out] p_length          Pointer Length of the string. Amount of sent bytes.
 * @param[in]     conn_handle       Connection Handle of the destination client.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_gas_srv_gas4_config_update(ble_gas_srv_t * p_gas_srv,
                                          uint8_t     * p_data,
                                          uint16_t    * p_length,
                                          uint16_t      conn_handle);


/**@brief   Function for updating the config characteristic.
 *
 * @details This function sends the config as characteristic notification to the
 *          peer.
 *
 * @param[in]     p_gas_srv       Pointer to the Gas Sensor Service structure.
 * @param[in]     p_data            Pointer to data to be sent.
 * @param[in,out] p_length          Pointer Length of the string. Amount of sent bytes.
 * @param[in]     conn_handle       Connection Handle of the destination client.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_gas_srv_config_update(ble_gas_srv_t * p_gas_srv,
                                     uint8_t     * p_data,
                                     uint16_t    * p_length,
                                     uint16_t      conn_handle);


#ifdef __cplusplus
}
#endif

#endif /* BLE_GAS_SRV_H__*/