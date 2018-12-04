/** @file
 * @defgroup pwm_example_main main.c
 * @{
 * @ingroup pwm_example
 *
 * @brief PWM Example Application main file.
 *
 * This file contains the source code for a sample application using PWM.
 */

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_rtc.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_serial.h"
#include "aducm355_controller.h"
#include "aducm355_cmd_protocol.h"

// logging
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// In case not defined in board file
#ifndef ADUCM355_RX_PIN
#define ADUCM355_RX_PIN            24
#define ADUCM355_TX_PIN            25
#define ADUCM355_RTS_PIN            2
#define ADUCM355_CTS_PIN           26
#endif

#define SERIAL_FIFO_TX_SIZE 32
#define SERIAL_FIFO_RX_SIZE 32
#define SERIAL_BUFF_TX_SIZE 1
#define SERIAL_BUFF_RX_SIZE 1

#define MAX_UINT32          4294967295

#define UART_TIMEOUT_US     5000

#define RTC_FREQUENCY       32 // must be a factor of 32768

#define PRIMETIME_SEC       5

#define TIMEOUT_SEC         10

const  nrf_drv_rtc_t           rtc = NRF_DRV_RTC_INSTANCE(2); // Declaring an instance of nrf_drv_rtc for RTC2. */

static void serial_rx_cb(struct nrf_serial_s const *p_serial, nrf_serial_event_t event);

// Uart library defines
NRF_SERIAL_DRV_UART_CONFIG_DEF(m_uart0_drv_config,
                      ADUCM355_RX_PIN, ADUCM355_TX_PIN,
                      ADUCM355_RTS_PIN, ADUCM355_CTS_PIN,
                      UART_CONFIG_HWFC_Disabled, NRF_UART_PARITY_EXCLUDED,
                      NRF_UART_BAUDRATE_115200,
                      UART_DEFAULT_CONFIG_IRQ_PRIORITY);
NRF_SERIAL_QUEUES_DEF(serial_queues, SERIAL_FIFO_TX_SIZE, SERIAL_FIFO_RX_SIZE);
NRF_SERIAL_BUFFERS_DEF(serial_buffs, SERIAL_BUFF_TX_SIZE, SERIAL_BUFF_RX_SIZE);
NRF_SERIAL_CONFIG_DEF(serial_config, NRF_SERIAL_MODE_IRQ,
                      &serial_queues, &serial_buffs, serial_rx_cb, NULL);                    
NRF_SERIAL_UART_DEF(serial_uart, 0);

static uint32_t priming_counter_ticks = 0;
static uint32_t measure_counter_ticks = 0;

// Uart buffers
static uint8_t uart_tx_buffer[PKT_LENGTH]   = {0};
static uint8_t uart_rx_buffer[PKT_LENGTH*3] = {0}; // 2x margin
static uint8_t uart_rx_fill_pos = 0;
static uint8_t uart_rx_parse_pos = 0;
static bool buf_rdy = false;
static bool ack_rcvd = false;

// State init
aducm355_state_t aducm355_state = ADUCM355_SLEEP;
gas_sensing_state_t sensing_state = IDLE;

// To store current measurement
MeasResult_t aducm355_result;

// to track which sensors are enabled / connected
static uint16_t use_sensor = 0;

// Protect UART buffer flag
bool uart_buffer_mutex_lock = false;

static void get_sensor_specific_constants(uint16_t * num_avg, float * freq)
{
    switch(use_sensor)
    {
        case CO2:
          *num_avg = 10;
          *freq    = 15000.0f;
        case CO:
          *num_avg = 15;
          *freq    = 25000.0f;
        default:
          break;
    }
}

static void lock_mutex(bool *mutex)
{
    while(*mutex) nrf_delay_us(1);
    *mutex = true;
}

static void unlock_mutex(bool *mutex)
{
    *mutex = false;
}

static void serial_rx_cb(struct nrf_serial_s const *p_serial, nrf_serial_event_t event) 
{
    if (event == NRF_SERIAL_EVENT_RX_DATA) 
    {
        lock_mutex(&uart_buffer_mutex_lock);

        // copy received byte to buffer, set buf_rdy flag
        APP_ERROR_CHECK(nrf_serial_read(&serial_uart, &uart_rx_buffer[uart_rx_fill_pos++], 1, NULL, 0));
        if (uart_rx_fill_pos == sizeof(uart_rx_buffer)) // overflow, should not happen as long as packets are parsed quickly enough
        {
            uart_rx_fill_pos = 0;
            buf_rdy = false;
        }
        if (uart_rx_fill_pos >= PKT_LENGTH) 
        {
            buf_rdy = true;
            check_aducm355_rx_buffer();
        }

        unlock_mutex(&uart_buffer_mutex_lock);
    }
}

static void serial_tx(uint8_t *buf, uint8_t *len)
{
    for (uint8_t i = 0; i < *len; i++)
    {
        (void)nrf_serial_write(&serial_uart, buf+i, 1, NULL, 0);
    }
    (void)nrf_serial_flush(&serial_uart, 0);
}

static void wait_for_ack(void)
{
    uint32_t timeout_ct = 0;
    do
    {
        nrf_delay_us(100);
        timeout_ct += 100;
    } while (ack_rcvd == false && timeout_ct < UART_TIMEOUT_US);
}

static uint32_t tx_and_wait_for_ack(uint8_t *buf, uint8_t *len)
{
    // ack init
    ack_rcvd = false;
    serial_tx(buf, len);
    wait_for_ack(); // delay while we wait for ACK, 5ms timeout
    
    // return 0 for success, 1 for fail
    return (ack_rcvd == true) ? 0 : 1;
}

static void send_ack_aducm355(uart_packet *pkt_to_ack)
{
    uint8_t pkt_length;
    build_ack_packet(&uart_tx_buffer[0], &pkt_length, pkt_to_ack, DNC);
    serial_tx(&uart_tx_buffer[0], &pkt_length); // do not wait for ACK
}

static uint32_t send_ping_aducm355(void)
{
    uint8_t pkt_length;
    build_ping_packet(&uart_tx_buffer[0], &pkt_length);
    return tx_and_wait_for_ack(&uart_tx_buffer[0], &pkt_length);
}

static void send_measurement_aducm355(void)
{
    uint32_t ret;
    uint8_t pkt_length;

    uint16_t num_avg;
    float freq;
    get_sensor_specific_constants(&num_avg, &freq);
    build_start_measure_packet(&uart_tx_buffer[0], &pkt_length, (uint8_t) use_sensor, num_avg, freq);
    ret = tx_and_wait_for_ack(&uart_tx_buffer[0], &pkt_length);
    
    // ping the ADuCM355 to get current state
    ret = send_ping_aducm355();
    APP_ERROR_CHECK(ret);

    // ensure the ADI chip has transitioned to measure state
    if (aducm355_state == ADUCM355_MEASURE)
    {
        sensing_state = MEASURING; // transition state
        measure_counter_ticks = 0;
    }
}

static uint32_t send_sleep_aducm355(void)
{
    uint8_t pkt_length;
    build_sleep_packet(&uart_tx_buffer[0], &pkt_length);
    return tx_and_wait_for_ack(&uart_tx_buffer[0], &pkt_length);
}

static void parse_aducm355_ack_payload(ack_payload *payload)
{
    aducm355_state = payload->state;
}

static void parse_aducm355_measure_data(data_payload *payload)
{
    memcpy(&aducm355_result, &payload->ImpResult, sizeof(MeasResult_t));
    sensing_state = MEASUREMENT_DONE;
}

static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{
    if (int_type == NRF_DRV_RTC_INT_TICK)
    {
        // make sure we don't overflow
        priming_counter_ticks += (priming_counter_ticks == MAX_UINT32) ? 0 : 1; 
        measure_counter_ticks += 1;
    }
}

static void start_priming_timer(void)
{
    uint32_t err_code;

    //Initialize RTC instance
    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
    config.prescaler = 32768/RTC_FREQUENCY; // 32768 / X = Tick Frequency
    err_code = nrf_drv_rtc_init(&rtc, &config, rtc_handler);
    APP_ERROR_CHECK(err_code);

    //Enable tick event & interrupt
    nrf_drv_rtc_tick_enable(&rtc,true);
    APP_ERROR_CHECK(err_code);

    //Restart the counter
    priming_counter_ticks = 0;

    //Power on RTC instance
    nrf_drv_rtc_enable(&rtc);                                         
}

static void check_priming_state(void)
{
    // check to see if we've been primed the minimum amount of time
    uint32_t time_primed_sec = priming_counter_ticks / RTC_FREQUENCY;
    if (time_primed_sec >= PRIMETIME_SEC)
    {
        sensing_state = PRIMED; // transition state
    }
}

static void check_measuring_state(void)
{
    // check to see if we've exceeded the maximum amount of time for a measurement
    uint32_t time_measurement_sec = measure_counter_ticks / RTC_FREQUENCY;
    if (time_measurement_sec >= TIMEOUT_SEC)
    {
        sensing_state = PRIMED; // transition state back to primed, retry the measurement
    }
}

uint32_t init_aducm355_iface(void)
{
    ret_code_t ret;
    ret = nrf_serial_init(&serial_uart, &m_uart0_drv_config, &serial_config);
    APP_ERROR_CHECK(ret);

    // ping the ADuCM355, it should be asleep
    return send_ping_aducm355();
}

uint32_t start_aducm355_measurement_seq(gas_sensor_t gas_sensor)
{
    ret_code_t ret;

    // ping the ADuCM355, it should be asleep, this will wake it up
    ret = send_ping_aducm355();
    APP_ERROR_CHECK(ret);

    // return error if it didn't transition to sleep
    if (aducm355_state != ADUCM355_IDLE) return 1;

    // start heaters and configure load capacitor (PLACEHOLDER)
    use_sensor = gas_sensor;
    //start_heaters(use_sensor);
    //config_load_capacitor(use_sensor);

    if (sensing_state == IDLE)
    {
        // start timer to keep track of how long we've been priming
        start_priming_timer();

        // transition state
        sensing_state = PRIMING;
    }
     
    // success
    return 0;
}

// simple state machine but transitions are sequentially made
// this allows us to go to sleep between task handling
uint32_t continue_aducm355_measurement_seq(gas_sensor_results_t *gas_results, bool *measurement_done)
{
    // default
    *measurement_done = false;
    switch (sensing_state)
    {
        case IDLE:
            // do nothing
            break;
        case PRIMING:
            check_priming_state();
            break;
        case PRIMED:
            send_measurement_aducm355();
            break;
        case MEASURING:
            check_measuring_state();
            break;
        case MEASUREMENT_DONE:
            //run_gas_sensing_algorithim(&aducm355_result, gas_results);
            sensing_state = PRIMED; // we are still in the primed state
            gas_results->gas_sensor = aducm355_result.sensor;
            gas_results->gas_ppm = 0; // placeholder
            gas_results->freq = aducm355_result.freq;
            gas_results->Z_re = aducm355_result.Z_re;
            gas_results->Z_im = aducm355_result.Z_im;
            *measurement_done = true;
            break;
        default:
            return 1; // this should never happen
    }

    // success
    return 0;
}

uint32_t stop_aducm355_measurement_seq(void)
{
    ret_code_t ret;

    // toggle heaters to OFF and reset load cap (Placeholder)
    //reset_heaters(use_sensor);
    //reset_load_capacitor(use_sensor);

    // sleep cmd
    ret = send_sleep_aducm355();
    APP_ERROR_CHECK(ret);

    // return error if it didn't transition to sleep
    if (aducm355_state != ADUCM355_SLEEP) return 1;                    

    // Power off RTC instance
    nrf_drv_rtc_uninit(&rtc); 

    // to IDLE mode
    sensing_state = IDLE;

    // success
    return 0;
}

void check_gas_sensing_state(gas_sensing_state_t *buf)
{
    *buf = sensing_state;
}

void check_aducm355_rx_buffer(void)
{
    static uint8_t mid_ind = sizeof(uart_rx_buffer)/2;

    if (!buf_rdy) 
    {
        return;
    }

    // Pkt buffer, pkts don't come fast enough to justify a ring buffer
    uart_packet rx_packet = {0};
    uint8_t buffer_fill = uart_rx_fill_pos;
    while (look_for_packet(&uart_rx_buffer[uart_rx_parse_pos], buffer_fill, &rx_packet))
    {
       // move rx buffer pointer back to front
       uart_rx_parse_pos += PKT_LENGTH;
       buffer_fill =  uart_rx_parse_pos - uart_rx_fill_pos;

       // reset flag
       buf_rdy = false;

       // do different actions depending on packet type
       switch (rx_packet.cmd)
       {
          case CMD_ACK:
             ack_rcvd = true;
             parse_aducm355_ack_payload((ack_payload *)&rx_packet.payload);
             break;
          case CMD_SEND_DATA:
             send_ack_aducm355(&rx_packet);
             parse_aducm355_measure_data((data_payload *)&rx_packet.payload);
          default: // should never receive any other pkt cmd types
             break;
       }
    }

    // shift rx buffer over
    uint8_t bytes_to_parse = uart_rx_fill_pos - uart_rx_parse_pos;
    if (uart_rx_parse_pos != 0 && bytes_to_parse != 0)
    {
        lock_mutex(&uart_buffer_mutex_lock);

        // shift buffer data to beginning, reset pos indexes
        memcpy(&uart_rx_buffer[0], &uart_rx_buffer[uart_rx_parse_pos], bytes_to_parse); 
        uart_rx_fill_pos = bytes_to_parse;
        uart_rx_parse_pos = 0;

        unlock_mutex(&uart_buffer_mutex_lock);
    }
    else if (bytes_to_parse == 0) 
    {
        uart_rx_fill_pos = uart_rx_parse_pos = 0;
    }
}

