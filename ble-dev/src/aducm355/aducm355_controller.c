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
#include <math.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_rtc.h"
#include "nrf_drv_gpiote.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_serial.h"
#include "aducm355_gas_constants.h"
#include "aducm355_controller.h"
#include "aducm355_cmd_protocol.h"

// free rtos
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

// logging
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// external devices (eeprom and temp humidity sensor)
#include "../i2c_device_controller.h"
extern m24m02_storage_block_t latest_measurement_data;

#include "../../../common/Packet.h"

#define MS_TO_TICKS(xMS) pdMS_TO_TICKS(xMS)
#define US_TO_TICKS(xUS) (((xUS) * configTICK_RATE_HZ) / 1000000)
#define TICKS_TO_US(xTicks) (((xTicks) * 1000000) / configTICK_RATE_HZ)
#define TICKS_TO_MS(xTicks) (((xTicks) * 1000) / configTICK_RATE_HZ)

// In case not defined in board file
#ifndef ADUCM355_RX_PIN
#define ADUCM355_RX_PIN            24
#define ADUCM355_TX_PIN            25
#define ADUCM355_RTS_PIN            2
#define ADUCM355_CTS_PIN           26
#endif

//#define SERIAL_FIFO_TX_SIZE 32
#define SERIAL_FIFO_TX_SIZE 64
#define SERIAL_FIFO_RX_SIZE 32
#define SERIAL_BUFF_TX_SIZE 1
#define SERIAL_BUFF_RX_SIZE 1
#define MAX_UINT32          4294967295
#define UART_TIMEOUT_US     20000
#define RTC_FREQUENCY       32 // must be a factor of 32768
//#define TIMEOUT_SEC         10
#define TIMEOUT_SEC         3

const  nrf_drv_rtc_t           rtc = NRF_DRV_RTC_INSTANCE(2); // Declaring an instance of nrf_drv_rtc for RTC2. */

static void serial_rx_cb(struct nrf_serial_s const *p_serial, nrf_serial_event_t event);
void new_serial_rx_cb(struct nrf_serial_s const *p_serial, nrf_serial_event_t event);

// Uart library defines
NRF_SERIAL_DRV_UART_CONFIG_DEF(m_uart0_drv_config,
                      ADUCM355_RX_PIN, ADUCM355_TX_PIN,
                      NULL, NULL, //ADUCM355_RTS_PIN, ADUCM355_CTS_PIN,
                      UART_CONFIG_HWFC_Disabled, NRF_UART_PARITY_EXCLUDED,
                      NRF_UART_BAUDRATE_115200,
                      UART_DEFAULT_CONFIG_IRQ_PRIORITY);
NRF_SERIAL_QUEUES_DEF(serial_queues, SERIAL_FIFO_TX_SIZE, SERIAL_FIFO_RX_SIZE);
NRF_SERIAL_BUFFERS_DEF(serial_buffs, SERIAL_BUFF_TX_SIZE, SERIAL_BUFF_RX_SIZE);
NRF_SERIAL_CONFIG_DEF(serial_config, NRF_SERIAL_MODE_IRQ,
                      &serial_queues, &serial_buffs, new_serial_rx_cb, NULL);                    
NRF_SERIAL_UART_DEF(serial_uart, 0);

static uint32_t priming_counter_ticks = 0;
static uint32_t measure_counter_ticks = 0;

#define TX_PIN 17
#define RX_PIN 19

// Uart buffers
static uint8_t uart_tx_buffer[PKT_LENGTH]   = {0};
static uint8_t uart_rx_buffer[PKT_LENGTH*3] = {0}; // 2x margin
static uint8_t uart_rx_fill_pos = 0;
static uint8_t uart_rx_parse_pos = 0;
static bool buf_rdy = false;
static bool ack_rcvd = false;

static uint8_t measurement_index         = 0;
static uint8_t measurement_num           = 0;
static uint8_t measurement_setting_index = 0;

// State init
aducm355_state_t aducm355_state = ADUCM355_SLEEP;
gas_sensing_state_t sensing_state = IDLE;

// To store current measurement
MeasResult_t aducm355_result;

// Protect UART buffer flag
bool uart_buffer_mutex_lock = false;

static void clear_FPU_interrupts(void)
{
    __set_FPSCR(__get_FPSCR() & ~(0x0000009F)); 
    (void) __get_FPSCR();
    NVIC_ClearPendingIRQ(FPU_IRQn);
}

static void get_sensor_specific_measure_params(gas_sensor_t * current_sensor, uint16_t * num_avg, float * freq, bool * mux_en, bool * mux_s0, bool * mux_s1)
{
    *current_sensor = MeasureSettings[measurement_setting_index].sensor_num;
    *num_avg        = MeasureSettings[measurement_setting_index].num_avg;
    *freq           = MeasureSettings[measurement_setting_index].freq;
    *mux_en         = MeasureSettings[measurement_setting_index].mux_enable;
    *mux_s0         = MeasureSettings[measurement_setting_index].mux_s0;
    *mux_s1         = MeasureSettings[measurement_setting_index].mux_s1;
}

static void get_sensor_specific_calibration_params(float * a0, float * a1, float * a2, float * a3, float * a4)
{
    *a0 = MeasureSettings[measurement_setting_index].a0;
    *a1 = MeasureSettings[measurement_setting_index].a1;
    *a2 = MeasureSettings[measurement_setting_index].a2;
    *a3 = MeasureSettings[measurement_setting_index].a3;
    *a4 = MeasureSettings[measurement_setting_index].a4;
}

static void control_heaters(bool enable, gas_sensor_t gas_sensors)
{

#ifndef BOARD_MATCHBOX_V1
    return;
#else
    if (!enable)
    {
        nrf_drv_gpiote_out_clear(ADUCM355_HEATER1);
        nrf_drv_gpiote_out_clear(ADUCM355_HEATER2);
        nrf_drv_gpiote_out_clear(ADUCM355_HEATER3);
        nrf_drv_gpiote_out_clear(ADUCM355_HEATER4);
    }
    else
    {
        if (gas_sensors & GAS1)
        {
            nrf_drv_gpiote_out_set(ADUCM355_HEATER1);
        }
        if (gas_sensors & GAS2)
        {
            nrf_drv_gpiote_out_set(ADUCM355_HEATER2);
        }
        if (gas_sensors & GAS3)
        {
            nrf_drv_gpiote_out_set(ADUCM355_HEATER3);
        }
        if (gas_sensors & GAS4)
        {
            nrf_drv_gpiote_out_set(ADUCM355_HEATER4);
        }
    }
#endif
}

static void config_load_capacitor(bool enable, bool s0, bool s1)
{
#ifndef BOARD_MATCHBOX_V1
    return;
#else
    if (!enable)
    {
        nrf_drv_gpiote_out_set(ADUCM355_CAP_MUX_NE); // disable = HIGH
        return; //don't care about other settings
    }
    else
    {
        nrf_drv_gpiote_out_clear(ADUCM355_CAP_MUX_NE); // enable = LOW
    }

    // S0
    if (s0) // S0 = 0, S1 = 0
    {
        nrf_drv_gpiote_out_set(ADUCM355_CAP_MUX_SO);
    }
    else
    {
        nrf_drv_gpiote_out_clear(ADUCM355_CAP_MUX_SO);
    }

    if (s1)
    {
        nrf_drv_gpiote_out_set(ADUCM355_CAP_MUX_S1);
    }
    else
    {
        nrf_drv_gpiote_out_clear(ADUCM355_CAP_MUX_S1);
    }
#endif
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
#if 0
        APP_ERROR_CHECK(nrf_serial_read(&serial_uart, &uart_rx_buffer[uart_rx_fill_pos++], 1, NULL, 0));
#else
		size_t nRd = 1;
        ret_code_t err = NRF_SUCCESS;
		char cRet;
		for(int i = 0; i < 10 && (NRF_ERROR_BUSY == (err = nrf_serial_read(&serial_uart, &cRet, 1, &nRd, 0))); ++i);
		if(err == NRF_SUCCESS && nRd == 1) {
            uart_rx_buffer[uart_rx_fill_pos++] = cRet;
#endif
			if (uart_rx_fill_pos == sizeof(uart_rx_buffer)) // overflow, should not happen as long as packets are parsed quickly enough
			{
				NRF_LOG_INFO("*** serial buffer overflow");
				uart_rx_fill_pos = 0;
				buf_rdy = false;
			}
			if (uart_rx_fill_pos >= PKT_LENGTH) 
			{
				buf_rdy = true;
				check_aducm355_rx_buffer();
			}
		} else
			NRF_LOG_INFO("Serial Read Fail %d %d", err, uart_rx_fill_pos);
        unlock_mutex(&uart_buffer_mutex_lock);
    }
}


static void serial_tx(uint8_t *buf, uint8_t *len, uint8_t nPin) {
    static volatile unsigned int* pUARTTX = (volatile unsigned int*) 0x4000250C;
//	unsigned int nTXP = *pUARTTX;
	// n - how much we've send, nOut - size of last transmission, nSend - number of characters to send
	for(size_t n = 0, nOut = 0, nSend, nErr, ex = 10000; n < *len; n += nOut, ex = 20000) {
		// make sure it's empty before sending more (or switching the pin)
//		while(NRF_ERROR_TIMEOUT == nrf_serial_flush(&serial_uart, 0));
		for(size_t x = 0; x < ex && NRF_ERROR_TIMEOUT == (nErr = nrf_serial_flush(&serial_uart, 0)); ++x);
//			taskYIELD();
		if(nErr == NRF_SUCCESS) {
			// change the TX pin if needed
			if(*pUARTTX != nPin) *pUARTTX = nPin;
			// if it's more than our tx buffer, break it up
			if((nSend = *len - n) > SERIAL_FIFO_TX_SIZE) nSend = SERIAL_FIFO_TX_SIZE;
			// send it
	//		while(NRF_ERROR_BUSY == (nErr = nrf_serial_write(&serial_uart, buf + n, nSend, &nOut, 0)));
			for(size_t x = 0; x < ex && NRF_ERROR_BUSY == (nErr = nrf_serial_write(&serial_uart, buf + n, nSend, &nOut, 0)); ++x);
//				taskYIELD();
			if(nErr == NRF_SUCCESS) continue;
			NRF_LOG_ERROR("**** Serial TX Fail %d", nErr);
		} else
			NRF_LOG_ERROR("**** Serial Flush Fail %d %d", nErr, n);
		return;
	}
//	*pUARTTX = nTXP;
}

void serial_tx2(uint8_t *buf, uint8_t len, uint8_t nPin) { serial_tx(buf, &len, nPin); }

#if 0
static void serial_tx(uint8_t *buf, uint8_t *len)
{
    /*for (uint8_t i = 0; i < *len; i++)
    {
        (void)nrf_serial_write(&serial_uart, buf+i, 1, NULL, 0);
    }*/
    (void)nrf_serial_write(&serial_uart, buf, *len, NULL, 0);
//    (void)nrf_serial_flush(&serial_uart, 0);

	while(NRF_ERROR_TIMEOUT == nrf_serial_flush(&serial_uart, 0));


	static char szSend[] =  { "hello0\r\n" };
//	static volatile unsigned int* pUART = 0x40002000;
    static volatile unsigned int* pUARTTX = (volatile unsigned int*) 0x4000250C;
	unsigned int nTXP = *pUARTTX;
	*pUARTTX = TX_PIN;
//	if(NRF_SUCCESS == nrf_serial_write(&serial_uart, "hello\r\n", 7, NULL, 0)) {
	if(NRF_SUCCESS == nrf_serial_write(&serial_uart, szSend, 8, NULL, 0)) {
		while(NRF_ERROR_TIMEOUT == nrf_serial_flush(&serial_uart, 0));
//		if(NRF_SUCCESS == nrf_serial_flush(&serial_uart, 0)) {
			*pUARTTX = nTXP;
			if(szSend[5] == '9') szSend[5] = '0';
			else ++szSend[5];
			return;
//		}
	}
	NRF_LOG_INFO("TX ECHO FAIL");
/*	*/
}
#endif

static void wait_for_ack(void) {
//	for(uint32_t xStart = xTaskGetTickCount(); !ack_rcvd && TICKS_TO_US(xTaskGetTickCount() - xStart) < UART_TIMEOUT_US; vTaskDelay(1));
	for(uint32_t i = 0; !ack_rcvd && i < 100; ++i)
		vTaskDelay(2);
	return;

	uint32_t xStart = xTaskGetTickCount(); 
	while(!ack_rcvd) {
		uint32_t xNow = xTaskGetTickCount();
		uint32_t xDiff = xNow - xStart;
		xDiff = TICKS_TO_US(xDiff);
		if(xDiff > UART_TIMEOUT_US) return;
		vTaskDelay(1);
	}
	return;

	uint32_t timeout_ct = 0;
    do
    {
//		if(xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
//			nrf_delay_us(100);
//		else
			vTaskDelay(1);
        timeout_ct += 100;
    } while (ack_rcvd == false && timeout_ct < UART_TIMEOUT_US);
}

static uint32_t tx_and_wait_for_ack(uint8_t *buf, uint8_t *len)
{
    // ack init
    ack_rcvd = false;
    serial_tx(buf, len, ADUCM355_TX_PIN);
    wait_for_ack(); // delay while we wait for ACK, 5ms timeout
    
    // return 0 for success, 1 for fail
    return (ack_rcvd == true) ? 0 : 1;
}

static void send_ack_aducm355(uart_packet *pkt_to_ack)
{
    uint8_t pkt_length;
    build_ack_packet(&uart_tx_buffer[0], &pkt_length, pkt_to_ack, DNC);
    serial_tx(&uart_tx_buffer[0], &pkt_length, ADUCM355_TX_PIN); // do not wait for ACK
}

static uint32_t send_measurement_aducm355(void)
{
    uint32_t ret;
    uint8_t pkt_length;
    gas_sensor_t use_sensor;

NRF_LOG_INFO("Issue measure %d", measurement_index);
    // get the constants for the current measurement_setting_index (global var)
    uint16_t num_avg;
    float freq;
    bool mux_en, mux_s0, mux_s1;
    get_sensor_specific_measure_params(&use_sensor, &num_avg, &freq, &mux_en, &mux_s0, &mux_s1);

    // configure load capacitor prior to sending measurement command
    config_load_capacitor(mux_en, mux_s0, mux_s1);

    build_start_measure_packet(&uart_tx_buffer[0], &pkt_length, use_sensor, num_avg, freq);
    ret = tx_and_wait_for_ack(&uart_tx_buffer[0], &pkt_length);
    if(ret != 0)
    {
    NRF_LOG_INFO("Issue measure failed ack");
        return ret;
    }
    //APP_ERROR_CHECK(ret);
   // nrf_delay_ms(3);
#if 0
   vTaskDelay(10);

    // ping the ADuCM355 to get current state
    ret = send_ping_aducm355();
    if(ret != 0)
    {
		if(0 != send_ping_aducm355()) {
//		wait_for_ack();
//		if(!ack_rcvd) {
			NRF_LOG_INFO("Issue measure failed ping");
			return ret;
		}
    }
   // APP_ERROR_CHECK(ret);
#endif
    // ensure the ADI chip has transitioned to measure state
    if (aducm355_state == ADUCM355_MEASURE)
    {
        sensing_state = MEASURING; // transition state
        measure_counter_ticks = 0;
    } else {
        NRF_LOG_INFO("Issue measure failed transition");
		return 1;
	}

    return 0;
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

bool bRTCInit = false;

static void start_priming_timer(void)
{
    uint32_t err_code;

    //Initialize RTC instance
    nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
    config.prescaler = 32768/RTC_FREQUENCY; // 32768 / X = Tick Frequency
    if(!bRTCInit) {
		err_code = nrf_drv_rtc_init(&rtc, &config, rtc_handler);
		APP_ERROR_CHECK(err_code);
        bRTCInit = true;
	}

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
    if (bRTCInit == false || time_measurement_sec >= TIMEOUT_SEC) {
        NRF_LOG_INFO("Measurement Timeout -- restart");
        sensing_state = PRIMED; // transition state back to primed, retry the measurement
    } else NRF_LOG_INFO("checking %d", measurement_index);
}

static void run_gas_sensing_algorithim(float *gas_ppm)
{
    float a0,a1,a2,a3,a4;
    get_sensor_specific_calibration_params(&a0, &a1, &a2, &a3, &a4);

    float z_re, z_im;
    z_re = aducm355_result.Z_re;
    z_im = aducm355_result.Z_im;
    *gas_ppm = a0 + a1*z_re + a2*powf(z_re,2) + a3*z_im + a4*powf(z_im,2);
    clear_FPU_interrupts();
}

void new_flush_rx_buffer();

uint32_t init_aducm355_iface(void)
{

    static bool uart_init = false;
    static bool gpio_init = false;
#ifdef BOARD_MATCHBOX_V1
    if(!nrf_drv_gpiote_is_init())
    {
        APP_ERROR_CHECK(nrf_drv_gpiote_init());
    }
    if(!gpio_init)
    {
        gpio_init = true;
        nrf_drv_gpiote_out_config_t out_config_low = GPIOTE_CONFIG_OUT_SIMPLE(false); // false = init low = UART ENABLE
        nrf_drv_gpiote_out_config_t out_config_high = GPIOTE_CONFIG_OUT_SIMPLE(true); // true = init High = MUX DISABLE
        APP_ERROR_CHECK(nrf_drv_gpiote_out_init(ADUCM355_UART_ENABLE, &out_config_low));
        APP_ERROR_CHECK(nrf_drv_gpiote_out_init(ADUCM355_HEATER1, &out_config_low));
        APP_ERROR_CHECK(nrf_drv_gpiote_out_init(ADUCM355_HEATER2, &out_config_low));
        APP_ERROR_CHECK(nrf_drv_gpiote_out_init(ADUCM355_HEATER3, &out_config_low));
        APP_ERROR_CHECK(nrf_drv_gpiote_out_init(ADUCM355_HEATER4, &out_config_low));
        APP_ERROR_CHECK(nrf_drv_gpiote_out_init(ADUCM355_CAP_MUX_NE, &out_config_high));

		// BCA - make sure these pins stay high
        APP_ERROR_CHECK(nrf_drv_gpiote_out_init(UART_TX_PIN, &out_config_high));
        APP_ERROR_CHECK(nrf_drv_gpiote_out_init(ADUCM355_TX_PIN, &out_config_high));
    }
#endif

    ret_code_t ret;
    if (uart_init)
    {
        ret = nrf_serial_uninit(&serial_uart);
		//
		//new_flush_rx_buffer();
		ResetBuffer();
        APP_ERROR_CHECK(ret);
    }
    uart_rx_fill_pos = uart_rx_parse_pos = 0;
    ret = nrf_serial_init(&serial_uart, &m_uart0_drv_config, &serial_config);
    APP_ERROR_CHECK(ret);
    uart_init = true;

    // ping the ADuCM355, it should be asleep
    return send_ping_aducm355();
}

uint32_t start_aducm355_measurement_seq(gas_sensor_t gas_sensors)
{
    ret_code_t ret;

    // ping the ADuCM355, it should be asleep, this will wake it up
    ret = send_ping_aducm355();
    if(ret != 0)
    {
        return ret;
    }
    //APP_ERROR_CHECK(ret);

    // return error if it didn't transition to sleep
    //if (aducm355_state != ADUCM355_IDLE) return 1;

    // start heaters for gas sensors
    control_heaters(true, gas_sensors);

    if (sensing_state == IDLE)
    {
        // start timer to keep track of how long we've been priming
        start_priming_timer();

        // transition state
        sensing_state = PRIMING;
    }
     
    // reset measurement_index to 0
    measurement_index = 0;
    if (gas_sensors == ALL)
    {
        measurement_setting_index = 0;
        measurement_num = NUM_SENSOR_READINGS; //NUM_GAS_SENSORS * NUM_FREQS_PER_SENSOR;
    }
    else
    {
        measurement_setting_index = 0; //gas_sensors*NUM_FREQS_PER_SENSOR;
        measurement_num = NUM_SENSOR_READINGS; //NUM_FREQS_PER_SENSOR;
    }

    // success
    return 0;
}

char txbuffer[64];

// simple state machine but transitions are sequentially made
// this allows us to go to sleep between task handling
uint32_t continue_aducm355_measurement_seq(gas_sensor_results_t *gas_results, bool *measurement_done)
{
    uint32_t ret = 0;
    float calc_ppm = 0.0f;

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
            ret = send_measurement_aducm355();
            break;
        case MEASURING:
            check_measuring_state();
            break;
        case MEASUREMENT_DONE:
            NRF_LOG_INFO("Done Step %d", measurement_index);
            sensing_state = PRIMED; // we are still in the primed state
            
            // copy to results struct
            gas_results[measurement_index].gas_sensor = aducm355_result.sensor;
            gas_results[measurement_index].freq = (uint8_t) (aducm355_result.freq/1000.0f);
            gas_results[measurement_index].Z_re = aducm355_result.Z_re;
            gas_results[measurement_index].Z_im = aducm355_result.Z_im;

            // calculate ppm using 2nd order
            run_gas_sensing_algorithim(&calc_ppm);
            gas_results[measurement_index].gas_ppm = calc_ppm;

			{
				if(!measurement_index) {
					uint8_t nLen = sprintf(txbuffer, "\r\n%d,", TICKS_TO_MS(xTaskGetTickCount()));
					serial_tx(txbuffer, &nLen, UART_TX_PIN);
				}
//				int nRe = aducm355_result.Z_re * 10000.f;
//				int nIm = aducm355_result.Z_im * 10000.f;
//				div_t dRe = div(nRe, 10000);
//				div_t dIm = div(nIm, 10000);
				int nRe = aducm355_result.Z_re;
				int nIm = aducm355_result.Z_im;
				int nReF = (nRe < 0 ? ((float) nRe - aducm355_result.Z_re) : (aducm355_result.Z_re - (float) nRe)) * 10000.f;
				int nImF = (nIm < 0 ? ((float) nIm - aducm355_result.Z_im) : (aducm355_result.Z_im - (float) nIm)) * 10000.f;
//				div_t dRe = div(aducm355_result.Z_re * 10000.f, 10000);
//				div_t dIm = div(aducm355_result.Z_im * 10000.f, 10000);
//				uint8_t nLen = sprintf(txbuffer, "%d,%d,%d,%d.%04d,%d.%04d\r\n", measurement_index, aducm355_result.sensor, (int) (aducm355_result.freq), dRe.quot, dRe.rem, dIm.quot, dIm.rem);
//				uint8_t nLen = sprintf(txbuffer, "%d,%d,%d,%d.%04d,%d.%04d,", measurement_index, aducm355_result.sensor, (int) (aducm355_result.freq), nRe, nReF, nIm, nImF);
				uint8_t nLen = sprintf(txbuffer, "%d.%04d,%d.%04d,", nRe, nReF, nIm, nImF);
//				if(3 == aducm355_result.sensor)
//					NRF_LOG_INFO("Sending Senor 3");
				serial_tx(txbuffer, &nLen, UART_TX_PIN);
			}
            // increment for next measurement
            measurement_index++;
            measurement_setting_index++;
          
            // check if done
            if (measurement_index == measurement_num)
            {
                measurement_num = 0;
                *measurement_done = true;

				uint8_t nLen = sprintf(txbuffer, "%d,%d,%d,%d", latest_measurement_data.temperature, latest_measurement_data.pressure, latest_measurement_data.humidity,measurement_index);
				serial_tx(txbuffer, &nLen, UART_TX_PIN);
            }
            break;
        default:
            return 1; // this should never happen
    }

    // success
    return ret;
}

uint32_t stop_aducm355_measurement_seq(void)
{
    ret_code_t ret;

    // turn off heaters for gas sensors
    control_heaters(false, 0);

    // turn off capacitor mux for gas sensors
    config_load_capacitor(false, false, false);

    // Power off RTC instance
	if(bRTCInit) {
//    if(nrfx_drv_rtc_isinit(&rtc))
	 nrf_drv_rtc_uninit(&rtc); 
	 bRTCInit = false;
	}

    // to IDLE mode
    sensing_state = IDLE;

    // sleep cmd
    ret = send_sleep_aducm355();
    if(ret != 0)
    {
        return ret;
    }
    //APP_ERROR_CHECK(ret);

    // return error if it didn't transition to sleep
    if (aducm355_state != ADUCM355_SLEEP) return 1;       

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
             //send_ack_aducm355(&rx_packet);
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

void reset_aducm355_state_vars(void)
{
    measurement_index = 0;
    measurement_setting_index = 0;
    stop_aducm355_measurement_seq();
}

uint32_t send_ping_aducm355(void)
{
    uint8_t pkt_length;
    build_ping_packet(&uart_tx_buffer[0], &pkt_length);
    return tx_and_wait_for_ack(&uart_tx_buffer[0], &pkt_length);
}

void get_num_aducm355_measurements(uint8_t *num_meas)
{
//    *num_meas = NUM_GAS_SENSORS*NUM_FREQS_PER_SENSOR;
    *num_meas = NUM_SENSOR_READINGS;
}

//////////////////////////////////////////////////////////////////////
// the new uart processing

#if 0

#define UART_MAX_LEN PKT_LENGTH*3


// a circular buffer
typedef struct uart_type {
	uint8_t m_buffer[UART_MAX_LEN]; // = {0}; // 2x margin
	uint8_t m_nRead;
	uint8_t m_nWrite;
} Uart;

Uart g_uart = { {0}, 0, 0 };

// push a new character into the buffer
// return the number of characters available for reading
size_t push(Uart* p, uint8_t nChar) {
	p->m_buffer[p->m_nWrite++] = nChar;
	if(p->m_nWrite == UART_MAX_LEN) p->m_nWrite = 0;
	return (p->m_nRead <= p->m_nWrite) ? p->m_nWrite - p->m_nRead : (UART_MAX_LEN - p->m_nRead) + p->m_nWrite;
}

// return the number of characters available for reading
size_t avail(const Uart* p) {
	uint8_t nWrite = p->m_nWrite; // m_nWrite could change due to the ISR
	return (p->m_nRead <= nWrite) ? nWrite - p->m_nRead : (UART_MAX_LEN - p->m_nRead) + nWrite;
}

// transfer UPTO nMax characters from the buffer into pBuf
// return the number of characters transfered (between 0 and nMax)
size_t transfer(Uart* p, uint8_t* pBuf, uint8_t nMax) {
	uint8_t ret = 0, nWrite = p->m_nWrite; // m_nWrite could change due to the ISR
	// if the read position is ahead for the write position
	if(p->m_nRead > nWrite && ret < nMax) {
		// read from nRead to the end of the buffer
		// how many characters are available
		ret = UART_MAX_LEN - p->m_nRead;
		// more than we need ? trim back to nMax
		if(ret > nMax) ret = nMax;
		memcpy(pBuf, p->m_buffer + p->m_nRead, ret);
		// move nRead
		if((p->m_nRead += ret) == UART_MAX_LEN)
			p->m_nRead = 0;
	}
	// nRead is before nWrite
	// read from m_nRead up to nWrite
	if(ret < nMax) {
		// amount available
		uint8_t nRead = nWrite - p->m_nRead;
		// more than we need ?
		if(ret + nRead > nMax) nRead = nMax - ret;
		memcpy(pBuf + ret, p->m_buffer + p->m_nRead, nRead);
		ret += nRead;
		// move nRead
		if((p->m_nRead += nRead) == UART_MAX_LEN)
			p->m_nRead = 0;
	}
	return ret;
}

// move the read position to the next occurance of nChar
// return false if no such character found
bool adv_to(Uart* p, uint8_t nChar) {
	uint8_t nWrite = p->m_nWrite; // m_nWrite could change (but it will only increment) due to the ISR
	while(p->m_nRead != nWrite) {
		if(p->m_buffer[p->m_nRead] == nChar) return true;
		// those were not the droids we're looking for, advance to the next character (and check for a wrap around)
		if(++p->m_nRead == UART_MAX_LEN) p->m_nRead = 0;
	}
	return false;
}

void vProcessInput(void *pvParameter1, uint32_t ulParameter2) {
	// if we found (and moved to) a start byte
	if(adv_to(&g_uart, START_BYTE)) {
		// is it (atleast) a full packet length?
		if(avail(&g_uart) >= PKT_LENGTH) {
			// figure out the buffer offset of the stop byte
			uint8_t xLoc = (g_uart.m_nRead + PKT_LENGTH - 1);
			if(xLoc > UART_MAX_LEN) xLoc -= UART_MAX_LEN;
			// check for a stop byte
			if(g_uart.m_buffer[xLoc] == STOP_BYTE) {
				// move it to a buffer so it's guaranteed to be contiguous
				transfer(&g_uart, uart_rx_buffer, PKT_LENGTH);
				// check the CRC
				uart_packet* pPacket = (uart_packet*) uart_rx_buffer;
				if(check_crc16(uart_rx_buffer)) {
					// it had a start byte, stop byte and a valid crc
					switch (pPacket->cmd) {
					case CMD_ACK:
						ack_rcvd = true;
						parse_aducm355_ack_payload((ack_payload *)&pPacket->payload);
						return;
					case CMD_SEND_DATA:
						parse_aducm355_measure_data((data_payload *)&pPacket->payload);
						return;
					}
					// if we get here, it was a valid packet but had an unknown command
					NRF_LOG_INFO("*** Packet Bad Cmd");
				} else NRF_LOG_INFO("*** Packet Bad CRC");
			} else {
				NRF_LOG_INFO("*** Packet No Stop Byte, skipping start byte");
				// move our read pointer up one so we skip this start byte
				if(++g_uart.m_nRead == UART_MAX_LEN) g_uart.m_nRead = 0;
			}
			// debug point, something is wrong with this packet if we get here
			NRF_LOG_INFO("***Break");
		}
    }
}

////
// ISR callback -- this is still INSIDE the ISR -- so we need to be quick -- NO PACKET PROCESSING HERE
void new_serial_rx_cb(struct nrf_serial_s const *p_serial, nrf_serial_event_t event) {
    if (event == NRF_SERIAL_EVENT_RX_DATA) { // we got a character
		size_t nRd = 1;
        ret_code_t err = NRF_SUCCESS;
		char cRet;
		// try to read the character
		for(int i = 0; i < 10 && (NRF_ERROR_BUSY == (err = nrf_serial_read(&serial_uart, &cRet, 1, &nRd, 0))); ++i);
		if(err == NRF_SUCCESS && nRd == 1) {
			// push the read character to the circular buffer
			if(push(&g_uart, cRet) >= PKT_LENGTH) {
				// if there is atleast PKT_LENGTH in the buffer, call the Process routine
				BaseType_t xHigherPriorityTaskWoken = pdFALSE;
				if(pdFALSE == xTimerPendFunctionCallFromISR(&vProcessInput, NULL, 0, &xHigherPriorityTaskWoken))
					NRF_LOG_INFO("*** Serial Process Call Failed");
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
				//portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
			}
		} else
			NRF_LOG_INFO("*** Serial Read Fail %d", err);
	}
}

void new_flush_rx_buffer() {
	g_uart.m_nRead = g_uart.m_nWrite = 0;
}

#else

extern TaskHandle_t process_uart_input_handle;

uart_packet packet;
void vProcessInput(void* pvParameter) {
	for(;;) {
		// wait for ISR to signal
//		NRF_LOG_INFO("** Awaiting Packet");
		ulTaskNotifyTake(pdTRUE,         /* Clear the notification value before exiting (equivalent to the binary semaphore). */
                      portMAX_DELAY); //) /* Block indefinitely	*/
		if(FindPacket(&packet)) {
			switch(packet.cmd) {
			case CMD_ACK:
				ack_rcvd = true;
				parse_aducm355_ack_payload((ack_payload *)&packet.payload);
				break;
			case CMD_SEND_DATA:
				parse_aducm355_measure_data((data_payload *)&packet.payload);
				break;
			default:
				NRF_LOG_INFO("*** Packet Bad Cmd");
			}
		}
	}
}


//volatile int nInUARTISR = 0;

////
// ISR callback -- this is still INSIDE the ISR -- so we need to be quick -- NO PACKET PROCESSING HERE
void new_serial_rx_cb(struct nrf_serial_s const *p_serial, nrf_serial_event_t event) {
//	volatile int* pInUARTISR = &nInUARTISR;
//	nInUARTISR = 1;
    if (event == NRF_SERIAL_EVENT_RX_DATA) { // we got a character
//		nInUARTISR = 2;
		size_t nRd = 1;
        ret_code_t err = NRF_SUCCESS;
		char cRet;
		// try to read the character
		for(int i = 0; i < 10 && (NRF_ERROR_BUSY == (err = nrf_serial_read(&serial_uart, &cRet, 1, &nRd, 0))); ++i);
		if(err == NRF_SUCCESS && nRd == 1) {
//			nInUARTISR = 3;
			// push the read character to the circular buffer
			if(PushBuffer(cRet) >= PKT_LENGTH) {
//				nInUARTISR = 4;
				// if there is atleast PKT_LENGTH in the buffer, signal the Process thread
				BaseType_t xHigherPriorityTaskWoken = pdFALSE;
				vTaskNotifyGiveFromISR(process_uart_input_handle, &xHigherPriorityTaskWoken);
				if(xHigherPriorityTaskWoken == pdTRUE)
					portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
			}
		} else
			NRF_LOG_INFO("*** Serial Read Fail %d", err);
	}
//    nInUARTISR = 0;
}
#endif


