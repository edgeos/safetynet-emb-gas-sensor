#ifndef ADUCM355_CMD_PROTOCOL_H__
#define ADUCM355_CMD_PROTOCOL_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif
  
#define CMD_PING                0x00
#define CMD_START_MEASURE       0x01
#define CMD_STOP_MEASURE        0x02
#define CMD_SEND_DATA           0x03
#define CMD_SLEEP               0x04
#define CMD_ACK                 0x05
 
#define START_BYTE              0xAA
#define STOP_BYTE               0xFF

#define PKT_ST_LENGTH           3  // start, cmd, len
#define PKT_PAYLOAD_LENGTH      15 // sizeOf(ImpResult_t)
#define PKT_END_LENGTH          3
#define PKT_LENGTH              (PKT_ST_LENGTH + PKT_PAYLOAD_LENGTH + PKT_END_LENGTH)

/**@brief Gas Sensor State */
typedef enum
{
    ADUCM355_SLEEP,
    ADUCM355_IDLE,
    ADUCM355_MEASURE,
    DNC // DO NOT CARE - always sent by Nordic chip in Acks
} aducm355_state_t;

#pragma pack(1) 
typedef struct
{
   uint8_t      sensor;
   float        freq;
   float        Z_re;
   float        Z_im;
}MeasResult_t;

typedef struct
{ 
   uint8_t  sflag;
   uint8_t  cmd;
   uint8_t  len;
   uint8_t  payload[PKT_PAYLOAD_LENGTH];
   uint16_t crc;
   uint8_t  eflag;
} uart_packet;

typedef struct
{ 
   MeasResult_t ImpResult;
} data_payload;

typedef struct
{ 
   uint8_t      sensor;
   uint8_t      num_avg;
   float        freq;
   uint16_t     dft_num;
   uint8_t filler[PKT_PAYLOAD_LENGTH-8];
} measure_payload;   

typedef struct
{ 
   uint8_t cmd_ackd;
   aducm355_state_t state;
   uint8_t filler[PKT_PAYLOAD_LENGTH-2];
} ack_payload;

/**@brief Try to parse a packet from a RX buffer
 *
 */
uint8_t look_for_packet(uint8_t * uart_rx_buffer, uint8_t len, uart_packet * pkt);

/**@brief Build a sleep packet to send 
 *
 */
void build_sleep_packet(uint8_t * uart_tx_buffer, uint8_t * pkt_len);

/**@brief Build a ping packet to send 
 *
 */
void build_ping_packet(uint8_t * uart_tx_buffer, uint8_t * pkt_len); 

/**@brief Build an ack packet to send back
 *
 */
void build_ack_packet(uint8_t * uart_tx_buffer, uint8_t * pkt_len, uart_packet * pkt_to_ack, aducm355_state_t state); 

/**@brief Build a measure command packet to send
 *
 */
void build_start_measure_packet(uint8_t * uart_tx_buffer, uint8_t * pkt_len, uint8_t sensor, uint8_t num_avg, float freq);

/**@brief Build a data packet to send
 *
 */
void build_data_packet(uint8_t * uart_tx_buffer, uint8_t * pkt_len, MeasResult_t * data); 

/**@brief Check packet for CRC match
 *
 */
uint8_t check_crc16(uint8_t* data_check);

/**@brief Calculate crc16 CCITT
 *
 */
uint16_t crc16(uint8_t* data_p, uint8_t length);

#ifdef __cplusplus
}
#endif

#endif /* ADUCM355_CMD_PROTOCOL_H__ */