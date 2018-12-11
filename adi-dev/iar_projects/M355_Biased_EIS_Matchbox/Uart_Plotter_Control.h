#ifndef UART_PLOTTER_CONTROL_H__
#define UART_PLOTTER_CONTROL_H__

#include <stdint.h>
#include "M355_ECSns_EIS.h"

#ifdef __cplusplus
extern "C" {
#endif
  
#define PACKET_ST_LENGTH          7  // start, cmd, len
#define PAYLOAD_LENGTH            32 // sizeOf(ImpResult_t) + sizeOf(seg_num) + sizeOf(num_segs)
#define CRC_END_LENGTH            3
#define PACKET_LENGTH             (PACKET_ST_LENGTH+ PAYLOAD_LENGTH + CRC_END_LENGTH)
  
#define CMD_PING                0x00
#define CMD_START_MEASURE       0x01
#define CMD_STOP_MEASURE        0x02
#define CMD_SEND_DATA           0x03
#define CMD_ACK                 0x04
 
#define START_BYTE1             0x55
#define START_BYTE2             0x44
#define START_BYTE3             0x33
#define START_BYTE4             0x22
#define START_BYTE5             0x11
#define STOP_BYTE               0xFF

#pragma pack(1)  
typedef struct
{
   float freq;
   int32_t DFT_result[4];
   float   Mag;
   float   Phase;
}ImpResult_Reduced_t;

typedef struct
{ 
   uint8_t sflag[5];
   uint8_t cmd;
   uint8_t len;
   uint8_t payload[PAYLOAD_LENGTH];
   uint16_t crc;
   uint8_t eflag;
} uart_packet;

typedef struct
{ 
   uint16_t seg_num;
   uint16_t num_segs;
   ImpResult_Reduced_t ImpResult;
} data_payload;

typedef struct
{ 
   float freq_start;
   float freq_step;
   float freq_stop;
   uint8_t filler[PAYLOAD_LENGTH-12];
} measure_payload;   

typedef struct
{ 
   uint8_t cmd_ackd;
   uint8_t filler[PAYLOAD_LENGTH-1];
} ack_payload;

/**@brief Try to parse a packet from a RX buffer
 *
 */
uint8_t look_for_packet(uint8_t * uart_rx_buffer, uint8_t len, uart_packet * pkt);

/**@brief Build an ack packet to send back
 *
 */
void build_ack_packet(uint8_t * uart_tx_buffer, uint8_t * pkt_len, uart_packet * pkt_to_ack); 

/**@brief Build a data packet to send
 *
 */
void build_data_packet(uint8_t * uart_tx_buffer, uint8_t * pkt_len, ImpResult_t * data, uint16_t seg_num, uint16_t num_segs); 

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

#endif /* UART_PLOTTER_CONTROL_H__ */