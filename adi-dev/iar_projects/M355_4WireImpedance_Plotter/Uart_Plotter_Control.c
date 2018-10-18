#include <string.h>
#include "Uart_Plotter_Control.h"

static data_payload data_buffer = {0};
static measure_payload measure_buffer = {0};
static ack_payload ack_buffer = {0};

/**@brief Try to parse a packet from a RX buffer
 *
 */
uint8_t look_for_packet(uint8_t * uart_rx_buffer, uint8_t length, uart_packet * pkt)
{
   uint8_t search_buffer[256];
   
   // check for min pkt length, return immediately if not long enough
   if( length < PACKET_LENGTH)
   {
     return 0;
   }
   
   // copy to working buffer so no interrupt conflicts
   memcpy(&search_buffer[0], uart_rx_buffer, length);
   uint8_t pkt_end_ind = 0;
   for (uint8_t i = 0; i < length; i++)
   {
      if(search_buffer[i] == START_BYTE1 &
         search_buffer[i+1] == START_BYTE2 &
         search_buffer[i+2] == START_BYTE3 &
         search_buffer[i+3] == START_BYTE4 &
         search_buffer[i+4] == START_BYTE5)
      {
         pkt_end_ind = i + PACKET_LENGTH - 1;
         if(pkt_end_ind < (length-1))
         {
            return 0;
         }
         else if (search_buffer[pkt_end_ind] == STOP_BYTE)
         {
            // check for CRC match
            if(check_crc16(&search_buffer[i]))
            {
               memcpy(pkt, &search_buffer[i], PACKET_LENGTH);
               return 1;
            }
         }
         else
         {
            return 0;
         }
      }
   }
   
   // if we got here then no match ever found
   return 0;
}

/**@brief Build an ack packet to send back
 *
 */
void build_ack_packet(uint8_t * uart_tx_buffer, uint8_t * pkt_len, uart_packet * pkt_to_ack)
{
   // what we'll use to build the packet
   uart_packet ack_pkt = {0};
  
   // static packet lengths for now 
   *pkt_len = PACKET_LENGTH;
   
   // copy ack info over
   ack_pkt.cmd = CMD_ACK; // pkt_to_ack->cmd; 
   
   // build rest of packet
   ack_pkt.sflag[0] = START_BYTE1;
   ack_pkt.sflag[1] = START_BYTE2;
   ack_pkt.sflag[2] = START_BYTE3;
   ack_pkt.sflag[3] = START_BYTE4;
   ack_pkt.sflag[4] = START_BYTE5;
   ack_pkt.eflag = STOP_BYTE;
   ack_pkt.len = PAYLOAD_LENGTH + CRC_END_LENGTH; // 2 for crc, 1 for end byte
   memcpy(&ack_pkt.payload, &ack_buffer, sizeof(ack_payload));
   
   // calculate and add CRC
   ack_pkt.crc = crc16((uint8_t*) &ack_pkt, (uint8_t)(PACKET_LENGTH-CRC_END_LENGTH));
  
   // copy to uart buffer
   memcpy(uart_tx_buffer, &ack_pkt, *pkt_len);
}

/**@brief Build a data packet to send
 *
 */
void build_data_packet(uint8_t * uart_tx_buffer, uint8_t * pkt_len, ImpResult_t * data, uint16_t seg_num, uint16_t num_segs)
{
   // what we'll use to build the packet
   uart_packet data_pkt = {0};
  
   // static packet lengths for now 
   *pkt_len = PACKET_LENGTH;
   
   // copy ack info over
   data_pkt.cmd = CMD_SEND_DATA; // pkt_to_ack->cmd; 
   
   // build rest of packet
   data_pkt.sflag[0] = START_BYTE1;
   data_pkt.sflag[1] = START_BYTE2;
   data_pkt.sflag[2] = START_BYTE3;
   data_pkt.sflag[3] = START_BYTE4;
   data_pkt.sflag[4] = START_BYTE5;
   data_pkt.eflag = STOP_BYTE;
   data_pkt.len = PAYLOAD_LENGTH + CRC_END_LENGTH; // 2 for crc, 1 for end byte
   
   // copy over data
   memcpy(&data_buffer.ImpResult, data, sizeof(ImpResult_t));
   data_buffer.seg_num = seg_num;
   data_buffer.num_segs = num_segs;
   memcpy(&data_pkt.payload, &data_buffer, sizeof(data_payload));
   
   // calculate and add CRC
   data_pkt.crc = crc16((uint8_t*) &data_pkt, (uint8_t)(PACKET_LENGTH-CRC_END_LENGTH));
  
   // copy to uart buffer
   memcpy(uart_tx_buffer, &data_pkt, *pkt_len);
}

/**@brief Check crc16
 *
 */
uint8_t check_crc16(uint8_t* pkt)
{
    uint16_t rcv_crc;
    uint8_t crc_ind_st;
    
    // get recv crc
    crc_ind_st = PACKET_LENGTH-CRC_END_LENGTH;
    rcv_crc = *(pkt+crc_ind_st+1) << 8 | *(pkt+crc_ind_st); // big endian
    
    // check received vs calculated
    if (rcv_crc == crc16(pkt, PACKET_LENGTH-CRC_END_LENGTH))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**@brief Calculate crc16 CCITT
 *
 */
uint16_t crc16(uint8_t* data_p, uint8_t length)
{
    unsigned char x;
    unsigned short crc = 0xFFFF;

    while (length--){
        x = crc >> 8 ^ *data_p++;
        x ^= x>>4;
        crc = (crc << 8) ^ ((unsigned short)(x << 12)) ^ ((unsigned short)(x <<5)) ^ ((unsigned short)x);
    }
    return crc & 0xFFFF;
}