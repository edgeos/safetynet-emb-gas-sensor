#include <string.h>
#include "aducm355_cmd_protocol.h"

// logging
#ifdef NRF52
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#endif

static data_payload data_buffer = {0};
static measure_payload measure_buffer = {0};
static ack_payload ack_buffer = {0};

static void pkt_constants(uart_packet * pkt)
{
   pkt->sflag = START_BYTE;
   pkt->eflag = STOP_BYTE;
   pkt->len   = PKT_PAYLOAD_LENGTH + PKT_END_LENGTH; // 2 for crc, 1 for end byte
}

static void build_empty_payload_pkt(uart_packet * pkt, uint8_t cmd_byte)
{   
   // ping cmd
   pkt->cmd = cmd_byte;
   
   // add constants
   pkt_constants(pkt);

   // calculate and add CRC
   pkt->crc = crc16((uint8_t*)pkt, (uint8_t)(PKT_LENGTH-PKT_END_LENGTH));
}

volatile uint32_t nLook = 0;

/**@brief Try to parse a packet from a RX buffer
 *
 */
uint8_t look_for_packet(uint8_t * uart_rx_buffer, uint8_t length, uart_packet * pkt)
{
   uint8_t search_buffer[PKT_LENGTH*3];
   
   // check for min pkt length, return immediately if not long enough
   if( length < PKT_LENGTH)
   {
     return 0;
   }
   
  nLook = 1;
   // copy to working buffer so no interrupt conflicts
   memcpy(&search_buffer[0], uart_rx_buffer, length);
   ++nLook;
   uint8_t pkt_end_ind = 0;
   for (uint8_t i = 0; i < length; i++)
   {
      if(search_buffer[i] == START_BYTE) // found start
      {
         pkt_end_ind = i + PKT_LENGTH - 1;
         if(pkt_end_ind < (length-1))
         {
            return 0;
         }
         else if (search_buffer[pkt_end_ind] == STOP_BYTE) // found stop
         {
            // check for CRC match
            if(check_crc16(&search_buffer[i]))
            {
               memcpy(pkt, &search_buffer[i], PKT_LENGTH);
#ifdef NRF52
			   if(i) NRF_LOG_INFO("Packet found at %d", i);
#endif
                  nLook = 0;

               return 1;
            }
#ifdef NRF52
            else NRF_LOG_INFO("Packet failed crc %d", i);
#endif
                  nLook |= 0x100;
         }
         else
         {
                            nLook |= 0x400;

//            return 0;
         }
      }
   }
   
   // if we got here then no match ever found
   return 0;
}

/**@brief Build a sleep packet to send 
 *
 */
void build_sleep_packet(uint8_t * uart_tx_buffer, uint8_t * pkt_len)
{
    // what we'll use to build the packet
   uart_packet pkt = {0};
  
   // static packet lengths for now 
   *pkt_len = PKT_LENGTH;

   // payload is empty
   build_empty_payload_pkt(&pkt, CMD_SLEEP);
  
   // copy to uart buffer
   memcpy(uart_tx_buffer, &pkt, *pkt_len);
}


/**@brief Build a ping packet to send 
 *
 */
void build_ping_packet(uint8_t * uart_tx_buffer, uint8_t * pkt_len)
{
    // what we'll use to build the packet
   uart_packet pkt = {0};
  
   // static packet lengths for now 
   *pkt_len = PKT_LENGTH;

   // payload is empty
   build_empty_payload_pkt(&pkt, CMD_PING);
  
   // copy to uart buffer
   memcpy(uart_tx_buffer, &pkt, *pkt_len);
}

/**@brief Build an ack packet to send back
 *
 */
void build_ack_packet(uint8_t * uart_tx_buffer, uint8_t * pkt_len, uart_packet * pkt_to_ack, aducm355_state_t state)
{
   // what we'll use to build the packet
   uart_packet pkt = {0};
  
   // static packet lengths for now 
   *pkt_len = PKT_LENGTH;
   
   // copy ack info over
   pkt.cmd = CMD_ACK;
   
   // add constants
   pkt_constants(&pkt);

   ack_buffer.cmd_ackd = pkt_to_ack->cmd; 
   ack_buffer.state = state;
   memcpy(&pkt.payload, &ack_buffer, sizeof(ack_payload));
   
   // calculate and add CRC
   pkt.crc = crc16((uint8_t*) &pkt, (uint8_t)(PKT_LENGTH-PKT_END_LENGTH));
  
   // copy to uart buffer
   memcpy(uart_tx_buffer, &pkt, *pkt_len);
}

/**@brief Build a measure cmd packet to send
 *
 */
void build_start_measure_packet(uint8_t * uart_tx_buffer, uint8_t * pkt_len, uint8_t sensor, uint8_t num_avg, float freq)
{
   // what we'll use to build the packet
   uart_packet pkt = {0};
  
   // static packet lengths for now 
   *pkt_len = PKT_LENGTH;
   
   // copy ack info over
   pkt.cmd = CMD_START_MEASURE; 
   
   // add constants
   pkt_constants(&pkt);
   
   // payload
   measure_buffer.sensor  = sensor;
   measure_buffer.num_avg = num_avg;
   measure_buffer.freq    = freq;
   memcpy(&pkt.payload, &measure_buffer, sizeof(measure_buffer));
   
   // calculate and add CRC
   pkt.crc = crc16((uint8_t*) &pkt, (uint8_t)(PKT_LENGTH-PKT_END_LENGTH));
  
   // copy to uart buffer
   memcpy(uart_tx_buffer, &pkt, *pkt_len);
}

/**@brief Build a data packet to send
 *
 */
void build_data_packet(uint8_t * uart_tx_buffer, uint8_t * pkt_len,  MeasResult_t * data)
{
   // what we'll use to build the packet
   uart_packet pkt = {0};
  
   // static packet lengths for now 
   *pkt_len = PKT_LENGTH;
   
   // copy ack info over
   pkt.cmd = CMD_SEND_DATA; // pkt_to_ack->cmd; 
   
   // add constants
   pkt_constants(&pkt);
   
   // copy over data from full ImpResult to reduced size structure
   data_buffer.ImpResult.sensor = data->sensor;
   data_buffer.ImpResult.freq = data->freq;
   data_buffer.ImpResult.Z_im = data->Z_im;
   data_buffer.ImpResult.Z_re = data->Z_re;
   memcpy(&pkt.payload, &data_buffer, sizeof(data_payload));
   
   // calculate and add CRC
   pkt.crc = crc16((uint8_t*) &pkt, (uint8_t)(PKT_LENGTH-PKT_END_LENGTH));
  
   // copy to uart buffer
   memcpy(uart_tx_buffer, &pkt, *pkt_len);
}

/**@brief Check crc16
 *
 */
uint8_t check_crc16(uint8_t* pkt)
{
    uint16_t rcv_crc;
    uint8_t crc_ind_st;
    
    // get recv crc
    crc_ind_st = PKT_LENGTH-PKT_END_LENGTH;
    rcv_crc = *(pkt+crc_ind_st+1) << 8 | *(pkt+crc_ind_st); // big endian
    
    // check received vs calculated
    if (rcv_crc == crc16(pkt, PKT_LENGTH-PKT_END_LENGTH))
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

    while (length--)
    {
        x = crc >> 8 ^ *data_p++;
        x ^= x>>4;
        crc = (crc << 8) ^ ((unsigned short)(x << 12)) ^ ((unsigned short)(x <<5)) ^ ((unsigned short)x);
    }
    return crc & 0xFFFF;
}

