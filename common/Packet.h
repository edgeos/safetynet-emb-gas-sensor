#ifndef _PACKET_H_GAS_SENSOR_
#define _PACKET_H_GAS_SENSOR_

#include "aducm355_cmd_protocol.h"

#ifdef NRF52
// logging
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

uint16_t PushBuffer(uint8_t nChar);
void ResetBuffer();
bool FindPacket(uart_packet* p);

#ifdef __cplusplus
}

#include "CircularBuffer.h"

template <class P, uint16_t SIZE, class T, class S> bool findPacket(P& packet, CircularBuffer<SIZE,T,S>& buf) {
	while(buf.adv_to(START_BYTE)) {
		// we found (and moved to) a start byte
		// is it (atleast) a full packet length?
		if(buf.avail() < sizeof(P)) return false;
		// figure out the buffer offset of the stop byte
		S xLoc = (buf.Read() + sizeof(P) - 1);
		if(xLoc >= SIZE) xLoc -= SIZE;
		// check for a stop byte
		if(buf[xLoc] != STOP_BYTE) {
			// start but no stop byte found, skip that start byte
#ifdef NRF52
			NRF_LOG_INFO("No Stop Bit Found");
#endif
			buf.pop();
		} else {
			// move it to a buffer so it's guaranteed to be contiguous
			auto p = reinterpret_cast<T*>(&packet);
			buf.transfer(p, sizeof(P));
#ifdef NRF52
//			NRF_LOG_INFO("Packet cmd %d payload[0] %d R/W %d %d", packet.cmd, packet.payload[0], buf.Read(), buf.Write());
//			NRF_LOG_INFO("Packet cmd %d payload[0] %d", packet.cmd, packet.payload[0]);
#endif
			if(check_crc16(p)) return true;
#ifdef NRF52
			NRF_LOG_INFO("Failed CRC");
#endif
		}
	}
	return false;
}

#endif

#endif // _PACKET_H_GAS_SENSOR_
