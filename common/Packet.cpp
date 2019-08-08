#include "Packet.h"

#if 0
#define SIZE 256
uint16_t m_nRead = 0;
uint16_t m_nWrite = 0;
uint8_t m_Buffer[SIZE];

uint16_t PushBuffer(uint8_t nChar) { 
	m_Buffer[m_nWrite++] = nChar;
	// m_Read might change (and potentially wrap) between our check and calculation
	auto nRead = m_nRead;
	return nRead < m_nWrite ? m_nWrite - nRead : (SIZE - nRead) + m_nWrite;
}

void ResetBuffer() { m_nRead = m_nWrite = 0; }

bool FindPacket(uart_packet* p) { return findPacket(*p, uart_buffer); }
#endif

static CircularBuffer<> uart_buffer;

uint16_t PushBuffer(uint8_t nChar) { return uart_buffer.push(nChar); }

void ResetBuffer() { uart_buffer.Reset(); }

bool FindPacket(uart_packet* p) { return findPacket(*p, uart_buffer); }

