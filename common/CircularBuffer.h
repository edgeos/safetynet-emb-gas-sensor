#ifndef _CIRCULAR_BUFFER_H_GAS_SENSOR_
#define _CIRCULAR_BUFFER_H_GAS_SENSOR_

#include <stdint.h>
#include <string.h>

// a circular buffer
template <uint16_t SIZE = 256, class T = uint8_t, class S = uint16_t> class CircularBuffer {
public:
	using this_type = CircularBuffer<SIZE, T, S>;
	using value_type = T;
	using size_type = S;

protected:
	size_type m_nRead;
	size_type m_nWrite;
	value_type m_Buffer[SIZE];

	inline static size_type& inc(size_type& n) {
		if(++n == SIZE) n = 0;
		return n;
	}

public:
	CircularBuffer() : m_nRead(0), m_nWrite(0) {}

	void Reset() { m_nRead = m_nWrite = 0; }

	// add a character to the write position
	size_type push(value_type nChar) {
		m_Buffer[m_nWrite] = nChar;
		// m_Read might change (and potentially wrap) between our check and calculation
		auto nRead = m_nRead;
		return nRead < inc(m_nWrite) ? m_nWrite - nRead : (SIZE - nRead) + m_nWrite;
	}

	// return the number of characters available for reading
	inline size_type avail() const {
		// m_nWrite could change (and potentially wrap) due to the ISR
		auto nWrite = m_nWrite; 
		return (m_nRead <= nWrite) ? nWrite - m_nRead : (SIZE - m_nRead) + nWrite;
	}

	value_type pop() {
		value_type ret = -1;
		if(m_nRead != m_nWrite) {
			ret = m_Buffer[m_nRead];
			inc(m_nRead);
		}
		return ret;
	}

	// transfer UPTO nMax characters from the buffer into pBuf
	// return the number of characters transfered (between 0 and nMax)
	size_type transfer(value_type* pBuf, size_type nMax) {
		size_type ret = 0;
		// if the read position is ahead for the write position
		if(m_nRead > m_nWrite && ret < nMax) {
			// read from nRead to the end of the buffer
			// how many characters are available
			ret = SIZE - m_nRead;
			// more than we need ? trim back to nMax
			if(ret > nMax) ret = nMax;
			memcpy(pBuf, m_Buffer + m_nRead, ret);
			// move nRead
			if((m_nRead += ret) == SIZE)
				m_nRead = 0;
		}
		// nRead is before nWrite
		// read from m_nRead up to nWrite
		if(ret < nMax) {
			// amount available
			size_type nRead = m_nWrite - m_nRead;
			// more than we need ?
			if(ret + nRead > nMax) nRead = nMax - ret;
			memcpy(pBuf + ret, m_Buffer + m_nRead, nRead);
			ret += nRead;
			// move nRead
			if((m_nRead += nRead) == SIZE)
				m_nRead = 0;
		}
		return ret;
	}

	// move the read position to the next occurance of nChar
	// return false if no such character found
	bool adv_to(value_type nChar) {
		// m_nWrite could change (but it will only increment) due to the ISR
		while(m_nRead != m_nWrite) {
			if(m_Buffer[m_nRead] == nChar) return true;
			// those were not the droids we're looking for, advance to the next character (and check for a wrap around)
			inc(m_nRead);
		}
		return false;
	}

	inline size_type Read() const { return m_nRead; }
	inline size_type Write() const { return m_nWrite; }
	value_type& operator[](size_type n) { return m_Buffer[n]; }
	value_type operator[](size_type n) const { return m_Buffer[n]; }
};


#endif //_CIRCULAR_BUFFER_H_GAS_SENSOR_
