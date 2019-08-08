#ifndef _MAX_30101_H_DEV_MCL_
#define _MAX_30101_H_DEV_MCL_

#include "nrf_drv_twi.h"
#include "nrf_delay.h"
#include "ble_hrs.h"
//#include "FreeRTOS.h"

extern "C" {
uint32_t twi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
uint32_t twi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
};

#include <cstdint>
#define constexpr
#define noexcept
#define IOEx

namespace std {
  template <class A, class B> struct pair {
    pair(const A& a, const B& b) : first(a), second(b) {}
    pair() {}
    A first;
    B second;
  };
  typedef std::int32_t ptrdiff_t;
}

//#include "dev/header.h"
#define DevEx 

#ifdef _WIN32
#define PACKED
#else
#define PACKED __attribute__((packed))
#endif

//////////////////////////////////////////////////////////

//! BYTEOFF -- the byte off set to the register
//! T -- the data type of the register
template <std::ptrdiff_t BYTEOFF, class T> struct RegisterConstant {
	typedef T type; //using type = T;
	constexpr static const std::ptrdiff_t offset = BYTEOFF;
	constexpr static const std::ptrdiff_t index = BYTEOFF / sizeof(T);
	constexpr std::ptrdiff_t operator()() const noexcept { return index; }
};

template <std::ptrdiff_t BYTEOFF, class T> constexpr const std::ptrdiff_t RegisterConstant<BYTEOFF,T>::offset;
template <std::ptrdiff_t BYTEOFF, class T> constexpr const std::ptrdiff_t RegisterConstant<BYTEOFF,T>::index;


//////////////////////////////////////////////////////////

//////////////////////
//
template <class R> struct RegisterTraits {
//	using reference = typename R::reference;
//	using const_reference = typename R::const_reference;
//	using value_type = typename R::value_type;

	typedef typename R::reference reference;
	typedef typename R::const_reference const_reference;
	typedef typename R::value_type value_type;
};

template <class T> struct RegisterTraits<T*> {
//	using reference = T&;
//	using const_reference = const T&;
//	using value_type = T;
	typedef T& reference;
	typedef const T& const_reference;
	typedef T value_type;
};

template <class R> struct Registers : public R {
//	using regr_type = R;
//	using reference = typename RegisterTraits<R>::reference;
//	using const_reference = typename RegisterTraits<R>::const_reference;
//	using value_type = typename RegisterTraits<R>::value_type;
	typedef R regr_type;
	typedef typename RegisterTraits<R>::reference reference;
	typedef typename RegisterTraits<R>::const_reference const_reference;
	typedef typename RegisterTraits<R>::value_type value_type;

	inline reference at(std::ptrdiff_t n) { return this->operator[](n); }
	inline const_reference at(std::ptrdiff_t n) const { return this->operator[](n); }

	template <class T> T at() const { return T(this->operator[](T::index)); }
	template <class T> T at(std::ptrdiff_t n) const { return T(this->operator[](T::index + n)); }

	void Set(std::ptrdiff_t n, value_type m, value_type v) {
		at(n) = (at(n) & ~m) | (v & m);
	}
	inline void Set(std::ptrdiff_t n, value_type m) { Set(n, m, m); }

	template <class T> inline void Set(value_type m, value_type v) { Set(T::index, m, v); }
	template <class T> inline void Set(value_type v) { Set(T::index, ~0, v); }
	template <class T> inline void Set(const T& x) { this->operator[](T::index) = x; }

	Registers(const regr_type& r = regr_type()) : R(r) {}
};

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//! internal macro, common setup for a register class
//! \ingroup em_reg_macros
#define MCL_REGISTER_COMMON(xName, xType) \
	typedef xType value_type; \
	typedef xName this_type; \
	operator value_type() const { return Reg; } \
	xName(value_type r = 0) : Reg(r) {}

//! internal macro, common setup for a register class with bitfields
//! \ingroup em_reg_macros
#define MCL_REGISTER_BITFIELDS_COMMON \
	union { \
		value_type Reg; \
		struct

//! a simple register, only the value
//! \ingroup em_reg_macros
#define MCL_REGISTER(xName,xType,xN) \
    struct xName : public RegisterConstant<xN,xType> { \
		MCL_REGISTER_COMMON(xName, xType) \
		value_type Reg; \
	}

//! a register dervied from a base type
//! \ingroup em_reg_macros
#define MCL_REGISTER_BASE(xName,xType,xN,xBase) \
    struct xName : public xBase, public RegisterConstant<xN,xType> { \
		xName(xType r = 0) : xBase(r) {} \
	}

//! a register with bitfields
//! this still requires the opening {
//! \ingroup em_reg_macros
#define MCL_REGISTER_BITFIELDS(xName,xType,xN) \
    struct xName : public RegisterConstant<xN,xType> { \
		MCL_REGISTER_COMMON(xName, xType) \
		MCL_REGISTER_BITFIELDS_COMMON

//! ends the bitfields struct, allow placement of member functions
//! \ingroup em_reg_macros
#define MCL_REGISTER_BITFIELDS_DONE \
		} PACKED; \
	} PACKED;

//! closes a bitfields register, allow declaration of a variable
//! requires a ; after use
//! \ingroup em_reg_macros
#define MCL_REGISTER_BITFIELDS_CLOSE \
	} PACKED

//! \ingroup em_reg_macros
//! end a bitfields register with no function members, allow declaration of a variable
//! requires a ; after use
#define MCL_REGISTER_BITFIELDS_END MCL_REGISTER_BITFIELDS_DONE MCL_REGISTER_BITFIELDS_CLOSE

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

template <class R, class T = std::uint8_t> struct RegisterConstReference {
        typedef R type;
        typedef T value_type;
	const type* m_p;
	value_type m_n;
	operator typename type::value_type() const { return m_p->get(m_n); }
	RegisterConstReference(const type* p, value_type n) : m_p(p), m_n(n) {}
};

template <class R, class T = std::uint8_t> struct RegisterReference : public RegisterConstReference<R,T> {
	typedef RegisterConstReference<R,T> base_type;
	typedef RegisterReference<R,T> this_type;
	typedef typename base_type::type type;
	typedef typename base_type::value_type value_type;
	RegisterReference(const type* p, value_type n) : base_type(p,n) {}
	this_type& operator=(T val) { this->m_p->set(this->m_n, val); return *this; }
};


struct IOEx TwoWireRegister {
	typedef std::uint8_t value_type; //typename mcl::io::TwoWire8::char_type;
	typedef RegisterConstReference<TwoWireRegister, value_type> const_reference;
	typedef RegisterReference<TwoWireRegister, value_type> reference;
	typedef std::uint8_t addr_type; //typename mcl::io::TwoWire8::addr_type;

//	TwoWireRegister(mcl::io::TwoWire8* p, std::uint8_t nAddr) : m_p(p), m_addr(nAddr) {}
//	mcl::io::TwoWire8* m_p;
//typedef uint32_t (*m24m02_com_fptr_write_t)(uint8_t dev_id, uint8_t *reg_addr, uint8_t *data);
//typedef uint32_t (*m24m02_com_fptr_read_t) (uint8_t dev_id, uint8_t *reg_addr, uint8_t *data);
	TwoWireRegister(const nrf_drv_twi_t* p, std::uint8_t nAddr) : m_p(p), m_addr(nAddr) {}
//        TwoWireRegister(const TwoWireRegister& x) : m_p(x.m_p), m_addr(x.m_addr) {}
        const nrf_drv_twi_t* m_p;
	addr_type m_addr;

	std::uint8_t get(std::uint8_t reg) const {
          get(reg, &reg, 1);
          return reg;
	}

	std::uint8_t set(std::uint8_t reg, std::uint8_t value) const {
          return NRF_SUCCESS == twi_write(m_addr, reg, &value, 1) ? 1 : 0;
        }

	const_reference operator[](std::uint8_t n) const {
		return const_reference(this, n);
	}

	reference operator[](std::uint8_t n) {
		return reference(this, n);
	}

        void get(std::uint8_t reg, std::uint8_t* pnBuffer, std::uint8_t nLen) const {
          twi_read(m_addr, reg, pnBuffer, nLen);
        }

};


//////////////////////////////////////////////////////////

//BEGIN_MCL_DEV_NAMESPACE

//! https://datasheets.maximintegrated.com/en/ds/MAX30101.pdf
class DevEx max30101 {
public:
    typedef max30101 this_type;
    typedef std::uint8_t value_type;
    typedef std::uint8_t register_type;

	struct ireg0 {
    	MCL_REGISTER_COMMON(ireg0, max30101::value_type);
		MCL_REGISTER_BITFIELDS_COMMON {
			std::uint8_t ready:1;
			std::uint8_t dummy:4;
			std::uint8_t overflow:1; // ambient light cancellation overflow
			std::uint8_t data:1; // new data ready
			std::uint8_t full:1; // fifo almost full -- see fifo config for "almost"
	MCL_REGISTER_BITFIELDS_END;

    struct multi {
		MCL_REGISTER_COMMON(multi, max30101::value_type);
		MCL_REGISTER_BITFIELDS_COMMON {
			value_type slot0:3;
			value_type dummy:1;
			value_type slot1:3;
	MCL_REGISTER_BITFIELDS_END;

	//////////////
	// Registers

	MCL_REGISTER_BASE(IntVal0, max30101::value_type, 0, ireg0) intval0;
	MCL_REGISTER(IntVal1, max30101::value_type, 1) intval1;
	MCL_REGISTER_BASE(IntEn0, max30101::value_type, 2, ireg0) inten0;
	MCL_REGISTER(IntEn1, max30101::value_type, 3) inten1;

	MCL_REGISTER(WritePtr, max30101::value_type, 4) writePtr;
	MCL_REGISTER(OverflowCtr, max30101::value_type, 5) overflowCtr;
	MCL_REGISTER(ReadPtr, max30101::value_type, 6) readPtr;
	MCL_REGISTER(Data, max30101::value_type, 7) data;

	MCL_REGISTER_BITFIELDS(CfgFIFO, max30101::value_type, 8) {
		value_type full:4;
		value_type roll:1;
		value_type avg:3;
	MCL_REGISTER_BITFIELDS_END cfgfifo;

	MCL_REGISTER_BITFIELDS(CfgMode, max30101::value_type, 9) {
		value_type mode:3;
		value_type dummy:3;
		value_type reset:1;
		value_type shutdown:1;
	MCL_REGISTER_BITFIELDS_DONE
		enum { HR = 2, SPO2 = 3, MULTI = 7 }; // the modes
		this_type& HeartRate() { mode = HR; return *this; }
		this_type& SpO2() { mode = SPO2; return *this; }
		this_type& Multi() { mode = MULTI; return *this; }
	MCL_REGISTER_BITFIELDS_CLOSE cfgmode;

	MCL_REGISTER_BITFIELDS(CfgSpO2, max30101::value_type, 10) {
		value_type pw:2; // 15-18 bits
		value_type rate:3;
		value_type range:2; // 2048 nA to 16384 nA
	MCL_REGISTER_BITFIELDS_DONE
		std::uint32_t SampleRate() const { return 50 * (rate + 1); }
		this_type& SampleRate(std::uint32_t nSamplesPerSecond) {
			rate = nSamplesPerSecond / 50;
			if(nSamplesPerSecond && (nSamplesPerSecond % 50) == 0) --rate;
			return *this;
		}
		std::uint8_t Bits() const { return pw + 15; }
		this_type& Bits(std::uint8_t nBits /* 15 to 18 */) {
			if(nBits > 18) pw = 3;
			else pw = (nBits < 15) ? 0 : (nBits - 15);
			return *this;
		}
	MCL_REGISTER_BITFIELDS_CLOSE cfgspo2;

	value_type dummy0;

	MCL_REGISTER(Led1, max30101::value_type, 12) led1;
	MCL_REGISTER(Led2, max30101::value_type, 13) led2;
	MCL_REGISTER(Led3, max30101::value_type, 14) led3;
	MCL_REGISTER(Led4, max30101::value_type, 15) led4;
	MCL_REGISTER(Pilot, max30101::value_type, 16) pilot;

	MCL_REGISTER_BASE(Multi0, max30101::value_type, 0x11, multi) multi0;
	MCL_REGISTER_BASE(Multi1, max30101::value_type, 0x12, multi) multi1;

	value_type dummy1[0x1F - 0x12 - 1];

	MCL_REGISTER(TempWhole, std::int8_t, 0x1F) tint;
	MCL_REGISTER(TempFrac, max30101::value_type, 0x20) tfrac;
	MCL_REGISTER(TempEn, max30101::value_type, 0x21) ten;

	value_type dummy2[0xFE - 0x21 - 1];

	MCL_REGISTER(Revision, max30101::value_type, 0xFE) rev;
	MCL_REGISTER(Part, max30101::value_type, 0xFF) part; // 0x15
};



template <class R, class S = max30101> struct Max30101 : public Registers<R> {
	typedef Max30101<R,S> this_type;
	typedef Registers<R> regr_type;
	typedef S struct_type;
	typedef std::uint8_t value_type;
	typedef typename regr_type::reference reference;
	typedef typename regr_type::const_reference const_reference;

	Max30101(const R& r = R()) : regr_type(r) {}

//	mcl::cmn::Fraction<std::int32_t> Temperature() {
//		reference r = at(S::TempEn::index);
//		// trigger a reading
//		r = 1;
//		// wait for it to finish
//		while(r);
//		std::int16_t w = at(S::TempWhole::index);
//		return mcl::cmn::Fraction<std::int32_t>((w << 4) + at(S::TempFrac::index), 16);
//	}

	bool Data() const { return this->template at<S::IntVal0>().data; }
	bool Ready() const { return this->template at<S::IntVal0>().ready; }

	std::uint32_t Bits() const { return this->template at<typename S::CfgSpO2>().Bits(); }
	std::uint32_t Overflows() const { return this->at(S::OverflowCtr::index); }
	inline std::uint8_t Status() const { return this->at(S::IntVal0::index); }
	inline std::uint8_t ReadAt() const { return this->at(S::ReadPtr::index); }
	inline std::uint8_t WriteAt() const { return this->at(S::WritePtr::index); }
	inline std::uint8_t Avail() const {
		std::uint8_t r = ReadAt(), w = WriteAt();
		return (w >= r) ? w - r : 32 - (r - w);
	}

	std::uint32_t SampleRate() const { return this->template at<typename S::CfgSpO2>().SampleRate(); }

	void Reset() {
		this->at(S::WritePtr::index) = 0;
		this->at(S::OverflowCtr::index) = 0;
		this->at(S::ReadPtr::index) = 0;
	}

	bool Init() {
		if(this->at(S::Part::index) != 0x15) return false;

//		Set(S::IntEn0(0xC0)); // INTR setting -- New data and FIFO almost full
		this->at(S::IntEn0::index) = 0xC0; // INTR setting -- New data and FIFO almost full
		this->at(S::IntEn1::index) = 0;

		typename S::CfgFIFO cf;
		cf.full = 0xF, cf.roll = 0, cf.avg = 0;  // no averaging, fifo rollover = false, fifo almost full = 17 (0xF)
		this->Set(cf);

		this->Set(typename S::CfgMode(S::CfgMode::SPO2)); // SpO2 mode

		typename S::CfgSpO2 cfo;
		cfo.SampleRate(100).Bits(18);
		cfo.range = 1; // range = 4096nA
		this->Set(cfo);

		this->at(S::Led1::index) = 0x24;
		this->at(S::Led2::index) = 0x24;
		this->at(S::Pilot::index) = 0x7F;

		Reset();
		return true;
	}

};

struct SpO2 : public Max30101<TwoWireRegister> {
	typedef Max30101<TwoWireRegister> base_type;

//	SpO2(mcl::io::TwoWire8* p) : base_type(TwoWireRegister(p, 0xAE)) {}
	SpO2(const nrf_drv_twi_t* p) : base_type(TwoWireRegister(p, 0xAE >> 1)) {}

	void Read(std::uint32_t *pnRed, std::uint32_t* pnIR) const {
		std::uint8_t buffer[6];
                this->get(struct_type::Data::index, buffer, 6);
		// we have to burst read 6 bytes to get all the data
//		m_p->Addr(m_addr);
//		if(6 == m_p->Query(struct_type::Data::index, buffer, 6)) {
			*pnRed = (buffer[0] << 16) | (buffer[1] << 8) | buffer[2];
			*pnIR = (buffer[3] << 16) | (buffer[4] << 8) | buffer[5];
//		} else *pnRed = *pnIR = 0xFFFFFFFF;
	}

	inline std::pair<std::uint32_t,std::uint32_t> Read(std::pair<std::uint32_t,std::uint32_t>& ret) const {
		Read(&ret.first, &ret.second);
		return ret;
	}

	inline std::pair<std::uint32_t,std::uint32_t> Read() const {
		std::pair<std::uint32_t,std::uint32_t> ret;
		return Read(ret);
	}

	std::uint8_t Avail() const {
		// we can burst read (write, overflow, read)
		std::uint8_t buffer[3];
                this->get(struct_type::WritePtr::index, buffer, 3);
		// we have to burst read 6 bytes to get all the data
//		m_p->Addr(m_addr);
//		if(3 == m_p->Query(struct_type::WritePtr::index, buffer, 3)) {
			const std::uint8_t& w = buffer[0];
			const std::uint8_t& r = buffer[2];
			if(buffer[1] == 31) return 1; // we've saturated the overflows
			return (w >= r) ? w - r : 32 - (r - w);
//		}
		return 0;
	}
protected:
};

#ifdef __cplusplus
extern "C" {
#endif
void DoHR(const nrf_drv_twi_t* pTWI, xSemaphoreHandle, ble_hrs_t* pHRS
//uint32_t twi_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);
//uint32_t twi_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
);
#ifdef __cplusplus
};
#endif

//END_MCL_DEV_NAMESPACE

#endif // _MAX_30101_H_DEV_MCL_
