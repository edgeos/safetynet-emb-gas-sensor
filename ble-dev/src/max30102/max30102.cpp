#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "nrf_log.h"

#include "max30101.h"
#include "CircularBuffer.h"

std::uint32_t nHR;

//! buffer for our last N readings
constexpr const std::uint32_t nReadingSize = 4;
std::uint32_t nReadingBuffer[nReadingSize << 1];
typedef mcl::MovingAverage<std::uint32_t,std::uint16_t> read_ma_type;

//! buffer for the gradient of our last N moving averages
constexpr const std::uint32_t nGradientSize = 2;
std::int32_t nGradientBuffer[nGradientSize << 1];
typedef mcl::MovingAverage<std::int32_t,std::uint16_t> grad_ma_type;

//! buffer for our window
//! this is the moving average of the gradient that we will apply our hamming value to
constexpr const std::uint32_t nWindowSize = 5;
std::int32_t nWindowBuffer[nWindowSize << 1];
typedef mcl::CircularBuffer<std::int32_t,std::uint16_t> window_type;

//! our final processed data output buffer (post window)
constexpr const std::uint32_t nOutputSize = 512;
std::int32_t nOutputBuffer[nOutputSize << 1];
typedef mcl::CircularBuffer<std::int32_t,std::uint16_t> output_type;

void DoHR(const nrf_drv_twi_t* pTWI, xSemaphoreHandle hMutex, ble_hrs_t* pHRS) {
//	mcl::em::ti::cc::TwoWire* pTWI = reinterpret_cast<mcl::em::ti::cc::TwoWire*>(p);
    if(pdFALSE == xSemaphoreTake(hMutex, portMAX_DELAY)) vTaskSuspend(NULL);
	SpO2 spo2(pTWI);
	if(!spo2.Init()) {
	  NRF_LOG_INFO("Did not find Max3010x on I2C bus");
	  xSemaphoreGive(hMutex);
	  vTaskSuspend(NULL);
	}
	std::uint32_t nPeriodMS = 500 / spo2.SampleRate();
	// the reading number of the minimum before the current one
	std::uint32_t nPrevMinimumIndex, nCurrentMimumIndex, nDiff, nSamplesPerMinute = spo2.SampleRate() * 60, nBelow = 0, nAbove = 0;
        xSemaphoreGive(hMutex);

	//! our current reading
	std::pair<std::uint32_t, std::uint32_t> xReading;

	//! moving avarage of the last N readings
	read_ma_type red(nReadingBuffer, nReadingSize), ir(nReadingBuffer + nReadingSize, nReadingSize);

	//! current and previous moving average sums (using int32_t so we get a signed value on subtraction)
	std::int32_t nMARed = 0, nMAIR = 0, nMARedPrev, nMAIRPrev;

	//! moving average of the gradient
	grad_ma_type Dred(nGradientBuffer, nGradientSize), Dir(nGradientBuffer + nGradientSize, nGradientSize);

	//! current and previous moving average delta
	window_type winred(nWindowBuffer, nWindowSize), winir(nWindowBuffer + nWindowSize, nWindowSize);

	//! processed values
	output_type cbRed(nOutputBuffer, nOutputSize), cbIR(nOutputBuffer + nOutputSize, nOutputSize);

	constexpr const std::int32_t nHamWin[] = { 82, 553, 1024, 553, 82 };

	std::uint16_t nHRbuf[5] = { 60, 60, 60, 60, 60 };
	mcl::MovingAverage<std::uint16_t,std::uint16_t> hr(nHRbuf, 5);

	//////////
	// minimum finding
//	constexpr const std::int32_t nThresh = -250000;
//	constexpr const std::int32_t nThresh = -400000;
	std::int32_t nMax = -400000;
	std::int32_t nThresh = nMax * 7 / 8;

	std::int32_t nMinimum;

	std::int32_t nLast, nG;
	std::uint8_t nA;
	std::uint32_t nReading = 0;


	std::uint32_t nNothing = 0, nGood = 0;

//	mcl::em::ti::cc::Time t, tLast;
	if(nPeriodMS > 100 || nPeriodMS < 1) nPeriodMS = 10;
//	t.Print();

//	mcl::os::Print(" Beginning SpO2 Heart Rate Monitoring... %ums\r\n", nPeriodMS);
        NRF_LOG_INFO("Beginning SpO2 Heart Rate Monitoring... %ums", nPeriodMS);
	spo2.Reset();
	while(true) {
        vTaskDelay(nPeriodMS);
          nA = 0;
          if(xSemaphoreTake(hMutex,1)) {
            nA = spo2.Avail();
            xSemaphoreGive(hMutex);
          }
		if(nA) {
                    if(!xSemaphoreTake(hMutex,1)) continue;
			spo2.Read(xReading);
                    xSemaphoreGive(hMutex);
			nNothing = 0;
			if((++nReading % 500) == 0) {
//				mcl::em::ti::cc::Time t2;
//				auto tdiff = t2.toMS() - tLast.toMS();
//				tLast = t2;
                          NRF_LOG_INFO("HR: IR %u, Last %d, Min %d", xReading.second, nLast, nMinimum);
			}

			if(xReading.second < 1000000) {
//				if(!t.SleepUntil(nPeriodMS)) t.Get();
				nGood = 0;
				continue;
			}
			++nGood;

			// remember our previous sums
//			nMARedPrev = nMARed;
			nMAIRPrev = nMAIR;

			// moving average, and gradient of our last N readings
//			winred.push(Dred.push((nMARed = red.push(xReading.first)) - nMARedPrev));
			winir.push(nG = Dir.push((nMAIR = ir.push(xReading.second)) - nMAIRPrev));

			// window our values
//			cbRed.push(winred[0] * nHamWin[0] + winred[1] * nHamWin[1] + winred[2] * nHamWin[2] + winred[3] * nHamWin[3] + winred[4] * nHamWin[4]);
//			cbIR.push(nLast = winir[0] * nHamWin[0] + winir[1] * nHamWin[1] + winir[2] * nHamWin[2] + winir[3] * nHamWin[3] + winir[4] * nHamWin[4]);
			nLast = winir[0] * nHamWin[0] + winir[1] * nHamWin[1] + winir[2] * nHamWin[2] + winir[3] * nHamWin[3] + winir[4] * nHamWin[4];

//			if(nGood > 50 && nLast < nMax) {
//				nThresh = (nMax = nLast) * 5 / 8;
//				mcl::os::Print("HR: Moving Threshold %d,%d\r\n", nThresh, nMax);
//			}

//			mcl::os::Print("%u,%u,%d\r\n", xReading.first, xReading.second, nLast);
			if(nLast < nThresh) {
				// this value is below the threshold
				if(++nBelow == 1 || nLast < nMinimum) {
					// if this is the start of a new search or this is a lower point, update our minimum
					nMinimum = nLast;
					nCurrentMimumIndex = nReading;
//					if(nBelow == 1) mcl::os::Print("below %u\r\n", nAbove);
				}
				nAbove = 0;
			} else {
				// this value is above the threshold
				if(++nAbove == 5 && nBelow > 3) {
					// but the previous reading was below it, we just finished finding a minimum
					// update the HR
					nDiff = nSamplesPerMinute / (nCurrentMimumIndex - nPrevMinimumIndex);
					if(nDiff > 20 && nDiff < 250) {
						hr.push(nDiff);
						nHR = hr.Avg();
                                                ret_code_t err_code = ble_hrs_heart_rate_measurement_send(pHRS, nHR);
                                                if(NRF_SUCCESS != err_code)
                                                  NRF_LOG_INFO("HR: notifiy error %d", err_code);
//						nHR = nDiff;
            NRF_LOG_INFO("HR: %u", nHR);
//						mcl::os::Print(",%u", nHR);
					}
//					mcl::os::Print("%u,%u\r\n", nDiff, nHR);
					nPrevMinimumIndex = nCurrentMimumIndex;
					nBelow = 0;
				}
//				if(nAbove == 1)
//					mcl::os::Print("above %d,%u\r\n", nMinimum,nBelow);
			}
//			mcl::os::Sleep(1);
//			mcl::os::Print("\r\n");
		} else {
			nGood = 0;
			if((++nNothing % 500) == 0)
                          NRF_LOG_INFO("HR: not responding, status 0x%x, write %u, read %u, over %u", spo2.Status(), spo2.WriteAt(), spo2.ReadAt(), spo2.Overflows());
//				mcl::os::Print("HR: not responding, status 0x%x, write %u, read %u, over %u\r\n", spo2.Status(), spo2.WriteAt(), spo2.ReadAt(), spo2.Overflows());
//			mcl::os::Sleep(8);
		}
	}
}

