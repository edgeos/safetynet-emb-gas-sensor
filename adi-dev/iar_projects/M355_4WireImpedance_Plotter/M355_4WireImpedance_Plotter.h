/**
 *****************************************************************************
   @addtogroup EC sensor
   @{
   @file     M355_4WireImpedance.h
   @brief    4 Wire Impedance measurement
   @version  V0.1
   @author   ADI
   @date     April 28th 2017
   @par Revision History:
   - V0.1, Sept 19th 2017: initial version.


All files provided by ADI, including this file, are
provided  as is without warranty of any kind, either expressed or implied.
The user assumes any and all risk from the use of this code.
It is the responsibility of the person integrating this code into an application
to ensure that the resulting application performs as required and is safe.

**/

#ifndef M355_4WIRE_IMPEDANCE_H
#define M355_4WIRE_IMPEDANCE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ADuCM355_Peri.h"
#include <stdio.h>  
  
/*
   uncomment this to use CMSIS_DSP library to calculate Impedance
*/
#include <math.h>
#define PI  3.14159265f

/*AC test Macros*/
/*
   RCAL, external calibration resistor
   will be used in impedance calculation
*/
#define AFE_RCAL  200u                                 // Assume 200 Ohms RCAL
//#define AFE_RCAL  160000u                                 // Assume 160k Ohms RCAL
/*
   High power DAC update rate
   controlled by HSDACCON[8:1], rate = 16MHz/HSDACCON[8:1]
   these bits need to be configured depending on the AC excitaion signal frequency.
   higher frequency excitation signal requires higher update rate, results in better performance but higher power consumption
*/

/*
   Excitaion sine wave amplitude, this value could be applied on calibration resistor or unknown sensor
   Controlled by WGAMPLITUDE[10:0], HSDACCON[0] and HSDACCON[12]
   ----------------------------------------------------------------------------------
     Gain   |  DAC attenuator(HSDACCON[0])  |  Excitaion Amplifier Gain(HSDACCON[12])   |
   ---------|-----------------------------|-----------------------------------------|
       2    |           1(0)              |                  2(0)                   |
   ---------|-----------------------------|-----------------------------------------|
      1/5   |           1(0)              |                  1/5(1)                 |
   ---------|-----------------------------|-----------------------------------------|
      1/4   |           1/4(1)            |                  2(0)                   |
   ---------|-----------------------------|-----------------------------------------|
      1/20  |           1/4(1)            |                  1/5(1)                 |
   ----------------------------------------------------------------------------------
   AC amplitude = WGAMPLITUDE[10:0]*HPDAC_LSB*Gain
   HPDAC_LSB = 800mV/(2^12-1)
*/
#define SINE_AMPLITUDE   15u //15mV
#define SINE_AMPLITUDE_REG (uint16_t)(SINE_AMPLITUDE*20/HPDAC_LSB+0.5)  //assuming Gain = 1/20
/*
   Excitation sine wave phase and DC offset
*/
#define SINE_PHASE 0
/*
   Excitation sine wave phase and DC offset
*/
#define SINE_OFFSET 0
#define SINE_OFFSET_REG (uint16_t)(SINE_OFFSET/HPDAC_LSB+0.5)


/*=========================== Function declarations ====================*/
void UartRxParser();
void SendMeasurement();
void ConfigImpMeasurement();
void StartMeasurement();
void ImpInit();
uint8_t ImpSigChainCfg(float freq);
void ImpMeasurement();
void ImpCalculation();
uint8_t TxUartBuffer(unsigned char length, bool waitForAck);

#ifdef __cplusplus
}
#endif

#endif // #define M355_ECSNS_EIS
