#ifndef ADUCM355_GAS_CONSTANTS_H
#define ADUCM355_GAS_CONSTANTS_H

#include <stdint.h>
#include <stdbool.h>

#ifdef BOARD_MATCHBOX_V1
#define PRIMETIME_SEC       1
#else
#define PRIMETIME_SEC       1
#endif


#define NUM_SENSOR_READINGS	40
//#define NUM_SENSOR_READINGS	16

#if 0
#define NUM_GAS_SENSORS      4
#define NUM_FREQS_PER_SENSOR 4
#endif

typedef struct
{
   uint8_t sensor_num;
   float freq;
   uint16_t num_avg;
   float a0;
   float a1;
   float a2;
   float a3;
   float a4;
   bool heater_always_on;
   bool mux_enable;
   bool mux_s0;
   bool mux_s1;
} SensorSettings;
   
//extern SensorSettings MeasureSettings[NUM_GAS_SENSORS*NUM_FREQS_PER_SENSOR];
extern SensorSettings MeasureSettings[NUM_SENSOR_READINGS];
#endif