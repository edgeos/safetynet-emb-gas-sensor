#include "aducm355_gas_constants.h"


//GAS1 = 1 << 0,            
//GAS2 = 1 << 1,
//GAS3 = 1 << 2,
//GAS4 = 1 << 3

SensorSettings MeasureSettings[NUM_GAS_SENSORS*NUM_FREQS_PER_SENSOR] = 
{
   // {SENSOR#, FREQ [Hz], NUM_TO_AVG, A0, A1, A2, A3, A4, A5, HEATER_ALWAYS_ON, MUX_ENABLE, MUX_S0, MUX_S1}
   {1, 10000.0, 7, 1.0, -2.0, 3.0, -4.0, false, false, false, false}, 
   {1, 15000.0, 7, 1.0, -2.0, 3.0, -4.0, false, false, false, false}, 
   {1, 20000.0, 7, 1.0, -2.0, 3.0, -4.0, false, false, false, false},
   {1, 25000.0, 7, 1.0, -2.0, 3.0, -4.0, false, false, false, false},
   {2, 30000.0, 7, 1.0, -2.0, 3.0, -4.0, false, false, false, false},
   {2, 35000.0, 7, 1.0, -2.0, 3.0, -4.0, false, false, false, false},
   {2, 40000.0, 7, 1.0, -2.0, 3.0, -4.0, false, false, false, false},
   {2, 45000.0, 7, 1.0, -2.0, 3.0, -4.0, false, false, false, false},
   {3, 50000.0, 7, 1.0, -2.0, 3.0, -4.0, false, false, false, false},
   {3, 55000.0, 7, 1.0, -2.0, 3.0, -4.0, false, false, false, false},
   {3, 60000.0, 7, 1.0, -2.0, 3.0, -4.0, false, false, false, false},
   {3, 65000.0, 7, 1.0, -2.0, 3.0, -4.0, false, false, false, false},
   {4, 70000.0, 7, 1.0, -2.0, 3.0, -4.0, false, false, false, false},
   {4, 75000.0, 7, 1.0, -2.0, 3.0, -4.0, false, false, false, false},
   {4, 80000.0, 7, 1.0, -2.0, 3.0, -4.0, false, false, false, false},
   {4, 85000.0, 7, 1.0, -2.0, 3.0, -4.0, false, false, false, false}
};