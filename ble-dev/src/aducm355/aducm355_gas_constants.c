#include "aducm355_gas_constants.h"

void GetSetting(int n, int* pnSensor, int* pnFreq) {
	if(pnSensor) *pnSensor = MeasureSettings[n].sensor_num;
	if(pnFreq) *pnFreq = MeasureSettings[n].freq;
}

#ifdef NUM_SENSOR_READINGS

// {SENSOR#, FREQ [Hz], NUM_TO_AVG, A0, A1, A2, A3, A4, A5, HEATER_ALWAYS_ON, MUX_ENABLE, MUX_S0, MUX_S1}
SensorSettings MeasureSettings[NUM_SENSOR_READINGS] = {

#if NUM_SENSOR_READINGS == 6
//   {1, 50000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
//   {1,100000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
//   {1,150000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
//   {2, 50000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
//   {2,100000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
//   {2,150000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false}
   {1, 10000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {1, 20000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {1, 50000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2, 10000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2, 20000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2, 50000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false}
#elif NUM_SENSOR_READINGS == 40
	{1, 10000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false}, 
   {1, 20000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {1, 30000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {1, 40000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {1, 50000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {1, 60000.0, 1, -50.0f, 0.1f, 0.0, 0.0, 0.0, false, false, false, false},
   {1, 70000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {1, 80000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {1, 90000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {1,100000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {1,110000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false}, 
   {1,120000.0, 1, -50.0f, -1.0, 0.02f, -0.05f, 0.0000001f, false, false, false, false},
   {1,130000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {1,140000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {1,150000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {1,160000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {1,170000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {1,180000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {1,190000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {1,200000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},

   {2, 10000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false}, 
   {2, 20000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2, 30000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2, 40000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2, 50000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2, 60000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2, 70000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2, 80000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2, 90000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2,100000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2,110000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false}, 
   {2,120000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2,130000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2,140000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2,150000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2,160000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2,170000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2,180000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2,190000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2,200000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false}
#else NUM_SENSOR_READINGS == 16
   {1, 10000.0, 7, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false}, 
   {1, 15000.0, 7, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false}, 
   {1, 20000.0, 7, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {1, 25000.0, 7, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2, 30000.0, 7, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2, 35000.0, 7, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2, 40000.0, 7, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2, 45000.0, 7, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {3, 50000.0, 7, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {3, 55000.0, 7, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {3, 60000.0, 7, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {3, 65000.0, 7, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {4, 70000.0, 7, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {4, 75000.0, 7, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {4, 80000.0, 7, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {4, 85000.0, 7, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false}
#endif // NUM_SENSOR_READINGS == value
};
#endif // NUM_SENSOR_READINGS




#if 0

//GAS1 = 1 << 0,            
//GAS2 = 1 << 1,
//GAS3 = 1 << 2,
//GAS4 = 1 << 3

// PPM = gas_ppm = a0 + a1*z_re + a2*powf(z_re,2) + a3*z_im + a4*powf(z_im,2);
/*
SensorSettings MeasureSettings[NUM_SENSOR_READINGS] = //NUM_GAS_SENSORS*NUM_FREQS_PER_SENSOR] = 
{
   // {SENSOR#, FREQ [Hz], NUM_TO_AVG, A0, A1, A2, A3, A4, A5, HEATER_ALWAYS_ON, MUX_ENABLE, MUX_S0, MUX_S1}
   {1, 10000.0, 7, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false}, 
   {1, 15000.0, 7, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false}, 
   {1, 20000.0, 7, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {1, 25000.0, 7, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2, 30000.0, 7, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2, 35000.0, 7, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2, 40000.0, 7, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2, 45000.0, 7, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {3, 50000.0, 7, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {3, 55000.0, 7, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {3, 60000.0, 7, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {3, 65000.0, 7, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {4, 70000.0, 7, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {4, 75000.0, 7, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {4, 80000.0, 7, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {4, 85000.0, 7, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false}
};
/**/

/**/
SensorSettings MeasureSettings[NUM_SENSOR_READINGS] = {
   // {SENSOR#, FREQ [Hz], NUM_TO_AVG, A0, A1, A2, A3, A4, A5, HEATER_ALWAYS_ON, MUX_ENABLE, MUX_S0, MUX_S1}
   {1, 10000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false}, 
   {1, 20000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {1, 30000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {1, 40000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {1, 50000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {1, 60000.0, 1, -50.0f, 0.1f, 0.0, 0.0, 0.0, false, false, false, false},
   {1, 70000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {1, 80000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {1, 90000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {1,100000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {1,110000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false}, 
   {1,120000.0, 1, -50.0f, -1.0, 0.02f, -0.05f, 0.0000001f, false, false, false, false},
   {1,130000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {1,140000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {1,150000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {1,160000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {1,170000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {1,180000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {1,190000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {1,200000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},

   {2, 10000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false}, 
   {2, 20000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2, 30000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2, 40000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2, 50000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2, 60000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2, 70000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2, 80000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2, 90000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2,100000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2,110000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false}, 
   {2,120000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2,130000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2,140000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2,150000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2,160000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2,170000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2,180000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2,190000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2,200000.0, 1, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false}
};/**/
/*
SensorSettings MeasureSettings[NUM_SENSOR_READINGS] = {
   // {SENSOR#, FREQ [Hz], NUM_TO_AVG, A0, A1, A2, A3, A4, A5, HEATER_ALWAYS_ON, MUX_ENABLE, MUX_S0, MUX_S1}
   {1, 50000.0, 5, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false}, 
   {1,100000.0, 5, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {1,150000.0, 5, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {1,200000.0, 5, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2, 50000.0, 5, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false}, 
   {2,100000.0, 5, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2,150000.0, 5, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {2,200000.0, 5, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {3, 50000.0, 5, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false}, 
   {3,100000.0, 5, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {3,150000.0, 5, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {3,200000.0, 5, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {4, 50000.0, 5, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false}, 
   {4,100000.0, 5, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {4,150000.0, 5, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false},
   {4,200000.0, 5, 1.0, -2.0, 3.0, -4.0, 5.0, false, false, false, false}
};
*/

#endif

