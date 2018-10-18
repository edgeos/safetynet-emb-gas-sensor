/*
This file is auto generated
Array for Impedance measurement result with frequency
*/
#ifndef IMP_RESULT_ARRAY_H
#define IMP_RESULT_ARRAY_H

#include "stdint.h"
typedef struct
{
   float freq;
   int32_t DFT_result[4];
   float   Mag;
   float   Phase;
}ImpResult_t;
   

extern ImpResult_t ImpResult[100];
#endif
