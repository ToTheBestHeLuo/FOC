#ifndef _MC_SENSOR_H_
#define _MC_SENSOR_H_

#include "mcVar.h"

extern f32_t IncAbzCalculateRealEleAngle(_FORCE IncABZEncoder* pABZ);
extern f32_t IncAbzCalculateRealEleSpeed(_FORCE IncABZEncoder* pABZ,f32_t targetEleSpeed);

extern f32_t AbsEncoderCalculateRealEleAngle(_FORCE AbsEncoderHandler* pAbs);
extern f32_t AbsEndoerCalculateRealEleSpeed(_FORCE AbsEncoderHandler* pAbs);

#endif


