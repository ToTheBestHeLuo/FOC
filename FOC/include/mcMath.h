/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-11-12 13:28:22
 * @LastEditors: ToTheBestHeLuo 2950083986@qq.com
 * @LastEditTime: 2024-07-28 10:06:21
 * @FilePath: \MDK-ARMd:\stm32cube\stm32g431rbt6_mc_ABZ\FOC\include\mcMath.h
 * @Description: 
 * 
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef _MC_MATH_H_
#define _MC_MATH_H_

#include "mcType.h"

#define MATH_PI 3.141592653589793f
#define abs(x) ((x) > 0.f) ? (x) : -(x)

extern Components2 Abc_AlphaBeta_Trans(_FORCE Components2* ab);
extern Components3 AlphaBeta_Abc_Trans(_FORCE Components2* alphaBeta);
extern Components2 AlphaBeta_Dq_Trans(_FORCE Components2* alphaBeta,_FORCE Components2* sinCos);
extern Components2 Dq_AlphaBeta_Trans(_FORCE Components2* dq,_FORCE Components2* sinCos);

extern bool CalculateIsInLimited(f32_t in,f32_t limit);
extern f32_t CalculateSquare(f32_t in);
extern Components2 CalculateSinCosValue(f32_t eleAngle);
extern f32_t FastSquareRoot(f32_t x);
extern f32_t FastReciprocalSquareRoot(f32_t x);
extern f32_t Min3(f32_t x1,f32_t x2,f32_t x3);
extern f32_t Max3(f32_t x1,f32_t x2,f32_t x3);

extern f32_t MedianFilter(f32_t datBuffer[],uint16_t length);
extern f32_t atan2Rad(f32_t x1,f32_t x2);

#endif

