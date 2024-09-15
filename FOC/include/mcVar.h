/*
 * @Author: ToTheBestHeLuo 2950083986@qq.com
 * @Date: 2024-07-04 09:16:17
 * @LastEditors: ToTheBestHeLuo 2950083986@qq.com
 * @LastEditTime: 2024-08-09 09:17:53
 * @FilePath: \MDK-ARMd:\stm32cube\stm32g431rbt6_mc_ABZ\FOC\include\mcVar.h
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */

#ifndef _MC_VAR_H_
#define _MC_VAR_H_

#include "mcType.h"

extern _FORCE MCSysHandler mcSystemHandler;
extern _FORCE SvpwmHandler svpwmHandler;
extern _FORCE MC_MotorPar motorParHandler;
extern _FORCE IncABZEncoder incABZHandler;
extern _FORCE HFPIHandler hfpiHandler;
extern _FORCE HFSIHandler hfsiHandler;
extern _FORCE SensorHandler sensorHandler;
extern _FORCE PIC currentIdPICHandler;
extern _FORCE PIC currentIqPICHandler;
extern _FORCE PIC speedPICHandler;
extern _FORCE NSIdentifyProcessHandler NSIdendityHandler;
extern _FORCE MC_ParameterIdentify_Handler MCParameterIdentifyHandler;
extern _FORCE NSCheckHandler NSHandler;
extern _FORCE NonlinearFluxObsHandler NonlinearFluxHandler;
extern _FORCE OpenLoop_IF_Handler IFHandler;
extern _FORCE LuenbergerObsHandler luenbergerObsHandler;
extern _FORCE AbsEncoderHandler absEncoderHandler;

extern _FORCE MCSysHandler* pSys;
extern _FORCE SvpwmHandler* pSVP;
extern _FORCE MC_MotorPar* pMotor;
extern _FORCE IncABZEncoder* pIncABZ;
extern _FORCE SensorHandler* pSens;
extern _FORCE HFPIHandler* pHFPI;
extern _FORCE HFSIHandler* pHFSI;
extern _FORCE PIC* pIdPIC;
extern _FORCE PIC* pIqPIC;
extern _FORCE PIC* pSpPIC;
extern _FORCE NSIdentifyProcessHandler* pNSIdentify;
extern _FORCE MC_ParameterIdentify_Handler* pParmeterIndentify;
extern _FORCE NSCheckHandler* pNS;
extern _FORCE NonlinearFluxObsHandler* pNonlinearFlux;
extern _FORCE OpenLoop_IF_Handler* pIF;
extern _FORCE LuenbergerObsHandler* pLuenberger;
extern _FORCE AbsEncoderHandler* pAbs;

extern void reset_All(void);
extern void reset_MCSysHandler(void);
extern void reset_SvpwmHandler(void);
extern void reset_MotorParHandler(void);
extern void reset_IncABZHandler(void);
extern void reset_SensorHandler(void);
extern void reset_HFPIHandler(void);
extern void reset_HFSIHandler(void);
extern void reset_CurrentPICHandler(void);
extern void reset_SpeedPICHandler(void);
extern void reset_NSIdentifyHandler(void);
extern void reset_ParmeterHandler(void);
extern void reset_NSCheckHandler(void);
extern void reset_NonlinearFluxObsHandler(void);
extern void reset_IFHandler(void);
extern void reset_Luenberger(void);
extern void reset_AbsEncoderHandler(void);
#endif

