/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-11-14 10:55:42
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-09-15 16:46:36
 * @FilePath: \MDK-ARMd:\stm32cube\stm32g431rbt6_advanced\FOC\source\mcVar.c
 * @Description: 
 * 
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
 */
#include "../include/mcVar.h"
#include "../interface/mcConfig.h"
#include "../include/mcMath.h"

_FORCE MCSysHandler mcSystemHandler = {
    .sysStu = eWaitSysReset
};
_FORCE SvpwmHandler svpwmHandler;
_FORCE MC_MotorPar motorParHandler;
_FORCE IncABZEncoder incABZHandler;
_FORCE SensorHandler sensorHandler;
_FORCE HFPIHandler hfpiHandler;
_FORCE HFSIHandler hfsiHandler;
_FORCE PIC currentIdPICHandler;
_FORCE PIC currentIqPICHandler;
_FORCE PIC speedPICHandler;
_FORCE NSIdentifyProcessHandler NSIdendityHandler;
_FORCE MC_ParameterIdentify_Handler MCParameterIdentifyHandler;
_FORCE NSCheckHandler NSHandler;
_FORCE NonlinearFluxObsHandler NonlinearFluxHandler;
_FORCE OpenLoop_IF_Handler IFHandler;
_FORCE LuenbergerObsHandler luenbergerObsHandler;
_FORCE AbsEncoderHandler absEncoderHandler;

_FORCE MCSysHandler* pSys = &mcSystemHandler;
_FORCE SvpwmHandler* pSVP = &svpwmHandler;
_FORCE MC_MotorPar* pMotor = &motorParHandler;
_FORCE IncABZEncoder* pIncABZ = &incABZHandler;
_FORCE SensorHandler* pSens = &sensorHandler;
_FORCE HFPIHandler* pHFPI = &hfpiHandler;
_FORCE HFSIHandler* pHFSI = &hfsiHandler;
_FORCE PIC* pIdPIC = &currentIdPICHandler;
_FORCE PIC* pIqPIC = &currentIqPICHandler;
_FORCE PIC* pSpPIC = &speedPICHandler;
_FORCE NSIdentifyProcessHandler* pNSIdentify = &NSIdendityHandler;
_FORCE MC_ParameterIdentify_Handler* pParmeterIndentify = &MCParameterIdentifyHandler;
_FORCE NSCheckHandler* pNS = &NSHandler;
_FORCE NonlinearFluxObsHandler* pNonlinearFlux = &NonlinearFluxHandler;
_FORCE OpenLoop_IF_Handler* pIF = &IFHandler;
_FORCE LuenbergerObsHandler* pLuenberger = &luenbergerObsHandler;
_FORCE AbsEncoderHandler* pAbs = &absEncoderHandler;

void reset_All(void)
{
    reset_MCSysHandler();
    reset_SvpwmHandler();
    reset_MotorParHandler();
    reset_SensorHandler();
    reset_HFPIHandler();
    reset_HFSIHandler();
    
    reset_CurrentPICHandler();
    reset_SpeedPICHandler();

    reset_NSIdentifyHandler();
    reset_ParmeterHandler();
    reset_NSCheckHandler();
    reset_NonlinearFluxObsHandler();
    reset_IncABZHandler();

    reset_IFHandler();
    reset_Luenberger();
    reset_AbsEncoderHandler();
}
void reset_MCSysHandler(void)
{
    mcSystemHandler.isPhaseCurrenfOffsetFinished = false;
    mcSystemHandler.sysError = eOKFlag;
    mcSystemHandler.sysStu = eWaitSysReset;
    mcSystemHandler.sysRunTime = 0u;
    mcSystemHandler.safeTaskTimeCnt = 0u;
    mcSystemHandler.focTaskTimeCnt = 0u;
    mcSystemHandler.controlMethod = eMethod_IF_Luenberger_Debug;
    mcSystemHandler.focStep = eFOC_Step_1;
    mcSystemHandler.lowSpeedClock = 0.001f;
    mcSystemHandler.highSpeedClock = 0.0001f;
    mcSystemHandler.pulseSpeedClock = 1.f / 170000000.f;
}

void reset_SvpwmHandler(void)
{
    svpwmHandler.svpFrequency = PerformanceCriticalTask_Timer_Frequency;
    svpwmHandler.timerARR = Timer_Period_ARR;
    svpwmHandler.motorVoltage = MC_SafeVoltage;
    svpwmHandler.limitVoltage = MC_SafeVoltage / 1.732050807568877f;
    svpwmHandler.volAlphaBeta.com1 = 0.f;svpwmHandler.volAlphaBeta.com2 = 0.f;
    svpwmHandler.volDQ.com1 = 0.f;svpwmHandler.volDQ.com2 = 0.f;
    svpwmHandler.ccr[0] = 0;svpwmHandler.ccr[1] = 0;svpwmHandler.ccr[2] = 0;
    svpwmHandler.sector = 0;
}

void reset_MotorParHandler(void)
{
    motorParHandler.Flux = Motor_Flux;
    motorParHandler.J = Motor_J;
    motorParHandler.Ld = Motor_Ld;
    motorParHandler.Lq = Motor_Lq;
    motorParHandler.Ls = Motor_Ls;
    motorParHandler.polePairs = Motor_PolePairs;
    motorParHandler.Rs = Motor_Rs;
}

void reset_SensorHandler(void)
{
    sensorHandler.dqVoltage.com1 = sensorHandler.dqVoltage.com2 = 0.f;
    sensorHandler.terminalVoltage.com1 = sensorHandler.terminalVoltage.com2 = sensorHandler.terminalVoltage.com3 = 0.f;
    sensorHandler.phaseVoltage.com1 = sensorHandler.phaseVoltage.com2 = sensorHandler.phaseVoltage.com3 = 0.f;
    sensorHandler.sinCosVal.com1 = 0.f;sensorHandler.sinCosVal.com2 = 0.f;
    sensorHandler.busAndTemp.com1 = 0.f;sensorHandler.busAndTemp.com2 = 0.f;
    sensorHandler.currentOffset.com1 = 0.f;sensorHandler.currentOffset.com2 = 0.f;
    sensorHandler.currentAB.com1 = 0.f;sensorHandler.currentAB.com2 = 0.f;
    sensorHandler.currentDQ.com1 = sensorHandler.currentDQ.com2 = 0.f;
    sensorHandler.currentAlphaBeta.com1 = sensorHandler.currentAlphaBeta.com2 = 0.f;
}

void reset_HFPIHandler(void)
{
    hfpiHandler.injectVoltage = 0.2f;
    hfpiHandler.est_eleAngle = 0.f;hfpiHandler.est_eleSpeed = 0.f;
    hfpiHandler.inject_phase = 0.f;
    hfpiHandler.inject_phaseSinCos.com1 = 0.f;hfpiHandler.inject_phaseSinCos.com2 = 1.f;
    hfpiHandler.response_HF_iDQ.com1 = hfpiHandler.response_HF_iDQ.com2 = 0.f;
    hfpiHandler.response_iAlphaBeta.com1 = 0.f;hfpiHandler.response_iAlphaBeta.com2 = 0.f;
    hfpiHandler.response_iDQ.com1 = 0.f;hfpiHandler.response_iDQ.com2 = 0.f;
    hfpiHandler.est_err = 0.f;
    hfpiHandler.errGain = 1000.f;
    hfpiHandler.int1 = hfpiHandler.int2 = 0.f;
    hfpiHandler.maxId = hfpiHandler.minId = 0.f;
    hfpiHandler.injectFrequency = 400.f;
    hfpiHandler.PLL_Kp = 0.1f;
    hfpiHandler.PLL_Ki = 1.f;
}

void reset_HFSIHandler(void)
{
    hfsiHandler.est_eleAngle = 0.f;hfsiHandler.est_eleSpeed = 0.f;
    hfsiHandler.inject_voltage = 0.1f;
    hfsiHandler.inject_polarity = true;
    hfsiHandler.response_HF_iAlphaBeta.com1 = 0.f;hfsiHandler.response_HF_iAlphaBeta.com2 = 0.f;
    hfsiHandler.response_HF_iDQ.com1 = 0.f;hfsiHandler.response_HF_iDQ.com2 = 0.f;
    hfsiHandler.response_iAlphaBeta.com1 = 0.f;hfsiHandler.response_iAlphaBeta.com2 = 0.f;
    hfsiHandler.response_HF_iAlphaBetaPerUnit.com1 = 0.f;hfsiHandler.response_HF_iAlphaBetaPerUnit.com2 = 0.f;
    hfsiHandler.est_err = 0.f;
    hfsiHandler.int1 = 0.f;hfsiHandler.int2 = 0.f;
    hfsiHandler.response_iDQ.com1 = hfsiHandler.response_iDQ.com2 = 0.f;
    hfsiHandler.kP = 0.1f;
    hfsiHandler.kI = 1.f;
    hfsiHandler.iAlphaBetaLast.com1 = hfsiHandler.iAlphaBetaLast.com2 = 0.f;
    hfsiHandler.iDQLast.com1 = hfsiHandler.iDQLast.com2 = 0.f;
}

void reset_CurrentPICHandler(void)
{
    currentIdPICHandler.errInt = 0.f;
    currentIqPICHandler.errInt = 0.f;
}

void reset_SpeedPICHandler(void)
{
    speedPICHandler.errInt = 0.f;
    speedPICHandler.target = 0.f;
}

void reset_NSIdentifyHandler(void)
{
    NSIdendityHandler.inject_voltage = 4.0f;
    NSIdendityHandler.inject_amp = 4.0f;
    NSIdendityHandler.posGate = 0.03f;
    NSIdendityHandler.negGate = -0.03f;
    NSIdendityHandler.polarityCntGate = 5;
    NSIdendityHandler.isCompensateFinished = false;
    NSIdendityHandler.maxId = 0.f;
    NSIdendityHandler.minId = 0.f;
    NSIdendityHandler.nsCompensate = 0.f;
    NSIdendityHandler.polarityCnt = 0;
    NSIdendityHandler.responseId = 0.f;
    NSIdendityHandler.pulseWidth = 20;
    NSIdendityHandler.pulseWidthCnt = 0;
}

void reset_ParmeterHandler(void)
{
    MCParameterIdentifyHandler.injectSigAmp = 0.3f;
    MCParameterIdentifyHandler.injectFre = 200.f;
    MCParameterIdentifyHandler.mc_Ls = 0.f;
    MCParameterIdentifyHandler.mc_Rs = 0.f;
    MCParameterIdentifyHandler.demodulation_phaseCompensate = -0.058293997016611f * 7.f;
    MCParameterIdentifyHandler.demodulation_ampCompensate = 1.f;
}

void reset_NSCheckHandler(void)
{
    NSHandler.compensateAngle = 0.f;
    NSHandler.iDQ.com1 = NSHandler.iDQ.com2 = 0.f;
    NSHandler.injectPosVoltage = 3.6f;
    NSHandler.injectNegVoltage = -3.6f;
    NSHandler.injectZeroVoltage = 0.f;
    NSHandler.injectVoltage = 0.f;
    NSHandler.isFinished = false;
    NSHandler.maxId = NSHandler.minId = 0.f;
    NSHandler.pulseWidth = 20;
    NSHandler.pulseWidthCnt = 0;
    NSHandler.status = 0u;
    NSHandler.polarityCnt = 0;
} 

void reset_NonlinearFluxObsHandler(void)
{
    NonlinearFluxHandler.delay1 = NonlinearFluxHandler.delay2 = 0.f;
    NonlinearFluxHandler.integrator1 = NonlinearFluxHandler.integrator2 = 0.f;
    NonlinearFluxHandler.gamma = 160000.f;
    NonlinearFluxHandler.kP = 2000.f;
    NonlinearFluxHandler.kI = 5000.f;
    NonlinearFluxHandler.est_eleAngle = 0.f;
    NonlinearFluxHandler.est_eleSpeed = 0.f;
    NonlinearFluxHandler.integrator3 = 0.f;
    NonlinearFluxHandler.integrator4 = 0.f;
    NonlinearFluxHandler.cos = NonlinearFluxHandler.sin = 0.f;
}

void reset_IncABZHandler(void)
{
    incABZHandler.isABZEncoderAlignment = false;
    incABZHandler.isABZEncoderFinished = false;
    incABZHandler.isAlignedOK = false;
    incABZHandler.zIndexTimCnt = 0u;
    incABZHandler.lastEncoderCnt = 0u;

    incABZHandler.abzCounterMode = eABZ_X4;
    incABZHandler.encoderPPR_Uint = ABZ_PPR;
    incABZHandler.encoderPPR_XX_Uint = incABZHandler.encoderPPR_Uint * (uint32_t)incABZHandler.abzCounterMode;
    incABZHandler.eleSpeedCalculateFacotr = 2.f * MATH_PI * (f32_t)Motor_PolePairs / ((f32_t)incABZHandler.encoderPPR_XX_Uint * pSys->lowSpeedClock);
    incABZHandler.eleAngleCalculateFacotr = (f32_t)incABZHandler.encoderPPR_XX_Uint / (f32_t)Motor_PolePairs / 2.f;
    incABZHandler.realEleSpeed = 0.f;
    incABZHandler.lowEleSpeedThreshold = 2.f * MATH_PI * 20.f;
    incABZHandler.highEleSpeedThreshold = 2.f * MATH_PI * 30.f;

    if(incABZHandler.lowEleSpeedThreshold < 0.f) incABZHandler.lowEleSpeedThreshold = -incABZHandler.lowEleSpeedThreshold;
}

void reset_IFHandler(void)
{
    IFHandler.accEleSpeed = MATH_PI * 2.f * 400.f;
    IFHandler.accSpeedTime = 1000u;
    IFHandler.eleAngle = 0.f;
    IFHandler.eleSpeed = 0.f;
    IFHandler.iqRef = 2.f;
}

void reset_Luenberger(void)
{
    luenbergerObsHandler.est_bemfAlphaBeta.com1 = luenbergerObsHandler.est_bemfAlphaBeta.com2 = 0.f;
    luenbergerObsHandler.est_eleAngle = 0.f;
    luenbergerObsHandler.est_eleSpeed = 0.f;
    luenbergerObsHandler.est_iAlphaBeta.com1 = luenbergerObsHandler.est_iAlphaBeta.com2 = 0.f;
    luenbergerObsHandler.gain1 = Motor_Rs / Motor_Ld * mcSystemHandler.highSpeedClock * 2.5f;
    luenbergerObsHandler.gain2 = 60.f * mcSystemHandler.highSpeedClock;
    luenbergerObsHandler.integrator1 = luenbergerObsHandler.integrator2 = 0.f;
    luenbergerObsHandler.par1 = (1.f - Motor_Rs * mcSystemHandler.highSpeedClock / Motor_Ls - luenbergerObsHandler.gain1);
    luenbergerObsHandler.par2 = -mcSystemHandler.highSpeedClock / Motor_Ls;
    luenbergerObsHandler.pllKp = 7000.f;luenbergerObsHandler.pllKi = 6000.f;
}

void reset_AbsEncoderHandler(void)
{
    pAbs->absOffsetFromRealEleAngle = -3.2390424;
    pAbs->encoderOutput = 0u;
    pAbs->realEleAngle = 0.f;
}

