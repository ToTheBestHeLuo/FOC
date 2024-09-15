/*
 * @Author: ToTheBestHeLuo 2950083986@qq.com
 * @Date: 2024-07-04 09:16:17
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-09-15 17:03:42
 * @FilePath: \MDK-ARMd:\stm32cube\stm32g431rbt6_advanced\FOC\interface\mcConfig.c
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
#include "mcConfig.h"
#include "main.h"
#include "arm_math.h"
#include "mcMath.h"
#include "usbd_cdc_if.h"

const uint8_t frameHeader[] = {'T','T',':'};
const uint8_t frameEnd[] = {'E','T','!'};

FrameForRTT frameForRTT;
FrameReceiveForUSART frameReceiveForUSART;

static uint32_t rngData[4] = {0,0,0,0};

void resetFrameReceiveForUSART(void)
{
    for(int i = 0;i < sizeof(FrameReceiveForUSART);i++){
        frameReceiveForUSART.receiveDat[i] = '\0';
    }
}

FrameSendForUSART frameSendForUSART = {
    .tail = {0x00,000,0x80,0x7f}
};


static void getRandomNumber(uint32_t* buffer)
{
    if(LL_RNG_IsActiveFlag_CEIS(RNG) && LL_RNG_IsActiveFlag_CECS(RNG)){
        LL_RNG_ClearFlag_CEIS(RNG);
        return;
    }else if(LL_RNG_IsActiveFlag_SEIS(RNG) && LL_RNG_IsActiveFlag_SECS(RNG)){
        LL_RNG_ClearFlag_SEIS(RNG);
        for(int i = 0;i < 12;i++) 
            LL_RNG_ReadRandData32(RNG);
        if(LL_RNG_IsActiveFlag_SEIS(RNG) != 0u) 
            LL_RNG_ClearFlag_SEIS(RNG);
        return;;
    }

    uint32_t dat = LL_RNG_ReadRandData32(RNG);

    if(LL_RNG_IsActiveFlag_DRDY(RNG) && dat != 0u){
        buffer[0] = dat;
        buffer[1] = LL_RNG_ReadRandData32(RNG);
        buffer[2] = LL_RNG_ReadRandData32(RNG);
        buffer[3] = LL_RNG_ReadRandData32(RNG);
    }
}

void Hardware_SafatyTaskEvent(void)
{
    static uint32_t timeBase = 0u;
    //1s温度采集任务
    if((timeBase % 1000) == 0u){
        
        pSens->busAndTemp.com2 = Hardware_GetTemperature();
        if(pSens->busAndTemp.com2 > 65.f){
            Hardware_StopPWM();
        }
        LL_GPIO_TogglePin(GPIOC,LL_GPIO_PIN_13);
    }
    
    //1ms解析一次上位机命令

    // /*Use RS232*/
    // static uint16_t index = 0u;
    // uint16_t index1 = index % sizeof(frameReceiveForUSART);
    // uint16_t index2 = (index + 1) % sizeof(frameReceiveForUSART);
    // uint16_t index3 = (index + 2) % sizeof(frameReceiveForUSART);
    // uint16_t index4 = (index + 3) % sizeof(frameReceiveForUSART);
    // if(frameReceiveForUSART.receiveDat[index1] == 'S' && frameReceiveForUSART.receiveDat[index2] == ':' && frameReceiveForUSART.receiveDat[index4] == '\n'){
    //     uint8_t receiveDat = frameReceiveForUSART.receiveDat[index3];
    //     if(receiveDat < 0xC9){
    //         pSpPIC->target = 2.f * MATH_PI * receiveDat;
    //     }else{
    //         pSpPIC->target = -pSpPIC->target;
    //     }
    //     frameReceiveForUSART.receiveDat[index1] = frameReceiveForUSART.receiveDat[index2] = '\0';
    //     frameReceiveForUSART.receiveDat[index3] = frameReceiveForUSART.receiveDat[index4] = '\0';
    // }
    // index++;

    /*Use Virtual Serial*/
    static uint16_t index = 0u;
    uint16_t index1 = index % sizeof(UserRxBufferFS);
    uint16_t index2 = (index + 1) % sizeof(UserRxBufferFS);
    uint16_t index3 = (index + 2) % sizeof(UserRxBufferFS);
    uint16_t index4 = (index + 3) % sizeof(UserRxBufferFS);
    if(UserRxBufferFS[index1] == 'S' && UserRxBufferFS[index2] == ':' && UserRxBufferFS[index4] == '\n'){
        uint8_t receiveDat = UserRxBufferFS[index3];
        if(receiveDat < 0xC9){
            pSpPIC->target = 2.f * MATH_PI * receiveDat;
        }else{
            pSpPIC->target = -pSpPIC->target;
        }
        UserRxBufferFS[index1] = UserRxBufferFS[index2] = '\0';
        UserRxBufferFS[index3] = UserRxBufferFS[index4] = '\0';
    }
    index++;
    
    getRandomNumber(rngData);

    timeBase++;
}

void Hardware_PerformanceTaskEvent(void)
{
    // /*Use Segger RTT*/
    // static uint16_t cnt = 0u;
    // frameForRTT.dat0 = pSens->currentAB.com1;
    // frameForRTT.dat1 = pSens->dqVoltage.com1;
    // frameForRTT.dat2 = pSVP->volDQ.com1;
    // frameForRTT.dat3 = pSens->dqVoltage.com2;
    // frameForRTT.dat4 = pSVP->volDQ.com2;
    // frameForRTT.dat5 = pSens->currentDQ.com1;
    // frameForRTT.dat6 = pSens->currentDQ.com2;
    // if(cnt == 0u){
    //     cnt++;
    //     SEGGER_RTT_WriteNoLock(0,frameHeader,3);
    //     SEGGER_RTT_WriteNoLock(0,&frameForRTT,sizeof(frameForRTT));
    // }
    // else if(cnt == 999u){
    //     cnt = 0u;
    //     SEGGER_RTT_WriteNoLock(0,&frameForRTT,sizeof(frameForRTT));
    //     SEGGER_RTT_WriteNoLock(0,frameEnd,3);
    // }
    // else{
    //     cnt++;
    //     SEGGER_RTT_WriteNoLock(0,&frameForRTT,sizeof(frameForRTT));
    // }

    // /*Use RS232*/
    // if(LL_DMA_GetDataLength(DMA1,LL_DMA_CHANNEL_1) == 0u){
    //     frameSendForUSART.dat0 = pSens->phaseA_Vol;
    //     frameSendForUSART.dat1 = pSens->phaseB_Vol;
    //     frameSendForUSART.dat2 = pSens->phaseC_Vol;
    //     frameSendForUSART.dat3 = pSens->currentAB.com2;
    //     LL_DMA_DisableChannel(DMA1,LL_DMA_CHANNEL_1);
    //     LL_DMA_SetDataLength(DMA1,LL_DMA_CHANNEL_1,sizeof(FrameSendForUSART));
    //     LL_DMA_EnableChannel(DMA1,LL_DMA_CHANNEL_1);
    // }

    /*Use Virtual Serial*/
    frameSendForUSART.dat0 = pIF->eleAngle;
    frameSendForUSART.dat1 = pSens->currentAB.com1;
    frameSendForUSART.dat2 = pLuenberger->est_eleAngle;
    frameSendForUSART.dat3 = pLuenberger->est_eleSpeed;
    frameSendForUSART.dat4 = pIncABZ->realEleAngle;
    CDC_Transmit_FS((uint8_t*)&frameSendForUSART,sizeof(FrameSendForUSART));
}
void Hardware_Init(void)
{

}
bool Hardware_MCStartOrStop(void)
{
    static bool isStart = false;
    static uint16_t key3Cnt = 0u;
    uint32_t key3Signal = LL_GPIO_ReadInputPort(GPIOB) & (LL_GPIO_PIN_6);
    if(key3Signal){
        key3Cnt = 0u;
    }
    else{
      if(++key3Cnt > 300u){
        key3Cnt = 0;
        isStart = !isStart;
      }
    }
    return isStart;
}
void Hardware_StartPWM(void)
{
    LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH1N);
    LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH2N);
    LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_EnableChannel(TIM1,LL_TIM_CHANNEL_CH3N);
    LL_TIM_EnableAllOutputs(TIM1);
    LL_GPIO_ResetOutputPin(GPIOC,LL_GPIO_PIN_0);
}

void Hardware_StopPWM(void)
{
    LL_TIM_DisableAllOutputs(TIM1);
    LL_TIM_CC_DisableChannel(TIM1,LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_DisableChannel(TIM1,LL_TIM_CHANNEL_CH1N);
    LL_TIM_CC_DisableChannel(TIM1,LL_TIM_CHANNEL_CH2);
    LL_TIM_CC_DisableChannel(TIM1,LL_TIM_CHANNEL_CH2N);
    LL_TIM_CC_DisableChannel(TIM1,LL_TIM_CHANNEL_CH3);
    LL_TIM_CC_DisableChannel(TIM1,LL_TIM_CHANNEL_CH3N);
    LL_GPIO_SetOutputPin(GPIOC,LL_GPIO_PIN_0);
}

void Hardware_SetCCR(int32_t ccr1,int32_t ccr2,int32_t ccr3)
{
    LL_TIM_OC_SetCompareCH1(TIM1,ccr1);
    LL_TIM_OC_SetCompareCH2(TIM1,ccr2);
    LL_TIM_OC_SetCompareCH3(TIM1,ccr3); 
}

void Hardware_GetPhaseVoltage(_FORCE SensorHandler* sens)
{
    f32_t voltage;
    voltage = -(float)LL_ADC_INJ_ReadConversionData12(ADC1,LL_ADC_INJ_RANK_2) / 65535.f * 31.f * 3.3f;
    sens->terminalVoltage.com3 = voltage;
    voltage = -(float)LL_ADC_INJ_ReadConversionData12(ADC2,LL_ADC_INJ_RANK_2) / 65535.f * 31.f * 3.3f;
    sens->terminalVoltage.com2 = voltage;
    voltage = -(float)LL_ADC_INJ_ReadConversionData12(ADC2,LL_ADC_INJ_RANK_3) / 65535.f * 31.f * 3.3f;
    sens->terminalVoltage.com1 = voltage;
}

Components2 Hardware_GetSinCosVal(f32_t angleRad)
{
    Components2 sinCos;

    int32_t angleFixed = (angleRad / 3.141592653589793f * 2147483648.f);

    LL_CORDIC_WriteData(CORDIC,angleFixed);

    int32_t sinFixed = LL_CORDIC_ReadData(CORDIC);
    int32_t cosFixed = LL_CORDIC_ReadData(CORDIC);

    sinCos.com1 = sinFixed / 2147483648.f;
    sinCos.com2 = cosFixed / 2147483648.f;

    // sinCos.com1 =  arm_sin_f32(angleRad);
    // sinCos.com2 = arm_cos_f32(angleRad);
    
    return sinCos;
}
f32_t Hardware_FastSquareRoot(f32_t x)
{
    f32_t out;
    __asm{
        VSQRT.F32 out,x;
    };
    return out;
}
f32_t Hardware_FastReciprocalSquareRoot(f32_t x)
{
    // int32_t i;
    // f32_t x2,y;
    // x2 = x * 0.5f;
    // y = x;
    // i = *(int32_t*)&y;
    // i = 0x5f3759df - (i >> 1);
    // y = *(f32_t*)&i;
    // y = y * (1.5f - (x2 * y * y));
    // return y;
    f32_t out;
    __asm{
        VSQRT.F32 out,x;
    }
    out = 1.f / out;
    return out;
}
Components2 Harware_GetCurrentAB(void)
{
    Components2 ab;
    f32_t a = (float)LL_ADC_INJ_ReadConversionData12(ADC1,LL_ADC_INJ_RANK_1);
    f32_t b = (float)LL_ADC_INJ_ReadConversionData12(ADC2,LL_ADC_INJ_RANK_1);
    ab.com1 = (1.65f - (a / 65535.f * 3.3f)) / 20.f * 200.f;
    ab.com2 = (1.65f - (b / 65535.f * 3.3f)) / 20.f * 200.f;
    return ab;
}

f32_t Hardware_GetBusVoltage(void)
{
    f32_t bus;
    LL_ADC_REG_StartConversion(ADC1);
    while(LL_ADC_IsActiveFlag_EOC(ADC1) == 0u);
    bus = (float)LL_ADC_REG_ReadConversionData12(ADC1) / 65535.f * 3.3f * 31.f;
    while(LL_ADC_IsActiveFlag_EOC(ADC1) == 0u);
    pSens->adcCorrectionCoefficient = 1.21142578125f / ((float)LL_ADC_REG_ReadConversionData12(ADC1) / 65535.f * 3.3f);
    return bus;
}

f32_t Hardware_GetTemperature(void)
{
    LL_I2C_GenerateStartCondition(I2C1);
    int16_t dat = ((tempDat) >> 8)|((tempDat) << 8);
    dat >>= 5;
    return (float)dat * 0.125f;
}

void Hardware_SetPulseCounter(uint32_t cnt)
{
    LL_TIM_SetCounter(TIM16,cnt);
}

uint32_t Hardware_GetPulseCounter(void)
{
    return LL_TIM_GetCounter(TIM16);
}

void Hardware_SetABZCounter(uint32_t cnt)
{
    LL_TIM_SetCounter(TIM3,cnt);
}

uint32_t Hardware_GetABZCounter(void)
{
    return LL_TIM_GetCounter(TIM3);
}

uint32_t Hardware_GetABZCounterDir(void)
{   
    return (LL_TIM_GetDirection(TIM3) == LL_TIM_COUNTERDIRECTION_UP) ? 1u : 0u;
}

uint16_t Hardware_GetAbsCounter(void)
{
    return (((uint16_t)(pAbs->encoderOutput << 3)) >> 3) % 410;
}

