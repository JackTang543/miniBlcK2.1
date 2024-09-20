#include "sDRV_DRV8870.h"

/**
  * sDRV_DRV8870.c
  * 用于miniBlcCAR的电机驱动,电机型号:轮趣科技WHEELTEC MG513P30-12V
  * 
  * v1.0 2024.06.09
  * 初版,说是DRV8870的驱动程序,其实只要是差不多的芯片都能用
  * 
  * 
  * By SIGHTSEER. inHNIP9607Lab
  * TIME 2024.06.09  端午节快乐,学弟们高考加油!
  * 
  **/


//TIM3_CH3 -> RIGHT
//TIM3_CH4 -> LEFT


/***************************************对底层接口*************************************/
static inline void portSetLeftP_PWM(float duty){
    sBSP_TIM5_LPSetDuty(duty);
}

static inline void portSetLeftN_PWM(float duty){
    sBSP_TIM5_LNSetDuty(duty);
}

static inline void portSetRightP_PWM(float duty){
    sBSP_TIM5_RPSetDuty(duty);
}

static inline void portSetRightN_PWM(float duty){
    sBSP_TIM5_RNSetDuty(duty);
}



//float格式的映射,类似Arduino的map
static inline float fmap(float x, float in_min, float in_max, float out_min, float out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//初始化
void sDRV_DRV8870_Init(){
    //初始化底层电机驱动TIM
    sBSP_TIM5_Init();
    //设置电机PWM驱动频率100KHz
    sBSP_TIM5_SetPWMFreq(100000);
    //首先关闭输出
    sBSP_TIM5_LPSetDuty(0);
    sBSP_TIM5_LNSetDuty(0);
    sBSP_TIM5_RPSetDuty(0);
    sBSP_TIM5_RNSetDuty(0);
    //再把输出使能
    sBSP_TIM5_LPSetEN(1);
    sBSP_TIM5_LNSetEN(1);
    sBSP_TIM5_RPSetEN(1);
    sBSP_TIM5_RNSetEN(1);
}

//设置左轮刹车
void sDRV_DRV8870_SetLeftBrake(bool is_short){
    if(is_short){
        //短路刹车
        portSetLeftP_PWM(100.0f);
        portSetLeftN_PWM(100.0f);
    }else{
        //开路刹车
        portSetLeftP_PWM(0.0f);
        portSetLeftN_PWM(0.0f);
    }
}

//设置右轮刹车
void sDRV_DRV8870_SetRightBrake(bool is_short){
    if(is_short){
        //短路刹车
        portSetRightP_PWM(100.0f);
        portSetRightN_PWM(100.0f);
    }else{
        //开路刹车
        portSetRightP_PWM(0.0f);
        portSetRightN_PWM(0.0f);
    }
}

//设置左轮PWM输出占空比(百分比)
void sDRV_DRV8870_SetLeftPct(float percent){
    if(percent >= 0.0f){
        portSetLeftP_PWM(0);
        percent = fmap(percent, 0.0f, 100.0f,50.0f,100.0f);
        portSetLeftN_PWM(percent);
    }else if(percent <= 0.0f){
        portSetLeftN_PWM(0);
        percent = -percent;
        percent = fmap(percent, 0.0f,100.0f,50.0f,100.0f);
        portSetLeftP_PWM(percent);
    }
}

//设置右轮PWM输出占空比(百分比)
void sDRV_DRV8870_SetRightPct(float percent){
    if(percent >= 0.0f){
        portSetRightN_PWM(0);
        percent = fmap(percent, 0.0f, 100.0f,50.0f,100.0f);
        portSetRightP_PWM(percent);
    }else if(percent <= 0.0f){
        portSetRightP_PWM(0);
        percent = -percent;
        percent = fmap(percent, 0.0f,100.0f,50.0f,100.0f);
        portSetRightN_PWM(percent);
    }
}



