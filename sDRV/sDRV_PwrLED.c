#include "sDRV_PwrLED.h"


void sDRV_PwrLED_Init(){
    sBSP_TIM1_Init();
    sBSP_TIM1_SetPWMFreq(10000);    //10K
    
}

void sDRV_PwrLED_SetEN(uint8_t en){
    sBSP_TIM1_CH1SetEN(en);
}

void sDRV_PwrLED_SetBrightness(float percent){
    sBSP_TIM1_CH1SetDuty(percent);
}


