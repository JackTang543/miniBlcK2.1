#ifndef __SDRV_PWRLED_H__
#define __SDRV_PWRLED_H__
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#include "sBSP_TIM.h"

void sDRV_PwrLED_Init();
void sDRV_PwrLED_SetEN(uint8_t en);
void sDRV_PwrLED_SetBrightness(float percent);



#ifdef __cplusplus
}
#endif
#endif
