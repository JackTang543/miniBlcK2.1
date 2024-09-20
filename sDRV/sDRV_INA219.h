#ifndef __SDRV_INA219_H__
#define __SDRV_INA219_H__
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

#include "sBSP_F4_I2C.h"

void sDRV_INA219_Init();


float sDRV_INA219_GetCurrA();
float sDRV_INA219_GetRshuntV();
float sDRV_INA219_GetBusV();
float sDRV_INA219_GetPwrW();



#ifdef __cplusplus
}
#endif
#endif