#ifndef __SBSP_GPIO_H__
#define __SBSP_GPIO_H__
#ifdef __cplusplus
extern "C"{
#endif

#include "stm32f4xx_hal.h"
#include "stdbool.h"

void sBSP_GPIO_LEDInit();
void sBSP_GPIO_KeyInit();


void sBSP_GPIO_SetLED(bool lv);

bool sBSP_GPIO_GetKey1();
bool sBSP_GPIO_GetKey2();




#ifdef __cplusplus
}
#endif
#endif
