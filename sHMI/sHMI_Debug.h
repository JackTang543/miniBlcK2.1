#ifndef __SHMI_DEBUG_H__
#define __SHMI_DEBUG_H__
#ifdef __cplusplus
extern "C"{
#endif

#include "main.h"

#include "stm32f4xx_hal_tim.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

void sHMI_Debug_Init(uint32_t bandrate);
void sHMI_Debug_Printf(char *p,...);

void sHMI_Debug_PrintTaskMang();


    
#ifdef __cplusplus
}
#endif
#endif

