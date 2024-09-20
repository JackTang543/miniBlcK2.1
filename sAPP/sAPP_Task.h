#ifndef __SAPP_TASK_H__
#define __SAPP_TASK_H__
#ifdef __cplusplus
extern "C"{
#endif

#include "main.h"


void sAPP_Task_OLEDUpdate(void* pvPara);
void sAPP_Task_AHRSUpdate(void* pvPara);
void sAPP_Task_BlcCtrl(void* pvPara);
void sAPP_Task_UARTPrint(void* pvPara);
void sAPP_Task_10ms(void* pvPara);
void sAPP_Task_50ms(void* pvPara);
void sAPP_Task_100ms(void* pvPara);
void sAPP_Task_TaskMang(void* pvPara);
void sAPP_Task_500ms(void* pvPara);
void sAPP_Task_1000ms(void* pvPara);

void sAPP_Task_CLIReci(void* pvPara);
void sAPP_Task_CLISend(void* pvPara);


#ifdef __cplusplus
}
#endif
#endif
