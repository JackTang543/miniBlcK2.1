#ifndef __SAPP_MOTOR_H__
#define __SAPP_MOTOR_H__
#ifdef __cplusplus
extern "C"{
#endif

#include "stm32f4xx_hal.h"

#include "sDRV_DRV8870.h"
#include "sDRV_GMR.h"

void sAPP_Motor_Init();

void sAPP_Motor_SetLeftSpdRPM(float spd_rpm);
void sAPP_Motor_SetLeftSpdPIDK(float kp,float ki,float kd);
void sAPP_Motor_SetLeftAcc(float acc);

void sAPP_Motor_SetRightSpdRPM(float spd_rpm);
void sAPP_Motor_SetRightSpdPIDK(float kp,float ki,float kd);
void sAPP_Motor_SetRightAcc(float acc);

void sAPP_Motor_Handler();


#ifdef __cplusplus
}
#endif
#endif
