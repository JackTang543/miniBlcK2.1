#ifndef __SBSP_TIM_H__
#define __SBSP_TIM_H__
#ifdef __cplusplus
extern "C"{
#endif

#include "stm32f4xx_hal.h"
#include <stdbool.h>


void sBSP_TIM1_Init();
void sBSP_TIM1_SetPWMFreq(uint32_t freq);
void sBSP_TIM1_CH1SetEN(bool is_en);
void sBSP_TIM1_CH1SetDuty(float percent);




void sBSP_TIM2_Init();
void sBSP_TIM2_SetPWMFreq(uint32_t freq);
void sBSP_TIM2_CH1SetEN(bool is_en);
void sBSP_TIM2_CH1SetDuty(float percent);





void sBSP_TIM3_Init();
uint32_t sBSP_TIM3_EnGet();
void sBSP_TIM3_EnSet(uint32_t count);
void sBSP_TIM3_EnSetEN(bool is_en);


void sBSP_TIM4_Init();
uint32_t sBSP_TIM4_EnGet();
void sBSP_TIM4_EnSet(uint32_t count);
void sBSP_TIM4_EnSetEN(bool is_en);


void sBSP_TIM5_Init();
void sBSP_TIM5_SetPWMFreq(uint32_t freq);

void sBSP_TIM5_LPSetEN(bool is_en);
void sBSP_TIM5_LNSetEN(bool is_en);
void sBSP_TIM5_RPSetEN(bool is_en);
void sBSP_TIM5_RNSetEN(bool is_en);

void sBSP_TIM5_LPSetDuty(float percent);
void sBSP_TIM5_LNSetDuty(float percent);
void sBSP_TIM5_RPSetDuty(float percent);
void sBSP_TIM5_RNSetDuty(float percent);



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim);
void HAL_TIM_OC_MspInit(TIM_HandleTypeDef* tim_ocHandle);
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle);
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* tim_encoderHandle);
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle);



#ifdef __cplusplus
}
#endif
#endif
