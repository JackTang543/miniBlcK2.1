#include "sBSP_TIM.h"

//APB1总线时钟频率,100MHz
#define PCLK1_FREQ (100000000)

//TIM1_CH1用于灯光PWM控制
TIM_HandleTypeDef htim1;
const uint32_t TIM1_ARRVal = 999;
const uint32_t TIM1_PSCVal = 0;
//TIM2_CH1用于蜂鸣器
TIM_HandleTypeDef htim2;
//PWM占空比精确度0.1%,频率100KHz
const uint32_t TIM2_ARRVal = 999;
const uint32_t TIM2_PSCVal = 0;

//用于Left电机的编码器读取
TIM_HandleTypeDef htim3;

//用于Right电机的编码器读取
TIM_HandleTypeDef htim4;

//用于电机控制
//TIM5_CH1:LEFT_P
//TIM5_CH2:LEFT_N
//TIM5_CH3:RIGHT_P
//TIM5_CH4:RIGHT_N
TIM_HandleTypeDef htim5;

TIM_HandleTypeDef htim10;

uint32_t TIM4_ARRVal = 1000;
uint32_t TIM4_PSCVal = 1;

//PWM占空比精确度0.1%,频率100KHz
uint32_t TIM5_ARRVal = 999;
uint32_t TIM5_PSCVal = 0;


void sBSP_TIM1_Init(){
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = TIM1_PSCVal;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = TIM1_ARRVal;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&htim1);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);

    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;

    HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);
    HAL_TIM_MspPostInit(&htim1);

    HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
}

void sBSP_TIM1_SetPWMFreq(uint32_t freq){
    //ARR = 999,输入频率范围3Hz~100KHz
    //PSC = (72M / (FREQ * (ARR + 1))) - 1
    __HAL_TIM_SET_PRESCALER(&htim1,((PCLK1_FREQ / (freq * (TIM1_ARRVal + 1))) - 1));
}

void sBSP_TIM1_CH1SetEN(bool is_en){
    if(is_en){
        HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
    }else{
        HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
    }
}

//duty范围:0~100%
void sBSP_TIM1_CH1SetDuty(float percent){
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,(uint32_t)(percent * 10.0f));
}




void sBSP_TIM2_Init(){
    TIM_MasterConfigTypeDef master = {0};
    TIM_OC_InitTypeDef oc = {0};
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = TIM2_PSCVal;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = TIM2_ARRVal;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&htim2);

    master.MasterOutputTrigger = TIM_TRGO_RESET;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim2, &master);

    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &oc, TIM_CHANNEL_1);

    HAL_TIM_MspPostInit(&htim2);
    
    HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
}

void sBSP_TIM2_SetPWMFreq(uint32_t freq){
    //ARR = 999,输入频率范围3Hz~100KHz
    //PSC = (72M / (FREQ * (ARR + 1))) - 1
    __HAL_TIM_SET_PRESCALER(&htim2,((PCLK1_FREQ / (freq * (TIM2_ARRVal + 1))) - 1));
}

void sBSP_TIM2_CH1SetEN(bool is_en){
    if(is_en){
        HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
    }else{
        HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
    }
}

//duty范围:0~100%
void sBSP_TIM2_CH1SetDuty(float percent){
    __HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,(uint32_t)(percent * 10.0f));
}




void sBSP_TIM3_Init(){
    TIM_Encoder_InitTypeDef encoder = {0};
    TIM_MasterConfigTypeDef master = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 65535;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    encoder.EncoderMode = TIM_ENCODERMODE_TI12;
    encoder.IC1Polarity = TIM_ICPOLARITY_RISING;
    encoder.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    encoder.IC1Prescaler = TIM_ICPSC_DIV1;
    encoder.IC1Filter = 0x7;
    encoder.IC2Polarity = TIM_ICPOLARITY_RISING;
    encoder.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    encoder.IC2Prescaler = TIM_ICPSC_DIV1;
    encoder.IC2Filter = 0x7;
    HAL_TIM_Encoder_Init(&htim3, &encoder);

    master.MasterOutputTrigger = TIM_TRGO_RESET;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim3, &master);
}

uint32_t sBSP_TIM3_EnGet(){
    return __HAL_TIM_GET_COUNTER(&htim3);
}

void sBSP_TIM3_EnSet(uint32_t count){
    __HAL_TIM_SET_COUNTER(&htim3,count);
}

void sBSP_TIM3_EnSetEN(bool is_en){
    if(is_en){
        HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
    }else{
        HAL_TIM_Encoder_Stop(&htim3,TIM_CHANNEL_ALL);
    }
}

void sBSP_TIM4_Init(){

    TIM_Encoder_InitTypeDef encoder = {0};
    TIM_MasterConfigTypeDef master = {0};

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 65535;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    encoder.EncoderMode = TIM_ENCODERMODE_TI12;
    encoder.IC1Polarity = TIM_ICPOLARITY_RISING;
    encoder.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    encoder.IC1Prescaler = TIM_ICPSC_DIV1;
    encoder.IC1Filter = 0x7;
    encoder.IC2Polarity = TIM_ICPOLARITY_RISING;
    encoder.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    encoder.IC2Prescaler = TIM_ICPSC_DIV1;
    encoder.IC2Filter = 0x7;
    HAL_TIM_Encoder_Init(&htim4, &encoder);

    master.MasterOutputTrigger = TIM_TRGO_RESET;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim4, &master);
}

uint32_t sBSP_TIM4_EnGet(){
    return __HAL_TIM_GET_COUNTER(&htim4);
}

void sBSP_TIM4_EnSet(uint32_t count){
    __HAL_TIM_SET_COUNTER(&htim4,count);
}

void sBSP_TIM4_EnSetEN(bool is_en){
    if(is_en){
        HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
    }else{
        HAL_TIM_Encoder_Stop(&htim4,TIM_CHANNEL_ALL);
    }
}


void sBSP_TIM5_Init(){
    TIM_MasterConfigTypeDef master = {0};
    TIM_OC_InitTypeDef oc = {0};
    htim5.Instance = TIM5;
    htim5.Init.Prescaler = TIM5_PSCVal;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = TIM5_ARRVal;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_PWM_Init(&htim5);

    master.MasterOutputTrigger = TIM_TRGO_RESET;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim5, &master);

    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim5, &oc, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim5, &oc, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim5, &oc, TIM_CHANNEL_3);
    HAL_TIM_PWM_ConfigChannel(&htim5, &oc, TIM_CHANNEL_4);

    HAL_TIM_MspPostInit(&htim5);
    
    HAL_TIM_PWM_Stop(&htim5,TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim5,TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim5,TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim5,TIM_CHANNEL_4);
}

void sBSP_TIM5_SetPWMFreq(uint32_t freq){
    //ARR = 999,输入频率范围3Hz~100KHz
    //PSC = (72M / (FREQ * (ARR + 1))) - 1
    __HAL_TIM_SET_PRESCALER(&htim5,((PCLK1_FREQ / (freq * (TIM5_ARRVal + 1))) - 1));
}

void sBSP_TIM5_LPSetEN(bool is_en){
    if(is_en){
        HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);
    }else{
        HAL_TIM_PWM_Stop(&htim5,TIM_CHANNEL_1);
    }
}

void sBSP_TIM5_LNSetEN(bool is_en){
    if(is_en){
        HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);
    }else{
        HAL_TIM_PWM_Stop(&htim5,TIM_CHANNEL_2);
    }
}

void sBSP_TIM5_RPSetEN(bool is_en){
    if(is_en){
        HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);
    }else{
        HAL_TIM_PWM_Stop(&htim5,TIM_CHANNEL_3);
    }
}

void sBSP_TIM5_RNSetEN(bool is_en){
    if(is_en){
        HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_4);
    }else{
        HAL_TIM_PWM_Stop(&htim5,TIM_CHANNEL_4);
    }
}

//duty范围:0~100%
void sBSP_TIM5_LPSetDuty(float percent){
    __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_1,(uint32_t)(percent * 10.0f));
}

void sBSP_TIM5_LNSetDuty(float percent){
    __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_2,(uint32_t)(percent * 10.0f));
}

void sBSP_TIM5_RPSetDuty(float percent){
    __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,(uint32_t)(percent * 10.0f));
}

void sBSP_TIM5_RNSetDuty(float percent){
    __HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,(uint32_t)(percent * 10.0f));
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim){
    if (htim->Instance == TIM11) {
        HAL_IncTick();
    }
}


void HAL_TIM_OC_MspInit(TIM_HandleTypeDef* tim_ocHandle){
    if(tim_ocHandle->Instance == TIM1){
        __HAL_RCC_TIM1_CLK_ENABLE();

        HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
        HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
        HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 15, 0);
        HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
        HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
    }
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle){
    if(tim_pwmHandle->Instance == TIM1){
        __HAL_RCC_TIM1_CLK_ENABLE();

        // HAL_NVIC_SetPriority(TIM1_BRK_TIM9_IRQn, 0, 0);
        // HAL_NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
        // HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0);
        // HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
        // HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 15, 0);
        // HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
        // HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
        // HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
    }
    else if(tim_pwmHandle->Instance == TIM2){
        __HAL_RCC_TIM2_CLK_ENABLE();

        HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM2_IRQn);
    }
    else if(tim_pwmHandle->Instance==TIM5){
        __HAL_RCC_TIM5_CLK_ENABLE();

        HAL_NVIC_SetPriority(TIM5_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM5_IRQn);

    }
}

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* tim_encoderHandle){
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(tim_encoderHandle->Instance == TIM3){
        __HAL_RCC_TIM3_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**TIM3 GPIO Configuration
        PB4     ------> TIM3_CH1
        PB5     ------> TIM3_CH2
        */
        GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        
        HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM3_IRQn);
    }
    else if(tim_encoderHandle->Instance == TIM4){
        __HAL_RCC_TIM4_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        /**TIM4 GPIO Configuration
        PB6     ------> TIM4_CH1
        PB7     ------> TIM4_CH2
        */
        GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM4_IRQn);
    }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle){
    if(tim_baseHandle->Instance == TIM10){
        __HAL_RCC_TIM10_CLK_ENABLE();

        HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
    }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle){
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(timHandle->Instance==TIM1){
        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**TIM1 GPIO Configuration
        PA8     ------> TIM1_CH1
        */
        GPIO_InitStruct.Pin = GPIO_PIN_8;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    else if(timHandle->Instance==TIM2){
        __HAL_RCC_GPIOA_CLK_ENABLE();
        //TIM2_CH1 -> PA15
        GPIO_InitStruct.Pin = GPIO_PIN_15;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    else if(timHandle->Instance==TIM5){
        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**TIM5 GPIO Configuration
        PA0-WKUP     ------> TIM5_CH1
        PA1     ------> TIM5_CH2
        PA2     ------> TIM5_CH3
        PA3     ------> TIM5_CH4
        */
        GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}

