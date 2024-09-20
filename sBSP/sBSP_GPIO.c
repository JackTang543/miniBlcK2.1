#include "sBSP_GPIO.h"

#define LED_GPIO         GPIOC
#define LED_GPIO_PIN     GPIO_PIN_13

#define KEY1_GPIO        GPIOB
#define KEY1_GPIO_PIN    GPIO_PIN_12
#define KEY2_GPIO        GPIOB
#define KEY2_GPIO_PIN    GPIO_PIN_14

#define LEFT_P_GPIO      GPIOA
#define LEFT_P_GPIO_PIN  GPIO_PIN_0

#define LEFT_N_GPIO      GPIOA
#define LEFT_N_GPIO_PIN  GPIO_PIN_1

#define RIGHT_P_GPIO     GPIOA
#define RIGHT_P_GPIO_PIN GPIO_PIN_2

#define RIGHT_N_GPIO     GPIOA
#define RIGHT_N_GPIO_PIN GPIO_PIN_3



void sBSP_GPIO_LEDInit(){
    __GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = LED_GPIO_PIN;
    gpio.Mode      = GPIO_MODE_OUTPUT_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = 0;
    HAL_GPIO_Init(LED_GPIO, &gpio);
}

void sBSP_GPIO_KeyInit(){
    __GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = KEY1_GPIO_PIN;
    gpio.Mode      = GPIO_MODE_INPUT;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = 0;
    HAL_GPIO_Init(KEY1_GPIO, &gpio);
    gpio.Pin       = KEY2_GPIO_PIN;
    HAL_GPIO_Init(KEY2_GPIO, &gpio);
}

void sBSP_GPIO_SetLED(bool lv){
    HAL_GPIO_WritePin(LED_GPIO, LED_GPIO_PIN,!lv);
}



bool sBSP_GPIO_GetKey1(){
    return HAL_GPIO_ReadPin(KEY1_GPIO, KEY1_GPIO_PIN);
}

bool sBSP_GPIO_GetKey2(){
    return HAL_GPIO_ReadPin(KEY2_GPIO, KEY2_GPIO_PIN);
}



