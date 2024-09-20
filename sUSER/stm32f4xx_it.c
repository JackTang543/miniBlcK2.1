#include "main.h"
#include "stm32f4xx_it.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;

extern TIM_HandleTypeDef  htim3;
extern TIM_HandleTypeDef  htim4;
extern TIM_HandleTypeDef  htim5;
extern TIM_HandleTypeDef  htim10;
extern TIM_HandleTypeDef  htim11;

extern I2C_HandleTypeDef  hi2c1;
extern I2C_HandleTypeDef  hi2c2;

extern SPI_HandleTypeDef  hspi1;

void NMI_Handler(void){
  while (1){
      
  }
}

void HardFault_Handler(void){
  while (1){
      
  }
}

void MemManage_Handler(void){
  while (1){
      
  }
}

void BusFault_Handler(void){
  while (1){
      
  }
}

void UsageFault_Handler(void){
  while (1){
      
  }
}



void DebugMon_Handler(void){
    
}


void USART1_IRQHandler(void){
    HAL_UART_IRQHandler(&huart1);
}

void USART6_IRQHandler(void){
    HAL_UART_IRQHandler(&huart6);
}

void TIM1_TRG_COM_TIM11_IRQHandler(void){
    HAL_TIM_IRQHandler(&htim11);
}

void I2C1_EV_IRQHandler(){
    HAL_I2C_EV_IRQHandler(&hi2c1);
}

void I2C1_ER_IRQHandler(){
    HAL_I2C_ER_IRQHandler(&hi2c1);
}

void I2C2_EV_IRQHandler(){
    HAL_I2C_EV_IRQHandler(&hi2c2);
}

void I2C2_ER_IRQHandler(){
    HAL_I2C_ER_IRQHandler(&hi2c2);
}

void TIM3_IRQHandler(void){
    HAL_TIM_IRQHandler(&htim3);
}

void TIM4_IRQHandler(void){
    HAL_TIM_IRQHandler(&htim4);
}

void TIM5_IRQHandler(void){
    HAL_TIM_IRQHandler(&htim5);
}

void TIM1_UP_TIM10_IRQHandler(void){
    HAL_TIM_IRQHandler(&htim10);
}

void SPI1_IRQHandler(){
    HAL_SPI_IRQHandler(&hspi1);
}


void SPI2_IRQHandler(){
    //HAL_SPI_IRQHandler(&hspi2);
}

