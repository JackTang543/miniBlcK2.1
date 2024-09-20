#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif


void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void DebugMon_Handler(void);


void USART1_IRQHandler(void);
void USART6_IRQHandler(void);
void TIM1_TRG_COM_TIM11_IRQHandler(void);
void SPI1_IRQHandler();
void SPI2_IRQHandler();


#ifdef __cplusplus
}
#endif

#endif
