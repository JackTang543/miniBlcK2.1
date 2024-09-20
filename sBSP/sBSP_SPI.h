#ifndef __SBSP_SPI_H__
#define __SBSP_SPI_H__
#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "main.h"
    
extern SPI_HandleTypeDef hspi1;

int8_t sBSP_SPI1M_Init(uint8_t SPI_BAUDRATE);
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi);
void sBSP_SPI1M_SetCS(uint8_t cs_en);
void sBSP_SPI1M_SendBytes(uint8_t *pData,uint16_t Size);
void sBSP_SPI1M_ReciBytes(uint8_t *pData,uint16_t Size);
void sBSP_SPI1M_SendByte(uint8_t byte);
uint8_t sBSP_SPI1M_ReciByte();
void sBSP_SPI1M_SetEN(uint8_t en);



#ifdef __cplusplus
}
#endif
#endif

