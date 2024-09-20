#ifndef __SBSP_UART_H__
#define __SBSP_UART_H__

#include "main.h"

#ifdef __cplusplus
extern "C"{
#endif

#define UART1_RX_BUF_SIZE 256

extern char g_UART1_RxBuf[UART1_RX_BUF_SIZE];

typedef void(*reci_data_end_cb_t)(char* pReciData,uint16_t length);

void sBSP_UART1_Init(uint32_t bandrate);
void sBSP_UART6_Init(uint32_t bandrate);


void sBSP_UART1_SendBytes(uint8_t* pData,uint16_t length);
void sBSP_UART6_SendBytes(uint8_t* pData,uint16_t length);
void sBSP_UART1_ReadBytesBegin(reci_data_end_cb_t end_cb);
void sBSP_UART6_ReadBytesBegin(reci_data_end_cb_t end_cb);


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
void HAL_UART_MspInit(UART_HandleTypeDef* huart);



    
#ifdef __cplusplus
}
#endif



#endif

