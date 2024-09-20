#include "sBSP_UART.h"


//这些是串口的GPIO口
#define UART1_GPIO       GPIOA
#define UART1_TX_PIN     GPIO_PIN_9
#define UART1_RX_PIN     GPIO_PIN_10

#define UART6_GPIO       GPIOA
#define UART6_TX_PIN     GPIO_PIN_11
#define UART6_RX_PIN     GPIO_PIN_12


//调试串口控制句柄
UART_HandleTypeDef huart1;
//串口6控制句柄
UART_HandleTypeDef huart6;
//保存用户传入的接收完成回调
reci_data_end_cb_t uart1_reci_data_end_cb;
reci_data_end_cb_t uart6_reci_data_end_cb;
//串口接收缓冲
char g_UART1_RxBuf[UART1_RX_BUF_SIZE];
uint8_t UART6_RxBuf[256];



//UART1初始化
void sBSP_UART1_Init(uint32_t bandrate){
    huart1.Instance = USART1;
    huart1.Init.BaudRate = bandrate;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    
    HAL_UART_Init(&huart1);
}

//UART6初始化
void sBSP_UART6_Init(uint32_t bandrate){
    huart6.Instance = USART6;
    huart6.Init.BaudRate = bandrate;
    huart6.Init.WordLength = UART_WORDLENGTH_8B;
    huart6.Init.StopBits = UART_STOPBITS_1;
    huart6.Init.Parity = UART_PARITY_NONE;
    huart6.Init.Mode = UART_MODE_TX_RX;
    huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_16;
    
    HAL_UART_Init(&huart6);
}

//覆写HAL库提供的弱定义
void HAL_UART_MspInit(UART_HandleTypeDef* huart){
    if(huart->Instance == USART1){
        __GPIOA_CLK_ENABLE();
        __USART1_CLK_ENABLE();

        GPIO_InitTypeDef gpio;
        gpio.Mode      = GPIO_MODE_AF_PP;
        gpio.Pin       = UART1_TX_PIN | UART1_RX_PIN;
        gpio.Pull      = GPIO_NOPULL;
        gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(UART1_GPIO,&gpio);
        
        HAL_NVIC_SetPriority(USART1_IRQn,5,0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }
    else if(huart->Instance == USART6){
        __GPIOA_CLK_ENABLE();
        __USART6_CLK_ENABLE();
        
        GPIO_InitTypeDef gpio;
        gpio.Mode      = GPIO_MODE_AF_PP;
        gpio.Pin       = UART6_TX_PIN | UART6_RX_PIN;
        gpio.Pull      = GPIO_NOPULL;
        gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF8_USART6;
        HAL_GPIO_Init(UART6_GPIO,&gpio);
        
        HAL_NVIC_SetPriority(USART6_IRQn,0,0);
        HAL_NVIC_EnableIRQ(USART6_IRQn);
    }
}


void sBSP_UART1_SendBytes(uint8_t* pData,uint16_t length){
    HAL_UART_Transmit(&huart1,pData,length,50);
    //HAL_UART_Transmit_IT(&huart1,pData,length);
}

void sBSP_UART6_SendBytes(uint8_t* pData,uint16_t length){
    HAL_UART_Transmit(&huart6,pData,length,50);
    //HAL_UART_Transmit_IT(&huart2,pData,length);
}



void sBSP_UART1_ReadBytesBegin(reci_data_end_cb_t end_cb){
    if(end_cb != NULL){
        uart1_reci_data_end_cb = end_cb;
    }else{
        return;
    }
    HAL_UARTEx_ReceiveToIdle_IT(&huart1, (char*)g_UART1_RxBuf, sizeof(g_UART1_RxBuf));
}

void sBSP_UART6_ReadBytesBegin(reci_data_end_cb_t end_cb){
    if(end_cb != NULL){
        uart6_reci_data_end_cb = end_cb;
    }else{
        return;
    }
    HAL_UARTEx_ReceiveToIdle_IT(&huart6, UART6_RxBuf, sizeof(g_UART1_RxBuf));
}



void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
    if (huart->Instance == USART1){
        if(uart1_reci_data_end_cb != NULL){
            //这里不清空缓冲区,因为有length在
            uart1_reci_data_end_cb(g_UART1_RxBuf,Size);
        }
    }
    else if(huart->Instance == USART6){
        if(uart6_reci_data_end_cb != NULL){
            //这里不清空缓冲区,因为有length在
            uart6_reci_data_end_cb(UART6_RxBuf,Size);
        }
    }
}


