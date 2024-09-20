#include "sBSP_DMA.h"

/**
 * DMA2 Stream0:用于把一块内存置任意数
 * 
 * 
 */




DMA_HandleTypeDef hdma2_stream0;
DMA_HandleTypeDef hdma2_stream1;
uint32_t dma2_s0_src_data;


void sBSP_DMA2S0_Init(){
    __HAL_RCC_DMA2_CLK_ENABLE();

    hdma2_stream0.Instance = DMA2_Stream0;
    hdma2_stream0.Init.Channel = DMA_CHANNEL_0;
    hdma2_stream0.Init.Direction = DMA_MEMORY_TO_MEMORY;
    hdma2_stream0.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma2_stream0.Init.MemInc = DMA_MINC_ENABLE;
    hdma2_stream0.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma2_stream0.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma2_stream0.Init.Mode = DMA_NORMAL;
    hdma2_stream0.Init.Priority = DMA_PRIORITY_LOW;
    hdma2_stream0.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma2_stream0.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma2_stream0.Init.MemBurst = DMA_MBURST_INC4;
    hdma2_stream0.Init.PeriphBurst = DMA_PBURST_SINGLE;
    HAL_DMA_Init(&hdma2_stream0);
}

//使用DMA2流0把长度为字长的倍数的一段内存地址设置为1,注意输入len_bytes得是16的整数倍
void sBSP_DMA2S0_32MemSet(uint32_t value,uint32_t* pDst,uint32_t len_bytes){
    dma2_s0_src_data = value;
    HAL_DMA_Start(&hdma2_stream0,(uint32_t)&dma2_s0_src_data,(uint32_t)pDst,len_bytes / 4);
    HAL_DMA_PollForTransfer(&hdma2_stream0, HAL_DMA_FULL_TRANSFER, 100);
}

void sBSP_DMA2S1_Init(void){
    __HAL_RCC_DMA2_CLK_ENABLE(); // 启用DMA时钟

    
    hdma2_stream1.Instance = DMA2_Stream1;
    hdma2_stream1.Init.Channel = DMA_CHANNEL_2;
    hdma2_stream1.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma2_stream1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma2_stream1.Init.MemInc = DMA_MINC_ENABLE;
    hdma2_stream1.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma2_stream1.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma2_stream1.Init.Mode = DMA_NORMAL;
    hdma2_stream1.Init.Priority = DMA_PRIORITY_LOW;
    hdma2_stream1.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
    hdma2_stream1.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
    hdma2_stream1.Init.MemBurst = DMA_MBURST_SINGLE;
    hdma2_stream1.Init.PeriphBurst = DMA_PBURST_SINGLE;
    
    // 将DMA与SPI关联
    __HAL_LINKDMA(&hspi1, hdmatx, hdma2_stream1);
    
    // 初始化DMA
    HAL_DMA_Init(&hdma2_stream1);
    
    // 启用DMA传输完成中断
    //HAL_NVIC_SetPriority(DMAx_STREAMx_IRQn, 0, 1);
    //HAL_NVIC_EnableIRQ(DMAx_STREAMx_IRQn);
}

void sBSP_DMA2S1_32MemToSPI1(uint32_t* pSrc,uint32_t len_bytes){
    HAL_SPI_Transmit_DMA(&hspi1,(uint8_t*)pSrc,len_bytes / 4);
    //HAL_Delay(10);
    //vTaskDelay(10);
    //HAL_DMA_Start(&hdma2_stream1,(uint32_t)pSrc,(uint32_t)&hspi1.Instance->DR,len_bytes / 4);
    //HAL_DMA_PollForTransfer(&hdma2_stream1, HAL_DMA_FULL_TRANSFER, 100);
}




