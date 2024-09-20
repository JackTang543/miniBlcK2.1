#include "sBSP_SPI.h"

/**
 * SPI资源分配
 * 
 * SPI1 -> SSD1306 只发送模式
 * SCK  -> PA5
 * MOSI -> PA7
 * NSS  -> PA4
 * 
 * 
 * 
 */

//GPIOA
#define SPI1_SCK_PIN    GPIO_PIN_5
#define SPI1_MOSI_PIN   GPIO_PIN_7
#define SPI1_NSS_PIN    GPIO_PIN_4



SPI_HandleTypeDef hspi1;

//SPI_BAUDRATEPRESCALER_2 ~ SPI_BAUDRATEPRESCALER_256
int8_t sBSP_SPI1M_Init(uint8_t SPI_BAUDRATE){
    hspi1.Instance = SPI1;
    //SPI主模式
    hspi1.Init.Mode = SPI_MODE_MASTER;
    //双线全双工
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    //数据大小8Bit
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    //时钟极性,空闲低:CPOL(Clock Polarity):0
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    //时钟相位,第一个边沿采样:CPHA(Clock Phase):0
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    //硬件NSS信号
    hspi1.Init.NSS = SPI_NSS_SOFT;
    //72MHZ 256分频
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATE;
    //高位先行
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    //TI(Texas Instruments)模式关闭
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    //CRC校验关闭
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    //CRC多项式7
    hspi1.Init.CRCPolynomial = 10;

    //模式0,CPOL = 0 CPHA = 0
    if(HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }
    __HAL_SPI_ENABLE(&hspi1);
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,0);
    return 0;
}

//重写HAL的MSP(MCU Support Package)Init
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi){
    if(hspi->Instance == SPI1){
        //! 先配置时钟,再配置外设!!!
        __GPIOA_CLK_ENABLE();
        __SPI1_CLK_ENABLE();
        
        GPIO_InitTypeDef gpio = {0};
        
        gpio.Mode = GPIO_MODE_AF_PP;
        gpio.Pin = SPI1_SCK_PIN | SPI1_MOSI_PIN;
        //GPIO_InitStruct.Pull = GPIO_NOPULL;
        gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        gpio.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOA,&gpio);

        //PB12:CS
        gpio.Mode = GPIO_MODE_OUTPUT_PP;
        gpio.Pin = SPI1_NSS_PIN;
        gpio.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA,&gpio);

        HAL_NVIC_SetPriority(SPI1_IRQn, 1, 0);
        HAL_NVIC_EnableIRQ(SPI1_IRQn);
        
        sBSP_SPI1M_SetCS(1);
    }
}

void sBSP_SPI1M_SetCS(uint8_t cs_en){
    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,(GPIO_PinState)cs_en);
}

void sBSP_SPI1M_SendBytes(uint8_t *pData,uint16_t Size){
    HAL_SPI_Transmit(&hspi1,pData,Size,1000);
}

void sBSP_SPI1M_ReciBytes(uint8_t *pData,uint16_t Size){
    HAL_SPI_Receive(&hspi1,pData,Size,1000);
}

void sBSP_SPI1M_SendByte(uint8_t byte){
    HAL_SPI_Transmit(&hspi1,&byte,1,100);
}

uint8_t sBSP_SPI1M_ReciByte(){
    uint8_t send_byte = 0;
    HAL_SPI_Receive (&hspi1,&send_byte,1,100);
    return send_byte;
}

void sBSP_SPI1M_SetEN(uint8_t en){
    if(en){
        __HAL_SPI_ENABLE(&hspi1);
    }else{
        __HAL_SPI_DISABLE(&hspi1);
    }
}



