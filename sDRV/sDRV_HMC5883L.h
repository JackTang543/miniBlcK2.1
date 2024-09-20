#ifndef __SAPP_HMC5883L_H__
#define __SAPP_HMC5883L_H__
#ifdef __cplusplus
extern "C"{
#endif

#include "main.h"

#include "stm32f4xx_hal.h"
#include "sBSP_F4_I2C.h"

//平均值数目
typedef enum{
    SDRV_HMC5883L_AVER_NUM_1 = 0x00u,
    SDRV_HMC5883L_AVER_NUM_2 = 0x01u,
    SDRV_HMC5883L_AVER_NUM_4 = 0x02u,
    SDRV_HMC5883L_AVER_NUM_8 = 0x03u,
}sDRV_HMC5883L_AVER_NUM_t;

//数据输出速率
typedef enum{
    SDRV_HMC5883L_OUTPUT_RATE_0D75HZ  = 0x00u,
    SDRV_HMC5883L_OUTPUT_RATE_1D5HZ   = 0x01u,
    SDRV_HMC5883L_OUTPUT_RATE_3HZ     = 0x02u,
    SDRV_HMC5883L_OUTPUT_RATE_7D5HZ   = 0x03u,
    SDRV_HMC5883L_OUTPUT_RATE_15HZ    = 0x04u,
    SDRV_HMC5883L_OUTPUT_RATE_30HZ    = 0x05u,
    SDRV_HMC5883L_OUTPUT_RATE_75HZ    = 0x06u,
}sDRV_HMC5883L_OUTPUT_RATE_t;


//测量模式
typedef enum{
    SDRV_HMC5883L_MEASURE_MODE_NORMAL = 0x00u,
    SDRV_HMC5883L_MEASURE_MODE_P_BIAS = 0x01u,
    SDRV_HMC5883L_MEASURE_MODE_N_BIAS = 0x02u,
}sDRV_HMC5883L_MEASURE_MODE_t;

//测量范围
typedef enum{
    SDRV_HMC5883L_RANGE_0D88GA        = 0x00u,
    SDRV_HMC5883L_RANGE_1D3GA         = 0x01u,
    SDRV_HMC5883L_RANGE_1D9GA         = 0x02u,
    SDRV_HMC5883L_RANGE_2D5GA         = 0x03u,
    SDRV_HMC5883L_RANGE_4D0GA         = 0x04u,
    SDRV_HMC5883L_RANGE_4D7GA         = 0x05u,
    SDRV_HMC5883L_RANGE_5D6GA         = 0x06u,
    SDRV_HMC5883L_RANGE_8D1GA         = 0x07u,
}sDRV_HMC5883L_RANGE_t;

//操作模式
typedef enum{
    SDRV_HMC5883L_OP_MODE_CONT_MEAS   = 0x00u,
    SDRV_HMC5883L_OP_MODE_SINGLE_MEAS = 0x01u,
    SDRV_HMC5883L_OP_MODE_IDLE        = 0x02u,
}sDRV_HMC5883L_OP_MODE_t;

//HMC5883L数据,单位mGa
typedef struct{
    float mag_x;
    float mag_y;
    float mag_z;
}sDRV_HMC5883L_DATA_t;

int8_t sDRV_HMC5883L_Init();
void   sDRV_HMC5883L_SetRange(sDRV_HMC5883L_RANGE_t _range);
void   sDRV_HMC5883L_Read(sDRV_HMC5883L_DATA_t* pData);




#ifdef __cplusplus
}
#endif
#endif
