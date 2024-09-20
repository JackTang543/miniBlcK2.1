#ifndef __SDRV_MPU6050_H__
#define __SDRV_MPU6050_H__
#ifdef __cplusplus
extern "C"{
#endif

#include "stm32f4xx_hal.h"
#include "sBSP_F4_I2C.h"
#include "main.h"



//外部同步
typedef enum{
	SDRV_MPU6050_EXTSYNC_DIS     = 0x00u,
	SDRV_MPU6050_EXTSYNC_TEMP_L  = 0x01u,
	SDRV_MPU6050_EXTSYNC_GYROX_L = 0x02u,
    SDRV_MPU6050_EXTSYNC_GYROY_L = 0x03u,
    SDRV_MPU6050_EXTSYNC_GYROZ_L = 0x04u,
	SDRV_MPU6050_EXTSYNC_ACCX_L  = 0x05u,
	SDRV_MPU6050_EXTSYNC_ACCY_L  = 0x06u,
    SDRV_MPU6050_EXTSYNC_ACCZ_L  = 0x07u,
}sDRV_MPU6050_EXTSYNC_t;

//设置数字低通滤波器
// DLPF_CFG	 加速度输出频率Hz(Fs=1KHz)	 延迟(ms) 	陀螺仪输出频率(Hz) 	延迟(ms) 	Fs(KHz)
// 0                 260                0             256               0.98        8      
// 1                 184                2             188               1.9         1
// 2                 94                 3             98                2.8         1
// 3                 44                 4.9           42                4.8         1
// 4	             21                 8.5           20                8.3         1
// 5                 10	                13.8          10                13.4        1
// 6                 5                  19            5                 18.6        1
typedef enum{
	SDRV_MPU6050_DLPF_0   = 0x00u,
	SDRV_MPU6050_DLPF_1   = 0x01u,
	SDRV_MPU6050_DLPF_2   = 0x02u,
    SDRV_MPU6050_DLPF_3   = 0x03u,
    SDRV_MPU6050_DLPF_4   = 0x04u,
	SDRV_MPU6050_DLPF_5   = 0x05u,
	SDRV_MPU6050_DLPF_6   = 0x06u,
    SDRV_MPU6050_DLPF_RES = 0x07u,
}sDRV_MPU6050_DLPF_t;

//设置MEMS自检
typedef enum{
	SDRV_MPU6050_SELFTEST_X   = 0b100u,
    SDRV_MPU6050_SELFTEST_Y   = 0b010u,
	SDRV_MPU6050_SELFTEST_Z   = 0b001u,
    SDRV_MPU6050_SELFTEST_ALL = 0b111u,
}sDRV_MPU6050_SELFTEST_t;

//陀螺仪满量程范围
typedef enum{
	SDRV_MPU6050_GYRO_FS_250   = 0x00u,
    SDRV_MPU6050_GYRO_FS_500   = 0x01u,
    SDRV_MPU6050_GYRO_FS_1000  = 0x02u,
    SDRV_MPU6050_GYRO_FS_2000  = 0x03u,
}SDRV_MPU6050_GYRO_FS_t;

//加速度计满量程范围
typedef enum{
    SDRV_MPU6050_ACCEL_FS_2G   = 0x00u,
    SDRV_MPU6050_ACCEL_FS_4G   = 0x01u,
    SDRV_MPU6050_ACCEL_FS_8G   = 0x02u,
    SDRV_MPU6050_ACCEL_FS_16G  = 0x03u,
}SDRV_MPU6050_ACCEL_FS_t;

//设置加速度计数字低通滤波器
typedef enum{
	SDRV_MPU6050_ACCEL_DLPF_RESET = 0x00u,
	SDRV_MPU6050_ACCEL_DLPF_5HZ   = 0x01u,
	SDRV_MPU6050_ACCEL_DLPF_2D5HZ = 0x02u,
	SDRV_MPU6050_ACCEL_DLPF_1D25HZ= 0x03u,
	SDRV_MPU6050_ACCEL_DLPF_0D63HZ= 0x04u,
	SDRV_MPU6050_ACCEL_DLPF_HOLD  = 0x07u,
}sDRV_MPU6050_ACCEL_DLPF_t;

//FIFO使能
typedef enum{
	SDRV_MPU6050_FIFO_EN_TEMP_EN  = 0b10000000u,
	SDRV_MPU6050_FIFO_EN_XG_EN    = 0b01000000u,
	SDRV_MPU6050_FIFO_EN_YG_EN    = 0b00100000u,
    SDRV_MPU6050_FIFO_EN_ZG_EN    = 0b00010000u,
	SDRV_MPU6050_FIFO_EN_ACCEL_EN = 0b00001000u,
	SDRV_MPU6050_FIFO_EN_SLV2_EN  = 0b00000100u,
	SDRV_MPU6050_FIFO_EN_SLV1_EN  = 0b00000010u,
    SDRV_MPU6050_FIFO_EN_SLV0_EN  = 0b00000001u,
	SDRV_MPU6050_FIFO_EN_MEMS_EN  = 0b11111000u,
}sDRV_MPU6050_FIFO_EN_t;

//信号路径复位
typedef enum{
	SDRV_MPU6050_SPRST_GYRO      = 0b100u,
	SDRV_MPU6050_SPRST_ACCEL     = 0b010u,
	SDRV_MPU6050_SPRST_TEMP      = 0b001u,
}sDRV_MPU6050_SPRst_t;

//时钟源选择
typedef enum{
	SDRV_MPU6050_CLKSEL_INTERNAL = 0x00u,
    SDRV_MPU6050_CLKSEL_PLLGYROX = 0x01u,
	SDRV_MPU6050_CLKSEL_PLLGYROY = 0x02u,
    SDRV_MPU6050_CLKSEL_PLLGYROZ = 0x03u,
    SDRV_MPU6050_CLKSEL_EXT32K   = 0x04u,
    SDRV_MPU6050_CLKSEL_EXT19M   = 0x05u,
    SDRV_MPU6050_CLKSEL_STOP     = 0x07u,
}sDRV_MPU6050_CLKSEL_t;

//周期唤醒频率
typedef enum{
	SDRV_MPU6050_LP_WAKE_CTRL_1D25HZ = 0x00u,
	SDRV_MPU6050_LP_WAKE_CTRL_2D5HZ,
	SDRV_MPU6050_LP_WAKE_CTRL_5HZ,
    SDRV_MPU6050_LP_WAKE_CTRL_10HZ,
}sDRV_MPU6050_LP_WAKE_CTRL_t;

//设置MEMS待机
typedef enum{
	SDRV_MPU6050_STBY_TYPE_XA  = 0b00100000u,
	SDRV_MPU6050_STBY_TYPE_YA  = 0b00010000u,
	SDRV_MPU6050_STBY_TYPE_ZA  = 0b00001000u,
    SDRV_MPU6050_STBY_TYPE_XG  = 0b00000100u,
    SDRV_MPU6050_STBY_TYPE_YG  = 0b00000010u,
	SDRV_MPU6050_STBY_TYPE_ZG  = 0b00000001u,
	SDRV_MPU6050_STBY_TYPE_ALL = 0b00111111u,
}sDRV_MPU6050_STBY_TYPE_t;

//储存MPU6050的配置信息
typedef struct{
	SDRV_MPU6050_GYRO_FS_t gyro_fs;
	SDRV_MPU6050_ACCEL_FS_t accel_fs;
}sDRV_MPU6050_t;

//数据缓冲
typedef struct{
	float AccX,AccY,AccZ;	//m/s^2
	float GyroX,GyroY,GyroZ;//度/秒
	float Temp;				//摄氏度
}sDRV_MPU6050_Data_t;





int8_t sDRV_MPU6050_Init();

void sDRV_MPU6050_SetAuxI2CLv(uint8_t is_vdd);
void sDRV_MPU6050_SetSmplrtDiv(uint8_t div);
void sDRV_MPU6050_SetExtSync(sDRV_MPU6050_EXTSYNC_t ext_sync);
void sDRV_MPU6050_SetDLPF(sDRV_MPU6050_DLPF_t dlpf);
void sDRV_MPU6050_SetGyroSelfTest(sDRV_MPU6050_SELFTEST_t gyro_selftest,uint8_t is_en);
void sDRV_MPU6050_SetGyroRange(SDRV_MPU6050_GYRO_FS_t fs);
void sDRV_MPU6050_SetAccelSelfTest(sDRV_MPU6050_SELFTEST_t accel_selftest,uint8_t is_en);
void sDRV_MPU6050_SetAccelRange(SDRV_MPU6050_ACCEL_FS_t fs);
void sDRV_MPU6050_SetAccelDLPF(sDRV_MPU6050_ACCEL_DLPF_t dlpf);
void sDRV_MPU6050_SetFIFOEn(sDRV_MPU6050_FIFO_EN_t fifo,uint8_t is_en);
void sDRV_MPU6050_SignalPathReset(sDRV_MPU6050_SPRst_t sprst);
void sDRV_MPU6050_Reset();
void sDRV_MPU6050_SetSleepMode(uint8_t is_sleep);
void sDRV_MPU6050_SetLoopWakeMode(uint8_t is_lpwake);
void sDRV_MPU6050_SetTempDis(uint8_t is_dis);
void sDRV_MPU6050_SetClkSel(sDRV_MPU6050_CLKSEL_t clk_sel);
void sDRV_MPU6050_SetLoopWakeFreq(sDRV_MPU6050_LP_WAKE_CTRL_t freq);


void sDRV_MPU6050_ReadData();
void sDRV_MPU6050_GetDataBuf(sDRV_MPU6050_Data_t* pData);



#ifdef __cplusplus
}
#endif
#endif
