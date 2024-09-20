#ifndef __SAPP_AHRS_H__
#define __SAPP_AHRS_H__
#ifdef __cplusplus
extern "C"{
#endif

#include "main.h"


typedef struct{
    bool is_free;       //系统是否处在自由落体中
}sAPP_AHRS_STATUS_t;


typedef struct{
    float pitch;        //单位deg
    float roll;
    float yaw;
    float acc_x;        //单位m/s^2
    float acc_y;
    float acc_z;
    float gyro_x;       //单位°/s
    float gyro_y;
    float gyro_z;
    float mag_x;        //单位mGs
    float mag_y;
    float mag_z;
    float apress;       //单位Pa
    float alt_m;        //单位m
    float bmp_temp;     //BMP280的温度传感器,摄氏度
    float mpu_temp;     //MPU6050的温度传感器
    float q0;           //姿态角四元数表示
    float q1;
    float q2;
    float q3;

    sAPP_AHRS_STATUS_t stat;    //保存状态信息

}sAPP_AHRS_Model_t;

//零偏数据
typedef struct{
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x ;
    float acc_y ;
    float acc_z ;
}sAPP_AHRS_IMU_StaticBias_t;


extern sAPP_AHRS_Model_t ahrs;



void sAPP_AHRS_Init();

void sAPP_AHRS_InitStaticBias();

void sAPP_AHRS_EstiUpdate();
void sAPP_AHRS_DataUpdate();

void sAPP_AHRS_Print();


float invSqrt(float x);


#ifdef __cplusplus
}
#endif
#endif