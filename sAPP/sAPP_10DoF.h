#ifndef __SAPP_10DOF_H__
#define __SAPP_10DOF_H__
#ifdef __cplusplus
extern "C" {
#endif


#include "main.h"

typedef struct{
    float press;
    float temp;
    float gyr_x;
    float gyr_y;
    float gyr_z;
    float acc_x;
    float acc_y;
    float acc_z;
    float mag_x;
    float mag_y;
    float mag_z;
}sAPP_10DoF_Data_t;



void sAPP_10DoF_Init();

int8_t sAPP_10DoF_GetSensor();
int8_t sAPP_10DoF_GetBufData(sAPP_10DoF_Data_t* data_buf);


#ifdef __cplusplus
}
#endif
#endif
