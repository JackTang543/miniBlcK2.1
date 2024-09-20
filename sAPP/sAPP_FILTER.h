#ifndef __SAPP_FILTER_H__
#define __SAPP_FILTER_H__
#ifdef __cplusplus
extern "C"{
#endif

#include "main.h"
#include "sAPP_AHRS.h"


//滑窗滤波
typedef struct {
    uint8_t size;     //窗口大小
    float *data;      //数据缓冲区
    uint8_t index;    //当前数据索引
    float sum;        //数据的总和，用于快速计算平均值
    float average;    //当前的平均值
}sAPP_FILTER_SWF_t;

//FIR
typedef struct {
    float* coefficients;  //滤波器系数
    float* data;          //输入数据缓冲区
    uint8_t size;         //滤波器阶数
    uint8_t index;        //当前数据索引
}sAPP_FILTER_FIR_t;

//IIR
typedef struct {
    float* a;      //IIR滤波器的反馈系数
    float* b;      //IIR滤波器的前馈系数
    float* x;      //输入数据缓冲区
    float* y;      //输出数据缓冲区
    uint8_t order; //滤波器阶数
    uint8_t index; //当前数据索引
} sAPP_FILTER_IIR_t;

//中值滤波
typedef struct {
    uint8_t size;     // 窗口大小
    float *data;      // 数据缓冲区
    uint8_t index;    // 当前数据索引
} sAPP_FILTER_MWF_t; // 中值滤波器结构体

//一阶指数移动平均滤波
typedef struct {
    float alpha;     // 平滑因子，决定了新数据在平均值中的权重
    float lastValue; // 上一次的滤波结果
} sAPP_FILTER_EMA1_t;

//二阶指数移动平均滤波
typedef struct {
    float alpha;        // 平滑因子
    float firstEMA;     // 第一阶EMA的结果
    float secondEMA;    // 第二阶EMA的结果
} sAPP_FILTER_EMA2_t;


void sAPP_FILTER_Init();

void sAPP_FILTER_ComplementaryFilter();

void sAPP_FILTER_SWFInit(sAPP_FILTER_SWF_t* pFliter,uint8_t size);
void sAPP_FILTER_SWFDeInit(sAPP_FILTER_SWF_t* pFliter);
float sAPP_FILTER_SWFUpdate(sAPP_FILTER_SWF_t* pFliter,float newData);

void sAPP_FILTER_FIRInit(sAPP_FILTER_FIR_t* pFilter, uint8_t size, const float* coefficients);
void sAPP_FILTER_FIRDeInit(sAPP_FILTER_FIR_t* pFilter);
float sAPP_FILTER_FIRUpdate(sAPP_FILTER_FIR_t* pFilter, float newData);

void sAPP_FILTER_IIRInit(sAPP_FILTER_IIR_t* pFilter, uint8_t order, const float* a, const float* b);
void sAPP_FILTER_IIRDeInit(sAPP_FILTER_IIR_t* pFilter);
float sAPP_FILTER_IIRUpdate(sAPP_FILTER_IIR_t* pFilter, float newData);

void sAPP_FILTER_MWFInit(sAPP_FILTER_MWF_t* pFilter, uint8_t size);
float sAPP_FILTER_MWFUpdate(sAPP_FILTER_MWF_t* pFilter, float newData);

void sAPP_FILTER_EMA1Init(sAPP_FILTER_EMA1_t* pFilter, float alpha);
float sAPP_FILTER_EMA1Update(sAPP_FILTER_EMA1_t* pFilter, float newData);

void sAPP_FILTER_EMA2Init(sAPP_FILTER_EMA2_t* pFilter, float alpha);
float sAPP_FILTER_EMA2Update(sAPP_FILTER_EMA2_t* pFilter, float newData);


#ifdef __cplusplus
}
#endif
#endif