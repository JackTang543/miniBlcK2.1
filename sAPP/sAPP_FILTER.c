#include "sAPP_FILTER.h"





void sAPP_FILTER_Init(){

}



void sAPP_FILTER_ComplementaryFilter(){
    #define DT_S 0.005f		//精确时间差

	static float Kp = 0.4f;
	static float Ki = 0.001f;

	//四元数
	static float q0 = 1.0f;
	static float q1 = 0.0f;
	static float q2 = 0.0f;
	static float q3 = 0.0f;	

	//误差积分累计
	float exInt = 0.0f;
	float eyInt = 0.0f;
	float ezInt = 0.0f;

	static float rMat[3][3];/*旋转矩阵*/

    //用于标准化
    float normalise;
	float ex, ey, ez;
	float accBuf[3] = {0.f};

    //用于缓存数据,便于修改
    sAPP_AHRS_Model_t data;
    //拷贝数据
    data.gyro_x = ahrs.gyro_x;
    data.gyro_y = ahrs.gyro_y;
    data.gyro_z = ahrs.gyro_z;
    data.acc_x = ahrs.acc_x;
    data.acc_y = ahrs.acc_y;
    data.acc_z = ahrs.acc_z;

    //把陀螺仪的角度单位换算成弧度
    data.gyro_x *= DEG2RAD;	/* 度转弧度 */
	data.gyro_y *= DEG2RAD;
	data.gyro_z *= DEG2RAD;

	//标准化加速计测量值
	normalise = sAPP_MATH_InvSqrt(data.acc_x * data.acc_x + data.acc_y * data.acc_y + data.acc_z * data.acc_z);
	data.acc_x *= normalise;
	data.acc_y *= normalise;
	data.acc_z *= normalise;

	//加速计读取的方向与重力加速计方向的差值，用向量叉乘计算
	ex = (data.acc_y * rMat[2][2] - data.acc_z * rMat[2][1]);
	ey = (data.acc_z * rMat[2][0] - data.acc_x * rMat[2][2]);
	ez = (data.acc_x * rMat[2][1] - data.acc_y * rMat[2][0]);

	//误差累计，与积分常数相乘
	exInt += Ki * ex * DT_S ;  
	eyInt += Ki * ey * DT_S ;
	ezInt += Ki * ez * DT_S ;

	//用叉积误差来做PI修正陀螺零偏，即抵消陀螺读数中的偏移量
	data.gyro_x += Kp * ex + exInt;
	data.gyro_y += Kp * ey + eyInt;
	data.gyro_z += Kp * ez + ezInt;

	//一阶近似算法，四元数运动学方程的离散化形式和积分
	float q0Last = q0;
	float q1Last = q1;
	float q2Last = q2;
	float q3Last = q3;
	q0 += (-q1Last * data.gyro_x - q2Last * data.gyro_y - q3Last * data.gyro_z) * 0.5f * DT_S;
	q1 += ( q0Last * data.gyro_x + q2Last * data.gyro_z - q3Last * data.gyro_y) * 0.5f * DT_S;
	q2 += ( q0Last * data.gyro_y - q1Last * data.gyro_z + q3Last * data.gyro_x) * 0.5f * DT_S;
	q3 += ( q0Last * data.gyro_z + q1Last * data.gyro_y - q2Last * data.gyro_x) * 0.5f * DT_S;

	//标准化四元数
	normalise = sAPP_MATH_InvSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= normalise;
	q1 *= normalise;
	q2 *= normalise;
	q3 *= normalise;

	//计算方向余弦矩阵
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3;

    rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    rMat[0][1] = 2.0f * (q1q2 + -q0q3);
    rMat[0][2] = 2.0f * (q1q3 - -q0q2);

    rMat[1][0] = 2.0f * (q1q2 - -q0q3);
    rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    rMat[1][2] = 2.0f * (q2q3 + -q0q1);

    rMat[2][0] = 2.0f * (q1q3 + -q0q2);
    rMat[2][1] = 2.0f * (q2q3 - -q0q1);
    rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;

    //计算roll pitch yaw 欧拉角
	ahrs.roll  = -asinf(rMat[2][0]) * RAD2DEG; 
	ahrs.pitch =  atan2f(rMat[2][1], rMat[2][2]) * RAD2DEG;
	ahrs.yaw   =  atan2f(rMat[1][0], rMat[0][0]) * RAD2DEG;
    //保存四元数
    ahrs.q0 = q0;
    ahrs.q1 = q1;
    ahrs.q2 = q2;
    ahrs.q3 = q3;

    #undef DT_S
}



void sAPP_FILTER_SWFInit(sAPP_FILTER_SWF_t* pFliter,uint8_t size){
    pFliter->size = size;
    pFliter->data = (float*)pvPortMalloc(size * sizeof(float));
    memset(pFliter->data, 0, size * sizeof(float));
    pFliter->index = 0;
    pFliter->sum = 0.0;
    pFliter->average = 0.0;
}

void sAPP_FILTER_SWFDeInit(sAPP_FILTER_SWF_t* pFliter){
    vPortFree(pFliter->data);
    pFliter->data = NULL;
    pFliter->size = 0;
    pFliter->index = 0;
    pFliter->sum = 0.0;
    pFliter->average = 0.0;
}

//传入滤波器,和新数据,返回平均值
float sAPP_FILTER_SWFUpdate(sAPP_FILTER_SWF_t* pFliter,float newData){
    //减去即将被覆盖的数据点的值
    pFliter->sum -= pFliter->data[pFliter->index];
    //添加新的数据到窗口并更新索引
    pFliter->data[pFliter->index] = newData;
    pFliter->sum += newData;
    pFliter->index = (pFliter->index + 1) % pFliter->size;
    //计算新的平均值
    pFliter->average = pFliter->sum / pFliter->size;
    return pFliter->average;
}



void sAPP_FILTER_FIRInit(sAPP_FILTER_FIR_t* pFilter, uint8_t size, const float* coefficients){
    pFilter->size = size;
    pFilter->coefficients = (float*)pvPortMalloc(size * sizeof(float));
    memcpy(pFilter->coefficients, coefficients, size * sizeof(float));
    pFilter->data = (float*)pvPortMalloc(size * sizeof(float));
    memset(pFilter->data, 0, size * sizeof(float));
    pFilter->index = 0;
}

void sAPP_FILTER_FIRDeInit(sAPP_FILTER_FIR_t* pFilter){
    vPortFree(pFilter->coefficients);
    pFilter->coefficients = NULL;
    vPortFree(pFilter->data);
    pFilter->data = NULL;
    pFilter->size = 0;
    pFilter->index = 0;
}

float sAPP_FILTER_FIRUpdate(sAPP_FILTER_FIR_t* pFilter, float newData){
    pFilter->data[pFilter->index] = newData;
    float result = 0.0f;
    uint8_t idx = pFilter->index;
    for (uint8_t i = 0; i < pFilter->size; i++) {
        result += pFilter->coefficients[i] * pFilter->data[idx];
        idx = (idx == 0) ? (pFilter->size - 1) : (idx - 1);
    }
    pFilter->index = (pFilter->index + 1) % pFilter->size;
    return result;
}


void sAPP_FILTER_IIRInit(sAPP_FILTER_IIR_t* pFilter, uint8_t order, const float* a, const float* b){
    pFilter->order = order;
    pFilter->a = (float*)pvPortMalloc((order + 1) * sizeof(float));
    pFilter->b = (float*)pvPortMalloc((order + 1) * sizeof(float));
    memcpy(pFilter->a, a, (order + 1) * sizeof(float));
    memcpy(pFilter->b, b, (order + 1) * sizeof(float));
    pFilter->x = (float*)pvPortMalloc((order + 1) * sizeof(float));
    pFilter->y = (float*)pvPortMalloc((order + 1) * sizeof(float));
    memset(pFilter->x, 0, (order + 1) * sizeof(float));
    memset(pFilter->y, 0, (order + 1) * sizeof(float));
    pFilter->index = 0;
}

void sAPP_FILTER_IIRDeInit(sAPP_FILTER_IIR_t* pFilter){
    vPortFree(pFilter->a);
    pFilter->a = NULL;
    vPortFree(pFilter->b);
    pFilter->b = NULL;
    vPortFree(pFilter->x);
    pFilter->x = NULL;
    vPortFree(pFilter->y);
    pFilter->y = NULL;
    pFilter->order = 0;
    pFilter->index = 0;
}

float sAPP_FILTER_IIRUpdate(sAPP_FILTER_IIR_t* pFilter, float newData){
    pFilter->x[pFilter->index] = newData;
    float result = 0.0f;
    uint8_t idx = pFilter->index;

    // 计算前馈部分
    for (uint8_t i = 0; i <= pFilter->order; i++) {
        result += pFilter->b[i] * pFilter->x[idx];
        idx = (idx == 0) ? pFilter->order : (idx - 1);
    }

    // 计算反馈部分
    idx = pFilter->index;
    for (uint8_t i = 1; i <= pFilter->order; i++) {
        result -= pFilter->a[i] * pFilter->y[idx];
        idx = (idx == 0) ? pFilter->order : (idx - 1);
    }

    pFilter->y[pFilter->index] = result;
    pFilter->index = (pFilter->index + 1) % (pFilter->order + 1);
    return result;
}

// 用于qsort的比较函数
static int compareFloat(const void *a, const void *b) {
    float fa = *(const float*)a;
    float fb = *(const float*)b;
    return (fa > fb) - (fa < fb);
}

void sAPP_FILTER_MWFInit(sAPP_FILTER_MWF_t* pFilter, uint8_t size) {
    pFilter->size = size;
    pFilter->data = (float*)pvPortMalloc(size * sizeof(float));
    memset(pFilter->data, 0, size * sizeof(float));
    pFilter->index = 0;
}

// 传入滤波器和新数据, 返回中值
float sAPP_FILTER_MWFUpdate(sAPP_FILTER_MWF_t* pFilter, float newData) {
    // 更新数据
    pFilter->data[pFilter->index] = newData;
    pFilter->index = (pFilter->index + 1) % pFilter->size;
    
    // 创建一个用于排序的临时数组
    float sortedData[pFilter->size];
    memcpy(sortedData, pFilter->data, pFilter->size * sizeof(float));
    qsort(sortedData, pFilter->size, sizeof(float), compareFloat);
    
    // 返回中值
    if (pFilter->size % 2 == 1) {
        return sortedData[pFilter->size / 2];
    } else {
        // 如果数据数量是偶数，则取中间两个数的平均值
        return (sortedData[pFilter->size / 2 - 1] + sortedData[pFilter->size / 2]) / 2.0;
    }
}

void sAPP_FILTER_EMA1Init(sAPP_FILTER_EMA1_t* pFilter, float alpha) {
    pFilter->alpha = alpha;
    pFilter->lastValue = 0.0; // 初始值设为0，或者可以设为第一个数据点的值
}

// 传入滤波器和新数据，返回滤波结果
float sAPP_FILTER_EMA1Update(sAPP_FILTER_EMA1_t* pFilter, float newData) {
    // 更新滤波值：EMA公式为 EMA_t = alpha * newData + (1 - alpha) * EMA_{t-1}
    pFilter->lastValue = pFilter->alpha * newData + (1 - pFilter->alpha) * pFilter->lastValue;
    return pFilter->lastValue;
}

void sAPP_FILTER_EMA2Init(sAPP_FILTER_EMA2_t* pFilter, float alpha) {
    pFilter->alpha = alpha;
    pFilter->firstEMA = 0.0;
    pFilter->secondEMA = 0.0;
}

float sAPP_FILTER_EMA2Update(sAPP_FILTER_EMA2_t* pFilter, float newData) {
    // 更新第一阶EMA
    pFilter->firstEMA = pFilter->alpha * newData + (1 - pFilter->alpha) * pFilter->firstEMA;
    // 使用第一阶EMA的结果来更新第二阶EMA
    pFilter->secondEMA = pFilter->alpha * pFilter->firstEMA + (1 - pFilter->alpha) * pFilter->secondEMA;
    
    return pFilter->secondEMA;
}





