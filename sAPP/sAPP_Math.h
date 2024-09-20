#ifndef __SAPP_MATH_H__
#define __SAPP_MATH_H__
#ifdef __cplusplus
extern "C"{
#endif

#include "main.h"

//PI
#define M_PI            (3.14159f)
//EXP
#define M_EXP           (2.71828f)

//重力常数
#define M_GRAVITY       (9.81398f)

//度转弧度
#define DEG2RAD		0.017453f
//弧度转度
#define RAD2DEG		57.29578f


float sAPP_MATH_InvSqrt(float x);
float sAPP_MATH_Press2Alt(float press_pa);


#ifdef __cplusplus
}
#endif
#endif
