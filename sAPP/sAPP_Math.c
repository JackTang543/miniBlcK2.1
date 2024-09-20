#include "sAPP_Math.h"


float sAPP_MATH_InvSqrt(float x){
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//匿名科创的算法,返回cm
float sAPP_MATH_Press2Alt(float press_pa){
    float x = (101000.0f - press_pa) / 1000.0f;
    return 0.82f * x * x * x + 0.9f * x * 10000.0f;
}





