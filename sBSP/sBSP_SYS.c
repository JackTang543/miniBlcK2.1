#include "sBSP_SYS.h"

void sBSP_SYS_Init(){
    HAL_Init();
    sBSP_RCC_Init();

    HAL_InitTick(0);
}


