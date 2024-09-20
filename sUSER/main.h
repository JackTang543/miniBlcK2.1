#ifndef __MAIN_H__
#define __MAIN_H__
#ifdef __cplusplus
extern "C"{
#endif


#include "stm32f4xx_hal.h"
#include <math.h>
#include "arm_math.h"
#include <stdbool.h>
#include <ctype.h>

//BSP
#include "sBSP_SYS.h"
#include "sBSP_RCC.h"
#include "sBSP_GPIO.h"
#include "sBSP_UART.h"
#include "sBSP_TIM.h"
#include "sBSP_RTC.h"
#include "sBSP_DWT.h"
#include "sBSP_DMA.h"
//HMI
#include "sGraphic2D.h"
#include "sGenBtnDrv2.h"
#include "sHMI_Debug.h"
#include "sHMI_BUZZER.h"
#include "sHMI_LED.h"
#include "sHMI_G75.h"
//DRV
#include "sDRV_MPU6050.h"
#include "sDRV_BMP280.h"
#include "sDRV_HMC5883L.h"
#include "sDRV_MB85RCxx.h"
#include "sDRV_DRV8870.h"
#include "sDRV_GMR.h"
#include "sDRV_PwrLED.h"
#include "sDRV_INA219.h"
//APP
#include "sAPP_FILTER.h"
#include "sAPP_AHRS.h"
#include "sAPP_Task.h"
#include "sAPP_CTRL.h"
#include "sAPP_Func.h"
#include "sAPP_Motor.h"
#include "sAPP_Math.h"
#include "sAPP_CLI.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "FreeRTOS_CLI.h"
#include "task.h"

    
extern SemaphoreHandle_t g_sem_debug_uart_recied;

//extern sAPP_CLI_SendDataPacket_t g_cli_send_data_packet;


    
void Error_Handler();
void vApplicationMallocFailedHook();
void vApplicationIdleHook();
void vApplicationTickHook();
void vApplicationStackOverflowHook( TaskHandle_t xTask,char * pcTaskName );

#ifdef __cplusplus
}
#endif
#endif

