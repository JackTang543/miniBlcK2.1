#ifndef __SDRV_LED_H__
#define __SDRV_LED_H__
#ifdef __cplusplus
extern "C"{
#endif

#include "stm32f4xx_hal.h"
#include "sBSP_GPIO.h"

//无定时器版本


typedef enum{
    SHMI_LED_MODE_OFF = 0,
    SHMI_LED_MODE_ON,
    SHMI_LED_MODE_PULSE_CYCLE,
    SHMI_LED_MODE_PULSE_SINGLE,
    SHMI_LED_MODE_BLINK,
}sHMI_LED_MODE_t;

typedef enum{
    SHMI_LED_STATUS_DARK = 0,
    SHMI_LED_STATUS_LIGHT = 1
}sHMI_LED_STATUS_t;

typedef struct{
    sHMI_LED_MODE_t mode;
    uint16_t time_ms;
    uint32_t prev_chrg_ts;
    uint32_t on_ts;
    uint32_t off_ts;
    sHMI_LED_STATUS_t status;
    uint8_t single_pulse_trig;
}sHMI_LED_t;

void sHMI_LED_Init();
void sHMI_LED_SetMode(sHMI_LED_MODE_t led_mode);
void sHMI_LED_StartSinglePulse();
void sHMI_LED_SetPulseTime_ms(uint32_t on_time, uint32_t period_time);
void sHMI_LED_SetBlinkTime_ms(uint16_t time);
void sHMI_LED_Handler();







#ifdef __cplusplus
}
#endif
#endif
