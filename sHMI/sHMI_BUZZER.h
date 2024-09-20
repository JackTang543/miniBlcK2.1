#ifndef __SHMI_BUZZER_H__
#define __SHMI_BUZZER_H__
#ifdef __cplusplus
extern "C"{
#endif

#include "stm32f4xx_hal.h"
#include "sBSP_TIM.h"

typedef enum{
    SHMI_BUZZER_MODE_OFF = 0,
    SHMI_BUZZER_MODE_FIXED_FREQ,
    SHMI_BUZZER_MODE_PULSE_CYCLE,
    SHMI_BUZZER_MODE_PULSE_SINGLE
}sHMI_BUZZER_MODE_t;

typedef enum{
    SHMI_BUZZER_STATUS_OFF = 0,
    SHMI_BUZZER_STATUS_ON
}sHMI_BUZZER_STATUS_t;

typedef struct{
    sHMI_BUZZER_MODE_t mode;
    volatile uint16_t freq_hz;
    uint32_t prev_chrg_ts;
    uint32_t on_ts;
    uint32_t off_ts;
    sHMI_BUZZER_STATUS_t status;
    uint8_t single_pulse_trig;
}sHMI_BUZZER_t;

void sHMI_BUZZER_Init();
void sHMI_BUZZER_SetMode(sHMI_BUZZER_MODE_t buzzer_mode);
void sHMI_BUZZER_SetFreq(uint16_t _freq_hz);
void sHMI_BUZZER_StartSinglePulse();
void sHMI_BUZZER_SetPulseTime_ms(uint32_t on_time, uint32_t period_time);
void sHMI_BUZZER_Handler();



#ifdef __cplusplus
}
#endif
#endif

