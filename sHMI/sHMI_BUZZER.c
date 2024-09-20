#include "sHMI_BUZZER.h"

//人机交互:蜂鸣器
//By Sightseer. inHNIP9607Lab TIME:2024.05.17

/**
 * 使用定时器控制蜂鸣器
 * 
 * 
 * 使用说明:
 * 1.调用sHMI_BUZZER_Init()初始化
 * 2.调用sHMI_BUZZER_SetMode(sHMI_BUZZER_MODE_t buzzer_mode)设置模式
 * 3.调用sHMI_BUZZER_SetFreq(uint16_t freq)设置发声频率
 * 
 * 1)如果使用固定频率模式,则调用一次周期处理函数就好
 * 2)如果使用循环脉冲模式和单脉冲模式则需要周期调用处理函数
 * 3)如果使用循环脉冲模式,单脉冲模式,则需要调用sHMI_BUZZER_SetPulseTime_ms(uint32_t on_time, uint32_t period_time)以设置发声时间和
 * 周期时间,然后使用sHMI_BUZZER_StartSinglePulse()启动一次脉冲
 * 
 * 更新记录
 * v1.0 2024.05.17 初始版本 By Sightseer.
 * 
 */

sHMI_BUZZER_t buzzer;


//****************************************板级支持包接口******************************************************
static inline void TIM_Init(){
    sBSP_TIM2_Init();
}
static inline void TIM_SetFreq(uint16_t freq_hz){
    sBSP_TIM2_SetPWMFreq(freq_hz);
}
static inline void TIM_SetEN(bool en){
    sBSP_TIM2_CH1SetEN(en);
}
static inline uint32_t getTick(void){
    return HAL_GetTick();
}


//BUZZER模块初始化
void sHMI_BUZZER_Init(){
    buzzer.mode = SHMI_BUZZER_MODE_OFF;
    buzzer.freq_hz = 0;
    buzzer.prev_chrg_ts = 0;
    buzzer.on_ts = 0;
    buzzer.off_ts = 0;
    buzzer.single_pulse_trig = 0;
    TIM_Init();
    sBSP_TIM2_CH1SetDuty(50);
    TIM_SetEN(0);
}

//设置BUZZER模式
void sHMI_BUZZER_SetMode(sHMI_BUZZER_MODE_t buzzer_mode){
    TIM_SetEN(0);
    buzzer.status = 0;
    buzzer.mode = buzzer_mode;
}

//设置BUZZER频率
void sHMI_BUZZER_SetFreq(uint16_t _freq_hz){
    //限幅
    buzzer.freq_hz = _freq_hz;
    TIM_SetEN(0);
    TIM_SetFreq(_freq_hz);
}

//设置脉冲模式的时间:亮起时长,一个循环的时长
void sHMI_BUZZER_SetPulseTime_ms(uint32_t on_time, uint32_t period_time){
    //参数检查
    if((on_time == 0) || (period_time < on_time)) return;
    buzzer.on_ts = on_time;
    //一个循环的总时长-亮起时间就是熄灭时间
    buzzer.off_ts = period_time - on_time;
}

//启动一次脉冲,只有单脉冲模式生效
void sHMI_BUZZER_StartSinglePulse(){
    if(buzzer.mode == SHMI_BUZZER_MODE_PULSE_SINGLE){
        //首先设置不输出
        TIM_SetEN(0);
        buzzer.status = 0;
        //清空单次脉冲触发标志位
        buzzer.single_pulse_trig = 0;
    }
}

//LED模块处理函数,需要定期调用
void sHMI_BUZZER_Handler(){
    //关闭
    if(buzzer.mode == SHMI_BUZZER_MODE_OFF){
        TIM_SetEN(0);
    }
    //固定频率模式
    else if(buzzer.mode == SHMI_BUZZER_MODE_FIXED_FREQ){
        //TIM_SetFreq(buzzer.freq_hz);
        TIM_SetEN(1);
    }
    //循环脉冲模式
    else if(buzzer.mode == SHMI_BUZZER_MODE_PULSE_CYCLE){
        //配置错误处理
        if(buzzer.on_ts == 0 || buzzer.on_ts == 0) return;
        //和闪烁模式大体一样原理,只是多了根据led的状态选择比较哪个时间
        if ((getTick() - buzzer.prev_chrg_ts) >= (buzzer.status ? buzzer.on_ts : buzzer.off_ts)) {
            buzzer.prev_chrg_ts = getTick();
            buzzer.status = !buzzer.status;
            TIM_SetEN(buzzer.status);
        }
    }
    //单次脉冲模式
    else if(buzzer.mode == SHMI_BUZZER_MODE_PULSE_SINGLE){
        //这个标志位使用来保证进入两次单脉冲模式的,进入两次才是一个完整脉冲
        if(buzzer.single_pulse_trig >= 2) return;
        //配置错误处理
        if(buzzer.on_ts == 0 || buzzer.on_ts == 0) return;
        //同上,只是只会触发2次
        if ((getTick() - buzzer.prev_chrg_ts) >= (buzzer.status ? buzzer.on_ts : buzzer.off_ts)) {
            buzzer.prev_chrg_ts = getTick();
            buzzer.status = !buzzer.status;
            TIM_SetEN(buzzer.status);
            //标志位++
            buzzer.single_pulse_trig++;
        }
    }
}

