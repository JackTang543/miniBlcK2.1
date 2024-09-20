#include "sHMI_LED.h"
 
 
sHMI_LED_t led;


//获取滴答定时器
static inline uint32_t getTick(void){
    return HAL_GetTick();
}

//LED模块初始化
void sHMI_LED_Init(){
    led.mode = SHMI_LED_MODE_OFF;
    led.time_ms = 0;
    led.prev_chrg_ts = 0;
    led.on_ts = 0;
    led.off_ts = 0;
    led.single_pulse_trig = 0;
    sBSP_GPIO_LEDInit();
}

//设置LED模式
void sHMI_LED_SetMode(sHMI_LED_MODE_t led_mode){
    sBSP_GPIO_SetLED(0);
    led.status = 0;
    led.mode = led_mode;
}


//设置闪烁时间
void sHMI_LED_SetBlinkTime_ms(uint16_t time){
    led.time_ms = time;
}

//设置脉冲模式的时间:亮起时长,一个循环的时长
void sHMI_LED_SetPulseTime_ms(uint32_t on_time, uint32_t period_time){
    //参数检查
    if((on_time == 0) || (period_time < on_time)) return;
    led.on_ts = on_time;
    //一个循环的总时长-亮起时间就是熄灭时间
    led.off_ts = period_time - on_time;
}

//启动一次脉冲,只有单脉冲模式生效
void sHMI_LED_StartSinglePulse(){
    if(led.mode == SHMI_LED_MODE_PULSE_SINGLE){
        //首先设置不输出
        sBSP_GPIO_SetLED(0);
        led.status = 0;
        //清空单次脉冲触发标志位
        led.single_pulse_trig = 0;
    }
}

//LED模块处理函数,需要定期调用
void sHMI_LED_Handler(){
    //关闭
    if(led.mode == SHMI_LED_MODE_OFF){
        sBSP_GPIO_SetLED(0);
    }
    //点亮
    else if(led.mode == SHMI_LED_MODE_ON){
        sBSP_GPIO_SetLED(1);
    }
    //闪烁模式
    else if(led.mode == SHMI_LED_MODE_BLINK){
        //如果闪烁时间为0,说明配置错误
        if(led.time_ms == 0) return;
        //如果当前时间-上一次变化的时间 >= 闪烁时间 就说明超时了,该翻转LED状态了
        if((getTick() - led.prev_chrg_ts) >= led.time_ms){ 
            //更新上一次变化时间
            led.prev_chrg_ts = getTick();
            //状态取反
            led.status = !led.status;
            //设置输出
            sBSP_GPIO_SetLED(led.status);
        }
    }
    //循环脉冲模式
    else if(led.mode == SHMI_LED_MODE_PULSE_CYCLE){
        //配置错误处理
        if(led.on_ts == 0 || led.on_ts == 0) return;
        //和闪烁模式大体一样原理,只是多了根据led的状态选择比较哪个时间
        if ((getTick() - led.prev_chrg_ts) >= (led.status ? led.on_ts : led.off_ts)) {
            led.prev_chrg_ts = getTick();
            led.status = !led.status;
            sBSP_GPIO_SetLED(led.status);
        }

    }
    //单次脉冲模式
    else if(led.mode == SHMI_LED_MODE_PULSE_SINGLE){
        //这个标志位使用来保证进入两次单脉冲模式的,进入两次才是一个完整脉冲
        if(led.single_pulse_trig >= 2) return;
        //配置错误处理
        if(led.on_ts == 0 || led.on_ts == 0) return;
        //同上,只是只会触发2次
        if ((getTick() - led.prev_chrg_ts) >= (led.status ? led.on_ts : led.off_ts)) {
            led.prev_chrg_ts = getTick();
            led.status = !led.status;
            sBSP_GPIO_SetLED(led.status);
            //标志位++
            led.single_pulse_trig++;
        }
    }
}

