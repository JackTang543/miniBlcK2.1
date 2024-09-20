#include "sAPP_Func.h"




static bool get_lv(uint8_t btn_id){
    if(btn_id == SGBD_KEY1_ID){
        return sBSP_GPIO_GetKey1();
    }
    else if(btn_id == SGBD_KEY2_ID){
        return sBSP_GPIO_GetKey2();
    }
    return false;
}

static void trig(uint8_t btn_id,ev_flag_t btn_ev){
    if(btn_id == SGBD_KEY1_ID){
        if(btn_ev == ev_dp){
            //sHMI_G75_IntoTransMode();
            //sHMI_Debug_Printf("蓝牙模块进入透传模式\n");
        }
        else if(btn_ev == ev_pres){
            ctrl.blc_en = !ctrl.blc_en;
        }
    }


    //打印按键id的事件
    if(btn_ev == ev_pres){
        sHMI_BUZZER_StartSinglePulse();
        sHMI_Debug_Printf("KEY%d:按键按下\n",btn_id + 1);
    }
    else if(btn_ev == ev_rlsd){
        sHMI_Debug_Printf("KEY%d:按键松手\n",btn_id + 1);
    }
    else if(btn_ev == ev_dp){
        sHMI_BUZZER_StartSinglePulse();
        sHMI_Debug_Printf("KEY%d:双击按下\n",btn_id + 1);
    }
    else if(btn_ev == ev_dp_rlsd){
        sHMI_Debug_Printf("KEY%d:双击松手\n",btn_id + 1);
    }
    else if(btn_ev == ev_lp){
        sHMI_BUZZER_StartSinglePulse();
        sHMI_Debug_Printf("KEY%d:长按触发\n",btn_id + 1);
    }
    else if(btn_ev == ev_lp_rlsd){
        sHMI_Debug_Printf("KEY%d:长按松手\n",btn_id + 1);
    }
    else if(btn_ev == ev_lp_loop){
        sHMI_Debug_Printf("KEY%d:长按循环触发\n",btn_id + 1);
    }
    //sHMI_Debug_Printf("btn_id:%d,btn_ev:%d\n",btn_id,btn_ev);
}

void sAPP_Func_BtnInit(){
    btn_init_t btn_init = {0};
    btn_init.en = 1;                //使能此按键
    btn_init.lv_rev = lv_non_reverse;   //空闲时的电平反转
    btn_init.dp_mode = dp_enable;   //禁用双击,可提高连续单击速度
    btn_init.lp_loop_pridt = 300;   //设置长按循环触发间隔每500ms触发一次
    btn_init.lp_trig_waitt = 1000;  //设置长按触发时间2000ms
    btn_init.dp_prid_waitt = 200;   //设置最大等待双击时间
    sGBD_SetAllBtnEnable(1);        //设置所有按键使能
    sGBD_SetAllBtnMode(&btn_init);  //装载btn_init的配置参数
    sGBD_Init(get_lv,trig,HAL_GetTick);
}


void sAPP_Func_IMUInit(){
    if(sBSP_I2C1_Init(400000) != 0){
        sHMI_Debug_Printf("AHRS I2C总线初始化失败\n");
        Error_Handler();
    }

    sDRV_BMP280_Conf_t bmp280;
    bmp280.filter = BMP280_FILTER_16;
    bmp280.mode   = BMP280_MODE_NORMAL;
    bmp280.osrs_p = BMP280_OSRS_X16;
    bmp280.osrs_t = BMP280_OSRS_X2;
    bmp280.tsb    = BMP280_TSB_0D5MS;

    if(sDRV_BMP280_Init(&bmp280) != 0){
        sHMI_Debug_Printf("BMP280初始化失败\n");
        //Error_Handler();
    }else{
        sHMI_Debug_Printf("BMP280初始化成功\n");
    }

    if(sDRV_MPU6050_Init() != 0){
        sHMI_Debug_Printf("MPU6050初始化失败\n");
        //Error_Handler();
    }else{
        sHMI_Debug_Printf("MPU6050初始化成功\n");
    }
    
    if(sDRV_HMC5883L_Init() != 0){
        sHMI_Debug_Printf("HMC5883L初始化失败\n");
        //Error_Handler();
    }else{
        sHMI_Debug_Printf("HMC5883L初始化成功\n");
    }
}
