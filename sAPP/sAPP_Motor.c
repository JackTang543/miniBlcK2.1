#include "sAPP_Motor.h"




#define MOTOR_RPM_MAX   ( 380.0f)
#define MOTOR_RPM_MIN   (-380.0f)

#define MATH_PI         (3.1415927f)
//rad/s^2 转换到 RPM/s^2
#define RADS2RPMS2      (3600.0f / (2.0f * MATH_PI))

//Left 速度环和GMR滤波
sAPP_CTRL_PosPID_t left_spd_pid;
sAPP_FILTER_SWF_t left_gmr_swf;
//Right
sAPP_CTRL_PosPID_t right_spd_pid;
sAPP_FILTER_SWF_t right_gmr_swf;

//这个值是上电默认的PID系数,这里是100Hz控制周期的值
const float  left_spd_pid_Kp = 1.25;
const float  left_spd_pid_Ki = 7.00;
const float  left_spd_pid_Kd = 0.025;
const float  left_pos_pid_Kp = 1.00;
const float  left_pos_pid_Ki = 0.10;
const float  left_pos_pid_Kd = 0.03;

const float right_spd_pid_Kp = 1.25;
const float right_spd_pid_Ki = 7.00;
const float right_spd_pid_Kd = 0.025;
const float right_pos_pid_Kp = 1.00;
const float right_pos_pid_Ki = 0.10;
const float right_pos_pid_Kd = 0.03;

/*Left*/
//加速度值,单位rad/s^2
float left_acc;
//速度环的输入
float left_spd;
//观测速度RPM值
float left_gmr;
//PWM输出占空比,单位%
float left_duty;

/*Right*/
float right_acc;
float right_spd;
float right_gmr;
float right_duty;

//初始化电机相关事项
void sAPP_Motor_Init(){
    //初始化电机驱动
    sDRV_DRV8870_Init();
    //初始化GMR编码器驱动
    sDRV_GMR_Init();

    //初始化轮子的速度环PID
    //输入:期望速度(-380RPM~380RPM),输出:PWM占空比(-100~100%),积分限幅-100~100
    sAPP_CTRL_PosPIDInit(&left_spd_pid, left_spd_pid_Kp, left_spd_pid_Ki, left_spd_pid_Kd,\
                        -100, 100,-100,100);
    sAPP_CTRL_PosPIDInit(&right_spd_pid, right_spd_pid_Kp, right_spd_pid_Ki, right_spd_pid_Kd,\
                        -100, 100,-100,100);
    //初始化轮子的编码器的SWF滤波
    sAPP_FILTER_SWFInit(&left_gmr_swf ,5);
    sAPP_FILTER_SWFInit(&right_gmr_swf ,5);
}

//设置Left速度RPM
void sAPP_Motor_SetLeftSpdRPM(float spd_rpm){
    sAPP_CTRL_PosPIDSetTarget(&left_spd_pid,spd_rpm);
}

//设置Left速度环PID的参数
void sAPP_Motor_SetLeftSpdPIDK(float kp,float ki,float kd){
    sAPP_CTRL_PosPIDSetK(&left_spd_pid,kp,ki,kd);
}

//设置Left加速度,单位rad/s^2
void sAPP_Motor_SetLeftAcc(float acc){
    left_acc = acc;
    //把输入的rad/s^2转换成RPM/s^2
    left_acc *= RADS2RPMS2;
}

//设置Right速度RPM
void sAPP_Motor_SetRightSpdRPM(float spd_rpm){
    sAPP_CTRL_PosPIDSetTarget(&right_spd_pid,spd_rpm);
}

//设置Right速度环PID的参数
void sAPP_Motor_SetRightSpdPIDK(float kp,float ki,float kd){
    sAPP_CTRL_PosPIDSetK(&right_spd_pid,kp,ki,kd);
}

//设置Right加速度,单位rad/s^2
void sAPP_Motor_SetRightAcc(float acc){
    right_acc = acc;
    //把输入的rad/s^2转换成RPM/s^2
    right_acc *= RADS2RPMS2;
}


//测试环境周期10ms,100Hz
void sAPP_Motor_Handler(){
    //更新GMR速度值
    sDRV_GMR_Handler();
    //得到时间间隔
    static uint32_t last_time;
    static uint32_t now_time;
    static uint32_t delta_time;
    static float dt;
    last_time = now_time;
    now_time = HAL_GetTick();
    delta_time = now_time - last_time;
    dt = (float)delta_time / 1000.0f;   //单位s

/*Left*/
    /*把加速度转换成速度*/
    //计算积分
    left_spd += left_acc * (dt / 1000.0f);
    //对速度限幅
    sAPP_CTRL_LimitF(&left_spd, MOTOR_RPM_MIN, MOTOR_RPM_MAX);
    //对速度环应用输入速度信息
    //sAPP_CTRL_PosPIDSetTarget(&left_spd_pid,left_spd);

    /*应用电机速度环PID*/
    //读取观测速度值,单位RPM
    left_gmr = sAPP_FILTER_SWFUpdate(&left_gmr_swf,sDRV_GMR_GetLeftRPM());
    //计算速度环PID,得到PWM占空比
    left_duty = sAPP_CTRL_PosPIDUpdate(&left_spd_pid, left_gmr, dt);
    

/*Right*/
/*把加速度转换成速度*/
    //计算积分
    right_spd += right_acc * (dt / 1000.0f);
    //对速度限幅
    sAPP_CTRL_LimitF(&right_spd, MOTOR_RPM_MIN, MOTOR_RPM_MAX);
    //对速度环应用输入速度信息
    //sAPP_CTRL_PosPIDSetTarget(&right_spd_pid,right_spd);

    /*应用电机速度环PID*/
    //读取观测速度值,单位RPM
    right_gmr = sAPP_FILTER_SWFUpdate(&right_gmr_swf,sDRV_GMR_GetRightRPM());
    //计算速度环PID,得到PWM占空比
    right_duty = sAPP_CTRL_PosPIDUpdate(&right_spd_pid, right_gmr, dt);

    //把PWM应用到电机
    sDRV_DRV8870_SetLeftPct(left_duty);
    //把PWM应用到电机
    sDRV_DRV8870_SetRightPct(right_duty);


    //sHMI_Debug_Printf("%.2f,%.2f,%.2f\n",gmr_left_rpm,
    //sAPP_CTRL_PosPIDGetTarget(&left_spd_pid),left_pwm_duty);

    //sHMI_Debug_Printf("%.2f,%.2f,%.2f,%.2f\n",left_acc,
    //left_spd,left_gmr,left_duty);

    //sHMI_Debug_Printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",left_acc,
    //left_spd,left_gmr,left_duty,right_acc,right_spd,right_gmr,right_duty);
    
    //sHMI_Debug_Printf("%.2f\n",dt);

}




