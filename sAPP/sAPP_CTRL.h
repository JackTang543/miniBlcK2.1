#ifndef __SAPP_CTRL_H__
#define __SAPP_CTRL_H__
#ifdef __cplusplus
extern "C"{
#endif

#include "main.h"




typedef struct{
    float Kp;           //比例增益
    float Ki;           //积分增益
    float Kd;           //微分增益
    float setpoint;     //设定目标值
    float integral;     //积分累计
    float prev_error;   //上一次的误差
    float output_min;   //输出限制最小值
    float output_max;   //输出限制最大值
    float integral_max; //积分限幅最大值
    float integral_min; //积分限幅最小值
}sAPP_CTRL_PosPID_t;

typedef struct {
    float Kp;           // 比例增益
    float Ki;           // 积分增益
    float Kd;           // 微分增益
    float setpoint;     // 设定目标值
    float integral;     // 积分累计
    float prev_error;   // 上一次的误差
    float delta_output; // 输出的变化量
    float output;       // 当前输出值
    float output_min;   // 输出限制最小值
    float output_max;   // 输出限制最大值
    float integral_max; // 积分限幅最大值
    float integral_min; // 积分限幅最小值
} sAPP_CTRL_IncPID_t;

//小车平衡参数
typedef struct{
    float stand_kp;     //直立环Kp
    float stand_kd;     //直立环Kd
    float spd_kp;       //速度环Kp
    float spd_ki;       //速度环Ki = Kp / 200
    float turn_kp;      //转向环Kp
    float turn_kd;      //转向环Kd

    float m_angle;      //机械中值,单位deg

    float left_pwm;     //左轮PWM,单位百分比,-100~100
    float right_pwm;    //右轮PWM
    float left_rpm;     //左轮编码器转速,单位RPM
    float right_rpm;    //右轮编码器转速
    
    float stand_out;    //直立环输出
    float spd_out;      //速度环输出
    float turn_out;     //转向环输出


    float inc_pos_kp;
    float inc_pos_ki;
    float inc_pos_kd;

    

} sAPP_CTRL_Blc_t;

//小车控制参数
typedef struct{
    float tar_spd;      //目标速度
    float tar_head;     //目标航向
    float tar_move;     //目标移动

    float tar_spd2;

    float turn_spd;     //转向速度
    
    bool  blc_en;       //平衡使能
} sAPP_CTRL_Ctrl_t;

//导航参数
typedef struct{
    float x_acc_pure;   //去除重力分量的纯加速度
    float y_acc_pure;
    float z_acc_pure;
    float x_spd;        //速度
    float y_spd;
    float x_pos;        //位置
    float y_pos;

} sAPP_CTRL_Nav_t;


extern sAPP_CTRL_Blc_t blc;
extern sAPP_CTRL_Ctrl_t ctrl;
extern sAPP_CTRL_Nav_t nav;

extern sAPP_CTRL_IncPID_t pid_inc_pos;

void sAPP_CTRL_BlcInit();

void sAPP_CTRL_BlcHandler();

void sAPP_CTRL_LimitF(float *val, float min, float max);

void sAPP_CTRL_PosPIDInit(sAPP_CTRL_PosPID_t *pid, float Kp, float Ki, float Kd, float out_min, \
float out_max, float int_min, float int_max);
void sAPP_CTRL_PosPIDSetK(sAPP_CTRL_PosPID_t *pid, float Kp, float Ki, float Kd);
void sAPP_CTRL_PosPIDSetTarget(sAPP_CTRL_PosPID_t *pid, float setpoint);
float sAPP_CTRL_PosPIDGetTarget(sAPP_CTRL_PosPID_t *pid);
float sAPP_CTRL_PosPIDUpdate(sAPP_CTRL_PosPID_t *pid, float measured_value, float dt);

void sAPP_CTRL_IncPIDInit(sAPP_CTRL_IncPID_t *pid, float Kp, float Ki, float Kd, float out_min, \
float out_max, float int_min, float int_max);
void sAPP_CTRL_IncPIDSetK(sAPP_CTRL_IncPID_t *pid, float Kp, float Ki, float Kd);
void sAPP_CTRL_IncPIDSetTarget(sAPP_CTRL_IncPID_t *pid, float setpoint);
float sAPP_CTRL_IncPIDGetTarget(sAPP_CTRL_IncPID_t *pid);
float sAPP_CTRL_IncPIDUpdate(sAPP_CTRL_IncPID_t *pid, float measured_value, float dt);



#ifdef __cplusplus
}
#endif
#endif
