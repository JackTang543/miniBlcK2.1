#include "sAPP_CTRL.h"

/*这些是上电默认值*/
//机械中值
#define MECHINE_CENTER_ANGLE    (  0.0f)
//直立环初始值
#define STAND_KP                (-12.00f * 0.6f)
#define STAND_KD                (- 0.60f * 0.6f)
//速度环初始值
#define SPD_KP                  ( 0.30f)
#define SPD_KI                  (SPD_KP / 200.0f)
//转向环初始值
#define TURN_KP                 (  0.20f)
#define TURN_KD                 (  0.05f)

#define ZERO_KP                 (200.00f)
#define ZERO_KI                 ( 80.00f)
#define ZERO_KD                 (  0.00f)



#define WHEEL_RADIUS (0.0672f)

//保存平衡状态信息
sAPP_CTRL_Blc_t blc;
//保存控制信息
sAPP_CTRL_Ctrl_t ctrl;
//保存导航信息
sAPP_CTRL_Nav_t nav;

//PD直立环
static inline float StandPDCtrler(float pitch_deg,float pitch_deg_spd);
//PI速度环
static float SpdPICtrler(float left_rpm,float right_rpm);
//PD转向环
static float TurnPDCtrler(float gyro_z);


sAPP_CTRL_IncPID_t pid_inc_pos;

//平衡控制初始化
void sAPP_CTRL_BlcInit(){
    //设置各个参数的初始值
    blc.m_angle = MECHINE_CENTER_ANGLE;
    blc.stand_kp = STAND_KP;
    blc.stand_kd = STAND_KD;
    blc.spd_kp   = SPD_KP;
    blc.spd_ki   = SPD_KI;
    blc.turn_kd  = TURN_KD;
    blc.turn_kp  = TURN_KP;

    blc.inc_pos_kp = ZERO_KP;
    blc.inc_pos_ki = ZERO_KI;
    blc.inc_pos_kd = ZERO_KD;

    //上电禁用平衡
    ctrl.blc_en = 0;



    sAPP_CTRL_IncPIDInit(&pid_inc_pos,blc.inc_pos_kp, blc.inc_pos_ki, blc.inc_pos_kd,-200,200,1000,1000);
    sAPP_CTRL_IncPIDSetTarget(&pid_inc_pos,0);
}

void sAPP_CTRL_BlcHandler(){
    #define DT_S (0.010f)    //10ms 100Hz控制周期

    //如果没有启用平衡就不计算
    if(ctrl.blc_en == 0){
        //电机设置开路
        sDRV_DRV8870_SetRightBrake(0);
        sDRV_DRV8870_SetLeftBrake(0);
        goto PRINT;
        //return;
    }
    
    //安全保护
    if(ahrs.pitch > 45 || ahrs.pitch < -45){
        sDRV_DRV8870_SetRightBrake(1);
        sDRV_DRV8870_SetLeftBrake(1);
        //关闭平衡开关
        ctrl.blc_en = 0;
        //Error_Handler();
    }

    //获取电机转速
    blc.left_rpm  = sDRV_GMR_GetLeftRPM ();
    blc.right_rpm = sDRV_GMR_GetRightRPM();

    //对编码器读到的转速值转换成线速度并积分
    nav.y_pos += (blc.left_rpm + blc.right_rpm) * ((2.0f * PI * WHEEL_RADIUS) / 60.0f) * DT_S;


    //static float inc_pos_tar;
    //inc_pos_tar += ctrl.tar_spd2 * DT_S;

    //sAPP_CTRL_IncPIDSetTarget(&pid_inc_pos,inc_pos_tar);

    float inc_pos_out;

    

    if(ctrl.tar_spd2 == 0){
        inc_pos_out = sAPP_CTRL_IncPIDUpdate(&pid_inc_pos,nav.y_pos,DT_S);
        ctrl.tar_spd = inc_pos_out;
    }else{
        nav.y_pos = 0;
        pid_inc_pos.integral = 0;
        ctrl.tar_spd = ctrl.tar_spd2;
    }

    
    
    //计算直立环
    blc.stand_out = StandPDCtrler(ahrs.pitch,ahrs.gyro_x);
    //计算速度环
    blc.spd_out   = SpdPICtrler  (blc.left_rpm,blc.right_rpm);
    //应用转向环
    blc.turn_out  = TurnPDCtrler (ahrs.yaw);
    

    //合并三环输出
    blc.left_pwm  = blc.stand_out + blc.spd_out + blc.turn_out;
    blc.right_pwm = blc.stand_out + blc.spd_out - blc.turn_out;

    //输出限幅
    sAPP_CTRL_LimitF(&blc.left_pwm,-100,100);
    sAPP_CTRL_LimitF(&blc.right_pwm,-100,100);


    if(ahrs.stat.is_free == 0){
        //短路刹车,确保落体中电机不动
        //sDRV_DRV8870_SetRightBrake(1);
        //sDRV_DRV8870_SetLeftBrake(1);
    }

    
        //应用输出
    sDRV_DRV8870_SetLeftPct(blc.left_pwm);
    sDRV_DRV8870_SetRightPct(blc.right_pwm);

    
PRINT:




    //sHMI_Debug_Printf("%.2f,%.2f,%.2f\n",nav.x_acc_pure,nav.y_acc_pure,nav.z_acc_pure);

    //sHMI_Debug_Printf("%.2f,%.2f,%.2f,%.2f\n",nav.y_pos,inc_pos_out,ctrl.tar_move,ctrl.tar_spd);

    //调试:平衡
    //sHMI_Debug_Printf("%.2f,%.2f,%.2f,%.2f,%.4f,%.2f\n",ahrs.pitch,ahrs.gyro_x,blc.left_pwm,blc.right_pwm,sDRV_GMR_GetLeftRPM(),sDRV_GMR_GetRightRPM());

    //调试:转向
    //sHMI_Debug_Printf("%.2f,%.2f,%.2f,%.2f\n",ctrl.tar_head,ahrs.yaw,ahrs.gyro_z,blc.turn_out);

    //调试:自由落体
    //sHMI_Debug_Printf("is_free:%d\n",ahrs.stat.is_free);
    
    //sHMI_Debug_Printf("%.2f,%.2f,%.2f\n",ahrs.pitch,ahrs.gyro_x,ahrs.alt_m);
    
    (void)0;


    #undef DT_S
}


//直立环PD控制,传参:俯仰角,俯仰角速度
static inline float StandPDCtrler(float pitch_deg,float pitch_deg_spd){
    //应用机械中值后,使用直立环PD控制器
    return blc.stand_kp * (pitch_deg - blc.m_angle) + blc.stand_kd * pitch_deg_spd;
}

//速度环PI控制,传参:左右电机编码器转速
static float SpdPICtrler(float left_rpm,float right_rpm){
    static float velocity, Encoder_Least, Encoder_bias;
    static float Encoder_Integral;

    //计算最新速度偏差 = 目标速度- 测量速度（左右编码器之和）
    Encoder_Least = ctrl.tar_spd - (left_rpm + right_rpm);
    //一阶低通滤波器,减缓速度变化
    Encoder_bias *= 0.86;
    Encoder_bias += Encoder_Least * 0.14;
    //积分出位移,积分时间：10ms
    Encoder_Integral += Encoder_bias;
    //接收遥控器数据,控制前进后退
    Encoder_Integral += ctrl.tar_move;
    //积分限幅
    sAPP_CTRL_LimitF(&Encoder_Integral, -1000, 1000);
    //计算速度PI
    velocity = -Encoder_bias * blc.spd_kp - Encoder_Integral * blc.spd_ki;
    return velocity;
}


float angleDifference(float angle1, float angle2) {
    float diff = angle2 - angle1;
    while (diff < -180.0) diff += 360.0;
    while (diff > 180.0) diff -= 360.0;
    return diff;
}

//转向环PD控制,传参:陀螺仪角度
static float TurnPDCtrler(float gyro_z){
    //转向值,用于返回
    static float turn_val;

    float error = angleDifference(ctrl.tar_head, gyro_z);

    //计算转向PD控制器
    //turn_val = (gyro_z - ctrl.tar_head) * blc.turn_kp + (gyro_z - ctrl.tar_head) * blc.turn_kd;

    turn_val = error * blc.turn_kp + error * blc.turn_kd;

    return turn_val;
}



void sAPP_CTRL_LimitF(float *val, float min, float max){
    if(*val > max) *val = max;
    if(*val < min) *val = min;
}

// 初始化PID控制器
void sAPP_CTRL_PosPIDInit(sAPP_CTRL_PosPID_t *pid, float Kp, float Ki, float Kd, float out_min, float out_max, float int_min, float int_max){
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = 0.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_min = out_min;
    pid->output_max = out_max;
    pid->integral_max = int_max;
    pid->integral_min = int_min;
}

void sAPP_CTRL_PosPIDSetK(sAPP_CTRL_PosPID_t *pid, float Kp, float Ki, float Kd){
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}

void sAPP_CTRL_PosPIDSetTarget(sAPP_CTRL_PosPID_t *pid, float setpoint){
    pid->setpoint = setpoint;
    //pid->integral = 0.0f;
    //pid->prev_error = 0.0f;
}

float sAPP_CTRL_PosPIDGetTarget(sAPP_CTRL_PosPID_t *pid){
    return pid->setpoint;
}

// 更新PID控制器
float sAPP_CTRL_PosPIDUpdate(sAPP_CTRL_PosPID_t *pid, float measured_value, float dt){
    float error = pid->setpoint - measured_value; // 计算误差
    float integral_temp = pid->integral + error * dt; // 计算新的积分
    // 积分限幅
    if (integral_temp > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if (integral_temp < pid->integral_min) {
        pid->integral = pid->integral_min;
    } else {
        pid->integral = integral_temp;
    }
    float derivative = (error - pid->prev_error) / dt; // 计算微分
    // 计算PID输出
    float output = (pid->Kp * error) + (pid->Ki * pid->integral) + (pid->Kd * derivative);
    // 保存当前误差为下一次计算的"上一次误差"
    pid->prev_error = error;
    // 限制输出范围
    if (output > pid->output_max) output = pid->output_max;
    else if (output < pid->output_min) output = pid->output_min;
    return output;
}


// 初始化增量式PID控制器
void sAPP_CTRL_IncPIDInit(sAPP_CTRL_IncPID_t *pid, float Kp, float Ki, float Kd, float out_min, float out_max, float int_min, float int_max) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = 0.0f;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f; // 初始化输出值
    pid->output_min = out_min;
    pid->output_max = out_max;
    pid->integral_max = int_max;
    pid->integral_min = int_min;
}

void sAPP_CTRL_IncPIDSetK(sAPP_CTRL_IncPID_t *pid, float Kp, float Ki, float Kd){
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
}

void sAPP_CTRL_IncPIDSetTarget(sAPP_CTRL_IncPID_t *pid, float setpoint) {
    pid->setpoint = setpoint;
}

float sAPP_CTRL_IncPIDGetTarget(sAPP_CTRL_IncPID_t *pid) {
    return pid->setpoint;
}

float sAPP_CTRL_IncPIDUpdate(sAPP_CTRL_IncPID_t *pid, float measured_value, float dt) {
    float error = pid->setpoint - measured_value; // 计算误差
    float delta_error = error - pid->prev_error; // 计算误差变化
    float integral_temp = pid->integral + error * dt; // 计算新的积分

    // 积分限幅
    if (integral_temp > pid->integral_max) {
        pid->integral = pid->integral_max;
    } else if (integral_temp < pid->integral_min) {
        pid->integral = pid->integral_min;
    } else {
        pid->integral = integral_temp;
    }

    // 计算微分
    float derivative = delta_error / dt;

    // 计算增量输出
    pid->delta_output = (pid->Kp * delta_error) + (pid->Ki * error * dt) + (pid->Kd * derivative);

    // 更新当前输出
    pid->output += pid->delta_output;

    // 限制输出范围
    if (pid->output > pid->output_max) pid->output = pid->output_max;
    else if (pid->output < pid->output_min) pid->output = pid->output_min;

    // 保存当前误差为下一次计算的"上一次误差"
    pid->prev_error = error;

    return pid->output;
}

