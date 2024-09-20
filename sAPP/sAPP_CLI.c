#include "sAPP_CLI.h"



//软件复位系统
static BaseType_t reset(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
const CLI_Command_Definition_t comm_reset = {
    "reset",                                 //命令的文本
    "reset:复位所有系统并重启MCU\n",          //命令的帮助文本
    reset,                                   //命令的处理函数
    0                                        //参数的数量
};

//让蜂鸣器响一下
static BaseType_t buzz1(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
const CLI_Command_Definition_t comm_buzz1 = {
    "buzz1",                                //命令的文本
    "buzz1:闪烁一次板载LED\n",               //命令的帮助文本
    buzz1,                                  //命令的处理函数
    0                                       //参数的数量
};

//启动或关闭小车平衡
static BaseType_t blcSet(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
const CLI_Command_Definition_t comm_blcSet = {
    "blcSet",                               
    "blcSet <en>:小车平衡使能 <en>:1或0\n", 
    blcSet,                                 
    1                                       
};

//显示欧拉角数据流
static BaseType_t showEuler(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
const CLI_Command_Definition_t comm_showEuler = {
    "showEuler",                               
    "showEuler:显示欧拉角数据流(50Hz),输入stop停止\n", 
    showEuler,                                 
    0                                       
};

//停止任何的循环打印数据流
static BaseType_t stop(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
const CLI_Command_Definition_t comm_stop = {
    "stop",                               
    "stop:停止任何的循环打印数据流\n", 
    stop,                                 
    0                                       
};

//打印一次任务管理器
static BaseType_t mang(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
const CLI_Command_Definition_t comm_mang = {
    "mang",                               
    "mang:打印一次任务管理器\n", 
    mang,                                 
    0                                       
};

//输出系统所有PID的系数
static BaseType_t showPIDK(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
const CLI_Command_Definition_t comm_showPIDK = {
    "showPIDK",                               
    "showPIDK:打印系统所有PID的系数\n", 
    showPIDK,                                 
    0                                       
};

//调整直立环参数
static BaseType_t setStandPDK(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString);
const CLI_Command_Definition_t comm_setStandPDK = {
    "setStandPDK",                               
    "setStandPDK:设置直立环系数\n", 
    setStandPDK,                                 
    2                                       
};



void sAPP_CLI_Init(){

    FreeRTOS_CLIRegisterCommand(&comm_reset);
    FreeRTOS_CLIRegisterCommand(&comm_buzz1);
    FreeRTOS_CLIRegisterCommand(&comm_blcSet);
    FreeRTOS_CLIRegisterCommand(&comm_showEuler);
    FreeRTOS_CLIRegisterCommand(&comm_stop);
    FreeRTOS_CLIRegisterCommand(&comm_mang);
    FreeRTOS_CLIRegisterCommand(&comm_showPIDK);
    FreeRTOS_CLIRegisterCommand(&comm_setStandPDK);
}



static BaseType_t reset(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString){
    //不做操作,因为MCU马上就要复位了
    NVIC_SystemReset();
    return pdFALSE; //表示不需要更多的输出 命令已处理完毕
}

static BaseType_t buzz1(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString){
    const char *pcMessage = "buzz1:OK\n";
    strncpy(pcWriteBuffer, pcMessage, xWriteBufferLen);
    sHMI_BUZZER_StartSinglePulse();
    return pdFALSE;
}

static BaseType_t blcSet(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString){
    char *pcParameter;
    BaseType_t xParameterStringLength, xReturn;
    static UBaseType_t uxParameterNumber = 0;

    pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength);
    pcWriteBuffer[xParameterStringLength] = 0x00;  // 确保字符串结尾

    int para_int = atoi(pcParameter);
    if(para_int == 0){
        ctrl.blc_en = 0;
    }else{
        ctrl.blc_en = 1;
    }
    snprintf(pcWriteBuffer, xWriteBufferLen, "blc_set:%d OK\r\n", para_int);

    return pdFALSE;
}

extern sAPP_CLI_SendDataPacket_t g_cli_send_data_packet;

static BaseType_t showEuler(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString){

    g_cli_send_data_packet.send_en = 1;
    g_cli_send_data_packet.send_intervals_ms = 20;
    g_cli_send_data_packet.type = SAPP_CLI_SEND_DATA_TYPE_EULER;

    return pdFALSE;
}


static BaseType_t stop(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString){
    if(g_cli_send_data_packet.send_en == 1){
        g_cli_send_data_packet.send_en = 0;
        strncpy(pcWriteBuffer, "stop:已停止打印数据流\n", xWriteBufferLen);
    }else{
        strncpy(pcWriteBuffer, "stop:数据流已在停止状态\n", xWriteBufferLen);
    }

    return pdFALSE;
}



static BaseType_t mang(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString){ 
    
    sHMI_Debug_Printf("\n\n");

    sHMI_Debug_PrintTaskMang();

    return pdFALSE;
}

static BaseType_t showPIDK(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString){
    
    sHMI_Debug_Printf("\n\n");
    
    sHMI_Debug_Printf("StandPD : Kp = %8.4f,Kd = %8.4f\n", blc.stand_kp,blc.stand_kd);
    sHMI_Debug_Printf("SpdPI   : Kp = %8.4f,Ki = %8.4f\n", blc.spd_kp,blc.spd_ki);
    sHMI_Debug_Printf("TurnPD  : Kp = %8.4f,Kd = %8.4f\n", blc.turn_kp,blc.turn_kd);

    sHMI_Debug_Printf("ZeroPID : Kp = %8.4f,Ki = %8.4f,Kd = %8.4f\n", blc.inc_pos_kp,blc.inc_pos_ki,blc.inc_pos_kd);

    snprintf(pcWriteBuffer, xWriteBufferLen, "\n\n");
    
    return pdFALSE;
}


static BaseType_t setStandPDK(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString){
    char *pcParameter;
    BaseType_t xParameterStringLength, xReturn;
    static UBaseType_t uxParameterNumber = 0;

    /* 如果命令处理被调用第一次，则处理参数 */
    if (uxParameterNumber == 0)
    {
        /* 查找第一个参数 */
        pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength);
        pcWriteBuffer[xParameterStringLength] = 0x00;  // 确保字符串结尾

        blc.stand_kp = atof(pcParameter);

        /* 查找第二个参数 */
        pcParameter = FreeRTOS_CLIGetParameter(pcCommandString, 2, &xParameterStringLength);
        pcWriteBuffer[xParameterStringLength] = 0x00;  // 确保字符串结尾

        blc.stand_kd = atof(pcParameter);

        uxParameterNumber++;
        xReturn = pdTRUE;  // 告诉CLI还有更多的输出要返回
    }
    else
    {
        /* 所有参数已处理，重置参数编号 */
        uxParameterNumber = 0;
        xReturn = pdFALSE;  // 告诉CLI没有更多输出
        snprintf(pcWriteBuffer, xWriteBufferLen, "setStandPDK: 已设置直立环系数: Kp=%8.4f, Kd=%8.4f\n", blc.stand_kp, blc.stand_kd);
    
    }   

    return xReturn;
}