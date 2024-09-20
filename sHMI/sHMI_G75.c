#include "sHMI_G75.h"

//用于集芯微的G75蓝牙模块的通信
//2024.06.17


#define BT_SHOW_NAME "miniBlcCARK"

char recied_buf[128];

//todo 串口初始化
static void uart_init(uint32_t baudrate){
    sBSP_UART6_Init(baudrate);
}

//todo 串口发送函数
static void uart_send(uint8_t* pData,uint16_t length){
    sBSP_UART6_SendBytes(pData,length);
}

//todo 串口接收函数
static void uart_read_begin(reci_data_end_cb_t cb){
    sBSP_UART6_ReadBytesBegin(cb);
}

//我的串口printf函数
void uart_printf(char *p,...){
    static char format_buf[128];

    va_list ap;
    va_start(ap,p);
    vsprintf(format_buf,p,ap);
    va_end(ap);
    
    uart_send((uint8_t*)format_buf,strlen(format_buf));
}


////todo 串口接收回调函数
//static void uart_recied_cb(uint8_t* pReciData,uint16_t length){
//    // if(sscanf(pReciData,"S:%s",&recied_buf) == 1){
//    //     sHMI_Debug_Printf("OK\n");
//    //     sHMI_Debug_Printf("%s\n",recied_buf);
//    // }

//    // if(sscanf(pReciData, "SPD:%f,MOVE:%f,HEAD:%f",\
//    //             &ctrl.tar_spd,&ctrl.tar_move,&ctrl.tar_head) == 3){
//        
//    //     sHMI_BUZZER_StartSinglePulse();
//    //     uart_printf("SETTED OK\n");
//    //     sHMI_Debug_Printf("SETTED OK\n");
//    // }
//    float head = 0;

//    if(sscanf(pReciData, "SPD:%f",&ctrl.tar_spd) == 1){
//        sHMI_BUZZER_StartSinglePulse();
//        uart_printf("SETTED OK\n");
//        sHMI_Debug_Printf("SETTED OK\n");
//    }
//    else if(sscanf(pReciData, "HEAD:%f",&head) == 1){
//        ctrl.tar_head +=head;
//        if(ctrl.tar_head > 360){
//            ctrl.tar_head -= 360;
//        }
//        sHMI_BUZZER_StartSinglePulse();
//        uart_printf("SETTED OK\n");
//        sHMI_Debug_Printf("SETTED OK\n");
//    }

//    //重新开始新的接收
//    //uart_read_begin(uart_recied_cb);
//}






void sHMI_G75_IntoTransMode(){
    //进入透传模式
    uart_printf("AT+BT_TRANS=1\n");
}



void sHMI_G75_Init(){
    //串口初始化
    uart_init(115200);

    //修改蓝牙名称
    uart_printf("AT+BT_NAME=%s\n",BT_SHOW_NAME);
    //设置射频功率11.5dBm:5, 10dBm:4, 4dBm:3, 0dBm:2, -8dBm:1, -20dBm:0
    uart_printf("AT+RF_POWER=5\n");
    //进入透传模式
    uart_printf("AT+BT_TRANS=1\n");

    //开始接收数据
    //uart_read_begin(uart_recied_cb);
}


