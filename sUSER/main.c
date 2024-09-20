#include "main.h"


extern UART_HandleTypeDef huart6;

//串口接收到数据信号量
SemaphoreHandle_t g_sem_debug_uart_recied;


sAPP_CLI_SendDataPacket_t g_cli_send_data_packet;

void uart_recied(char* pReciData,uint16_t length){
    static BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xSemaphoreGiveFromISR(g_sem_debug_uart_recied, &xHigherPriorityTaskWoken);

    sBSP_UART1_ReadBytesBegin(uart_recied);
    
    //请求上下文切换，如果有更高优先级的任务被唤醒
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// void uart6_recied(uint8_t* pReciData,uint16_t length){
//     //把串口6接收的数据转发到调试串口
//     sHMI_Debug_Printf("%s\n",pReciData);
//     sBSP_UART6_ReadBytesBegin(uart6_recied);
// }


//todo 优化控制算法结构,全局变量优化,CLI调整4个PID系数,优化上层对小车的控制方式
//没有输入时小车保持静止
//输入速度加入编码器的负反馈解决实际速度不符和有障碍物就卡住

//使用DMA传输串口,SPI屏幕
//把CLI映射到蓝牙

//OPENMV巡线优化,提高速度和稳定性




int main(){
    //初始化系统
    sBSP_SYS_Init();
    //初始化DWT计时单元
    sBSP_DWT_Init(HAL_RCC_GetSysClockFreq());
    //初始化调试串口
    sHMI_Debug_Init(115200);
    sBSP_UART1_ReadBytesBegin(uart_recied);

    uint32_t freq = HAL_RCC_GetSysClockFreq();
    sHMI_Debug_Printf("System Clock:%dHz\n",freq);
    sHMI_Debug_Printf("UART Debug text encoding by UTF-8\n\n");

    //初始化串口CLI
    sAPP_CLI_Init();
    sHMI_Debug_Printf("CLI串口命令行初始化完成\n");
    
    float32_t a = -14.1;
    float32_t b = 0;
    arm_abs_f32(&a, &b,1);

    //初始化按键
    sAPP_Func_BtnInit();
    //初始化蜂鸣器
    sHMI_BUZZER_Init();
    sHMI_BUZZER_SetMode(SHMI_BUZZER_MODE_PULSE_SINGLE);
    sHMI_BUZZER_SetFreq(1000);
    sHMI_BUZZER_SetPulseTime_ms(50,100);
    
    //初始化惯导单元
    sAPP_Func_IMUInit();
    //初始化航姿参考系统
    sAPP_AHRS_Init();
    //初始化电机控制
    sAPP_Motor_Init();
    //初始化平衡算法
    sAPP_CTRL_BlcInit();

    sBSP_DMA2S0_Init();
    sBSP_DMA2S1_Init();

    //初始化OLED
    sDRV_GenOLED_Init();
    //清空OLED屏幕
    sG2D_UpdateScreen();
    //初始化蓝牙
    sHMI_G75_Init();

    g_sem_debug_uart_recied = xSemaphoreCreateBinary();

    // __GPIOA_CLK_ENABLE();
    // GPIO_InitTypeDef gpio = {0};
    // gpio.Pin       = GPIO_PIN_8;
    // gpio.Mode      = GPIO_MODE_OUTPUT_PP;
    // gpio.Pull      = GPIO_NOPULL;
    // gpio.Speed     = GPIO_SPEED_FREQ_LOW;
    // gpio.Alternate = 0;
    // HAL_GPIO_Init(GPIOA, &gpio);

    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

    sDRV_PwrLED_Init();
    sDRV_PwrLED_SetBrightness(1);
    sDRV_PwrLED_SetEN(1);

    HAL_Delay(10);
    

    sDRV_INA219_Init();

    sAPP_AHRS_InitStaticBias();


    //OLED刷屏任务
    xTaskCreate(sAPP_Task_OLEDUpdate        , "OLEDUp"      ,  256, NULL, 1, NULL);
    //AHRS姿态获取和解算
    xTaskCreate(sAPP_Task_AHRSUpdate        , "AHRSUp"      ,  512, NULL, 4, NULL);
    //控制平衡
    xTaskCreate(sAPP_Task_BlcCtrl           , "BlcCtrl"     ,  512, NULL, 4, NULL);
    //串口打印数据
    xTaskCreate(sAPP_Task_UARTPrint         , "UARTPrint"   ,  512, NULL, 2, NULL);
    //10ms 100Hz任务
    xTaskCreate(sAPP_Task_10ms              , "10ms"        ,  128, NULL, 1, NULL);
    //50ms 20Hz任务
    xTaskCreate(sAPP_Task_50ms              , "50ms"        ,  512, NULL, 2, NULL);
    //500ms 2Hz任务
    xTaskCreate(sAPP_Task_500ms             , "500ms"       ,  128, NULL, 1, NULL);
    //1000ms 1Hz任务
    xTaskCreate(sAPP_Task_1000ms            , "1000ms"      ,  128, NULL, 1, NULL);

    //CLI接收数据任务
    xTaskCreate(sAPP_Task_CLIReci           , "CLIReci"     ,  512, NULL, 2, NULL);
    //CLI发送数据任务
    xTaskCreate(sAPP_Task_CLISend           , "CLISend"     ,  256, NULL, 1, NULL);
    
    //任务管理器打印任务
    //xTaskCreate(sAPP_Task_TaskMang          , "TaskMang"    , 512, NULL, 1, NULL);

    sHMI_Debug_Printf("FreeRTOS启动任务调度\n");
    vTaskStartScheduler();
    while(1){
        float amp_a = sDRV_INA219_GetCurrA();
        float rs_v = sDRV_INA219_GetRshuntV();
        float bus_v = sDRV_INA219_GetBusV();
        float pwr_w = sDRV_INA219_GetPwrW();

        sHMI_Debug_Printf("amp_a:%.6f, rs_v:%.6f, bus_v:%.6f, pwr_w:%.6f,calc_ma:%.2f\n", amp_a, rs_v, bus_v, pwr_w,(rs_v * 1000)/50);


        HAL_Delay(50);

    }
}



void Error_Handler(){
    __disable_irq();
    while (1){
        sHMI_Debug_Printf("WARNING警告:错误! 禁用IRQ,死循环...\n");
        HAL_Delay(500);
    }
}

void assert_failed(uint8_t* file, uint32_t line){
    __disable_irq();
    while (1){
        sHMI_Debug_Printf("WARNING警告:断言! 文件名:%s,行:%u 禁用IRQ,死循环...\n",file,line);
        HAL_Delay(500);
    }
}

void vApplicationMallocFailedHook(){
    sHMI_Debug_Printf("WARNING警告:内存申请失败! 禁用IRQ\n");
}

void vApplicationIdleHook(){
    
}   

void vApplicationTickHook(){
}


void vApplicationStackOverflowHook(TaskHandle_t xTask,char* pcTaskName){
    sHMI_Debug_Printf("WARNING警告:触发栈金丝雀机制! 任务句柄:0x%X,任务名:%s\n",xTask,pcTaskName);
}

