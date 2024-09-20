#include "sAPP_Task.h"

void sAPP_Task_OLEDUpdate(void* pvPara){
    
    for(;;){
        
        char buf[30];
        sG2D_WriteString(1, 2, "miniBlcCAR-K");
        sG2D_RevRectArea(0,0,127,10);

        snprintf(buf,sizeof(buf),"Pitch:%8.2f Deg",ahrs.pitch);
        sG2D_WriteString(0,20,buf);
        snprintf(buf,sizeof(buf),"GyroX:%8.2f Deg/s",ahrs.gyro_x);
        sG2D_WriteString(0,30,buf);
        snprintf(buf,sizeof(buf),"Power:%8.2f %%",(blc.left_pwm + blc.right_pwm) / 2);
        sG2D_WriteString(0,40,buf);
        snprintf(buf,sizeof(buf),"Yaw:  %8.2f Deg",ahrs.yaw);
        sG2D_WriteString(0,50,buf);
        

        
        
        
        sBSP_DWT_MeasureStart();
        sG2D_UpdateScreen();        //30Hz
        sBSP_DWT_MeasureEnd();
        
        //sG2D_SetAllGRAM(0);
        sG2D_FastSetAllGRAM(0);
        

        //sHMI_Debug_Printf("%uus\n",sBSP_DWT_GetMeasure_us());
        
        vTaskDelay(33 / portTICK_PERIOD_MS);
    }
}

//AHRS的姿态解算和数据更新200Hz(5ms)
void sAPP_Task_AHRSUpdate(void* pvPara){
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    for(;;){
        //AHRS从传感器获取姿态数据
        sAPP_AHRS_DataUpdate();
        //AHRS进行姿态解算
        sAPP_AHRS_EstiUpdate();

        //高精确度延时5ms
        xTaskDelayUntil(&xLastWakeTime,5 / portTICK_PERIOD_MS);
    }
}

//小车平衡控制任务100Hz(10ms)
void sAPP_Task_BlcCtrl(void* pvPara){
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    for(;;){
        //轮胎编码器读取
        sDRV_GMR_Handler();
        //小车平衡控制算法
        sAPP_CTRL_BlcHandler();

        //高精确度延时10ms
        xTaskDelayUntil(&xLastWakeTime,10 / portTICK_PERIOD_MS);
    }
}

void sAPP_Task_UARTPrint(void* pvPara){
    for(;;){    //20Hz
        //sG2D_SetAllGRAM(0);
        //sAPP_AHRS_Print();
        //sHMI_Debug_Printf("%.2f,%.2f,%.2f,%.2f\n",ahrs.pitch,ahrs.roll,ahrs.yaw,ahrs.gyro_x);
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void sAPP_Task_10ms(void* pvPara){
    for(;;){
        sHMI_LED_Handler();
        sHMI_BUZZER_Handler();
        sGBD_Handler();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void sAPP_Task_50ms(void* pvPara){
    for(;;){

        float amp_a = sDRV_INA219_GetCurrA();
        float rs_v = sDRV_INA219_GetRshuntV();
        float bus_v = sDRV_INA219_GetBusV();
        float pwr_w = sDRV_INA219_GetPwrW();

        //sHMI_Debug_Printf("amp_a:%.6f, rs_v:%.6f, bus_v:%.6f, pwr_w:%.6f,calc_ma:%.2f\n", amp_a, rs_v, bus_v, pwr_w,(rs_v * 1000)/50*1000);



        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void sAPP_Task_100ms(void* pvPara){
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();
    for(;;){
        //sDRV_GMR_Handler();
        vTaskDelay(100 / portTICK_PERIOD_MS);
        //xTaskDelayUntil(&xLastWakeTime,100 / portTICK_PERIOD_MS);
    }
}


void sAPP_Task_500ms(void* pvPara){
    for(;;){
        sHMI_LED_StartSinglePulse();
        //sHMI_Debug_Printf("left: %.2f, right: %.2f\n",sDRV_GMR_GetLeftRPM(),sDRV_GMR_GetRightRPM());
        //sHMI_BUZZER_StartSinglePulse();
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void sAPP_Task_1000ms(void* pvPara){
    for(;;){
        // RTC_TimeTypeDef time;
        // sBSP_RTC_GetTime(&time);
        // RTC_DateTypeDef date;
        // sBSP_RTC_GetDate(&date);

        // sHMI_Debug_Printf("RTC:%04d-%02d-%02d %02d:%02d:%02d\n",(uint16_t)date.Year + 2000,date.Month,\
        // date.Date,time.Hours,time.Minutes,time.Seconds);
        
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void sAPP_Task_TaskMang(void* pvPara){
    for(;;){
        sHMI_Debug_PrintTaskMang();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void sAPP_Task_CLIReci(void* pvPara){
    char* p_str_buf;
    BaseType_t more_data_flag;

    for(;;){
        //如果串口收到数据了
        if(xSemaphoreTake(g_sem_debug_uart_recied,portMAX_DELAY) == pdTRUE){
            p_str_buf = (char*)g_UART1_RxBuf;
            //确保字符串以0结尾
            p_str_buf[UART1_RX_BUF_SIZE - 1] = '\0';

            // 处理接收到的每一条命令
            do {
                // 调用 FreeRTOS+CLI 的命令处理器
                more_data_flag = FreeRTOS_CLIProcessCommand(
                    (const char*)p_str_buf,         // 命令字符串
                    cOutputBuffer,           // 命令输出缓冲区
                    sizeof(cOutputBuffer)    // 缓冲区大小
                );

                // 将命令处理的输出发送回UART
                sHMI_Debug_Printf("%s", cOutputBuffer);

            } while (more_data_flag != pdFALSE);  // 如果还有更多输出，继续处理

            // 清空接收缓冲区，准备下一次接收
            memset(g_UART1_RxBuf, 0, UART1_RX_BUF_SIZE);

        }
    }
}

extern sAPP_CLI_SendDataPacket_t g_cli_send_data_packet;

void sAPP_Task_CLISend(void* pvPara){

    for(;;){
        if(g_cli_send_data_packet.send_en == 1){
            //需要发送数据了

            //发送欧拉角
            if(g_cli_send_data_packet.type == SAPP_CLI_SEND_DATA_TYPE_EULER){
                sHMI_Debug_Printf("%.2f,%.2f,%.2f\n",ahrs.pitch,ahrs.roll,ahrs.yaw);
            }

            vTaskDelay(g_cli_send_data_packet.send_intervals_ms / portTICK_PERIOD_MS);
        }else{
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
}

