#include "sHMI_Debug.h"

#include "sBSP_UART.h"



//对外提供:debug初始化
void sHMI_Debug_Init(uint32_t bandrate){
    sBSP_UART1_Init(bandrate);
}

char format_buffer[128] = {0};

void sHMI_Debug_Printf(char *p,...){
    va_list ap;
    va_start(ap,p);
    vsprintf(format_buffer,p,ap);
    va_end(ap);
    
    sBSP_UART1_SendBytes((uint8_t*)format_buffer,strlen(format_buffer));
}


void sHMI_Debug_PrintTaskMang(){
    const char task_state[]={'r','R','B','S','D'};
    volatile UBaseType_t uxArraySize, x;
    unsigned portSHORT ulTotalRunTime,ulStatsAsPercentage;
    /* 获取任务总数目 */
    uxArraySize = uxTaskGetNumberOfTasks();
    sHMI_Debug_Printf("当前任务数量:%d\n",uxArraySize);
    TaskStatus_t pxTaskStatusArray[16];
    /*获取每个任务的状态信息 */
    uxArraySize = uxTaskGetSystemState(pxTaskStatusArray,uxArraySize,&ulTotalRunTime);
    sHMI_Debug_Printf("任务名        状态       ID       优先级       堆栈剩余      CPU使用率\n");
    /* 避免除零错误 */
    if( ulTotalRunTime > 0 ){
        /* 将获得的每一个任务状态信息部分的转化为程序员容易识别的字符串格式 */
        for(x = 0; x < uxArraySize; x++){
            char tmp[128];
            /* 计算任务运行时间与总运行时间的百分比。*/
            ulStatsAsPercentage =(uint16_t)(pxTaskStatusArray[x].ulRunTimeCounter)*100 / ulTotalRunTime;
 
            if( ulStatsAsPercentage > 0UL ){
               sprintf(tmp,"%-15s%-10c%-10lu%-12lu%-12d%d%%",pxTaskStatusArray[ x].pcTaskName,task_state[pxTaskStatusArray[ x ].eCurrentState],
                                                             pxTaskStatusArray[ x ].xTaskNumber,pxTaskStatusArray[ x].uxCurrentPriority,
                                                             pxTaskStatusArray[ x ].usStackHighWaterMark,ulStatsAsPercentage);
            }
            else{
                /* 任务运行时间不足总运行时间的1%*/
                sprintf(tmp,"%-15s%-10c%-10lu%-12lu%-12dt<1%%",pxTaskStatusArray[x ].pcTaskName,task_state[pxTaskStatusArray[ x ].eCurrentState],
                                                               pxTaskStatusArray[ x ].xTaskNumber,pxTaskStatusArray[ x].uxCurrentPriority,
                                                               pxTaskStatusArray[ x ].usStackHighWaterMark);               
            }
           sHMI_Debug_Printf("%s\n",tmp);
        }
    }
    sHMI_Debug_Printf("任务状态:   r-运行  R-就绪  B-阻塞  S-挂起  D-删除\n");
}



