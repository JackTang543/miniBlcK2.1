#ifndef __SAPP_CLI_H__
#define __SAPP_CLI_H__
#ifdef __cplusplus
extern "C" {
#endif


#include "main.h"

typedef enum{
    SAPP_CLI_SEND_DATA_TYPE_NONE  = 0,
    SAPP_CLI_SEND_DATA_TYPE_EULER = 1,

}sAPP_CLI_SendDataType_t;

typedef struct{
    sAPP_CLI_SendDataType_t type;
    uint16_t send_intervals_ms;
    bool send_en;
}sAPP_CLI_SendDataPacket_t;


void sAPP_CLI_Init();



#ifdef __cplusplus
}
#endif
#endif
