#include "sBSP_RTC.h"

RTC_HandleTypeDef hrtc;

RTC_TimeTypeDef time = {0};
RTC_DateTypeDef date = {0};

void sBSP_RTC_Init(){
    hrtc.Instance = RTC;
    hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
    hrtc.Init.AsynchPrediv = 127;
    hrtc.Init.SynchPrediv = 255;
    hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
    hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
    HAL_RTC_Init(&hrtc);
}

void sBSP_RTC_SetTime(uint8_t hour,uint8_t min,uint8_t sec){
    time.Hours = hour;
    time.Minutes = min;
    time.Seconds = sec;
    HAL_RTC_SetTime(&hrtc,&time,RTC_FORMAT_BIN);
}

//year(eg:2024),RTC_Month_Date_Definitions,day,RTC_WeekDay_Definitions
void sBSP_RTC_SetDate(uint16_t year,uint8_t month,uint8_t day,uint8_t weekday){
    date.Year = year - 2000;
    date.Month = month;
    date.Date = day;
    date.WeekDay = weekday;
    HAL_RTC_SetDate(&hrtc,&date,RTC_FORMAT_BIN);
}

void sBSP_RTC_GetTime(RTC_TimeTypeDef* param_time){
    HAL_RTC_GetTime(&hrtc,&time,RTC_FORMAT_BIN);
    param_time->Hours = time.Hours;
    param_time->Minutes = time.Minutes;
    param_time->Seconds = time.Seconds;
}

void sBSP_RTC_GetDate(RTC_DateTypeDef* param_date){
    HAL_RTC_GetDate(&hrtc,&date,RTC_FORMAT_BIN);
    param_date->Year = date.Year;
    param_date->Month = date.Month;
    param_date->Date = date.Date;
    param_date->WeekDay = date.WeekDay;
}


void HAL_RTC_MspInit(RTC_HandleTypeDef* rtcHandle){
    if(rtcHandle->Instance == RTC){
        __HAL_RCC_RTC_ENABLE();   
        HAL_PWR_EnableBkUpAccess();
        RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
        PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
        HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
    }
}
