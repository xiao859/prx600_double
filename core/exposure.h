#ifndef __EXPOSURE_H
#define __EXPOSURE_H
#include "stdint.h"



#include <stdint.h>


#define EXPO_MAXTIME_PROTECT        1000  

#define     Is_PulseMode_CT()           ((ctrl_data.xrayMode) == XRAY_MODE_PULSE)

#define     Is_ContinuousMode_CT()      ((ctrl_data.xrayMode) == XRAY_MODE_CONTINUOUS)



typedef struct
{
    uint32_t timmer_count;   


    uint32_t interLock_count;
    uint32_t pwr_24_overCount;
    uint32_t pwr_24_underCount;
    uint32_t tube_kv_overCount;
    uint32_t tube_kv_underCount;
    uint32_t tube_mA_overCount;
    uint32_t tube_mA_peak_overCount;
    uint32_t tube_mA_underCount;
    uint32_t fila_vol_overCount;
    uint32_t fila_vol_underCount;
    uint32_t fila_curr_overCount;
    uint32_t fila_curr_underCount;
    uint32_t sink_temp_errCount;
    uint32_t oil_temp_errCount;
    uint32_t oil_temp_warnCount;

    uint32_t tube_vol_broken_count;
    uint32_t tube_curr_broken_count;

    uint32_t tube_strike_count;    
    uint32_t tube_strike_times;     
    uint32_t oil_strike_count;      
    uint32_t oil_strike_times;      

    uint32_t isCheckAvailable;  
} xray_running_data;
extern volatile xray_running_data xray_data;

/* 4¡¢º¯ÊýÉùÃ÷ */
void ct_task(void);
void xray_CT_disable(void);


typedef enum {
    SOURCE_A = 0,
    SOURCE_B
} ExposureSource;

void Exposure_Start(ExposureSource src);         //启动曝光流程
void Exposure_Stop(ExposureSource src);          //停止曝光
uint8_t Exposure_IsDone(uint32_t dummy);         //曝光是否完成
void Exposure_Tick_Handler(void);                //1ms定时器调度函数

#endif
