#ifndef __EXPOSURE_H
#define __EXPOSURE_H
#include "stdint.h"

typedef enum {
    SOURCE_A = 0,
    SOURCE_B
} ExposureSource;

void Exposure_Start(ExposureSource src);         //启动曝光流程
void Exposure_Stop(ExposureSource src);          //停止曝光
uint8_t Exposure_IsDone(uint32_t dummy);         //曝光是否完成
void Exposure_Tick_Handler(void);                //1ms定时器调度函数

#endif
