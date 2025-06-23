#ifndef __PROTECT_H
#define __PROTECT_H

#include "exposure.h"

void Protect_Check_Quick(void);            // 中断中快速检测（互锁/过流）
void Protect_Check_Slow(void);             // 主循环中慢速检测
unsigned char Protect_GetTrigger(ExposureSource src);  //获取曝光状态

#endif

