#ifndef __XRAY_H
#define __XRAY_H

#include "exposure.h"

void Xray_SetVoltage(uint16_t kv);              // 设置Xray高压参考
void Xray_Enable(ExposureSource src);           // 打开高压通道
void Xray_Disable(ExposureSource src);          // 关闭高压通道

#endif
