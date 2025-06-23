#ifndef __FILAMENT_H
#define __FILAMENT_H

#include "exposure.h"

void Filament_Enable(ExposureSource src);        // 打开灯丝输出
void Filament_Disable(ExposureSource src);       // 关闭灯丝输出
void Filament_SetCurrent(ExposureSource src, float amp);  // 设置灯丝电流

#endif
