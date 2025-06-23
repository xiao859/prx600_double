#ifndef __PI_CONTROL_H
#define __PI_CONTROL_H

#include "exposure.h"

void PI_Control_Current(ExposureSource src, float i_target); // 电流闭环控制

#endif
