#ifndef __PI_CONTROL_H
#define __PI_CONTROL_H

#include "exposure.h"

typedef struct
{
	float currTarget;
	float currValue;
	float Kp;
	float Ti;
}User_PID;
extern User_PID   user_pid;

/* 过程数据 */
typedef struct
{
	float integral;
	float Error;
	float lastError;
	float Out_pid;
    float coeff;
    uint32_t config_ref;
}PARAM_PID;
extern PARAM_PID  param_pid;

/* 4、函数声明 */
void tube_current_piControl(uint8_t conflag);
void pid_Init(float target, uint32_t ref_init, uint8_t isPulseMode);

void PI_Control_Current(ExposureSource src, float i_target); // 电流闭环控制

#endif
