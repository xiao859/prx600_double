#ifndef __PI_CTL_H
#define __PI_CTL_H

/* 1、头文件包含 */
#include "stdint.h"

/* 2、宏定义 */


/* 3、数据类型定义 */
/* 参数 */
typedef struct
{
    float currTarget;
    float currValue;
    float Kp;
    float Ti;
} User_PID;
extern User_PID   user_pid;

/* 过程数据 */
typedef struct
{
    float integral;
    float Error;
    float Out_pid;
    float coeff;
    uint32_t config_ref;

    uint8_t  pulse_pi_flag;     /* 脉冲模式，PI开始标志 */
    uint32_t  pulse_count;      /* 脉冲个数计数 */

    uint8_t  pi_flag;           /* 连续模式，PI开始标志 */
    uint8_t  pi_fast_flag;      /* 连续模式快速调节标志 */
    uint8_t  ki_flag;           /* 积分项开始标志 */
    uint32_t ti_CycleCount;     /* 计数，用于开启积分项 */

    uint32_t threshold_offset;  /* 限幅门限值调整，因为连续模式，电流值逐渐减小，需要实时调整限幅 */
    uint32_t conu_start_ref;    /* 连续开始时的基准值，限幅基于此值，逐渐上调 */

    uint32_t thresh_achieve_count;  /*  达到门限值后的计数，超过一定次数，开始上调门限 */

} PARAM_PID;
extern PARAM_PID  param_pid;

typedef struct
{
    float currTarget[2];
    float currValue[2];
    float Kp;
    float Ki;
    float err[2];
    float last_err[2];
    uint32_t config_ref[2];
} User_PID_2;
extern User_PID_2   user_pid_2;

/* 4、函数声明 */
void tube_current_piControl(uint8_t conflag,uint8_t n);
void pid_Init(float target, uint32_t ref_init, uint8_t isPulseMode);
void tube_current_pid_pulseInit(uint8_t n);
void tube_current_piControl_v2(uint8_t n);
void pid_Init_2(float target, uint32_t ref_init, uint8_t isPulseMode,uint8_t n);


#endif

