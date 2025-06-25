#ifndef _CALIBRATE_H
#define _CALIBRATE_H


/* 1、头文件包含 */
#include <stdint.h>
#include "comm_protocol.h"
#include "comm_string.h"

/* 2、宏定义 */
#define     CALI_HV_REF                     80

#define     CALI_SINGLE_CURR_TIME           2.4 //2.04

/* 脉冲模式每个脉冲的时间长度，单位ms */
#define     CALI_PULSE_CURR_EXPO_TIME       15
/* 脉冲模式的周期，单位ms */
#define     CALI_PULSE_SIGLE_CURR_PERIOD    120

#define     Is_PulseMode()                  ((cali_data.mode) == XRAY_MODE_PULSE)

#define     Is_ContinuousMode()             ((cali_data.mode) == XRAY_MODE_CONTINUOUS)

/* 3、数据类型定义 */
typedef struct {
    xray_mode   mode;                   /* 校准模式：脉冲0或者连续1 */
    uint8_t     curr_index;             /* 校准阶段0~9代表 */
    uint8_t     finished_flag;          /* 结束标记 */

    float       tube_vol_realtime;
    float       tube_vol_step;

    uint32_t    timmer_count;           /* 计时，用于冷却或者控制曝光 */
    uint32_t    cycle_count;            /* 计数，单毫安循环 */

    uint32_t    expoTime_expect;        /* 设置的曝光时间，单个电流下，每个脉冲的持续时间 */
    uint32_t    coolTime_expect;        /* 设置的冷却时间，单个电流下，每个脉冲间的冷却时间 */
    uint32_t    expoCycle_perCurrent;   /* 校准，单毫安曝光次数 */

    uint32_t    para_save_flag;
} xray_calibrate_data;
extern volatile xray_calibrate_data cali_data;

/* 4、函数声明 */
void calibrate_para_init(void);
void calibrate_task(void);

#endif
