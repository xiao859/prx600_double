#ifndef _DEBUG_MODE_H
#define _DEBUG_MODE_H


/* 1、头文件包含 */
#include "stdint.h"
#include "comm_protocol.h"

/* 2、宏定义 */
/* 管电流基准值，单位0.001v */
#define IDLE_FILAMENT_REF_DEBUG       0

/* 3、数据类型定义 */

typedef struct {
    uint32_t    timmer_count[XRAY_NUMS];           /* 计时，用于冷却或者控制曝光 */
    uint32_t    cycle_count[XRAY_NUMS];            /* 计数，单毫安循环 */

    uint32_t    expoTime_expect[XRAY_NUMS];        /* 设置的曝光时间，单个电流下，每个脉冲的持续时间 */
    uint32_t    coolTime_expect[XRAY_NUMS];        /* 设置的冷却时间，单个电流下，每个脉冲间的冷却时间 */
    uint32_t    expoCycle_perCurrent[XRAY_NUMS];   /* 校准，单毫安曝光次数 */
} xray_debug_data;
extern volatile xray_debug_data debug_data;

/* 4、函数声明 */
void debug_task(void);
void xray_HV_enable_debug(uint16_t value);

#endif
