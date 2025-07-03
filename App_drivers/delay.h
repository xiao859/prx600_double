#ifndef __DELAY_H
#define __DELAY_H

/* 1、头文件包含 */
#include "stdint.h"
#include "sys.h"
/* 2、宏定义 */

/* 定时器6的频率为50kHz */
#define COUNTER_TIMER6_FREQ                  50000

/* 延时2.5秒需要的时钟周期 */
#define TIMER6_2P5_SECOND_CYCLES                (uint32_t)(2.5 * COUNTER_TIMER6_FREQ)

/* 延时8ms秒需要的时钟周期 */
#define TIMER6_8_MILSECOND_CYCLES              (uint32_t)(0.006 * COUNTER_TIMER6_FREQ)

/* 延时5ms秒需要的时钟周期 */
#define TIMER6_5_MILSECOND_CYCLES              (uint32_t)(0.005 * COUNTER_TIMER6_FREQ)

/* 延时10ms秒需要的时钟周期 */
#define TIMER6_10_MILSECOND_CYCLES              (uint32_t)(0.01 * COUNTER_TIMER6_FREQ)

/* 延时20ms秒需要的时钟周期 */
#define TIMER6_20_MILSECOND_CYCLES              (uint32_t)(0.02 * COUNTER_TIMER6_FREQ)

/* 延时5秒需要的时钟周期 */
#define TIMER6_5_SECOND_CYCLES                  (uint32_t)(5 * COUNTER_TIMER6_FREQ)

/* 4m秒需要的时钟周期 */
#define TIMER6_4_MILSECOND_CYCLES                (uint32_t)(0.004 * COUNTER_TIMER6_FREQ)

/* 6m秒需要的时钟周期 */
#define TIMER6_6_MILSECOND_CYCLES                (uint32_t)(0.006 * COUNTER_TIMER6_FREQ)

/* 12m秒需要的时钟周期 */
#define TIMER6_12_MILSECOND_CYCLES                (uint32_t)(0.012 * COUNTER_TIMER6_FREQ)

/* 延时500ms秒需要的时钟周期 */
#define TIMER6_500_MILSECOND_CYCLES              (uint32_t)(0.1 * COUNTER_TIMER6_FREQ)

/* 1m秒需要的时钟周期 */
#define TIMER6_1_MILSECOND_CYCLES                (uint32_t)(0.001 * COUNTER_TIMER6_FREQ)

/* 0.5m秒需要的时钟周期 */
#define TIMER6_0P5_MILSECOND_CYCLES                (uint32_t)(0.0005 * COUNTER_TIMER6_FREQ)

/* 0.1m秒需要的时钟周期 */
#define TIMER6_0P1_MILSECOND_CYCLES                (uint32_t)(0.0004 * COUNTER_TIMER6_FREQ)

/* 3、数据类型定义 */

/* 4、函数声明 */
void delay_init(uint16_t sysclk);           /* 初始化延迟函数 */
void delay_ms(uint16_t nms);                /* 延时nms */
void delay_us(uint32_t nus);                /* 延时nus */

void HAL_Delay(uint32_t Delay);             /* HAL库的延时函数，SDIO等需要用到 */
uint32_t get_tick_ms(void);
#endif

