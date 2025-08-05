#ifndef __DELAY_H
#define __DELAY_H


#include "stdint.h"
#include "sys.h"


/*50kHz */
#define COUNTER_TIMER6_FREQ                  20000

/*2.5s*/
#define TIMER6_2P5_SECOND_CYCLES                (uint32_t)(2.5 * COUNTER_TIMER6_FREQ)

/*6ms*/
#define TIMER6_8_MILSECOND_CYCLES              (uint32_t)(0.006 * COUNTER_TIMER6_FREQ)

/*5ms*/
#define TIMER6_5_MILSECOND_CYCLES              (uint32_t)(0.005 * COUNTER_TIMER6_FREQ)

/*10ms*/
#define TIMER6_10_MILSECOND_CYCLES              (uint32_t)(0.01 * COUNTER_TIMER6_FREQ)

/*20ms*/
#define TIMER6_20_MILSECOND_CYCLES              (uint32_t)(0.02 * COUNTER_TIMER6_FREQ)

/*5s*/
#define TIMER6_2_SECOND_CYCLES                  (uint32_t)(2 * COUNTER_TIMER6_FREQ)

/*4ms*/
#define TIMER6_4_MILSECOND_CYCLES                (uint32_t)(0.004 * COUNTER_TIMER6_FREQ)

/*6ms*/
#define TIMER6_6_MILSECOND_CYCLES                (uint32_t)(0.006 * COUNTER_TIMER6_FREQ)

/*12ms*/
#define TIMER6_12_MILSECOND_CYCLES                (uint32_t)(0.012 * COUNTER_TIMER6_FREQ)

/*500ms*/
#define TIMER6_500_MILSECOND_CYCLES              (uint32_t)(0.1 * COUNTER_TIMER6_FREQ)

/*1ms*/
#define TIMER6_1_MILSECOND_CYCLES                (uint32_t)(0.001 * COUNTER_TIMER6_FREQ)

/*0.5ms*/
#define TIMER6_0P5_MILSECOND_CYCLES                (uint32_t)(0.0005 * COUNTER_TIMER6_FREQ)

/*0.1ms*/
#define TIMER6_0P1_MILSECOND_CYCLES                (uint32_t)(0.0004 * COUNTER_TIMER6_FREQ)




void delay_init(uint16_t sysclk);          
void delay_ms(uint16_t nms);                
void delay_us(uint32_t nus);                

void HAL_Delay(uint32_t Delay);            
uint32_t get_tick_ms(void);
#endif

