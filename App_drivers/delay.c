/**
 *
 */

#include "delay.h"


static uint32_t g_fac_us = 0;       

/**

 */
void delay_init(uint16_t sysclk)
{
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
    g_fac_us = sysclk;                                 
}


void delay_us(uint32_t nus)
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;
    uint32_t reload = SysTick->LOAD;        
    ticks = nus * g_fac_us;                 

    told = SysTick->VAL;                   
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;       
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;                      
            }
        }
    }

}


void delay_ms(uint16_t nms)
{
    delay_us((uint32_t)(nms * 1000));                   
}


void HAL_Delay(uint32_t Delay)
{
     delay_ms(Delay);
}

// 获取系统启动以来的毫秒数（来自 SysTick 中断累计）
uint32_t get_tick_ms()
{
    return HAL_GetTick();
}








