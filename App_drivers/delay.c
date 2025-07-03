/**
 *
 */

#include "delay.h"


static uint32_t g_fac_us = 0;       /* usÑÓÊ±±¶³ËÊı */

/**
 * ³õÊ¼»¯ÑÓ³Ùº¯Êı
 */
void delay_init(uint16_t sysclk)
{
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);/* SYSTICKÊ¹ÓÃÍâ²¿Ê±ÖÓÔ´,ÆµÂÊÎªHCLK */
    g_fac_us = sysclk;                                  /* ²»ÂÛÊÇ·ñÊ¹ÓÃOS,g_fac_us¶¼ĞèÒªÊ¹ÓÃ */
}

/**
 * ÑÓÊ±nus
 * ÎŞÂÛÊÇ·ñÊ¹ÓÃOS, ¶¼ÊÇÓÃÊ±ÖÓÕªÈ¡·¨À´×öusÑÓÊ±
 */
void delay_us(uint32_t nus)
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 0;
    uint32_t reload = SysTick->LOAD;        /* LOADµÄÖµ */
    ticks = nus * g_fac_us;                 /* ĞèÒªµÄ½ÚÅÄÊı */

    told = SysTick->VAL;                    /* ¸Õ½øÈëÊ±µÄ¼ÆÊıÆ÷Öµ */
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;        /* ÕâÀï×¢ÒâÒ»ÏÂSYSTICKÊÇÒ»¸öµİ¼õµÄ¼ÆÊıÆ÷¾Í¿ÉÒÔÁË */
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;                      /* Ê±¼ä³¬¹ı/µÈÓÚÒªÑÓ³ÙµÄÊ±¼ä,ÔòÍË³ö */
            }
        }
    }

}

/**
 * @brief     ÑÓÊ±nms
 * @param     nms: ÒªÑÓÊ±µÄmsÊı (0< nms <= (2^32 / fac_us / 1000))(fac_usÒ»°ãµÈÓÚÏµÍ³Ö÷Æµ, ×ÔĞĞÌ×Èë¼ÆËã)
 * @retval    ÎŞ
 */
void delay_ms(uint16_t nms)
{
    delay_us((uint32_t)(nms * 1000));                   /* ÆÕÍ¨·½Ê½ÑÓÊ± */
}

/**
 * HAL¿âÄÚ²¿º¯ÊıÓÃµ½µÄÑÓÊ±
 * HAL¿âµÄÑÓÊ±Ä¬ÈÏÓÃSystick£¬Èç¹ûÎÒÃÇÃ»ÓĞ¿ªSystickµÄÖĞ¶Ï»áµ¼ÖÂµ÷ÓÃÕâ¸öÑÓÊ±ºóÎŞ·¨ÍË³ö
 */
void HAL_Delay(uint32_t Delay)
{
     delay_ms(Delay);
}

// è·å–ç³»ç»Ÿå¯åŠ¨ä»¥æ¥çš„æ¯«ç§’æ•°ï¼ˆæ¥è‡ª SysTick ä¸­æ–­ç´¯è®¡ï¼‰
uint32_t get_tick_ms()
{
    return HAL_GetTick();
}








