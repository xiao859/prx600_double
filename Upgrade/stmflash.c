/**
 *
 */

#include "stmflash.h"
#include "app_uart.h"
#include "updata.h"


uint32_t g_flash_bankx = 0;     /* ¼ÇÂ¼²Ù×÷ÄÄ¸öBANK */

/**
 * @brief       µÃµ½FLASHµÄ´íÎó×´Ì¬
 * @param       ÎÞ
 * @retval      Ö´ÐÐ½á¹û
 *   @arg       0    : ÒÑÍê³É
 *   @arg       ÆäËû : ´íÎó±àºÅ
 */
static uint8_t stmflash_get_error_status(void)
{
    uint32_t res = 0;
    res = FLASH->SR;

    if (res & (1 << 16)) return 1;  /* BSY=1, ·±Ã¦ */
    if (res & (1 << 15)) return 2;  /* OPTVERR=1,Ñ¡ÏîÓÐÐ§ÐÔ´íÎó */
    if (res & (1 << 14)) return 3;  /* RDERR=1,¶Á±£»¤´íÎó */
    if (res & (1 << 9))  return 4;  /* FASTERR=1,¿ìËÙ±à³Ì´íÎó */
    if (res & (1 << 8))  return 5;  /* MISSERR=1,¿ìËÙ±à³ÌÊý¾Ý¶ªÊ§´íÎó */
    if (res & (1 << 7))  return 6;  /* PGSERR=1,±à³ÌÐòÁÐ´íÎó */
    if (res & (1 << 6))  return 7;  /* SIZERR=1,Êý¾Ý´óÐ¡´íÎó */
    if (res & (1 << 5))  return 8;  /* PGAERR=1,±à³Ì¶ÔÆë´íÎó */
    if (res & (1 << 4))  return 9;  /* WRPERR=1,Ð´±£»¤´íÎó */
    if (res & (1 << 3))  return 10; /* PROGERR=1,±à³Ì´íÎó */
    if (res & (1 << 1))  return 11; /* OPERR=1,Ð´/²Á³ý´íÎó */

    return 0;                       /* Ã»ÓÐÈÎºÎ×´Ì¬/²Ù×÷Íê³É. */
}

/**
 * @brief       µÈ´ý²Ù×÷Íê³É
 * @param       time : ÒªÑÓÊ±µÄ³¤¶Ì
 * @retval      Ö´ÐÐ½á¹û
 *   @arg       0   : ÒÑÍê³É
 *   @arg       0XFF: ³¬Ê±
 *   @arg       ÆäËû : ´íÎó±àºÅ
 */
static uint8_t stmflash_wait_done(uint32_t time)
{
    uint8_t res;

    do
    {
        res = stmflash_get_error_status();

        if (res != 1)
        {
            break;              /* ·ÇÃ¦, ÎÞÐèµÈ´ýÁË, Ö±½ÓÍË³ö */
        }

        time--;
    } while (time);

    if (time == 0)res = 0XFF;   /* ³¬Ê± */

    return res;
}

/**
 * @brief       ÔÚFLASHÖ¸¶¨µØÖ·Ð´Á½¸ö×Ö (64Î»Êý¾Ý)
 *   @note      ÕâÁËÐ´ÈëÁ½¸ö×Ö, ÊÇÖ¸8¸ö×Ö½Ú
 * @param       faddr   : Ð´ÈëµØÖ· (´ËµØÖ·±ØÐëÎª4µÄ±¶Êý!!)
 * @param       data    : ÒªÐ´ÈëµÄÊý¾Ý(32Î»)
 * @param       data2   : ÒªÐ´ÈëµÄÊý¾Ý(32Î»)
 * @retval      Ö´ÐÐ½á¹û
 *   @arg       0    : ÒÑÍê³É
 *   @arg       0XFF : ³¬Ê±
 *   @arg       ÆäËû : ´íÎó±àºÅ
 */
static uint8_t stmflash_write_double_word(uint32_t faddr, uint32_t data, uint32_t data2)
{
    uint8_t res;
    res = stmflash_wait_done(0XFFFF);
    if (res == 0)                                   /* OK */
    {
        FLASH->CR |= 1 << 0;                        /* ±à³ÌÊ¹ÄÜ */
        *(volatile uint32_t *)faddr = data;         /* Ð´ÈëµÚÒ»¸ö×Ö½ÚÊý¾Ý */
        *(volatile uint32_t *)(faddr + 4) = data2;  /* Ð´ÈëµÚ¶þ¸ö×Ö½ÚÊý¾Ý */
        res = stmflash_wait_done(0XFFFF);           /* µÈ´ý²Ù×÷Íê³É,2¸ö×Ö±à³Ì */

        if (res != 1)                               /* ²Ù×÷³É¹¦ */
        {
            FLASH->CR &= ~(1 << 0);                 /* Çå³ýPGÎ» */
        }
    }
    return res;
}

/**
 * @brief       ´ÓÖ¸¶¨µØÖ·¶ÁÈ¡Ò»¸ö×Ö (32Î»Êý¾Ý)
 * @param       faddr   : ¶ÁÈ¡µØÖ· (´ËµØÖ·±ØÐëÎª4±¶Êý!!)
 * @retval      ¶ÁÈ¡µ½µÄÊý¾Ý (32Î»)
 */
uint32_t stmflash_read_word(uint32_t faddr)
{
    return *(volatile uint32_t *)faddr;
}

/**
 * @brief       »ñÈ¡Ä³¸öµØÖ·ËùÔÚµÄflashÒ³
 * @param       addr    : lashµØÖ·
 * @retval      µØÖ·ËùÔÚÒ³
 */
uint32_t  stmflash_get_flash_page(uint32_t addr)
{
    uint32_t page = 0;

    if (addr < (FLASH_BASE + FLASH_BANK_SIZE))
    {
        /* Bank 1 */
        page = (addr - FLASH_BASE) / FLASH_PAGE_SIZE;
        g_flash_bankx = FLASH_BANK_1;
    }
    else
    {
        /* Bank 2 */
        page = (addr - (FLASH_BASE + FLASH_BANK_SIZE)) / FLASH_PAGE_SIZE;
        g_flash_bankx = FLASH_BANK_2;
    }

    return page;
}

/**
 * @brief       ÔÚFLASH Ö¸¶¨Î»ÖÃ, Ð´ÈëÖ¸¶¨³¤¶ÈµÄÊý¾Ý(×Ô¶¯²Á³ý)
 * @note        ±¾º¯ÊýÐ´µØÖ·Èç¹û·Ç0XFFÄÇÃ´»áÏÈ²Á³ýÕû¸öÒ³ÇøÇÒ²»±£´æÒ³ÇøÊý¾Ý.ËùÒÔÐ´·Ç0XFFµÄµØÖ·,½«µ¼ÖÂÕû¸ö
 *              Ò³ÇøÊý¾Ý¶ªÊ§.½¨ÒéÐ´Ö®Ç°È·±£Ò³ÇøÀïÃ»ÓÐÖØÒªÊý¾Ý,×îºÃÊÇÕû¸öÉÈÒ³ÇøÏÈ²Á³ýÁË,È»ºóÂýÂýÍùºóÐ´.
 *              ¸Ãº¯Êý¶ÔOTPÇøÓòÒ²ÓÐÐ§!¿ÉÒÔÓÃÀ´Ð´OTPÇø!
 *              OTPÇøÓòµØÖ··¶Î§:0X1FFF7800~0X1FFF7A0F(×¢Òâ£º×îºó16×Ö½Ú£¬ÓÃÓÚOTPÊý¾Ý¿éËø¶¨£¬±ðÂÒÐ´£¡£¡)
 * @param       waddr   : ÆðÊ¼µØÖ· (´ËµØÖ·±ØÐëÎª4µÄ±¶Êý!!,·ñÔòÐ´Èë³ö´í!)
 * @param       pbuf    : Êý¾ÝÖ¸Õë
 * @param       length  : ÒªÐ´ÈëµÄ ×Ö(32Î»)Êý(¾ÍÊÇÒªÐ´ÈëµÄ32Î»Êý¾ÝµÄ¸öÊý)
 * @retval      ÎÞ
 */
void stmflash_write(uint32_t waddr, uint32_t *pbuf, uint32_t length)
{
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError = 0;
    uint8_t status = 0;
    uint32_t addrx = 0;
    uint32_t endaddr = 0;

    if (waddr < STM32_FLASH_BASE || waddr % 4 ||        /* Ð´ÈëµØÖ·Ð¡ÓÚ STM32_FLASH_BASE, »ò²»ÊÇ4µÄÕûÊý±¶, ·Ç·¨. */
        waddr > (STM32_FLASH_BASE + STM32_FLASH_SIZE))  /* Ð´ÈëµØÖ·´óÓÚ STM32_FLASH_BASE + STM32_FLASH_SIZE, ·Ç·¨. */
    {
        return;
    }

    HAL_FLASH_Unlock();             /* ½âËø */
    FLASH->ACR &= ~(1 << 10);       /* FLASH²Á³ýÆÚ¼ä,±ØÐë½ûÖ¹Êý¾Ý»º´æ!!! */
    FLASH->OPTR |= 1 << 22;         /* Ê¹ÓÃË«´æ´¢Ä£Ê½£¨Ò»´ÎÐ´64bitÒ²¾ÍÊÇ8×Ö½Ú£© */
    addrx = waddr;                  /* Ð´ÈëµÄÆðÊ¼µØÖ· */
    endaddr = waddr + length * 4;   /* Ð´ÈëµÄ½áÊøµØÖ· */

    if (addrx < 0X1FFF0000)         /* Ö»ÓÐÖ÷´æ´¢Çø,²ÅÐèÒªÖ´ÐÐ²Á³ý²Ù×÷!! */
    {
        while (addrx < endaddr)     /* É¨ÇåÒ»ÇÐÕÏ°­.(¶Ô·ÇFFFFFFFFµÄµØ·½,ÏÈ²Á³ý) */
        {
            if (stmflash_read_word(addrx) != 0XFFFFFFFF)    /* ÓÐ·Ç0XFFFFFFFFµÄµØ·½,Òª²Á³ýÕâ¸öÉÈÇø */
            {
                EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;                                                    /* Ñ¡ÔñÒ³²Á³ý */
                EraseInitStruct.Page        = stmflash_get_flash_page(addrx);                                           /* »ñÈ¡ÐèÒª²Á³ýÄÄ¸öÒ³ */
                EraseInitStruct.Banks       = g_flash_bankx;                                                            /* ÐèÒª²Á³ýµÄÒ³µØÖ·£¬ËùÊôBANK */
                EraseInitStruct.NbPages     = stmflash_get_flash_page(endaddr) - stmflash_get_flash_page(addrx) + 1;    /* ²Á³ýµÄÒ³ÊýÁ¿ */
                if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)
                {
                    break;
                }
                stmflash_wait_done(0XFFFF);
            }
            else
            {
                addrx += 4;
            }
            status = stmflash_wait_done(0XFFFF);                                /* µÈ´ýÉÏ´Î²Ù×÷Íê³É */
        }
    }
    status = stmflash_wait_done(0XFFFF);                                        /* µÈ´ýÉÏ´Î²Ù×÷Íê³É */
    if (status == HAL_OK)
    {
        while (waddr < endaddr)     /* Ð´Êý¾Ý */
        {
            if(stmflash_write_double_word(waddr, *pbuf, *(pbuf + 1)))           /* ÓÉÓÚÊ¹ÓÃµÄdoubleÄ£Ê½£¬ËùÒÔÒ»´ÎÐ´8×Ö½Ú£¬Èç¹ûÒ»´ÎÐ´4×Ö½Ú£¬»á³öÏÖÒ»´ÎÐ´³É¹¦Ò»´ÎÐ´Ê§°ÜµÄÇé¿ö */
            {
                break;              /* Ð´ÈëÒì³£ */
            }

            waddr += 8;
            pbuf += 2;
        }
    }

    FLASH->ACR |= 1 << 10;          /* FLASH²Á³ý½áÊø,¿ªÆôÊý¾Ýfetch */
    HAL_FLASH_Lock();               /* ÉÏËø */
}

/**
 * @brief       ´ÓÖ¸¶¨µØÖ·¿ªÊ¼¶Á³öÖ¸¶¨³¤¶ÈµÄÊý¾Ý
 * @param       raddr : ÆðÊ¼µØÖ·
 * @param       pbuf  : Êý¾ÝÖ¸Õë
 * @param       length: Òª¶ÁÈ¡µÄ×Ö(32)Êý,¼´4¸ö×Ö½ÚµÄÕûÊý±¶
 * @retval      ÎÞ
 */
void stmflash_read(uint32_t raddr, uint32_t *pbuf, uint32_t length)
{
    uint32_t i;

    for (i = 0; i < length; i++)
    {
        pbuf[i] = stmflash_read_word(raddr);    /* ¶ÁÈ¡4¸ö×Ö½Ú. */
        raddr += 4;                             /* Æ«ÒÆ4¸ö×Ö½Ú. */
    }
}

void update_paraFlash(void)
{
    uint32_t buff[7];

    for (int i = 0; i < 7; i++) {
        buff[i] = stmflash_read_word(PARAM_FLASH_START_ADDR + 0x04 * i);
    }
    buff[4] = PARAM_CODE_VALID_FLAG;
    stmflash_write(PARAM_FLASH_START_ADDR, buff, sizeof(buff));

    return;
}

void refresh_paraFlash(void)
{
    uint32_t buff[7];

    for (int i = 0; i < 7; i++) {
        buff[i] = stmflash_read_word(PARAM_FLASH_START_ADDR + 0x04 * i);
    }
    buff[4] = PARAM_CODE_INVALID_FLAG;
    stmflash_write(PARAM_FLASH_START_ADDR, buff, sizeof(buff));

    return;
}
