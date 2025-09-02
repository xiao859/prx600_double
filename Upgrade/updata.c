#include "updata.h"
#include "stdbool.h"
#include "app_uart.h"
#include "app_spi.h"
#include "stmflash.h"

iapfun jump2app;

void iap_load_app(uint32_t appxaddr)
{
    if (((*(volatile  uint32_t *)appxaddr) & 0x2FFE0000) == 0x20000000)     /* 检查栈顶地址是否合法.可以放在内部SRAM共64KB(0x20000000) */
    {
        /* 用户代码区第二个字为程序开始地址(复位地址) */
        jump2app = (iapfun) * (volatile uint32_t *)(appxaddr + 4);

        /* 初始化APP堆栈指针(用户代码区的第一个字用于存放栈顶地址) */
        sys_msr_msp(*(volatile uint32_t *)appxaddr);

        /* 跳转到APP */
        jump2app();
    }
}

void app_jump_init()
{
    HAL_RCC_DeInit();

    HAL_UART_DeInit(&huart4);
    HAL_UART_DeInit(&huart5);

    __HAL_RCC_UART4_FORCE_RESET();
    __HAL_RCC_UART4_RELEASE_RESET();
    NVIC_DisableIRQ(UART4_IRQn);

    __HAL_RCC_UART5_FORCE_RESET();
    __HAL_RCC_UART5_RELEASE_RESET();
    NVIC_DisableIRQ(UART5_IRQn);

    __disable_irq();
    // HAL_NVIC_DisableIRQ();
    // __set_PRIMASK(1);


    return;
}

void update_process()
{
    app_jump_init();

    refresh_paraFlash();

    uint32_t appxaddr = 0x8000000;

    iap_load_app(appxaddr);

    return;
}
