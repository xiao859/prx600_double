/*
 *
 */
#ifndef _SYS_H
#define _SYS_H

#include "stm32g4xx.h"
#include "core_cm4.h"
#include "stm32g4xx_hal.h"

void sys_nvic_set_vector_table(uint32_t baseaddr, uint32_t offset);                                     
void sys_standby(void);                                                                                 
void sys_soft_reset(void);                                                                                 
uint8_t sys_stm32_clock_init(uint32_t plln, uint32_t pllm, uint32_t pllr,uint32_t pllp, uint32_t pllq);    



void sys_wfi_set(void);           
void sys_intx_disable(void);      
void sys_intx_enable(void);      
void sys_msr_msp(uint32_t addr);   

#endif

