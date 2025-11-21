#ifndef __XRAY_H
#define __XRAY_H

#include "stdint.h"
//低电平使能
#define Calc_Gpio_State_N(value)      ((value == 1) ? (GPIO_PIN_RESET) : (GPIO_PIN_SET))

/* 高电平使能*/
#define Calc_Gpio_State_P(value)      ((value == 0) ? (GPIO_PIN_RESET) : (GPIO_PIN_SET))

void config_ready_signal(uint16_t value);
void config_xrayOn_signal(uint16_t value);
void config_fault_signal(uint16_t value);
uint16_t get_expo_pin(uint16_t n);
uint16_t get_enable_pin(uint16_t n);
uint16_t get_interLock_pin(void);

void config_HVEn_signal(uint16_t value,uint16_t n);
void config_mcuLock_signal(uint16_t value);
void config_reset_signal(uint16_t value);
uint16_t get_tube_curr_fault_pin(void);
uint16_t get_tube_vol_fault_pin(void);

void config_filamentOn_signal(uint16_t value,uint16_t n);
uint16_t get_filament_pin(uint16_t n);

void heartBeat_led(void);
void xray_on_led(uint16_t value);
void falut_led(uint16_t value);

void config_enable_sw(uint16_t n);
void config_disable_sw(uint16_t n);


#endif


