#ifndef __XRAY_H
#define __XRAY_H

#include "exposure.h"

#define Calc_Gpio_State_N(value)      ((value == 1) ? (GPIO_PIN_RESET) : (GPIO_PIN_SET))

/* 高电平使能的转换 */
#define Calc_Gpio_State_P(value)      ((value == 0) ? (GPIO_PIN_RESET) : (GPIO_PIN_SET))

/* 3、数据类型定义 */

/* 4、函数声明 */

void config_ready_signal(uint16_t value);
void config_xrayOn_signal(uint16_t value);
void config_fault_signal(uint16_t value);
uint16_t get_expo_pin(uint16_t n);
uint16_t get_enable_pin(uint16_t n);
uint16_t get_interLock_pin(void);

void config_HVEn_signal(uint16_t value);
void config_mcuLock_signal(uint16_t value);
void config_reset_signal(uint16_t value);
uint16_t get_tube_curr_fault_pin(void);
uint16_t get_tube_vol_fault_pin(void);

void config_filamentOn_signal(uint16_t value,uint16_t n);
uint16_t get_filament_pin(uint16_t n);

void heartBeat_led(void);
void xray_on_led(uint16_t value);
void falut_led(uint16_t value);



void Xray_SetVoltage(uint16_t kv);              // 设置Xray高压参考
void Xray_Enable(ExposureSource src);           // 打开高压通道
void Xray_Disable(ExposureSource src);          // 关闭高压通道

#endif
