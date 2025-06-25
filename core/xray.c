#include "xray.h"
#include "main.h"

/* 输出给MCU----------------------------------------------------------------------------------------*/
void config_ready_signal(uint16_t value)
{
    /* 低电平有效 */
    HAL_GPIO_WritePin(READY_GPIO_Port, READY_Pin, Calc_Gpio_State_N(value));
    return;
}

void config_xrayOn_signal(uint16_t value)
{
    /* 低电平有效 */
    HAL_GPIO_WritePin(XRAY_ON_GPIO_Port, XRAY_ON_Pin, Calc_Gpio_State_N(value));
    return;
}

void config_fault_signal(uint16_t value)
{
    /* 低电平有效 */
    HAL_GPIO_WritePin(FAULT_GPIO_Port, FAULT_Pin, Calc_Gpio_State_N(value));
    return;
}

/* 低电平有效 */
uint16_t get_enable_pin()
{
    return ((HAL_GPIO_ReadPin(ENABLE_GPIO_Port, ENABLE_Pin) == GPIO_PIN_RESET) ? 1 : 0);
}

/* 低电平有效 */
uint16_t get_interLock_pin()
{
    return ((HAL_GPIO_ReadPin(INTERLOCK_GPIO_Port, INTERLOCK_Pin) == GPIO_PIN_RESET) ? 1 : 0);
}

/* 低电平有效 */
uint16_t get_expo_pin()
{
    return ((HAL_GPIO_ReadPin(EXP_GPIO_Port, EXP_Pin) == GPIO_PIN_RESET) ? 1 : 0);
}

/* 输出给高压电源----------------------------------------------------------------------------------------*/
void config_HVEn_signal(uint16_t value)
{
    /* 高电平开，低电平关 */
    HAL_GPIO_WritePin(HV_EN_GPIO_Port, HV_EN_Pin, Calc_Gpio_State_P(value));
    return;
}

void config_mcuLock_signal(uint16_t value)
{
    /* 高电平开，低电平关 */
    HAL_GPIO_WritePin(MCU_LOCK_GPIO_Port, MCU_LOCK_Pin, Calc_Gpio_State_P(value));
    return;
}

void config_reset_signal(uint16_t value)
{
    /* 高电平有效 */
    HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, Calc_Gpio_State_P(value));
    return;
}

/* 配置高压基准 */
void config_HV_REF(uint32_t value)
{

    return;
}

/* 高电平有效 */
uint16_t get_tube_curr_fault_pin()
{
    return ((HAL_GPIO_ReadPin(HV_C_FAULT_GPIO_Port, HV_C_FAULT_Pin) == GPIO_PIN_RESET) ? 0 : 1);
}

/* 高电平有效 */
uint16_t get_tube_vol_fault_pin()
{
    return ((HAL_GPIO_ReadPin(HV_V_FAULT_GPIO_Port, HV_V_FAULT_Pin) == GPIO_PIN_RESET) ? 0 : 1);
}

/* 输出给灯丝电源----------------------------------------------------------------------------------------*/
void config_filamentOn_signal(uint16_t value)
{
    /* 高电平开，低电平关 */
    HAL_GPIO_WritePin(FILAMENT_EN_GPIO_Port, FILAMENT_EN_Pin, Calc_Gpio_State_P(value));
    return;
}

uint16_t get_filament_pin()
{
    return ((HAL_GPIO_ReadPin(FILAMENT_EN_GPIO_Port, FILAMENT_EN_Pin) == GPIO_PIN_RESET) ? 0 : 1);
}

void heartBeat_led()
{
    HAL_GPIO_TogglePin(HEART_LED_GPIO_Port, HEART_LED_Pin);
}

void xray_on_led(uint16_t value)
{
    /* 低电平有效 */
    HAL_GPIO_WritePin(XRAY_LED_GPIO_Port, XRAY_LED_Pin, Calc_Gpio_State_N(value));

    return;
}

void falut_led(uint16_t value)
{
    /* 低电平有效 */
    HAL_GPIO_WritePin(FAULT_LED_GPIO_Port, FAULT_LED_Pin, Calc_Gpio_State_N(value));
}
