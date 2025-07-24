///*
// *  calibrate.c
// *
// *  Created on: Mar 04, 2025
// *  Author: Administrator
// *
// */

#include "debug_mode.h"
#include "calibrate.h"
#include "ct_exposure.h"
#include <stdio.h>
#include "comm_string.h"
#include "xray.h"
#include "delay.h"
#include <math.h>
#include "adc.h"
#include "tim.h"

volatile xray_debug_data debug_data;

///**/
void xray_HV_enable_debug(uint16_t value)
{
    config_HVEn_signal(value);      /*高压电源*/
    config_xrayOn_signal(value);    /*准备信号*/
    config_mcuLock_signal(value);

    return;
}

void xray_disable_ref_debug()
{
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
    //B PWM
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
    return;
}

void config_filament_ref_slop_debug(uint8_t n)
{
    if (ctrl_data.filament_on[n] == 1)
    {
        if (config_data.fila_ref_realtime[n] < config_data.fila_ref_target[n])
            config_data.fila_ref_realtime[n] += config_data.fila_ref_step[n];
    }
    else
    {
        if (config_data.fila_ref_realtime[n] > IDLE_FILAMENT_REF_DEBUG)
            config_data.fila_ref_realtime[n] -= config_data.fila_ref_step[n];
    }

    config_data.fila_ref_realtime[n] = MAX(MIN(config_data.fila_ref_realtime[n], config_data.fila_ref_target[n]), IDLE_FILAMENT_REF_DEBUG);
    if (n == 0)
    {
        uint32_t filament_ref = (uint32_t)floor(((config_data.fila_ref_realtime[n] / ADDA_FULL_SCALE_VIL_VALUE) * 4095));

        HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, filament_ref);
    }
    else
    {}
}


void debug_task()
{
    static uint8_t xray_active;            // 当前射源
    static uint32_t last_switch_tick = 0;   // 上次切换采样开关时间戳
    xray_active = ctrl_data.xray_current - 1;

    // 检查射线模式
    if ((ctrl_data.xrayMode == XRAY_MODE_D_CONTINUOUS) || (ctrl_data.xrayMode == XRAY_MODE_D_PULSE) ||
            (ctrl_data.xrayMode == XRAY_MODE_S_CONTINUOUS) || (ctrl_data.xrayMode == XRAY_MODE_S_PULSE))
    {
        if ((ctrl_data.enable[0] == 0) && (ctrl_data.enable[1] == 0))      // 若系统关闭，强制退出
        {
            xray_HV_enable_debug(0);
            xray_disable_ref_debug();
            set_hv_state(HVPS_SM_ID_TRAIN_END, 0);
            set_hv_state(HVPS_SM_ID_TRAIN_END, 1);
            return;
        }
    }
    else
    {
        if (ctrl_data.enable[xray_active] == 0)
        {
            config_disable_sw(xray_active);  // 关闭采样通道
            xray_HV_enable_debug(0);
        }
    }

    hvps_sm_state debug_source_state = get_hv_state(xray_active);
    // Tick自增
    if (debug_data.timmer_count >= 1)
        debug_data.timmer_count++;

    config_filament_ref_slop_debug(xray_active);

    switch (debug_source_state)
    {
    case HVPS_SM_ID_TRAIN_PREPARE:
        if (ctrl_data.interlock == 1)
        {
            config_mcuLock_signal(1);
        }
        break;
    case HVPS_SM_ID_TRAIN_RUN:
        set_hv_state(HVPS_SM_ID_TRAIN_EXPOSURING, xray_active);
        debug_data.timmer_count = 1;
        break;
    case HVPS_SM_ID_TRAIN_EXPOSURING:
        config_hvref_slope(xray_active);
        xray_HV_enable_debug(1);
        config_data.expo_count_total[xray_active]++;

        if (debug_data.timmer_count >= debug_data.expoTime_expect[xray_active])
        {
            xray_HV_enable_debug(0);
            set_hv_state(HVPS_SM_ID_TRAIN_COOLING, xray_active);
            debug_data.timmer_count = 1;
            last_switch_tick = HAL_GetTick();  // 记录曝光结束时间
        }

        xray_data.isCheckAvailable[xray_active] = (debug_data.timmer_count >= TIMER6_5_MILSECOND_CYCLES);
        break;
    case HVPS_SM_ID_TRAIN_COOLING:
        // 冷却中，等待时间达到切换阈值
        if (debug_data.timmer_count < debug_data.coolTime_expect[xray_active])
            return;

        debug_data.cycle_count[xray_active]++;

        if (ctrl_data.xrayMode == XRAY_MODE_D_CONTINUOUS)
        {
            // 双源连续模式：每个源曝光一次后切换
            if (xray_active == 0 && debug_data.cycle_count[xray_active] >= debug_data.expoCycle_perCurrent[0])  // A源曝光一次
            {
                config_disable_sw(xray_active);
                if (HAL_GetTick() - last_switch_tick >= 8)// 切B源
                {
                    parm_table[xray_active].expo_count_total++;
                    parm_table[xray_active].expo_times_total += config_data.expo_count_total[xray_active] / 3000000;
                    config_enable_sw(1);
                    last_switch_tick = HAL_GetTick();
                    ctrl_data.xray_current =  2 ;
                    set_hv_state(HVPS_SM_ID_TRAIN_EXPOSURING, 1);
                    debug_data.timmer_count = 1;
                }
            }
            else if (xray_active == 1 && debug_data.cycle_count[xray_active] >= debug_data.expoCycle_perCurrent[1])  // B源曝光一次,结束
            {
                // 曝光完成，进入结束状态
                parm_table[xray_active].expo_count_total++;
                parm_table[xray_active].expo_times_total += config_data.expo_count_total[xray_active] / 3000000;
                cali_data.para_save_flag = 1;
                set_hv_state(HVPS_SM_ID_TRAIN_END, xray_active);
            }
        }
        else if (ctrl_data.xrayMode == XRAY_MODE_D_PULSE)
        {
            parm_table[xray_active].expo_count_total++;
            parm_table[xray_active].expo_times_total += config_data.expo_count_total[xray_active] / 3000000;
            // 双源脉冲模式，A源和B源交替曝光，直到达到最大曝光次数
            if (xray_active == 0)
            {
                config_disable_sw(xray_active);
                if (HAL_GetTick() - last_switch_tick >= 8)// 切B源
                {
                    config_enable_sw(1);
                    last_switch_tick = HAL_GetTick();
                    ctrl_data.xray_current =  2 ;
                    set_hv_state(HVPS_SM_ID_TRAIN_EXPOSURING, 1);
                    debug_data.timmer_count = 1;
                }
            }
            else if (xray_active == 1 && debug_data.cycle_count[xray_active] < debug_data.expoCycle_perCurrent[1])
            {
                config_disable_sw(xray_active);
                if (HAL_GetTick() - last_switch_tick >= 8)// 切A源
                {
                    config_enable_sw(0);
                    last_switch_tick = HAL_GetTick();
                    ctrl_data.xray_current =  1 ;
                    set_hv_state(HVPS_SM_ID_TRAIN_EXPOSURING, 0);
                    debug_data.timmer_count = 1;
                }
            }
            else
            {
                cali_data.para_save_flag = 1;
                set_hv_state(HVPS_SM_ID_TRAIN_END, xray_active);
            }
        }
        else if (ctrl_data.xrayMode == XRAY_MODE_S_CONTINUOUS)
        {
            // 单源连续模式，选择A源或B源曝光一次，结束
            parm_table[xray_active].expo_count_total++;
            parm_table[xray_active].expo_times_total += config_data.expo_count_total[xray_active] / 3000000;
            cali_data.para_save_flag = 1;
            set_hv_state(HVPS_SM_ID_TRAIN_END, xray_active);
        }
        else if (ctrl_data.xrayMode == XRAY_MODE_S_PULSE)
        {
            parm_table[xray_active].expo_count_total++;
            parm_table[xray_active].expo_times_total += config_data.expo_count_total[xray_active] / 3000000;
            // 单源脉冲模式，选择A源或B源曝光，直到达到最大曝光次数
            if (xray_active == 0 && debug_data.cycle_count[xray_active] < debug_data.expoCycle_perCurrent[0])
            {
                set_hv_state(HVPS_SM_ID_TRAIN_EXPOSURING, 0); // A源曝光
                debug_data.timmer_count = 1;
            }
            else if (xray_active == 1 && debug_data.cycle_count[xray_active] < debug_data.expoCycle_perCurrent[1])
            {
                set_hv_state(HVPS_SM_ID_TRAIN_EXPOSURING, 1); // B源曝光
                debug_data.timmer_count = 1;
            }
            else
            {
                cali_data.para_save_flag = 1;
                set_hv_state(HVPS_SM_ID_TRAIN_END, xray_active);
            }
        }
        break;
    case HVPS_SM_ID_TRAIN_END:

        xray_HV_enable_debug(0);
        xray_disable_ref_debug();
        if (xray_active == 1)
        {
            config_disable_sw(xray_active);
            if (HAL_GetTick() - last_switch_tick >= 8)  //A源不用切
            {
                config_enable_sw(0);
                last_switch_tick = HAL_GetTick();
                set_hv_state(HVPS_SM_ID_IDLE, 0);
                set_hv_state(HVPS_SM_ID_IDLE, 1);
            }
        }
        else
        {

            set_hv_state(HVPS_SM_ID_IDLE, 0);
            set_hv_state(HVPS_SM_ID_IDLE, 1);
        }

        break;
    default:
        break;
    }

    // LED指示
    xray_on_led((get_hv_state(0) == HVPS_SM_ID_TRAIN_EXPOSURING) || (get_hv_state(1) == HVPS_SM_ID_TRAIN_EXPOSURING));
}


