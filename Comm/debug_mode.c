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
    config_mcuLock_signal(value);
    config_HVEn_signal(value);      /*高压电源*/
    config_xrayOn_signal(value);    /*准备信号*/


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

void config_filament_ref_slop_debug(uint8_t n)//到目标值时跳出
{
//    if (config_data.fila_ref_target[n] == 0)
//        return;
//    if (ctrl_data.filament_on[n] == 1)
//    {
//        if (config_data.fila_ref_realtime[n] < config_data.fila_ref_target[n])
//            config_data.fila_ref_realtime[n] += config_data.fila_ref_step[n];
//    }
//    else
//    {
//        if (config_data.fila_ref_realtime[n] > (float)IDLE_FILAMENT_REF_DEBUG)
//            config_data.fila_ref_realtime[n] -= config_data.fila_ref_step[n];
//    }


//    if (n == 0)
//    {
//        config_data.fila_ref_realtime[n] = MAX(MIN(config_data.fila_ref_realtime[n], config_data.fila_ref_target[n]), IDLE_FILAMENT_REF_DEBUG);
//        uint32_t filament_ref = (uint32_t)floor(((config_data.fila_ref_realtime[n] / ADDA_FULL_SCALE_VIL_VALUE) * 4095));

//        HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, filament_ref);
//    }
//    else
//    {
//        config_data.fila_ref_realtime[n] = MAX(MIN(config_data.fila_ref_realtime[n], config_data.fila_ref_target[n]), (float)IDLE_FILAMENT_REF_DEBUG);
//        uint32_t filament_ref = (uint32_t)floor(((config_data.fila_ref_realtime[n] / ADDA_FULL_SCALE_VIL_VALUE) * 2999));
//        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, filament_ref);
//    }
}


void debug_task()
{
    static uint8_t xray_active;            // 当前射源
    static uint32_t last_exp_tick = 0;
    static uint8_t switching = 0;
    static uint32_t count = 0;

    xray_active = ctrl_data.xray_current - 1;

    // Tick自增
    if (debug_data.timmer_count >= 1)
        debug_data.timmer_count++;

    hvps_sm_state debug_source_state = get_hv_state(xray_active);


//    config_filament_ref_slop_debug(0);
//    config_filament_ref_slop_debug(1);

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
        // 曝光后 3.7ms 开始允许采样检查
        xray_data.isCheckAvailable = (debug_data.timmer_count > 80) ? 1 : 0;
        config_hvref_slope(xray_active);
        if (HAL_GetTick() - last_exp_tick >= 10)
        {

            xray_HV_enable_debug(1);
            config_data.expo_count_total[xray_active]++;
            if (debug_data.timmer_count >= debug_data.expoTime_expect[xray_active])
            {
                xray_HV_enable_debug(0);
                xray_data.isCheckAvailable = 0;
                last_exp_tick = HAL_GetTick(); // 记录曝光结束时间
                set_hv_state(HVPS_SM_ID_TRAIN_COOLING, xray_active);
                debug_data.timmer_count = 1;
            }
        }

        break;
    case HVPS_SM_ID_TRAIN_COOLING:
        // 冷却中，等待时间达到切换阈值
        if (debug_data.timmer_count < debug_data.coolTime_expect[xray_active])
            return;
        if (count >= 1)
            count++;
        if (ctrl_data.xrayMode == XRAY_MODE_D_CONTINUOUS)
        {
            // 双源连续模式：每个源曝光一次后切换
            if (xray_active == 0)   // A源曝光一次   && debug_data.cycle_count[xray_active] >= debug_data.expoCycle_perCurrent[0]
            {
                if (HAL_GetTick() - last_exp_tick >= 9)// 切B源
                {
                    if (switching == 0)
                    {
                        config_disable_sw(xray_active);
//                        last_switch_tick = HAL_GetTick();
                        switching = 1;
                        count = 1;
                    }

                    if (count >= 20)//uint32_t tuck = HAL_GetTick();tuck - last_switch_tick >= 1
                    {
                        if (switching == 1)
                        {
                            config_enable_sw(1);
                            count = 1; //last_switch_tick = HAL_GetTick();
                            switching = 2;
                        }
                        if ((count >= 20) && (switching == 2))//HAL_GetTick() - last_switch_tick >= 1
                        {
                            count = 0;
                            switching = 0;
                            ctrl_data.xray_current =  2 ;
                            set_hv_state(HVPS_SM_ID_TRAIN_EXPOSURING, 1);
                            debug_data.timmer_count = 1;
                        }
                    }
                }
            }
            else if (xray_active == 1)  // B源曝光一次,结束 && debug_data.cycle_count[xray_active] >= debug_data.expoCycle_perCurrent[1]
            {
                if (HAL_GetTick() - last_exp_tick >= 9)// 切B源
                {

                    if (switching == 0)
                    {
                        config_disable_sw(xray_active);
                        count = 1;//last_switch_tick = HAL_GetTick();
                        switching = 1;
                    }
                    if (count >= 20)
                    {
                        // 曝光完成，进入结束状态
                        parm_table[0].expo_count_total++;
                        parm_table[1].expo_count_total++;
                        parm_table[0].expo_times_total += config_data.expo_count_total[0] / 1200000;
                        parm_table[1].expo_times_total += config_data.expo_count_total[1] / 1200000;
                        config_data.expo_count_total[xray_active] = 0;
                        config_data.expo_count_total[xray_active] = 0;
                        set_hv_state(HVPS_SM_ID_TRAIN_END, xray_active);
                        config_enable_sw(0);
                        switching = 0;
                        count = 0;
                    }
                }
            }
        }
        else if (ctrl_data.xrayMode == XRAY_MODE_D_PULSE)
        {
            if (count > 0)
                count++;
            // 双源脉冲模式，A源和B源交替曝光，直到达到最大曝光次数
            if (xray_active == 0)
            {
                if (HAL_GetTick() - last_exp_tick >= 9)// 切B源
                {

                    if (switching == 0)
                    {
                        config_disable_sw(xray_active);
                        count = 1; //last_switch_tick = HAL_GetTick();
                        switching = 1;
                    }

                    if (count >= 20)//uint32_t tuck = HAL_GetTick();tuck - last_switch_tick >= 1
                    {
                        if (switching == 1)
                        {
                            config_enable_sw(1);
                            count = 1; //last_switch_tick = HAL_GetTick();
                            switching = 2;
                        }
                        if ((count >= 20) && (switching == 2))//HAL_GetTick() - last_switch_tick >= 1
                        {
                            ctrl_data.xray_current =  2 ;
                            set_hv_state(HVPS_SM_ID_TRAIN_EXPOSURING, 1);
                            switching = 0;
                            debug_data.timmer_count = 1;
                            config_data.expo_count_total[xray_active] = 0;
                            count = 0;
                        }
                    }
                }
            }
            else if (xray_active == 1 && debug_data.cycle_count[xray_active] < debug_data.expoCycle_perCurrent[1])
            {

                if (HAL_GetTick() - last_exp_tick >= 9)// 切A源
                {
                    if (switching == 0)
                    {
                        config_disable_sw(1);
                        count = 1; //last_switch_tick = HAL_GetTick();
                        switching = 1;
                    }
                    if (count >= 20)//HAL_GetTick() - last_switch_tick >= 1
                    {
                        if (switching == 1)
                        {
                            config_enable_sw(0);
                            count = 1; //last_switch_tick = HAL_GetTick();
                            switching = 2;
                        }
                        if ((count >= 20) && (switching == 2))//HAL_GetTick() - last_switch_tick >= 1
                        {
                            switching = 0;
                            debug_data.cycle_count[xray_active]++;
                            ctrl_data.xray_current =  1 ;
                            set_hv_state(HVPS_SM_ID_TRAIN_EXPOSURING, 0);
                            count = 0;
                            debug_data.timmer_count = 1;
                            config_data.expo_count_total[xray_active] = 0;
                        }
                    }
                }
            }
            else
            {
                if (switching == 0)
                {
                    config_disable_sw(1);
                    count = 1; //last_switch_tick = HAL_GetTick();
                    switching = 1;
                }
                if (count >= 20)//HAL_GetTick() - last_switch_tick >= 1
                {
                    if (switching == 1)
                    {
                        config_enable_sw(0);
                        count = 1; //last_switch_tick = HAL_GetTick();
                        switching = 2;
                    }
                    if ((count >= 20) && (switching == 2))//HAL_GetTick() - last_switch_tick >= 1
                    {
                        switching = 0;
                        count = 0;
                        debug_data.cycle_count[xray_active]++;
                        debug_data.cycle_count[0] = 0;
                        debug_data.cycle_count[1] = 0;
                        set_hv_state(HVPS_SM_ID_TRAIN_END, xray_active);
                        parm_table[0].expo_count_total++;
                        parm_table[1].expo_count_total++;
                        parm_table[0].expo_times_total += config_data.expo_count_total[0] / 1200000;
                        parm_table[1].expo_times_total += config_data.expo_count_total[1] / 1200000;
                        config_data.expo_count_total[xray_active] = 0;
                    }
                }
            }
        }
        else if (ctrl_data.xrayMode == XRAY_MODE_S_CONTINUOUS)
        {
            // 单源连续模式，选择A源或B源曝光一次，结束
						parm_table[xray_active].expo_count_total++;
						exp_count[xray_active] += config_data.expo_count_total[xray_active];
						parm_table[xray_active].expo_times_total += exp_count[xray_active] / 1200000;
						exp_count[xray_active] = exp_count[xray_active] % 1200000;
            config_data.expo_count_total[xray_active] = 0;
            debug_data.timmer_count = 0;
            set_hv_state(HVPS_SM_ID_TRAIN_END, xray_active);
        }
        else if (ctrl_data.xrayMode == XRAY_MODE_S_PULSE)
        {
            debug_data.cycle_count[xray_active]++;
            config_data.expo_count_total[xray_active] = 0;
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
                debug_data.cycle_count[xray_active] = 0;
                set_hv_state(HVPS_SM_ID_TRAIN_END, xray_active);
                parm_table[xray_active].expo_count_total++;
                exp_count[xray_active] += config_data.expo_count_total[xray_active];
                parm_table[xray_active].expo_times_total += exp_count[xray_active] / 1200000;
                exp_count[xray_active] = exp_count[xray_active] % 1200000;
            }
        }
        break;
    case HVPS_SM_ID_TRAIN_END:

        xray_HV_enable_debug(0);
        count++;
        xray_disable_ref_debug();
        config_filamentOn_signal(0, 0);
        config_filamentOn_signal(0, 1);
        ctrl_data.interlock = 0;
        ctrl_data.enable[0] = 0;
        ctrl_data.enable[0] = 0;
        if ((xray_active == 1) & (count >= 180))
        {
            config_disable_sw(xray_active);
            if (count >= 200)
            {
                count = 0;
                config_enable_sw(0);
                set_hv_state(HVPS_SM_ID_IDLE, 0);
                set_hv_state(HVPS_SM_ID_IDLE, 1);
								 cali_data.para_save_flag = 1;
            }
        }
        else if (xray_active == 0)
        {
            set_hv_state(HVPS_SM_ID_IDLE, 0);
            set_hv_state(HVPS_SM_ID_IDLE, 1);
						cali_data.para_save_flag = 1;
        }

        break;
    default:
        break;
    }

// LED指示
    xray_on_led((get_hv_state(0) == HVPS_SM_ID_TRAIN_EXPOSURING) || (get_hv_state(1) == HVPS_SM_ID_TRAIN_EXPOSURING));
}


