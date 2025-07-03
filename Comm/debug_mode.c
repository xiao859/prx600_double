///*
// *  calibrate.c
// *
// *  Created on: Mar 04, 2025
// *  Author: Administrator
// *
// */

#include "debug_mode.h"
#include "calibrate.h"
#include "HV_exposure.h"
#include <stdio.h>
#include "comm_string.h"
#include "xray.h"
#include "delay.h"
#include <math.h>

volatile xray_debug_data debug_data;

///**/
//void xray_HV_enable_debug(uint16_t value)
//{
//    config_HVEn_signal(value);      /*高压电源*/
//    config_xrayOn_signal(value);    /*准备信号*/
//    config_mcuLock_signal(value);

//    return;
//}

//void xray_disable_ref_debug()
//{
//    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
//    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
//  //B PWM

//    return;
//}

//void config_filament_ref_slop_debug()
//{
//    if (ctrl_data.filament_on == 1) {
//        if (config_data.fila_ref_realtime < config_data.fila_ref_target) config_data.fila_ref_realtime += config_data.fila_ref_step;
//    } else {
//        if (config_data.fila_ref_realtime > IDLE_FILAMENT_REF_DEBUG) config_data.fila_ref_realtime -= config_data.fila_ref_step;
//    }

//    config_data.fila_ref_realtime = MAX(MIN(config_data.fila_ref_realtime, config_data.fila_ref_target), IDLE_FILAMENT_REF_DEBUG);

//    uint32_t filament_ref = (uint32_t)floor(((config_data.fila_ref_realtime / ADDA_FULL_SCALE_VIL_VALUE) * 4095));

//    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, filament_ref);
//}

/////**/
void debug_task()
{
//    static uint8_t xray_active = 0;            // 当前激活射源：0 = 无, 1 = SW1, 2 = SW2
//    static uint32_t last_switch_tick = 0;      // 上次切换采样开关时间戳

//    if (debug_data.timmer_count >= 1) debug_data.timmer_count++;

//    config_filament_ref_slop_debug();

//    // 若系统关闭，强制退出
//    if (ctrl_data.enable[xray_active] == 0) {
//        xray_HV_enable_debug(0);
//        xray_disable_ref_debug();
//        switch_sw1(0);  // 关闭采样通道1
//        switch_sw2(0);  // 关闭采样通道2
//        set_hv_state(HVPS_SM_ID_IDLE);
//        return;
//    }

//    switch (get_hv_state()) {
//        case HVPS_SM_ID_TRAIN_PREPARE:
//            if (ctrl_data.interlock == 1) {
//                config_mcuLock_signal(1);
//            }
//            if (debug_data.timmer_count > TIMER6_10_MILSECOND_CYCLES) {
//                // 进入运行状态，默认从SW1开始
//                xray_active = 1;
//                switch_sw1(1);
//                switch_sw2(0);
//                last_switch_tick = HAL_GetTick();
//                set_hv_state(HVPS_SM_ID_TRAIN_RUN);
//                debug_data.timmer_count = 1;
//            }
//            break;

//        case HVPS_SM_ID_TRAIN_RUN:
//            set_hv_state(HVPS_SM_ID_TRAIN_EXPOSURING);
//            debug_data.timmer_count = 1;
//            break;

//        case HVPS_SM_ID_TRAIN_EXPOSURING:
//            config_hvref_slope();
//            xray_HV_enable_debug(1);
//            config_data.expo_count_total++;

//            if (debug_data.timmer_count >= debug_data.expoTime_expect) {
//                xray_HV_enable_debug(0);
//                set_hv_state(HVPS_SM_ID_TRAIN_COOLING);
//                debug_data.timmer_count = 1;
//                last_switch_tick = HAL_GetTick();  // 记录曝光结束时间
//            }

//            xray_data.isCheckAvailable = (debug_data.timmer_count >= TIMER6_5_MILSECOND_CYCLES);
//            break;

//        case HVPS_SM_ID_TRAIN_COOLING:
//            // 冷却中，等待时间达到切换阈值
//            if (debug_data.timmer_count < debug_data.coolTime_expect) return;

//            debug_data.cycle_count++;
//            if (debug_data.cycle_count >= debug_data.expoCycle_perCurrent) {
//                parm_table.expo_count_total++;
//                parm_table.expo_times_total += config_data.expo_count_total / 3000000;
//                cali_data.para_save_flag = 1;
//                set_hv_state(HVPS_SM_ID_TRAIN_END);
//            } else {
//                // 切换射源：需确保距离上次关闭 >8ms
//                if (HAL_GetTick() - last_switch_tick >= 8) {
//                    if (xray_active == 1) {
//                        switch_sw1(0);
//                        switch_sw2(1);
//                        xray_active = 2;
//                    } else {
//                        switch_sw2(0);
//                        switch_sw1(1);
//                        xray_active = 1;
//                    }
//                    last_switch_tick = HAL_GetTick();
//                    set_hv_state(HVPS_SM_ID_TRAIN_EXPOSURING);
//                    debug_data.timmer_count = 1;
//                }
//            }
//            break;

//        case HVPS_SM_ID_TRAIN_END:
//            xray_HV_enable_debug(0);
//            xray_disable_ref_debug();
//            switch_sw1(0);
//            switch_sw2(0);
//            set_hv_state(HVPS_SM_ID_IDLE);
//            break;

//        default:
//            set_hv_state(HVPS_SM_ID_IDLE);
//            break;
//    }

//    xray_on_led(get_hv_state() == HVPS_SM_ID_TRAIN_EXPOSURING);
}



