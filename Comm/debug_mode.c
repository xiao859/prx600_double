///*
// *  calibrate.c
// *
// *  Created on: Mar 04, 2025
// *  Author: Administrator
// *
// */

#include "debug_mode.h"
#include "calibrate.h"
#include "exposure.h"
#include <stdio.h>
#include "comm_protocol.h"
#include "comm_string.h"
#include "xray.h"
#include "delay.h"
#include <math.h>

volatile xray_debug_data debug_data;

///* 使能高压触发信号 */
//void xray_HV_enable_debug(uint16_t value)
//{
//    config_HVEn_signal(value);      /* 高电平开 */
//    config_xrayOn_signal(value);    /* 低电平开 */
//    config_mcuLock_signal(value);

//    return;
//}

///* 高压基准和灯丝基准都关掉 */
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

///* 开发者模式任务，大体流程和校准类似 */
//void debug_task()
//{
//    if (debug_data.timmer_count >= 1) debug_data.timmer_count++;

//    config_filament_ref_slop_debug();

//    /* 使能关闭直接退出 */
//    if (ctrl_data.enable == 0) {
//        xray_HV_enable_debug(0);
//        xray_disable_ref_debug();
//        set_hv_state(HVPS_SM_ID_IDLE);

//        return;
//    }

//    if (get_hv_state() == HVPS_SM_ID_TRAIN_PREPARE) {
//        if (ctrl_data.interlock == 1) {
//            config_mcuLock_signal(1);
//        }

//        if (debug_data.timmer_count > TIMER6_10_MILSECOND_CYCLES) {
//            // xray_HV_enable_debug(1);
//            // debug_data.timmer_count = 1;
//        }
//    } else if (get_hv_state() == HVPS_SM_ID_TRAIN_RUN) {
//        // config_hvref_slope();
//        set_hv_state(HVPS_SM_ID_TRAIN_EXPOSURING);
//        debug_data.timmer_count = 1;
//    } else if (get_hv_state() == HVPS_SM_ID_TRAIN_EXPOSURING) {
//        config_hvref_slope();  /* 开始曝光 */
//        config_data.expo_count_total++;
//        /* 曝光计时 */
//        if (debug_data.timmer_count >= debug_data.expoTime_expect) {
//            xray_HV_enable_debug(0);
//            set_hv_state(HVPS_SM_ID_TRAIN_COOLING);
//            debug_data.timmer_count = 1;
//        }
//        if (debug_data.timmer_count >= TIMER6_5_MILSECOND_CYCLES) {
//            xray_data.isCheckAvailable = 1;
//        } else {
//            xray_data.isCheckAvailable = 0;
//        }

//    } else if (get_hv_state() == HVPS_SM_ID_TRAIN_COOLING) {
//        // config_hvref_slope();
//        /* 脉冲之间的时间 */
//        if (debug_data.timmer_count < debug_data.coolTime_expect) return;

//        debug_data.cycle_count++;
//        hvps_sm_state next_cal_state;

//        if (debug_data.cycle_count >= debug_data.expoCycle_perCurrent) {

//            debug_data.cycle_count = 0;
//            parm_table.expo_count_total++;
//            parm_table.expo_times_total += config_data.expo_count_total / 3000000;
//            cali_data.para_save_flag = 1;
//            /* 单个电流的脉冲完毕 */
//            next_cal_state = HVPS_SM_ID_TRAIN_END;

//        } else {
//            xray_HV_enable_debug(1);      /* 单电流的脉冲没有执行完，继续曝光 */
//            next_cal_state = HVPS_SM_ID_TRAIN_EXPOSURING;
//        }
//        set_hv_state(next_cal_state);
//        debug_data.timmer_count = 1;
//    } else if (get_hv_state() == HVPS_SM_ID_TRAIN_END) {
//        xray_HV_enable_debug(0);
//        xray_disable_ref_debug();
//        set_hv_state(HVPS_SM_ID_IDLE);
//    }

//    (get_hv_state() == HVPS_SM_ID_TRAIN_EXPOSURING) ? xray_on_led(1) : xray_on_led(0);

//    return;
//}
