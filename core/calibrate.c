/*
 *  calibrate.c
 *
 *  Created on: Mar 04, 2025
 *  Author: Administrator
 *
 */

#include "calibrate.h"
#include "exposure.h"
#include <stdio.h>
#include "comm_string.h"
#include "comm_protocol.h"
#include "xray.h"
#include "pi_control.h"
#include "delay.h"
#include "app_spi.h"
#include "app_uart.h"

volatile xray_calibrate_data cali_data;

/* 校准参数：
 * 单电流校准时间：3.2s
 * 脉冲：120ms周周期，15ms曝光
 * */
void calibrate_mode_config()
{
    if (Is_PulseMode()) {
        cali_data.expoCycle_perCurrent[0] = (uint32_t)((CALI_SINGLE_CURR_TIME * 1000) / CALI_PULSE_SIGLE_CURR_PERIOD);
        cali_data.expoTime_expect[0]      = CALI_PULSE_CURR_EXPO_TIME * 50;
        cali_data.coolTime_expect[0]      = (CALI_PULSE_SIGLE_CURR_PERIOD - CALI_PULSE_CURR_EXPO_TIME) * 50;
    } else {
        cali_data.expoCycle_perCurrent[0] = 1;
        cali_data.expoTime_expect[0]      = CALI_SINGLE_CURR_TIME * 1000 * 50;
        cali_data.coolTime_expect[0]      = 0;
    }

    return;
}

void calibrate_para_init()
{
    cali_data.mode = XRAY_MODE_D_PULSE;
    cali_data.curr_index[0]    = 0;
    cali_data.cycle_count[0]   = 0;
    cali_data.timmer_count[0]  = 1;
    ctrl_data.filament_on[0]   = 1;
    cali_data.para_save_flag[0] = 0;

    cali_data.tube_vol_step[0] = ((float)(CALI_HV_REF - IDLE_HV_REF) / 100);

    config_data.fila_ref_realtime[0] = 0;
    config_data.fila_ref_step[0] = IDLE_FILAMENT_REF / (20 * 50);          /* 20ms上升时间 */
	
	  cali_data.curr_index[1]    = 0;
    cali_data.cycle_count[1]   = 0;
    cali_data.timmer_count[1]  = 1;
    ctrl_data.filament_on[1]   = 1;
    cali_data.para_save_flag[1] = 0;

    cali_data.tube_vol_step[1] = ((float)(CALI_HV_REF - IDLE_HV_REF) / 100);

    config_data.fila_ref_realtime[1] = 0;
    config_data.fila_ref_step[1] = IDLE_FILAMENT_REF / (20 * 50);          /* 20ms上升时间 */

    return;
}

/* 使能高压触发信号 */
void xray_HV_enable(uint16_t value)
{
    config_HVEn_signal(value);      /* 高电平开 */
    config_xrayOn_signal(value);    /* 低电平开 */

    return;
}

/* 更新校准结果到表中 */
void filament_ref_update(uint8_t curr_idx)
{
    if (Is_PulseMode()) {
        parm_table[0].currRef[curr_idx]   = param_pid.config_ref;
    } else {
        parm_table[0].currRef_c[curr_idx] = param_pid.config_ref;
    }


    return;
}

void config_filamentRef_cali(uint8_t curr_index)
{
    uint32_t fila_vol_ref = (Is_PulseMode()) ?
        parm_table[0].currRef[curr_index] : parm_table[0].currRef_c[curr_index];

    /* 测试代码，为1.4v，对应1.4/3.3*4095 = 1737 */
    // fila_vol_ref = 1900;
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, fila_vol_ref);

    return;
}

void config_hvref_cali()
{
    if (get_hv_state(0) == HPVS_SM_ID_CAL_EXPOSURING) {
        if (cali_data.tube_vol_realtime[0] < CALI_HV_REF) cali_data.tube_vol_realtime[0] += cali_data.tube_vol_step[0];
    } else {
        if (cali_data.tube_vol_realtime[0] > IDLE_HV_REF) cali_data.tube_vol_realtime[0] -= cali_data.tube_vol_step[0];
    }

    cali_data.tube_vol_realtime[0] = MAX(MIN(cali_data.tube_vol_realtime[0], CALI_HV_REF), IDLE_HV_REF);

    uint32_t tube_vol_ref = (uint32_t)((cali_data.tube_vol_realtime[0] / 64) / ADDA_FULL_SCALE_VIL_VALUE * 4095);  /* 0~2.5V 对应 0~160kV */

    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, tube_vol_ref);

    return;
}

void calibrate_task()
{
    if (cali_data.timmer_count[0] >= 1) cali_data.timmer_count[0]++;
    xray_data.isCheckAvailable[0] = 0;

    /* 使能关闭直接退出 */
    if (ctrl_data.enable[0] == 0) {
        xray_system_disable();
        disable_hvref();
        disable_filamentref(0);
        set_hv_state(HVPS_SM_ID_IDLE,0);
        return;
    }

    if (get_hv_state(0) == HPVS_SM_ID_CAL_PREPARE) {
        // if (ctrl_data.interlock == 1) config_mcuLock_signal(1);
        config_mcuLock_signal(1);

        /* 参数预设 */
        calibrate_mode_config();

        /* 使能灯丝 */
        config_filamentOn_signal(1,0);
        config_filament_ref_slop(0);

        /* 灯丝预热2.5秒 */
        if (cali_data.timmer_count[0] > TIMER6_2P5_SECOND_CYCLES) {
            set_hv_state(HPVS_SM_ID_CAL_RUN,0);
            cali_data.timmer_count[0] = 1;
            /* PID系数重置，注意每次开始RUN都需要重置，避免上次的遗留 */
            uint32_t currRef = (Is_PulseMode()) ?
                parm_table[0].currRef[cali_data.curr_index[0]] : parm_table[0].currRef_c[cali_data.curr_index[0]];
            pid_Init(parm_table[0].currValue[cali_data.curr_index[0]], currRef, Is_PulseMode());
        }

    } else if (get_hv_state(0) == HPVS_SM_ID_CAL_RUN) {
        /* 配置灯丝电源和高压电源 DAC值 */
        config_hvref_cali();
        config_filamentRef_cali(cali_data.curr_index[0]);

        /* 每个电流间需要有5秒的间隔时间 */
        if (cali_data.timmer_count[0] >= TIMER6_5_SECOND_CYCLES) {
            // xray_HV_enable(1);  /* 开始曝光 */
            set_hv_state(HPVS_SM_ID_CAL_EXPOSURING,0);
            cali_data.timmer_count[0] = 1;
        }
    } else if (get_hv_state(0) == HPVS_SM_ID_CAL_EXPOSURING) {
        xray_HV_enable(1);  /* 开始曝光，曝光和基准一起给 */
        config_hvref_cali();
        config_data.expo_count_total[0]++;
        /* 曝光计时 */
        if (cali_data.timmer_count[0] >= cali_data.expoTime_expect[0]) {
            set_hv_state(HPVS_SM_ID_CAL_COOLING,0);
            cali_data.timmer_count[0] = 1;
        }

        // if (cali_data.timmer_count == (uint32_t)(cali_data.expoTime_expect * 2 / 3)) {
        if (cali_data.timmer_count[0] == 650) {
            user_pid.currValue = sampled_data.tube_curr_value;
            // debug_tx3("管电流: %f, %f\n", user_pid.currValue, sampled_data.tube_curr_value);
        }

        if (cali_data.timmer_count[0] >= TIMER6_5_MILSECOND_CYCLES) {
            // xray_data.isCheckAvailable = 1;
        } else {
            xray_data.isCheckAvailable[0] = 0;
        }

        /* 连续模式下，每隔10ms调一次 */
        if (Is_ContinuousMode() && (cali_data.timmer_count[0] > TIMER6_4_MILSECOND_CYCLES)) {
            if (cali_data.timmer_count[0] % TIMER6_10_MILSECOND_CYCLES == 0) {
                user_pid.currValue = sampled_data.tube_curr_value;
                user_pid.Kp = 3.7;
	            user_pid.Ti = 0.0009;
                tube_current_piControl(1);
            }
        }
        // if (cali_data.timmer_count % 20 == 0) {
        //     debug_tx3("%d, %f\n", cali_data.timmer_count, sampled_data.tube_curr_value);
        // }

    } else if (get_hv_state(0) == HPVS_SM_ID_CAL_COOLING) {
        config_hvref_cali();
        /* 脉冲之间的时间 */

        if ((Is_PulseMode()) &&
            (cali_data.timmer_count[0] == (uint32_t)(cali_data.coolTime_expect[0] / 2))) {
            user_pid.Kp = 20;
	        user_pid.Ti = 1;
            tube_current_piControl(0);
        }

        if (cali_data.timmer_count < cali_data.coolTime_expect) return;

        cali_data.cycle_count[0]++;
        hvps_sm_state next_cal_state;

        if (cali_data.cycle_count >= cali_data.expoCycle_perCurrent) {
            filament_ref_update(cali_data.curr_index[0]);

            cali_data.cycle_count[0] = 0;
            parm_table[0].expo_count_total++;
            parm_table[0].expo_times_total += config_data.expo_count_total[0] / 3000000;
            /* 单个电流的脉冲校准完毕 */
            cali_data.curr_index[0]++;
            if (cali_data.curr_index[0] >= FILAMENT_CURRENT_TABLE_ORDER) {
            // if (cali_data.curr_index >= 1) {
                /* 上报本次的校准结果 */
                uint32_t last_idx = cali_data.curr_index[0] - 1;
                debug_tx3("闭环结果: %f, %d, %d\n",
                    parm_table[0].currValue[last_idx], parm_table[0].currRef[last_idx], param_pid.config_ref);
                cali_data.curr_index[0] = 0;

                /* 该模式下所有电流校准完毕，脉冲或连续 */
                if (Is_PulseMode()) {
                    next_cal_state = HPVS_SM_ID_CAL_RUN;
                    cali_data.mode = XRAY_MODE_S_CONTINUOUS;
                    calibrate_mode_config();
                    uint32_t currRef = (Is_PulseMode()) ?
                        parm_table[0].currRef[cali_data.curr_index[0]] : parm_table[0].currRef_c[cali_data.curr_index[0]];
                    pid_Init(parm_table[0].currValue[cali_data.curr_index[0]], currRef, Is_PulseMode());
                } else {
                    next_cal_state = HPVS_SM_ID_CAL_END;
                }
            } else {
                /* 上报本次的校准结果 */
                uint32_t last_idx = cali_data.curr_index[0] - 1;
                debug_tx3("闭环结果: %f, %d, %d\n",
                    parm_table[0].currValue[last_idx], parm_table[0].currRef[last_idx], param_pid.config_ref);
                /* 准备下个电流的校准 */
                next_cal_state = HPVS_SM_ID_CAL_RUN;
                uint32_t currRef = (Is_PulseMode()) ?
                    parm_table[0].currRef[cali_data.curr_index[0]] : parm_table[0].currRef_c[cali_data.curr_index[0]];
                pid_Init(parm_table[0].currValue[cali_data.curr_index[0]], currRef, Is_PulseMode());
            }
        } else {
            /* 单电流的脉冲没有执行完，继续曝光 */
            next_cal_state = HPVS_SM_ID_CAL_EXPOSURING;
        }
        set_hv_state(next_cal_state,0);
        cali_data.timmer_count[0] = 1;
    } else if (get_hv_state(0) == HPVS_SM_ID_CAL_END) {
        /* 记录校准数据 */
        // save_parament_to_flash();
        cali_data.para_save_flag[0] = 1;
        xray_system_disable();
        disable_hvref();
        disable_filamentref(0);
        cali_data.timmer_count[0] = 0;
        set_hv_state(HVPS_SM_ID_IDLE,0);
    }

    (get_hv_state(0) == HPVS_SM_ID_CAL_EXPOSURING) ? xray_on_led(1) : xray_on_led(0);

    return;
}
