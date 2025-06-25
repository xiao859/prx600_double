/*
 *  app_comm.c
 *
 *  Created on: Feb 24, 2025
 *  Author: Administrator
 *
 */

#include "comm_protocol.h"
#include <stdio.h>
#include "main.h"
#include "xray_io_config.h"
#include "app_usart.h"
#include "app_funcs.h"
#include "log.h"
#include <string.h>
#include "math.h"
#include "stmflash.h"
#include "spi_flash.h"
#include "CT_mode.h"
#include "debug_mode.h"
#include "calibrate.h"
#include "temperature.h"

uint16_t adc_buffer2[ADC_2_CHANNEL_NUM * ADC_SAMPLE_CYCLE_NUM];
uint16_t adc_buffer3[ADC_3_CHANNEL_NUM * ADC_SAMPLE_CYCLE_NUM];

volatile hvps_sm_state hv_state = HVPS_SM_ID_IDLE;
volatile xray_mode xrayMode = XRAY_MODE_PULSE;
volatile cmd_control_data ctrl_data;
volatile xray_config_data config_data;
volatile adc_sampled_value sampled_data = {
    30, 30, 10, 1000, 24, 0, 24, 0, 20};
volatile adc_sampled_value sampled_data_last = {0};

volatile xray_parament_range para_range = {
    140, 50,            /* 管电压保护值 */
    145, 5, 750,        /* 管电流保护值：最大最小和瞬时 */

    70, -25, 60, 57,     /* 油箱温度 */

    28, 20,           /* 24V供电 */
    100, 100,           /* 散热器温度 */
    30, 8,           /* 灯丝电压保护 */
    100, 100,           /* 灯丝电流保护 */

    100, 60,            /* 管电压配置门限 */
    120, 10,             /* 管电流配置门限 */

    1000, 1000,         /* 曝光时间保护 */

    4500000,            /* 最大曝光时间，1.5min */

    6000000             /* 灯丝开启未曝光时间保护 */
};

volatile xray_parament_table parm_table = {
    0, 0, 1,
    {1,    2,    3,    4,    5,    6,    7,    8,    9,    10,   11,   12},
    {1606, 1734, 1830, 1911, 1980, 2039, 2091, 2134, 2174, 2210, 2246, 2282},
    {1500, 1660, 1770, 1850, 1920, 1980, 2030, 2080, 2120, 2160, 2200, 2250},
};

volatile xray_version version = {
    1, 1,
    1, 0, 2,
    2, 1, 19, 0,
    17, 23,
    3, 6, 0
};

hvps_sm_state get_hv_state()
{
    return hv_state;
}

void set_hv_state(hvps_sm_state state)
{
    hv_state = state;

    return;
}

void save_parament_to_flash()
{
    wirte_flash_parament((uint8_t *)&parm_table, sizeof(xray_parament_table));

    return;
}

void func_test(uint8_t *buff)
{
    return;
}

void flash_table_init()
{
    get_flash_parament((uint8_t *)&parm_table, sizeof(xray_parament_table));

    uint32_t currRef[FILAMENT_CURRENT_TABLE_ORDER] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
    int i;

    if(memcmp((uint8_t *)&parm_table.currValue[0], (uint8_t *)&currRef[0], FILAMENT_CURRENT_TABLE_ORDER * sizeof(uint32_t)) != 0) {
        // FLASH地址参数错误
    }

    for (i = 0; i < FILAMENT_CURRENT_TABLE_ORDER; i++) {
        parm_table.currRef[i]   = MIN(MAX(parm_table.currRef[i], 1000), 2450);
        parm_table.currRef_c[i] = MIN(MAX(parm_table.currRef_c[i], 1000), 2450);
    }

    config_data.expo_count_total=0;       /* 开始前将曝光周期计时清零 */
}

void parament_init(void)
{
    set_hv_state(HVPS_SM_ID_IDLE);

    config_data.tube_vol  = 60;
    config_data.tube_curr = 10;

    /* FLASH中参数是否擦写：
       判断到valid_flag == 0，说明从boot擦写过来的：需要把参数重新擦写到FLASH中。（为了满足用户手动改参数）
       否则，说明是正常的上电动作，直接把FLASH更新到参数中
    */
   if (stmflash_read_word(PARAM_CODE_VALID_ADDR) != PARAM_CODE_VALID_FLAG) {
        /* 1、设置标志位 */
        update_paraFlash();

        /* 2、把参数擦写到FLASH中 */
        xray_parament_table table_local;
        /* 2.1、读取全部FLASH */
        get_flash_parament((uint8_t *)&table_local, sizeof(xray_parament_table));

        /* 2.2、赋值需要的参数，曝光次数和时间不需要刷新 */
        parm_table.expo_times_total = table_local.expo_times_total;
        parm_table.expo_count_total = table_local.expo_count_total;

        memcpy((uint8_t *)&table_local.currValue[0], (uint8_t *)&parm_table.currValue[0], sizeof(parm_table.currValue));
        memcpy((uint8_t *)&table_local.currRef[0], (uint8_t *)&parm_table.currRef[0], sizeof(parm_table.currRef));
        memcpy((uint8_t *)&table_local.currRef_c[0], (uint8_t *)&parm_table.currRef_c[0], sizeof(parm_table.currRef_c));

        /* 2.3、写入到FLASH中 */
        wirte_flash_parament((uint8_t *)&table_local, sizeof(xray_parament_table));

   } else {
        /* 把校准过的值赋值给表 */
        // xray_parament_table table_local;
        get_flash_parament((uint8_t *)&parm_table, sizeof(xray_parament_table));
        // memcpy((uint8_t *)&parm_table, (uint8_t *)&table_local, sizeof(xray_parament_table));
   }

   return;
}

/* AA 55 33 00 00 CD */
uint8_t message_check(USART_TypeDef *Instance)
{
    uint8_t result = 0;

    if (Instance == UART4) {
        if ((uart4.uart_rx_buf[0] == 0xAA) && (uart4.uart_rx_buf[1] == 0x55) &&
            (((uart4.uart_rx_buf[2] + uart4.uart_rx_buf[3] + uart4.uart_rx_buf[4] + uart4.uart_rx_buf[5])&0xFF) == 0)) {
            result = 1;
        }
    } else if (Instance == UART5) {
        if ((uart5.uart_rx_buf[0] == 0xAA) && (uart5.uart_rx_buf[1] == 0x55) &&
            (((uart5.uart_rx_buf[2] + uart5.uart_rx_buf[3] + uart5.uart_rx_buf[4] + uart5.uart_rx_buf[5])&0xFF) == 0)) {
            result = 1;
        }
    }

    return result;
}

void cmd_process(int32_t message_idx, USART_TypeDef *Instance)
{
    uint32_t func_idx;

    if (message_idx >= SCI_MSG_TEST_1) {
        func_idx = message_idx - SCI_MSG_TEST_1 + 40;
    } else if (message_idx >= SCI_MSG_DEBUG_LAMP_I_SET) {
        func_idx = message_idx - SCI_MSG_DEBUG_LAMP_I_SET + 36;
    } else if (message_idx >= SCI_MSG_SET_PFCTHRESHOLD) {
        func_idx = message_idx - SCI_MSG_SET_PFCTHRESHOLD + 32;
    } else if (message_idx >= SCI_MSG_CTRL_RST) {
        func_idx = message_idx - SCI_MSG_CTRL_RST + 24;
    } else if (message_idx >= SCI_MSG_SET_MODE) {
        func_idx = message_idx - SCI_MSG_SET_MODE + 18;
    } else if (message_idx >= SCI_MSG_INQ_MODE) {
        func_idx = message_idx - SCI_MSG_INQ_MODE;
    } else {
        //invalid_cmd_reply();
        restart_usart_receive(Instance);

        return;
    }

    message_protocol msg;
    if (Instance == UART4) {
        memcpy((uint8_t *)&msg, (uint8_t *)&uart4.uart_rx_buf[0], MESSAGE_PACK_LENGTH);
    } else if (Instance == UART5) {
        memcpy((uint8_t *)&msg, (uint8_t *)&uart5.uart_rx_buf[0], MESSAGE_PACK_LENGTH);
    }

    // func_idx = 0;
    /* check parament */
    if ((func_idx >= APP_FUNC_NUM) || (funcs[func_idx].msgId != message_idx)) {
        invalid_cmd_reply();
    } else {
        funcs[func_idx].func_ptr(&msg);
    }

    restart_usart_receive(Instance);

}

void cmd_parser()
{
    int32_t  message_idx;

    if ((uart4.recv_complete == 1) && message_check(UART4)) {
        message_idx = (int32_t)uart4.uart_rx_buf[2];
        cmd_process(message_idx, UART4);
    }

    if ((uart5.recv_complete == 1) && message_check(UART5)) {
        message_idx = (int32_t)uart5.uart_rx_buf[2];
        cmd_process(message_idx, UART5);
    }

    return;
}

void xray_fast_protect()
{
    if (!Is_Exposing() || !xray_data.isCheckAvailable) {
        return;
    }

    /* 管电压 */
    float tube_vol_p = sampled_data.tube_vol_p_value * 2;
    float tube_vol_n = sampled_data.tube_vol_n_value * 2;

    /* 过压判断 */
    if (tube_vol_p > para_range.tube_vol_max_protected) {
        xray_data.tube_kv_overCount++;
        if (xray_data.tube_kv_overCount > FAST_PROTECT_TIME_RANGE) mHVPS_Fault.FAULT_REG2.bit.KV1_OVER = 1;
    } else {
        xray_data.tube_kv_overCount = 0;
    }

    /* 欠压判断 */
    if (tube_vol_p < para_range.tube_vol_min_protected) {
        xray_data.tube_kv_underCount++;
        if (xray_data.tube_kv_underCount > FAST_PROTECT_TIME_RANGE) mHVPS_Fault.FAULT_REG2.bit.KV1_UNDER = 1;
    } else {
        xray_data.tube_kv_underCount = 0;
    }

    float tube_vol_target = Is_CalibrateMode() ? CALI_HV_REF : (float)config_data.tube_vol;
    /* 油箱打火 oil_strike_count */
    if (((tube_vol_p > tube_vol_target - 5) && (tube_vol_n < tube_vol_target - 5)) ||
        ((tube_vol_n > tube_vol_target - 5) && (tube_vol_p < tube_vol_target - 5))) {
            xray_data.oil_strike_count++;
            if (xray_data.oil_strike_count > FAST_PROTECT_TIME_RANGE) xray_data.oil_strike_times++;

            if (xray_data.oil_strike_times >= STRIKE_TIEMS_RANGE) mHVPS_Fault.FAULT_REG4.bit.ARC1 = 1;
    } else {
        xray_data.oil_strike_count = 0;
    }

    /*  球管打火 tube_strike_count */
    if ((tube_vol_p < tube_vol_target - 5) && (tube_vol_n < tube_vol_target - 5)) {
            xray_data.tube_strike_count++;
            if (xray_data.tube_strike_count > OVER_RANGE_TIME_LIMIT) xray_data.tube_strike_times++;
            if (xray_data.tube_strike_times >= STRIKE_TIEMS_RANGE)  mHVPS_Fault.FAULT_REG4.bit.ARC2 = 1;
    } else {
        xray_data.tube_strike_count = 0;
    }

    /* 电压断线 */
    if (tube_vol_p < 1 || tube_vol_n < 1) {
        xray_data.tube_vol_broken_count++;
        if (xray_data.tube_vol_broken_count > FAST_PROTECT_TIME_RANGE) mHVPS_Fault.FAULT_REG4.bit.HV_broken1 = 1;
    } else {
        xray_data.tube_vol_broken_count = 0;
    }

    /* 管电流 */
    float tube_curr = sampled_data.tube_curr_value * 10; /* 比较时的单位：0.1mA */

    /* 过流判断 */
    if (tube_curr > para_range.tube_curr_max_protected) {
        xray_data.tube_mA_overCount++;
        if (xray_data.tube_mA_overCount > FAST_PROTECT_TIME_RANGE) mHVPS_Fault.FAULT_REG2.bit.MA1_OVER = 1;
    } else {
        xray_data.tube_mA_overCount = 0;
    }

    /* 欠流判断 */
    if (tube_curr < para_range.tube_curr_min_protected) {
        xray_data.tube_mA_underCount++;
        if (xray_data.tube_mA_underCount > FAST_PROTECT_TIME_RANGE) mHVPS_Fault.FAULT_REG2.bit.MA1_UNDER = 1;
    } else {
        xray_data.tube_mA_underCount = 0;
    }

    /* 电流采样断线 */
    if (tube_curr < 1) {
        xray_data.tube_curr_broken_count++;
        if (xray_data.tube_curr_broken_count > FAST_PROTECT_TIME_RANGE) mHVPS_Fault.FAULT_REG4.bit.current_broken1 = 1;
    } else {
        xray_data.tube_curr_broken_count = 0;
    }

    float fila_vol = sampled_data.filament_vol_value;
    if (fila_vol > para_range.filament_vol_max_protected) {
        xray_data.fila_vol_overCount++;
        if (xray_data.fila_vol_overCount > FAST_PROTECT_TIME_RANGE) mHVPS_Fault.FAULT_REG4.bit.LAMP1_OV = 1;
    } else {
        xray_data.fila_vol_overCount = 0;
    }

    if (fila_vol < para_range.filament_vol_min_protected) {
        xray_data.fila_vol_underCount++;
        if (xray_data.fila_vol_underCount > FAST_PROTECT_TIME_RANGE) mHVPS_Fault.FAULT_REG1.bit.LAMP1_UV = 1;
    } else {
        xray_data.fila_vol_underCount = 0;
    }

    return;
}

void xray_parament_protect()
{
    /* 灯丝开启，长时间不曝光 */
    if (config_data.fila_protect_cnt > para_range.fila_protect_cnt_max) {
        mHVPS_Fault.FAULT_REG4.bit.lamp_wait_overtime=1;
    }

    if (ctrl_data.interlock == 0) {
        xray_data.interLock_count++;
        if (xray_data.interLock_count > OVER_RANGE_TIME_LIMIT) mHVPS_Fault.FAULT_REG1.bit.INTERLOCK1 = 1;
        /* 计时，超限报警 */
    } else {
        xray_data.interLock_count = 0;
    }

    /* 单次曝光超过预计时间 */
    if (config_data.expo_count > para_range.expo_time_limit) {
        mHVPS_Fault.FAULT_REG3.bit.EXPO1_OVERTIME = 1;
    }

    if (sampled_data.power_24v_value > para_range.power_24v_max_protected) {
        xray_data.pwr_24_overCount++;
        if (xray_data.pwr_24_overCount > 20 * OVER_RANGE_TIME_LIMIT) mHVPS_Fault.FAULT_REG2.bit.V24_OV = 1;
    } else {
        xray_data.pwr_24_overCount = 0;
        mHVPS_Fault.FAULT_REG2.bit.V24_OV = 0;
    }

    if (sampled_data.power_24v_value < para_range.power_24v_min_protected) {
        xray_data.pwr_24_underCount++;
        if (xray_data.pwr_24_underCount > 20 * OVER_RANGE_TIME_LIMIT) mHVPS_Fault.FAULT_REG2.bit.V24_UV = 1;
    } else {
        xray_data.pwr_24_underCount = 0;
        mHVPS_Fault.FAULT_REG2.bit.V24_UV = 0;
    }

    float temp_oil = get_temp(sampled_data.temp_oil_value);
    sampled_data.oil_temp = temp_oil;
    if (temp_oil > para_range.temp_oil_max_protected || temp_oil < para_range.temp_oil_min_protected) {
        mHVPS_Fault.FAULT_REG1.bit.OIL_TEMP1 = 1;
    } else {
        mHVPS_Fault.FAULT_REG1.bit.OIL_TEMP1 = 0;
    }

    if (temp_oil > para_range.temp_oil_warning) {
        mHVPS_Fault.FAULT_REG1.bit.OIL_OT1 = 1;
        // xray_data.oil_temp_errCount++;
        // if (xray_data.oil_temp_errCount > OVER_RANGE_TIME_LIMIT) mHVPS_Fault.FAULT_REG1.bit.OIL_OT1 = 1;
    }

    mHVPS_Fault.FAULT_REG1.bit.OIL_OT1 = (mHVPS_Fault.FAULT_REG1.bit.OIL_OT1 & (temp_oil > para_range.temp_oil_recovery));

    // /* 油箱直接报过来的故障 */
    // if (ctrl_data.hv_vol_fault || ctrl_data.hv_curr_fault) {
    //     mHVPS_Fault.FAULT_REG1.bit.HV_HARDW_FAULT = 1;
    // }

    // mHVPS_Fault.FAULT_REG2.bit.MA1_OVER = 1;
    // mHVPS_Fault.FAULT_REG2.bit.MA1_UNDER = 1;
    // mHVPS_Fault.FAULT_REG2.bit.KV1_OVER = 1;
    // mHVPS_Fault.FAULT_REG2.bit.KV1_UNDER = 1;

    return;
}

/* 进行一些开始前的初始化， */
void hvState_ilde_init()
{
    xray_data.isCheckAvailable = 0;

    /* 把曝光中的计数清零 */
    debug_data.timmer_count = 0;
    cali_data.timmer_count = 0;

}

/* 由实际值，找到索引；由索引找到REF */
uint32_t get_filamentRef(float tube_current)
{
    int index = 0;

    for (index = 0; index < FILAMENT_CURRENT_TABLE_ORDER; index++) {
        if (tube_current >= parm_table.currValue[FILAMENT_CURRENT_TABLE_ORDER - 1]) {
            config_data.tube_curr_index = FILAMENT_CURRENT_TABLE_ORDER - 1;
            break;
        }

        /* 向下取整 */
        if ((tube_current >= parm_table.currValue[index]) && (tube_current < parm_table.currValue[index+1])) {
            config_data.tube_curr_index = index;
            break;
        }
    }

    if (tube_current >= parm_table.currValue[FILAMENT_CURRENT_TABLE_ORDER - 1]) {
        return parm_table.currRef[config_data.tube_curr_index];
    }

    uint32_t currRef_uplimit;
    uint32_t currRef_downlimit;

    // if (ctrl_data.xrayMode == XRAY_MODE_PULSE) {
    //     currRef_uplimit   = parm_table.currRef[config_data.tube_curr_index+1];
    //     currRef_downlimit = parm_table.currRef[config_data.tube_curr_index];
    // } else {
    //     currRef_uplimit   = parm_table.currRef_c[config_data.tube_curr_index+1];
    //     currRef_downlimit = parm_table.currRef_c[config_data.tube_curr_index];
    // }
    currRef_uplimit   = parm_table.currRef[config_data.tube_curr_index+1];
    currRef_downlimit = parm_table.currRef[config_data.tube_curr_index];

    return (currRef_downlimit + (tube_current - parm_table.currValue[index]) * (currRef_uplimit - currRef_downlimit));

}

void config_filamentRef()
{
    config_data.fila_ref_realtime = get_filamentRef(config_data.tube_curr);

    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, config_data.fila_ref_realtime);

    return;
}

/* 控制管电压基准上升曲线  offset = 0.22405*config_data.tube_vol_realtime - 6.89364*/
void config_hvref_slope()
{
    if ((get_hv_state() == HVPS_SM_ID_EXPOSURING) ||
        (get_hv_state() == HPVS_SM_ID_CAL_EXPOSURING) ||
        (get_hv_state() == HVPS_SM_ID_TRAIN_EXPOSURING)) {
        if (config_data.tube_vol_realtime < config_data.tube_vol) config_data.tube_vol_realtime += config_data.tube_vol_step;
    } else {
        if (config_data.tube_vol_realtime > IDLE_HV_REF) config_data.tube_vol_realtime -= config_data.tube_vol_step;
    }

    config_data.tube_vol_realtime = MAX(MIN(config_data.tube_vol_realtime ,config_data.tube_vol), IDLE_HV_REF);

    int32_t tube_vol_ref = (int32_t)((config_data.tube_vol_realtime / 64) / ADDA_FULL_SCALE_VIL_VALUE * 4095);  /* 0~2.5V 对应 0~160kV */
    int32_t offset = (int32_t)(0.22405f * config_data.tube_vol_realtime - 6.89364f);

    tube_vol_ref = tube_vol_ref + offset;
    tube_vol_ref = MAX(tube_vol_ref, 0);

    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)tube_vol_ref);
}

void config_filament_ref_slop()
{
    if (ctrl_data.filament_on == 1) {
        if (config_data.fila_ref_realtime < IDLE_FILAMENT_REF) config_data.fila_ref_realtime += config_data.fila_ref_step;
    } else {
        if (config_data.fila_ref_realtime > 0) config_data.fila_ref_realtime -= config_data.fila_ref_step;
    }

    config_data.fila_ref_realtime = MAX(MIN(config_data.fila_ref_realtime, IDLE_FILAMENT_REF), 0);

    uint32_t filament_ref = (uint32_t)floor(((config_data.fila_ref_realtime / ADDA_FULL_SCALE_VIL_VALUE) * 4095));

    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (uint32_t)config_data.fila_ref_realtime);
}

void disable_hvref()
{
    // config_data.tube_vol_realtime = 0;
    // config_data.tube_vol = 0;
    // config_data.tube_vol_step = 0;

    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
}

void disable_filamentref()
{
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
}

/* 非曝光状态，各种信号初始化 */
void xray_system_disable()
{
    xray_CT_disable();

    ctrl_data.enable = 0;
    ctrl_data.expo = 0;
    ctrl_data.filament_on = 0;

    return;
}

void xray_system_fault_check()
{
    if (!Is_System_Without_Fault()) {
        set_hv_state(HVPS_SM_ID_FAULT);
        xray_system_disable();
        config_fault_signal(1);

        falut_led(1);
    } else {
        falut_led(0);
    }

    return;
}
