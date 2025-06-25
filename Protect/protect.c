#include "protect.h"
#include "xray.h"
#include "temperature.h"
#include "exposure.h"

void Protect_Check_Slow()
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

HVPS_FAULT_REGS mHVPS_Fault;

void InqHVPSFault(message_protocol *msg)
{
    static uint16_t fault_index ;
    uint16_t i;
    uint16_t *pFault;
    uint8_t data1, data2;
    fault_index ++;
    pFault = (uint16_t*)(&mHVPS_Fault);
    for(i=0;i<sizeof(HVPS_FAULT_REGS);i++)
    {
        uint16_t fault_count= 0;
        uint16_t i=0,j,fault_bit;
        uint16_t fault_temp;

        uint16_t start_fault_ID[sizeof(HVPS_FAULT_REGS)] = {0x00,0x20,0x30,0xA0,0xB0,0xC0};

        pFault = (uint16_t*)(&mHVPS_Fault);
        for(i=0;i<sizeof(HVPS_FAULT_REGS);i++) {
            fault_temp = *pFault++;
            for(j=0; j<16; j++) {
                fault_bit  = fault_temp & 0x1;  // 取最低位
                fault_temp = fault_temp>>1;
                if(fault_bit) { // 有故障
                    fault_count++;//当前故障计数
                    if(fault_index ==fault_count) {
                        data2 = start_fault_ID[i] + j;
                    }
                    data1 = fault_count;
                }
            }
        }
        if(fault_index >= fault_count) fault_index =0;
    }
    send_message(msg->msg_id, data1, data2);

    return;
}


