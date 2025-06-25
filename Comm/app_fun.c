/*
 *  app_funcs.c
 *
 *  Created on: Feb 25, 2025
 *  Author: Administrator
 *
 */

#include "app_fun.h"
#include "comm_protocol.h"
#include "comm_string.h"
#include <stdint.h>
//#include "debug.h"
#include "calibrate.h"
#include "math.h"
#include "xray.h"
#include "delay.h"
#include "exposure.h"
#include "protect.h"
#include <string.h>
//#include "debug_mode.h"

controler_cmd_funcs funcs[APP_FUNC_NUM] =
{
    {SCI_MSG_INQ_MODE,                      &fun_null},
    {SCI_MSG_INQ_TUBE_VSET,                 &Inqixay1HVPSCurrentset},
    {SCI_MSG_INQ_TUBE_ISET,                 &fun_null},
    {SCI_MSG_INQ_MAX_TIME,                  &Inqmaxtimeset},
    {SCI_MSG_INQ_TEMP,                      &InqHVPSTemp},
    {SCI_MSG_INQ_FAULT,                     &InqHVPSFault},
    {SCI_MSG_INQ_STATE,                     &fun_null},
    {SCI_MSG_INQ_SW,                        &HVswversion},
    {SCI_MSG_INQ_HW,                        &HVhwversion},
    {SCI_MSG_INQ_TUBE_LAST_V,               &InqHVPS1LastVandC},
    {SCI_MSG_INQ_TUBE_LAST_I,               &fun_null},
    {SCI_MSG_INQ_TUBE_LAST_EXPOTIME,        &fun_null},
    {SCI_MSG_INQ_LAMP_SW,                   &lampswversion},
    {SCI_MSG_INQ_LAMP_HW,                   &lamphwversion},
    {SCI_MSG_INQ_EXPO_TIME1,                &InqHVPSExpo_Time1},
    {SCI_MSG_INQ_EXPO_COUNT1,               &InqHVPSExpo_Count},
    {SCI_MSG_INQ_AUTOCALIBRA,               &Inqautocalibra},
    {SCI_MSG_INQ_XSOURCE_SW,                &xsourcetype},

    {SCI_MSG_SET_MODE,                      &SetHVPSMode},
    {SCI_MSG_SET_TUBE_V,                    &SetHV1TubeVoltageandcurrent},
    {SCI_MSG_SET_TUBE_I,                    &SetHV1TubeVoltageandcurrent},
    {SCI_MSG_SET_MAX_TIME,                  &Setmaxexpotime},
    {SCI_MSG_SET_EXP_COUNTCLR,              &exp1countclr},
    {SCI_MSG_SET_EXP_TIMECLR,               &exp1timeclr},

    {SCI_MSG_CTRL_RST,                      &FaultReset},
    {SCI_MSG_CTRL_CAL,                      &Autocalibra},
    {SCI_MSG_CTRL_TRAIN,                    &fun_null},
    {SCI_MSG_CTRL_UPDATE,                   &fun_null},
    {SCI_MSG_CTRL_STORE_TABLE,              &StoreLookupTable},
    {SCI_MSG_CTRL_STORE_TABLE_INQ,          &StoreLookupTableINQ},
    {SCI_MSG_CTRL_STORE_STATISTICS,         &StoreStatistics},
    {SCI_MSG_LAMP_CONTROL,                  &Lampcontrol},

    {SCI_MSG_SET_PFCTHRESHOLD,              &fun_null},
    {SCI_MSG_SET_24VTHRESHOLD,              &fun_null},
    {SCI_MSG_SET_KVMATHRESHOLD,             &fun_null},
    {SCI_MSG_SET_BUCKLLCTHRESHOLD,          &fun_null},

    {SCI_MSG_DEBUG_EXPO_CTRL,               &debug_expo_ctrl},

    {SCI_MSG_DEBUG_LAMP_I_SET,              &SetHVLampCurrent},
    {SCI_MSG_DEBUG_TUBE_VIDLE_SET,          &SetHVTubeIdleVoltage},
    {SCI_MSG_DEBUG_TUBE_VRISE_TIME_SET,     &SetHVTubeRisingTime},
    {SCI_MSG_DEBUG_ONLINE_PI,               &SetOnlinePI},

//    {SCI_MSG_TEST_1,                        &test_func1},
//    {SCI_MSG_TEST_2,                        &test_func2},
//    {SCI_MSG_TEST_3,                        &test_func3},
//    {SCI_MSG_TEST_4,                        &test_func4},
};

void fun_null(message_protocol *msg)
{
    return;
}

void invalid_cmd_reply()
{
    uint8_t data1 = 0xFF;
    uint8_t data2 = 0xFF;

    send_message(SCI_MSG_INQ_SW, data1, data2);

    return;
}

void InqHVPS1LastVandC(message_protocol *msg)
{
   // debug("get message\r\n");
    return;
}

void HVswversion(message_protocol *msg)
{
    uint16_t sw_value = (version.sw_ver_high << 12) | (version.sw_ver_mid << 8) | version.sw_ver_low;
    uint8_t data1 = sw_value >> 8;
    uint8_t data2 = sw_value & 0xFF;

    send_message(msg->msg_id, data1, data2);

    return;
}

void HVhwversion(message_protocol *msg)
{
    uint8_t data1 = version.hw_ver_high;
    uint8_t data2 = version.hw_ver_low;

    send_message(msg->msg_id, data1, data2);

    return;
}

void lampswversion(message_protocol *msg)
{
    uint8_t data1 = version.tube_ver_high;
    uint8_t data2 = version.tube_ver_low;

    send_message(msg->msg_id, data1, data2);

    return;
}

void lamphwversion(message_protocol *msg)
{
    uint16_t filament_value = (version.fila_ver_high << 11) | (version.fila_ver_mid1 << 7) |
                              (version.fila_ver_mid2 << 3)  | version.fila_ver_low;

    uint8_t data1 = filament_value >> 8;
    uint8_t data2 = filament_value & 0xFF;

    send_message(msg->msg_id, data1, data2);

    return;
}

void InqHVPSTemp(message_protocol *msg)
{
    /* 单位0.1°C, 向上偏30°C */
    uint16_t temperature = (uint16_t)(sampled_data.oil_temp);

    uint8_t data1 = temperature >> 8;
    uint8_t data2 = temperature & 0xFF;

    send_message(msg->msg_id, data1, data2);

    return;
}

void InqHVPSExpo_Time1(message_protocol *msg)
{
    /* 需要换算一下 */
    uint8_t data1 = parm_table.expo_times_total >> 8;
    uint8_t data2 = parm_table.expo_times_total & 0xFF;

    send_message(msg->msg_id, data1, data2);

    return;
}


void InqHVPSExpo_Count(message_protocol *msg)
{
    uint8_t data1 = parm_table.expo_count_total >> 8;
    uint8_t data2 = parm_table.expo_count_total & 0xFF;

    send_message(msg->msg_id, data1, data2);

    return;
}

void SetHVPSMode(message_protocol *msg)
{
    uint8_t data1;
    xray_mode xrayMode = (msg->data1 == 0) ? XRAY_MODE_PULSE : XRAY_MODE_CONTINUOUS;

    if (get_hv_state() == HVPS_SM_ID_IDLE) {
        data1 = SCI_SETUP_SUCCESS;
    } else {
        data1 = SCI_SETUP_OUT_LIMIT;
    }

    ctrl_data.xrayMode = xrayMode;
    send_message(msg->msg_id, data1, 0);

    return;
}

void SetHV1TubeVoltageandcurrent(message_protocol *msg)
{
    uint8_t data1, data2;

    /* 1、限幅 */
    if ((msg->data1 <= para_range.tube_vol_max_config) && (msg->data1 >= para_range.tube_vol_min_config)) {
        config_data.tube_vol  = msg->data1;     /* 单位kV */
        data1 = SCI_SETUP_SUCCESS;
    } else {
        mHVPS_Fault.FAULT_REG4.bit.VOL_CURR_OV = 1;
        data1 = SCI_SETUP_OUT_LIMIT;
    }

    if ((msg->data2 <= para_range.tube_curr_max_config) && (msg->data2 >= para_range.tube_curr_min_config)) {
        config_data.tube_curr = ((float)msg->data2) / 10;     /* 单位0.1mA */
        data1 = SCI_SETUP_SUCCESS;
    } else {
        mHVPS_Fault.FAULT_REG4.bit.VOL_CURR_OV = 1;
        data2 = SCI_SETUP_OUT_LIMIT;
    }

    if (get_hv_state() == HVPS_SM_ID_IDLE) {
        config_data.tube_vol_realtime = 0;
        config_data.tube_vol_step = (float)(config_data.tube_vol - IDLE_HV_REF) / (50 * parm_table.rising_time);
        xray_data.timmer_count = 1;
        // config_hvref_filamentRef();
    } else {
        data1 = SCI_SETUP_SM_ERROR;
        data2 = SCI_SETUP_SM_ERROR;
    }

    send_message(msg->msg_id, data1, data2);

    return;
}

void SetHVTubeIdleVoltage(message_protocol *msg)
{
    return;
}

void SetHVLampCurrent(message_protocol *msg)
{
    return;
}

void SetHVTubeRisingTime(message_protocol *msg)
{
    if (get_hv_state() != HVPS_SM_ID_IDLE) {
        send_message(msg->msg_id, SCI_SETUP_SM_ERROR, SCI_SETUP_SM_ERROR);
    }

    if (msg->data1 >= 2 && msg->data1 <= 20) {
        parm_table.rising_time = (uint32_t)msg->data1;
        send_message(msg->msg_id, SCI_SETUP_SUCCESS, SCI_SETUP_SUCCESS);
        save_parament_to_flash();
    } else {
        send_message(msg->msg_id, SCI_SETUP_OUT_LIMIT, SCI_SETUP_OUT_LIMIT);
    }

    return;
}

void SetOnlinePI(message_protocol *msg)
{
    return;
}

void FaultReset(message_protocol *msg)
{
    uint8_t data1, data2;

    if(get_hv_state() == HVPS_SM_ID_FAULT ) {
        HVPS_FAULT_GROUP1_REG FAULT_REG1_temp;
        FAULT_REG1_temp.value = 0;

        FAULT_REG1_temp.bit.OIL_OT1 = mHVPS_Fault.FAULT_REG1.bit.OIL_OT1;

        mHVPS_Fault.FAULT_REG1.value = 0x0000;
        mHVPS_Fault.FAULT_REG2.value = 0x0000;
        mHVPS_Fault.FAULT_REG3.value = 0x0000;
        mHVPS_Fault.FAULT_REG4.value = 0x0000;
        mHVPS_Fault.FAULT_REG5.value = 0x0000;
        mHVPS_Fault.FAULT_REG6.value = 0x0000;

        mHVPS_Fault.FAULT_REG1.value = FAULT_REG1_temp.value;

        memset((uint8_t *)(&xray_data), 0, sizeof(xray_data));

        set_hv_state(HVPS_SM_ID_IDLE);
        data1 = 0x00;
        data2 = 0x00;

    } else {
            data1 = 0x01;
            data2 = 0x00;
    }

    falut_led(0);

    send_message(msg->msg_id, data1, data2);

    return;
}

void StoreLookupTable(message_protocol *msg)
{
    return;
}

void StoreLookupTableINQ(message_protocol *msg)
{
    return;
}

void StoreStatistics(message_protocol *msg)
{
    return;
}

void Setmaxexpotime(message_protocol *msg)
{
    uint8_t expoTime_expect = msg->data1;

    if (get_hv_state() != HVPS_SM_ID_IDLE) {
        send_message(msg->msg_id, SCI_SETUP_SM_ERROR, SCI_SETUP_SM_ERROR);
    }

    if ((expoTime_expect > para_range.expo_time_min) && (expoTime_expect > para_range.expo_time_max)) {
        /* 换算成实际的周期数 */
        config_data.expo_time_expect = expoTime_expect * COUNTER_TIMER6_FREQ;

        send_message(msg->msg_id, SCI_SETUP_SUCCESS, SCI_SETUP_SUCCESS);
    } else {

        send_message(msg->msg_id, SCI_SETUP_OUT_LIMIT, SCI_SETUP_OUT_LIMIT);
    }

    return;
}

void Inqixay1HVPSCurrentset(message_protocol *msg)
{
    return;
}

void Inqmaxtimeset(message_protocol *msg)
{
    uint8_t data1 = (uint8_t)(config_data.expo_time_expect / COUNTER_TIMER6_FREQ);
    uint8_t data2 = 0;

    send_message(msg->msg_id, data1, data2);

    return;
}

void Lampcontrol(message_protocol *msg)
{
    if (get_hv_state() == HVPS_SM_ID_IDLE) {
        uint16_t value = (msg->data1 == 1) ? 1 : 0;

        config_filamentOn_signal(value);
        ctrl_data.filament_on = value;
        xray_data.timmer_count = 1;
        config_data.fila_ref_step = (float)IDLE_FILAMENT_REF / (20 * 50); /* 20ms上升时间 */

        if (value == 0) disable_filamentref();

        send_message(msg->msg_id, msg->data1, msg->data2);

    }

    return;
}

void Autocalibra(message_protocol *msg)
{
    hvps_sm_state state = get_hv_state();

    if ((msg->data2 == 0) && (state == HVPS_SM_ID_IDLE)) {
        ctrl_data.enable = 1;
        calibrate_para_init();
        set_hv_state(HPVS_SM_ID_CAL_PREPARE);
        send_message(msg->msg_id, 0, 0);
    } else if ((msg->data2 == 1) && Is_CalibrateMode()) {
        ctrl_data.enable = 0;
        send_message(msg->msg_id, 0, 1);
    } else {
        send_message(msg->msg_id, msg->data1, msg->data2);
    }

    return;
}

void xsourcetype(message_protocol *msg)
{
    uint16_t xsource_value = (version.xsrc_ver_high << 11) | (version.xsrc_ver_high << 3) | version.xsrc_ver_high;

    uint8_t data1 = xsource_value >> 8;
    uint8_t data2 = xsource_value & 0xFF;

    send_message(msg->msg_id, data1, data2);

    return;
}

void Inqautocalibra(message_protocol *msg)
{
    uint8_t reply;

    if (Is_CalibrateMode()) {
        reply = 0;
    } else if (get_hv_state() == HVPS_SM_ID_FAULT) {
        reply = 2;
    } else {
        reply = 1;
    }

    send_message(msg->msg_id, 0, reply);

    return;
}

void exp1countclr(message_protocol *msg)
{
    uint8_t data1, data2;
    if (get_hv_state() == HVPS_SM_ID_IDLE) {
        parm_table.expo_count_total = (msg->data1 << 8) + msg->data2;
        data1 = 0;
        data2 = 0;
        save_parament_to_flash();
    } else {
        data1 = 0;
        data2 = 1;
    }

    send_message(msg->msg_id, data1, data2);

    return;
}

void exp1timeclr(message_protocol *msg)
{
    uint8_t data1, data2;
    if (get_hv_state() == HVPS_SM_ID_IDLE) {
        parm_table.expo_times_total = (msg->data1 << 8) + msg->data2;
        data1 = 0;
        data2 = 0;
        save_parament_to_flash();
    } else {
        data1 = 0;
        data2 = 1;
    }

    send_message(msg->msg_id, data1, data2);

    return;
}

void setpfcthreshold(message_protocol *msg)
{
    return;
}

void set24vthreshold(message_protocol *msg)
{
    return;
}

void setkvmathreshold(message_protocol *msg)
{
    return;
}

void setbuckllcthreshold(message_protocol *msg)
{
    return;
}

void debug_expo_ctrl(message_protocol *msg)
{
    hvps_sm_state state = get_hv_state();

    if ((msg->data2 == 0) && (state == HVPS_SM_ID_IDLE)) {
        ctrl_data.enable = 1;
        set_hv_state(HVPS_SM_ID_TRAIN_PREPARE);
        send_message(msg->msg_id, 0, 0);
    } else if ((msg->data2 == 1) && Is_DebugMode()) {
        ctrl_data.enable = 0;
        send_message(msg->msg_id, 0, 1);
    } else {
        send_message(msg->msg_id, msg->data1, msg->data2);
    }

    return;
}
