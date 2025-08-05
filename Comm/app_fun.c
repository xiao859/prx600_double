/*
 *  app_funcs.c
 *
 *  Created on: Feb 25, 2025
 *  Author: Administrator
 *
 */

#include "app_fun.h"
#include <stdint.h>
#include "math.h"
#include "xray.h"
#include "delay.h"
#include "protect.h"
#include <string.h>
#include "ct_exposure.h"
#include "calibrate.h"

volatile  xray_version version =
{
    1, 1,
    3, 0, 0,            /*软件版本*/
    2, 1, 9, 0,
    17, 18,             /*球管类型*/
    3, 6, 0
};

void invalid_cmd_reply()
{
    uint8_t data1 = 0xFF;
    uint8_t data2 = 0xFF;

    send_message(SCI_MSG_INQ_SW, data1, data2);

    return;
}

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
    {SCI_MSG_INQ_TUBE_LAST_V,               &fun_null},
    {SCI_MSG_INQ_TUBE_LAST_I,               &fun_null},
    {SCI_MSG_INQ_TUBE_LAST_EXPOTIME,        &fun_null},
    {SCI_MSG_INQ_LAMP_SW,                   &lampswversion},
    {SCI_MSG_INQ_LAMP_HW,                   &lamphwversion},
    {SCI_MSG_INQ_EXPO_TIME1,                &InqHVPSExpo_Time1},
    {SCI_MSG_INQ_EXPO_TIME2,                &InqHVPSExpo_Time2},
    {SCI_MSG_INQ_EXPO_COUNT1,               &InqHVPSExpo_Count1},
    {SCI_MSG_INQ_EXPO_COUNT2,               &InqHVPSExpo_Count2},
    {SCI_MSG_INQ_AUTOCALIBRA,               &Inqautocalibra},
    {SCI_MSG_INQ_XSOURCE_SW,                &xsourcetype},

    {SCI_MSG_SET_MODE,                      &SetHVPSMode},
    {SCI_MSG_SET_TUBE_V,                    &SetHV1TubeVoltageandcurrent},
    {SCI_MSG_SET_TUBE_I,                    &SetHV2TubeVoltageandcurrent},
    {SCI_MSG_SET_MAX_TIME,                  &Setmaxexpotime},
    {SCI_MSG_SET_EXP1_COUNTCLR,             &exp1countclr},
    {SCI_MSG_SET_EXP2_COUNTCLR,             &exp2countclr},
    {SCI_MSG_SET_EXP1_TIMECLR,              &exp1timeclr},
    {SCI_MSG_SET_EXP2_TIMECLR,              &exp2timeclr},
    {SCI_MSG_SET_NULL,                      &fun_null},
    {SCI_MSG_SET_ENABLE,                    &Setenable},

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



    {SCI_MSG_DEBUG_LAMP_I_SET,              &SetHVLampCurrent},
    {SCI_MSG_DEBUG_TUBE_VIDLE_SET,          &SetHVTubeIdleVoltage},
    {SCI_MSG_DEBUG_TUBE_VRISE_TIME_SET,     &SetHVTubeRisingTime},
    {SCI_MSG_DEBUG_ONLINE_PI,               &SetOnlinePI},
    {SCI_MSG_DEBUG_EXPO_CTRL,               &debug_expo_ctrl},

    {SCI_MSG_TEST_1,                        &test_func1},
    {SCI_MSG_TEST_2,                        &test_func2},
    {SCI_MSG_TEST_3,                        &test_func3},
    {SCI_MSG_TEST_4,                        &test_func4},
};

void fun_null(message_protocol *msg)
{
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
    /* 单位0.1°*/
    uint16_t temperature = (uint16_t)(sampled_data.oil_temp);

    uint8_t data1 = temperature >> 8;
    uint8_t data2 = temperature & 0xFF;

    send_message(msg->msg_id, data1, data2);

    return;
}

void InqHVPSExpo_Time1(message_protocol *msg)
{

    uint8_t data1 = parm_table[0].expo_times_total >> 8;
    uint8_t data2 = parm_table[0].expo_times_total & 0xFF;

    send_message(msg->msg_id, data1, data2);

    return;
}

void InqHVPSExpo_Time2(message_protocol *msg)
{

    uint8_t data1 = parm_table[1].expo_times_total >> 8;
    uint8_t data2 = parm_table[1].expo_times_total & 0xFF;

    send_message(msg->msg_id, data1, data2);

    return;
}

void InqHVPSExpo_Count1(message_protocol *msg)
{
    uint8_t data1 = parm_table[0].expo_count_total >> 8;
    uint8_t data2 = parm_table[0].expo_count_total & 0xFF;

    send_message(msg->msg_id, data1, data2);

    return;
}

void InqHVPSExpo_Count2(message_protocol *msg)
{
    uint8_t data1 = parm_table[1].expo_count_total >> 8;
    uint8_t data2 = parm_table[1].expo_count_total & 0xFF;

    send_message(msg->msg_id, data1, data2);

    return;
}

void SetHVPSMode(message_protocol *msg)
{
    msg->data2 = msg->data1;
    if ((get_hv_state(0) == HVPS_SM_ID_IDLE) && get_hv_state(1) == HVPS_SM_ID_IDLE)
    {
        if (msg->data1 < 4)
        {
            switch (msg->data1)
            {
            case HVPS_MODE_S_CONTINUOUS:
                ctrl_data.xrayMode = XRAY_MODE_S_CONTINUOUS;
                //DMA地址
                ctrl_data.xray_current = 1;
                msg->data1 = SETUP_SUCCESS;
                break;
            case HVPS_MODE_S_PULSE:
                ctrl_data.xrayMode = XRAY_MODE_S_PULSE;
                ctrl_data.xray_current = 1;
                msg->data1 = SETUP_SUCCESS;
                break;
            case HVPS_MODE_D_CONTINUOUS:
                ctrl_data.xrayMode = XRAY_MODE_D_CONTINUOUS;
                msg->data1 = SETUP_SUCCESS;
                break;
            case HVPS_MODE_D_PULSE:
                ctrl_data.xrayMode = XRAY_MODE_D_PULSE;
                msg->data1 = SETUP_SUCCESS;
                break;
            default:
                msg->data2 = SETUP_OUT_LIMIT;
            }
            config_disable_sw(1);
            config_enable_sw(0); // 选取射源0作为采样
        }
        else
            msg->data1 = SETUP_OUT_LIMIT;
    }
    else
        msg->data1 = SETUP_SM_ERROR;

    send_message(msg->msg_id, msg->data1, msg->data2);

    return;
}


void SetHV1TubeVoltageandcurrent(message_protocol *msg)
{
    uint8_t data1, data2;
    if ((get_hv_state(0) == HVPS_SM_ID_IDLE) && (get_hv_state(1) == HVPS_SM_ID_IDLE))
    {
        if ((msg->data2 <= para_range.tube_vol_max_config) && (msg->data2 >= para_range.tube_vol_min_config))
        {
            config_data.tube_vol[0]  = msg->data2;
            data2 = SETUP_SUCCESS;
        }
        else
        {
            mHVPS_Fault.FAULT_REG4.bit.VOL_CURR_OV = 1;
            data1 = SETUP_OUT_LIMIT;
        }

        if ((msg->data1 <= para_range.tube_curr_max_config) && (msg->data1 >= para_range.tube_curr_min_config))
        {
            config_data.tube_curr[0] = ((float)msg->data1) / 10;
            config_data.tube_vol_realtime[0] = 0;
            config_data.tube_vol_step[0] = (float)(config_data.tube_vol[0] - IDLE_HV_REF) / (20 * parm_table[0].rising_time);
            data1 = SETUP_SUCCESS;
        }
        else
        {
            mHVPS_Fault.FAULT_REG4.bit.VOL_CURR_OV = 1;
            data1 = SETUP_OUT_LIMIT;
        }
    }
    else
    {
        data1 = SETUP_SM_ERROR;
        data2 = SETUP_SM_ERROR;
    }

    send_message(msg->msg_id, data1, data2);

    return;
}

void SetHV2TubeVoltageandcurrent(message_protocol *msg)
{
    uint8_t data1, data2;
    if ((get_hv_state(0) == HVPS_SM_ID_IDLE) && (get_hv_state(1) == HVPS_SM_ID_IDLE))
    {
        if ((msg->data2 <= para_range.tube_vol_max_config) && (msg->data2 >= para_range.tube_vol_min_config))
        {
            config_data.tube_vol[1]  = msg->data2;
            data2 = SETUP_SUCCESS;
        }
        else
        {
            mHVPS_Fault.FAULT_REG4.bit.VOL_CURR_OV = 1;
            data1 = SETUP_OUT_LIMIT;
        }

        if ((msg->data1 <= para_range.tube_curr_max_config) && (msg->data1 >= para_range.tube_curr_min_config))
        {
            config_data.tube_curr[1] = ((float)msg->data1) / 10;
            config_data.tube_vol_realtime[1] = 0;
            config_data.tube_vol_step[1] = (float)(config_data.tube_vol[1] - IDLE_HV_REF) / (20 * parm_table[1].rising_time);
            data1 = SETUP_SUCCESS;
        }
        else
        {
            mHVPS_Fault.FAULT_REG4.bit.VOL_CURR_OV = 1;
            data2 = SETUP_OUT_LIMIT;
        }
        //      xray_data.timmer_count = 1;
    }
    else
    {
        data1 = SETUP_SM_ERROR;
        data2 = SETUP_SM_ERROR;
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
    if ((get_hv_state(0) != HVPS_SM_ID_IDLE) && (get_hv_state(1) != HVPS_SM_ID_IDLE))
    {
        send_message(msg->msg_id, SETUP_SM_ERROR, SETUP_SM_ERROR);
    }

    if (msg->data1 >= 2 && msg->data1 <= 20)
    {
        parm_table[msg->data2].rising_time = (uint32_t)msg->data1;
        send_message(msg->msg_id, SETUP_SUCCESS, SETUP_SUCCESS);
        save_parament_to_flash();
    }
    else
    {
        send_message(msg->msg_id, SETUP_OUT_LIMIT, SETUP_OUT_LIMIT);
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

    if ((get_hv_state(0) == HVPS_SM_ID_FAULT) || (get_hv_state(1) == HVPS_SM_ID_FAULT))
    {
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

        set_hv_state(HVPS_SM_ID_IDLE, 0);
        set_hv_state(HVPS_SM_ID_IDLE, 1);
        data1 = 0x00;
        data2 = 0x00;

    }
    else
    {
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
    if ((get_hv_state(0) != HVPS_SM_ID_IDLE) && (get_hv_state(1) != HVPS_SM_ID_IDLE))
    {
        send_message(msg->msg_id, SETUP_SM_ERROR, SETUP_SM_ERROR);
        return;
    }

    if ((msg->data1 > para_range.expo_time_min) && (msg->data1 > para_range.expo_time_max))
    {
        /* 换算成实际周期值 */
        config_data.expo_time_expect[1] = msg->data1 * COUNTER_TIMER6_FREQ;

        send_message(msg->msg_id, SETUP_SUCCESS, SETUP_SUCCESS);
    }
    else
        send_message(msg->msg_id, SETUP_OUT_LIMIT, SETUP_OUT_LIMIT);

    if ((msg->data2 > para_range.expo_time_min) && (msg->data2 > para_range.expo_time_max))
    {
        config_data.expo_time_expect[0] = msg->data2 * COUNTER_TIMER6_FREQ;

        send_message(msg->msg_id, SETUP_SUCCESS, SETUP_SUCCESS);
    }
    else
        send_message(msg->msg_id, SETUP_OUT_LIMIT, SETUP_OUT_LIMIT);

    return;
}

void Inqixay1HVPSCurrentset(message_protocol *msg)
{
    uint8_t data1, data2;

    data1 = 90;
    data2 = 60;
    send_message(msg->msg_id, data1, data2);
    return;
}

void Inqmaxtimeset(message_protocol *msg)
{
    uint8_t data1 = (uint8_t)(config_data.expo_time_expect[0] / COUNTER_TIMER6_FREQ);
    uint8_t data2 = (uint8_t)(config_data.expo_time_expect[0] / COUNTER_TIMER6_FREQ);

    send_message(msg->msg_id, data1, data2);

    return;
}

void Lampcontrol(message_protocol *msg)
{
    uint16_t combined = ((uint16_t)(msg->data2 & 0xFF) << 8) | (msg->data1 & 0xFF);

    switch (combined)
    {
    case 0x0100:  // data2=0x01, data1=0x00
        if ((get_hv_state(0) == HVPS_SM_ID_IDLE) && (get_hv_state(1) == HVPS_SM_ID_IDLE))
        {
            config_filamentOn_signal(1, 0);
            ctrl_data.filament_on[0] = 1;
            xray_data.timmer_count[0] = 1;
            config_data.fila_ref_step[0] = (float)IDLE_FILAMENT1_REF / (20 * 50); /* 20ms上升时间*/
            msg->data2 = SETUP_SUCCESS;  // 明确设置成功码
            config_disable_sw(1);
            config_enable_sw(0);
        }
        else
        {
            msg->data2 = SETUP_SM_ERROR;
        }
        break;

    case 0x0101:  // data2=0x01, data1=0x01
        if ((get_hv_state(0) == HVPS_SM_ID_IDLE) && (get_hv_state(1) == HVPS_SM_ID_IDLE))
        {
            config_filamentOn_signal(1, 0);
            ctrl_data.filament_on[0] = 1;
            xray_data.timmer_count[0] = 1;
            config_data.fila_ref_step[0] = (float)IDLE_FILAMENT1_REF / (20 * 50); /* 20ms上升时间*/
            config_filamentOn_signal(1, 1);
            ctrl_data.filament_on[1] = 1;
            xray_data.timmer_count[1] = 1;
            config_data.fila_ref_step[1] = (float)IDLE_FILAMENT2_REF / (20 * 50); /* 20ms上升时间*/
            msg->data1 = SETUP_SUCCESS;
            msg->data2 = SETUP_SUCCESS;
            config_disable_sw(1);
            config_enable_sw(0);
        }
        else
        {
            msg->data1 = SETUP_SM_ERROR;
            msg->data2 = SETUP_SM_ERROR;
        }
        break;
    case 0x0000:
        config_filamentOn_signal(0, 0);
        ctrl_data.filament_on[0] = 0;
        config_filamentOn_signal(0, 1);
        ctrl_data.filament_on[1] = 0;
        msg->data1 = SETUP_SUCCESS;
        msg->data2 = SETUP_SUCCESS;
        config_disable_sw(1);
        config_enable_sw(0);
        break;
    case 0x0001:  // data2=0x00, data1=0x01
        if ((get_hv_state(0) == HVPS_SM_ID_IDLE) && (get_hv_state(1) == HVPS_SM_ID_IDLE))
        {
            config_filamentOn_signal(1, 1);
            ctrl_data.filament_on[1] = 1;
            xray_data.timmer_count[1] = 1;
            config_data.fila_ref_step[1] = (float)IDLE_FILAMENT2_REF / (20 * 50); /* 20ms上升时间*/
            msg->data1 = SETUP_SUCCESS;  // 明确设置成功码
            config_disable_sw(0);
            config_enable_sw(1);
        }
        else
        {
            msg->data1 = SETUP_SM_ERROR;  // 状态不符合条件
        }
        break;
    default:
        msg->data1 = SETUP_OUT_LIMIT;
        msg->data2 = SETUP_OUT_LIMIT;

    }

    send_message(msg->msg_id, msg->data1, msg->data2);
    return;
}

void Autocalibra(message_protocol *msg)
{
    if ((get_hv_state(0) == HVPS_SM_ID_IDLE) && (get_hv_state(1) == HVPS_SM_ID_IDLE))
    {
        if (msg->data2 == 0)
        {
            ctrl_data.enable[0] = 1;
            ctrl_data.enable[1] = 1;
            calibrate_para_init();
            set_hv_state(HPVS_SM_ID_CAL_PREPARE, 0);
            set_hv_state(HPVS_SM_ID_CAL_PREPARE, 1);
            send_message(msg->msg_id, 0, 0);
        }
        else if ((msg->data2 == 1) && (Is_CalibrateMode()))
        {
            ctrl_data.enable[0] = 0;
            ctrl_data.enable[1] = 0;
            send_message(msg->msg_id, 0, 1);
        }
        else
        {
            send_message(msg->msg_id, msg->data1, msg->data2);
        }
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

    if (Is_CalibrateMode())
    {
        reply = 0;
    }
    else if ((get_hv_state(0) == HVPS_SM_ID_FAULT) && (get_hv_state(0) == HVPS_SM_ID_FAULT))
    {
        reply = 2;
    }
    else
    {
        reply = 1;
    }

    send_message(msg->msg_id, 0, reply);

    return;
}

void exp1countclr(message_protocol *msg)
{
    uint8_t data1, data2;
    if ((get_hv_state(0) == HVPS_SM_ID_IDLE) && (get_hv_state(1) == HVPS_SM_ID_IDLE))
    {
        parm_table[0].expo_count_total = (msg->data1 << 8) + msg->data2;
        data1 = 0;
        data2 = 0;
        save_parament_to_flash();
    }
    else
    {
        data1 = 0;
        data2 = 1;
    }

    send_message(msg->msg_id, data1, data2);

    return;
}

void exp2countclr(message_protocol *msg)
{
    uint8_t data1, data2;
    if ((get_hv_state(0) == HVPS_SM_ID_IDLE) && (get_hv_state(1) == HVPS_SM_ID_IDLE))
    {
        parm_table[1].expo_count_total = (msg->data1 << 8) + msg->data2;
        data1 = 0;
        data2 = 0;
        save_parament_to_flash();
    }
    else
    {
        data1 = 0;
        data2 = 1;
    }

    send_message(msg->msg_id, data1, data2);

    return;
}

void exp1timeclr(message_protocol *msg)
{
    if ((get_hv_state(0) == HVPS_SM_ID_IDLE) && (get_hv_state(1) == HVPS_SM_ID_IDLE))
    {
        parm_table[0].expo_times_total = (msg->data1 << 8) + msg->data2;
        msg->data1 = SETUP_SUCCESS;
        msg->data2 = SETUP_SUCCESS;
    }
    else
    {
        msg->data1 = SETUP_SUCCESS;
        msg->data2 = SETUP_SM_ERROR;
    }
    send_message(msg->msg_id, msg->data1, msg->data2);

    return;
}

void exp2timeclr(message_protocol *msg)
{
    if ((get_hv_state(0) == HVPS_SM_ID_IDLE) && (get_hv_state(1) == HVPS_SM_ID_IDLE))
    {
        parm_table[1].expo_times_total = (msg->data1 << 8) + msg->data2;
        msg->data1 = SETUP_SUCCESS;
        msg->data2 = SETUP_SUCCESS;
    }
    else
    {
        msg->data1 = SETUP_SM_ERROR;
        msg->data2 = SETUP_SUCCESS;
    }
    send_message(msg->msg_id, msg->data1, msg->data2);

    return;
}

void Setenable(message_protocol *msg)
{

        ctrl_data.enable[0] = msg->data2;
        ctrl_data.enable[1] = msg->data1;
        msg->data1 = SETUP_SUCCESS;
        msg->data2 = SETUP_SUCCESS;
   

 //   send_message(msg->msg_id, msg->data1, msg->data2);

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
    if ((get_hv_state(0) == HVPS_SM_ID_IDLE) && (get_hv_state(1) == HVPS_SM_ID_IDLE))
    {
        ctrl_data.enable[0] = msg->data2;
        ctrl_data.enable[1] = msg->data1;
        msg->data1 = SETUP_SUCCESS;
        msg->data2 = SETUP_SUCCESS;
    }
    else
    {
        msg->data1 = SETUP_SM_ERROR;
        msg->data2 = SETUP_SM_ERROR;
    }

    send_message(msg->msg_id, msg->data1, msg->data2);

    return;
}

void test_func1(message_protocol *msg)
{
//    gpio_input_trigger();

//    //return;

//    gpio_output_set1_test();

//    //return;
//    tube_vol_ref = (uint32_t)(0 / ADDA_FULL_SCALE_VIL_VALUE * 4095);

//    fila_vol_ref = (uint32_t)(1 / ADDA_FULL_SCALE_VIL_VALUE * 4095);

//    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, tube_vol_ref);
//    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, fila_vol_ref);

    return;
}

void test_func2(message_protocol *msg)
{
//    gpio_input_nottrigger();

//    //return;

//    gpio_output_set2_test();
//    //return;
//    tube_vol_ref = (uint32_t)(1 / ADDA_FULL_SCALE_VIL_VALUE * 4095);

//    fila_vol_ref = (uint32_t)(2 / ADDA_FULL_SCALE_VIL_VALUE * 4095);

//    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, tube_vol_ref);
//    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, fila_vol_ref);

    return;
}

void test_func3(message_protocol *msg)
{
//    tube_vol_ref = (uint32_t)(3.3f / ADDA_FULL_SCALE_VIL_VALUE * 4095);

//    fila_vol_ref = (uint32_t)(1 / ADDA_FULL_SCALE_VIL_VALUE * 4095);

//    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, tube_vol_ref);
//    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, fila_vol_ref);

    return;
}

void test_func4(message_protocol *msg)
{
//    tube_vol_ref = 4095;

//    fila_vol_ref = 4095;

//    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, tube_vol_ref);
//    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, fila_vol_ref);

    return;
}


void InqHVPSFault(message_protocol *msg)
{
    static uint16_t fault_index ;
    uint16_t i;
    uint16_t *pFault;
    uint8_t data1, data2;
    fault_index ++;
    pFault = (uint16_t*)(&mHVPS_Fault);
    for (i = 0; i < sizeof(HVPS_FAULT_REGS); i++)
    {
        uint16_t fault_count = 0;
        uint16_t i = 0, j, fault_bit;
        uint16_t fault_temp;

        uint16_t start_fault_ID[sizeof(HVPS_FAULT_REGS)] = {0x00, 0x20, 0x30, 0xA0, 0xB0, 0xC0};

        pFault = (uint16_t*)(&mHVPS_Fault);
        for (i = 0; i < sizeof(HVPS_FAULT_REGS); i++)
        {
            fault_temp = *pFault++;
            for (j = 0; j < 16; j++)
            {
                fault_bit  = fault_temp & 0x1;  // 取最低位
                fault_temp = fault_temp >> 1;
                if (fault_bit)  // 有故障
                {
                    fault_count++;//当前故障计数
                    if (fault_index == fault_count)
                    {
                        data2 = start_fault_ID[i] + j;
                    }
                    data1 = fault_count;
                }
            }
        }
        if (fault_index >= fault_count) fault_index = 0;
    }
    send_message(msg->msg_id, data1, data2);

    return;
}


void cmd_process(int32_t message_idx, message_protocol* msg, USART_TypeDef *Instance)
{
    uint32_t func_idx;

    if (message_idx >= SCI_MSG_TEST_1)
    {
        func_idx = message_idx - SCI_MSG_TEST_1 + 47;
    }
    else if (message_idx >= SCI_MSG_DEBUG_LAMP_I_SET)
    {
        func_idx = message_idx - SCI_MSG_DEBUG_LAMP_I_SET + 42;
    }
    else if (message_idx >= SCI_MSG_SET_PFCTHRESHOLD)
    {
        func_idx = message_idx - SCI_MSG_SET_PFCTHRESHOLD + 38;
    }
    else if (message_idx >= SCI_MSG_CTRL_RST)
    {
        func_idx = message_idx - SCI_MSG_CTRL_RST + 30;
    }
    else if (message_idx >= SCI_MSG_SET_MODE)
    {
        func_idx = message_idx - SCI_MSG_SET_MODE + 20;
    }
    else if (message_idx >= SCI_MSG_INQ_MODE)
    {
        func_idx = message_idx - SCI_MSG_INQ_MODE;
    }
    else
    {
        invalid_cmd_reply();
        return;
    }

    if ((func_idx >= APP_FUNC_NUM) || (funcs[func_idx].msgId != message_idx))
    {
        invalid_cmd_reply();
    }
    else
    {
        funcs[func_idx].func_ptr(msg);
    }
}

/* AA 55 33 00 00 CD */
uint8_t message_check(USART_TypeDef *Instance)
{
    uint8_t result = 0;

    if (Instance == UART4)
    {
        if ((uart4.uart_rx_buf[0] == 0xAA) && (uart4.uart_rx_buf[1] == 0x55) &&
                (((uart4.uart_rx_buf[2] + uart4.uart_rx_buf[3] + uart4.uart_rx_buf[4] + uart4.uart_rx_buf[5]) & 0xFF) == 0))
        {
            result = 1;
        }
    }
    else if (Instance == UART5)
    {
        if ((uart5.uart_rx_buf[0] == 0xAA) && (uart5.uart_rx_buf[1] == 0x55) &&
                (((uart5.uart_rx_buf[2] + uart5.uart_rx_buf[3] + uart5.uart_rx_buf[4] + uart5.uart_rx_buf[5]) & 0xFF) == 0))
        {
            result = 1;
        }
    }

    return result;
}

void cmd_parser()
{
//    if (uart4_frame_fifo.count > 0)
//    {
//        uint8_t idx = uart4_frame_fifo.head;
//        message_protocol* frame = & uart4_frame_fifo.data[idx];

//        cmd_process(frame->msg_id, frame, UART4);
//        // 出队
//        uart4_frame_fifo.head = (uart4_frame_fifo.head + 1) % FRAME_BUF_NUM;
//        uart4_frame_fifo.count--;
//    }

    if (uart5_frame_fifo.count > 0)
    {
        uint8_t idx = uart5_frame_fifo.head;
        message_protocol* frame = & uart5_frame_fifo.data[idx];

        // 出队
        uart5_frame_fifo.head = (uart5_frame_fifo.head + 1) % FRAME_BUF_NUM;
        uart5_frame_fifo.count--;

        cmd_process(frame->msg_id, frame, UART5);

    }
    return;
}

