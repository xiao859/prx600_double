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
    uint8_t data1 = parm_table[0].expo_times_total >> 8;
    uint8_t data2 = parm_table[0].expo_times_total & 0xFF;

    send_message(msg->msg_id, data1, data2);

    return;
}

void InqHVPSExpo_Time2(message_protocol *msg)
{
    /* 需要换算一下 */
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
	  if ((get_hv_state(0) == HVPS_SM_ID_IDLE)&& get_hv_state(1) == HVPS_SM_ID_IDLE) 
		{
			if(msg->data1 < 4)
			{
				switch(msg->data1)
				{
					case HVPS_MODE_S_CONTINUOUS:
						xrayMode = XRAY_MODE_S_CONTINUOUS;
						break;
					case HVPS_MODE_S_PULSE:
						xrayMode = XRAY_MODE_S_PULSE;
						break;
					case HVPS_MODE_D_CONTINUOUS:
						xrayMode = XRAY_MODE_D_CONTINUOUS;
						break;
					case HVPS_MODE_D_PULSE:
						xrayMode = XRAY_MODE_D_PULSE;
						break;
					default:
						msg->data2 = SETUP_SM_ERROR;
				}
        msg->data2 = SETUP_SUCCESS;
			}
			else
			 msg->data2 = SETUP_SM_ERROR;
    } else 
       msg->data2 = SETUP_OUT_LIMIT;
		
		ctrl_data.xrayMode = xrayMode;
    send_message(msg->msg_id, msg->data1, msg->data2);

    return;
}

void SetHV1TubeVoltageandcurrent(message_protocol *msg)
{
    uint8_t data1, data2;

    /* 1、限幅 */
    if ((msg->data1 <= para_range[0].tube_vol_max_config) && (msg->data1 >= para_range[0].tube_vol_min_config)) {
        config_data.tube_vol[0]  = msg->data1;     /* 单位kV */
        data1 = SETUP_SUCCESS;
    } else {
        mHVPS_Fault.FAULT_REG4.bit.VOL_CURR_OV = 1;
        data1 = SETUP_OUT_LIMIT;
    }

    if ((msg->data2 <= para_range[0].tube_curr_max_config) && (msg->data2 >= para_range[0].tube_curr_min_config)) {
        config_data.tube_curr[0] = ((float)msg->data2) / 10;     /* 单位0.1mA */
        data1 = SETUP_SUCCESS;
    } else {
        mHVPS_Fault.FAULT_REG4.bit.VOL_CURR_OV = 1;
        data2 = SETUP_OUT_LIMIT;
    }

    if ((get_hv_state(0) == HVPS_SM_ID_IDLE)&& (get_hv_state(1) == HVPS_SM_ID_IDLE)) {
        config_data.tube_vol_realtime[0] = 0;
        config_data.tube_vol_step[0] = (float)(config_data.tube_vol[0] - IDLE_HV_REF) / (50 * parm_table[0].rising_time);
        xray_data.timmer_count = 1;
        // config_hvref_filamentRef();
    } else {
        data1 = SETUP_SM_ERROR;
        data2 = SETUP_SM_ERROR;
    }

    send_message(msg->msg_id, data1, data2);

    return;
}

void SetHV2TubeVoltageandcurrent(message_protocol *msg)
{
    uint8_t data1, data2;

    /* 1、限幅 */
    if ((msg->data1 <= para_range[1].tube_vol_max_config) && (msg->data1 >= para_range[1].tube_vol_min_config)) {
        config_data.tube_vol[1]  = msg->data1;     /* 单位kV */
        data1 = SETUP_SUCCESS;
    } else {
        mHVPS_Fault.FAULT_REG4.bit.VOL_CURR_OV = 1;
        data1 = SETUP_OUT_LIMIT;
    }

    if ((msg->data2 <= para_range[1].tube_curr_max_config) && (msg->data2 >= para_range[1].tube_curr_min_config)) {
        config_data.tube_curr[1] = ((float)msg->data2) / 10;     /* 单位0.1mA */
        data1 = SETUP_SUCCESS;
    } else {
        mHVPS_Fault.FAULT_REG4.bit.VOL_CURR_OV = 1;
        data2 = SETUP_OUT_LIMIT;
    }

    if ((get_hv_state(0) == HVPS_SM_ID_IDLE)&& (get_hv_state(1) == HVPS_SM_ID_IDLE)) {
        config_data.tube_vol_realtime[1] = 0;
        config_data.tube_vol_step[1] = (float)(config_data.tube_vol[1] - IDLE_HV_REF) / (50 * parm_table[0].rising_time);
        xray_data.timmer_count = 1;
        // config_hvref_filamentRef();
    } else {
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
    if ((get_hv_state(0) != HVPS_SM_ID_IDLE)&& (get_hv_state(1) != HVPS_SM_ID_IDLE)) {
        send_message(msg->msg_id, SETUP_SM_ERROR, SETUP_SM_ERROR);
    }

    if (msg->data1 >= 2 && msg->data1 <= 20) {
        parm_table[msg->data2].rising_time = (uint32_t)msg->data1;
        send_message(msg->msg_id, SETUP_SUCCESS, SETUP_SUCCESS);
        save_parament_to_flash();
    } else {
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

    if((get_hv_state(0) != HVPS_SM_ID_IDLE)&& (get_hv_state(1) != HVPS_SM_ID_IDLE)) {
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

        set_hv_state(HVPS_SM_ID_IDLE,0);
	      set_hv_state(HVPS_SM_ID_IDLE,1);		
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
    if ((get_hv_state(0) != HVPS_SM_ID_IDLE)&& (get_hv_state(1) != HVPS_SM_ID_IDLE)) {
        send_message(msg->msg_id, SETUP_SM_ERROR, SETUP_SM_ERROR);
    }

    if ((msg->data1 > para_range[1].expo_time_min) && (msg->data1 > para_range[1].expo_time_max)) {
        /* 换算成实际的周期数 */
        config_data.expo_time_expect[1] = msg->data1 * COUNTER_TIMER6_FREQ;

        send_message(msg->msg_id, SETUP_SUCCESS, SETUP_SUCCESS);
    } else 
        send_message(msg->msg_id, SETUP_OUT_LIMIT, SETUP_OUT_LIMIT);
		
		if ((msg->data2 > para_range[0].expo_time_min) && (msg->data2 > para_range[0].expo_time_max)) {
		/* 换算成实际的周期数 */
		config_data.expo_time_expect[0] = msg->data2 * COUNTER_TIMER6_FREQ;

		send_message(msg->msg_id, SETUP_SUCCESS, SETUP_SUCCESS);
    } else 
        send_message(msg->msg_id, SETUP_OUT_LIMIT, SETUP_OUT_LIMIT);

    return;
}

void Inqixay1HVPSCurrentset(message_protocol *msg)
{
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
		if ((get_hv_state(0) == HVPS_SM_ID_IDLE)&& (get_hv_state(1) == HVPS_SM_ID_IDLE)) {
			config_filamentOn_signal(msg->data1,1);
			config_filamentOn_signal(msg->data2,0);

			ctrl_data.filament_on[0] = msg->data2;
			ctrl_data.filament_on[1] = msg->data1;
			xray_data.timmer_count = 1;
			config_data.fila_ref_step[0] = (float)IDLE_FILAMENT_REF / (20 * 50); /* 20ms上升时间 */
			config_data.fila_ref_step[1] = (float)IDLE_FILAMENT_REF / (20 * 50); /* 20ms上升时间 */

			if (msg->data1 == 0) 
				disable_filamentref(1);
			else if(msg->data2 == 0)
				disable_filamentref(0);

			send_message(msg->msg_id, msg->data1, msg->data2);
	}

    return;
}

void Autocalibra(message_protocol *msg)
{
	if ((get_hv_state(0) == HVPS_SM_ID_IDLE)&& (get_hv_state(1) == HVPS_SM_ID_IDLE)) {
		if (msg->data2 == 0)
		{
			ctrl_data.enable[0] = 1;
			ctrl_data.enable[1] = 1;
			calibrate_para_init();
			set_hv_state(HPVS_SM_ID_CAL_PREPARE,0);
			set_hv_state(HPVS_SM_ID_CAL_PREPARE,1);
			send_message(msg->msg_id, 0, 0);
		}
		else if((msg->data2 == 1) && (Is_CalibrateMode()))
		{
				ctrl_data.enable[0] = 0;
				ctrl_data.enable[1] = 0;
        send_message(msg->msg_id, 0, 1);
		}
		else {
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

    if (Is_CalibrateMode()) {
        reply = 0;
    } else if ((get_hv_state(0) == HVPS_SM_ID_FAULT) && (get_hv_state(0) == HVPS_SM_ID_FAULT)) 
		{
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
    if ((get_hv_state(0) == HVPS_SM_ID_IDLE)&& (get_hv_state(1) == HVPS_SM_ID_IDLE)) {
        parm_table[0].expo_count_total = (msg->data1 << 8) + msg->data2;
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

void exp2countclr(message_protocol *msg)
{
    uint8_t data1, data2;
    if ((get_hv_state(0) == HVPS_SM_ID_IDLE)&& (get_hv_state(1) == HVPS_SM_ID_IDLE)) {
        parm_table[1].expo_count_total = (msg->data1 << 8) + msg->data2;
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
    if ((get_hv_state(0) == HVPS_SM_ID_IDLE)&& (get_hv_state(1) == HVPS_SM_ID_IDLE)) 
		{
			parm_table[0].expo_times_total = (msg->data1 << 8) + msg->data2;
			msg->data1 = SETUP_SUCCESS;
      msg->data2 = SETUP_SUCCESS;
    } else {
        msg->data1 = SETUP_SUCCESS;
        msg->data2 = SETUP_SM_ERROR; 
    }
    send_message(msg->msg_id, msg->data1, msg->data1);

    return;
}

void exp2timeclr(message_protocol *msg)
{
    if ((get_hv_state(0) == HVPS_SM_ID_IDLE)&& (get_hv_state(1) == HVPS_SM_ID_IDLE)) 
		{
			parm_table[1].expo_times_total = (msg->data1 << 8) + msg->data2;
			msg->data1 = SETUP_SUCCESS;
      msg->data2 = SETUP_SUCCESS;
    } else {
        msg->data1 = SETUP_SM_ERROR;
        msg->data2 = SETUP_SUCCESS; 
    }
    send_message(msg->msg_id, msg->data1, msg->data1);

    return;
}

void Setenable(message_protocol *msg)
{
    if ((get_hv_state(0) == HVPS_SM_ID_IDLE)&& (get_hv_state(1) == HVPS_SM_ID_IDLE)) {
       ctrl_data.enable[0] = msg->data1;
			 ctrl_data.enable[1] = msg->data2;
        msg->data1 = SETUP_SUCCESS;
        msg->data2 = SETUP_SUCCESS;
    } else {
        msg->data1 = SETUP_SM_ERROR;
        msg->data2 = SETUP_SM_ERROR;
    }

    send_message(msg->msg_id, msg->data1, msg->data2);

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
	  if ((get_hv_state(0) == HVPS_SM_ID_IDLE)&& (get_hv_state(1) == HVPS_SM_ID_IDLE)) {
       ctrl_data.enable[0] = msg->data2;
			 ctrl_data.enable[1] = msg->data1;
        msg->data1 = SETUP_SUCCESS;
        msg->data2 = SETUP_SUCCESS;
    } else {
        msg->data1 = SETUP_SM_ERROR;
        msg->data2 = SETUP_SM_ERROR;
    }

    send_message(msg->msg_id, msg->data1, msg->data2);

    return;
}
