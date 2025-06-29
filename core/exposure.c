///*
// *  CT_mode.c
// *
// *  Created on: Feb 24, 2025
// *  Author: Administrator
// *
// */

#include "exposure.h"
//#include "calibrate.h"
//#include <stdio.h>
//#include "comm_string.h"
//#include "comm_protocol.h"
//#include "xray.h"
//#include "delay.h"
//#include "pi_control.h"
//#include "app_uart.h"

////struct CONVERTER_VAR_STRUCT HVPS_Regs;

//volatile xray_running_data xray_data = {0};

///* CT 模式关闭放线 */


//float kp_test = 3.5;
//float ki_test = 0.0009;
//extern uint32_t test_flag;

//void lamp_on(uint16_t n)
//{
//	if(n<XRAY_NUMS)
//	{
//		config_filamentOn_signal(1,n);
//	}
//}

//void lamp_off(uint16_t n)
//{
//	if(n<XRAY_NUMS)
//	{
//		config_filamentOn_signal(0,n);
//	}
//}

//void ct_task()
//{
////    if (xray_data.timmer_count >= 1)
////			xray_data.timmer_count++;

//    /* 异常场景，立马关闭  */
////		if(HVPS_IsFault())
//		{
//			xray_CT_disable(0);
//			xray_CT_disable(1);	
//		}
//		uint8_t xray_num = 0;
//    switch(hv_state[xray_num])
//    {
//			case HVPS_SM_ID_IDLE:
//				hvState_ilde_init(xray_num);
//				config_ready_signal(1);
//				config_xrayOn_signal(1);

//			/* 开启灯丝后，需要预热2.5秒 */
//        if ((ctrl_data.enable[xray_num] == 1) && (ctrl_data.interlock == 1) &&
//            (ctrl_data.filament_on[xray_num] == 1) && (xray_data.timmer_count > TIMER6_2P5_SECOND_CYCLES))
//				{
//            config_mcuLock_signal(1);
//            set_hv_state(HVPS_SM_ID_PREPARE,xray_num);
//            xray_data.timmer_count = 1;
//            config_data.expo_count[xray_num] = 0;
//        }
//				
//				if (ctrl_data.filament_on[xray_num] == 1) {
//            config_filament_ref_slop(xray_num);
//        }
//			break;
//				
//			case HVPS_SM_ID_PREPARE:
//				config_filamentRef(xray_num);     /* 灯丝基准拉到预期值 */

//        /* 通知MCU已准备好 */
//        config_ready_signal(1);
//        set_hv_state(HVPS_SM_ID_READY,xray_num);
//        xray_data.timmer_count = 0;
//        uint32_t currRef = config_data.fila_ref_realtime[xray_num];

//        pid_Init(config_data.tube_curr[xray_num], currRef, Is_PulseMode_CT());
//				break;
//			
//			case HVPS_SM_ID_READY:
//				config_hvref_slope(xray_num);
//        if (ctrl_data.enable[xray_num] && ctrl_data.expo[xray_num]) 
//				{
//            /* 给高压射源 */
//            config_HVEn_signal(1);

//            /* 通知MCU正在曝光 */
//            config_xrayOn_signal(1);

//            set_hv_state(HVPS_SM_ID_EXPOSURING,xray_num);
//            xray_data.timmer_count = 1;
//        }
//				else
//					  config_filamentOn_signal(0,1);

//        /* EXP先关，ENABLE后关 */
//        if (ctrl_data.enable[xray_num] == 0) {
//            if (config_data.expo_count_total[xray_num] > 1) {
//                parm_table[xray_num].expo_count_total++;
//                /* 曝光60s加一次 */
//                parm_table[xray_num].expo_times_total += config_data.expo_count_total[xray_num] / 3000000;
//            }
//            config_xrayOn_signal(0);
//            set_hv_state(HVPS_SM_ID_EXPO_END,xray_num);
//        }
//				
//			case HVPS_SM_ID_EXPOSURING:
//				config_hvref_slope(xray_num);
//			
//				if (xray_data.timmer_count == TIMER6_8_MILSECOND_CYCLES) {
//            user_pid.currValue = sampled_data.tube_curr_value;
//        }

//        /* 连续模式下，每隔10ms调一次 */
//        // if (Is_ContinuousMode_CT() && (xray_data.timmer_count > TIMER6_8_MILSECOND_CYCLES)) {
//        if ((xray_data.timmer_count > TIMER6_5_MILSECOND_CYCLES)) {
//            xray_data.isCheckAvailable[xray_num] = 1;
//           if (xray_data.timmer_count % TIMER6_10_MILSECOND_CYCLES == 0) {
//               user_pid.currValue = sampled_data.tube_curr_value;
//               user_pid.Kp = kp_test;
//	           // user_pid.Ti = 0.005;
//               user_pid.Ti = ki_test;

//                tube_current_piControl(1);
//           }
//        } else {
//            xray_data.isCheckAvailable[xray_num] = 0;
//        }

//        /* ENABLE先关，EXP后关 */
//        if (ctrl_data.enable[xray_num] == 0) {
//            xray_data.isCheckAvailable[xray_num] = 0;
//            parm_table[xray_num].expo_count_total++;
//            /* 曝光60s加一次 */
//            parm_table[xray_num].expo_times_total += config_data.expo_count_total[xray_num] / 3000000;
//            config_xrayOn_signal(0);
//            set_hv_state(HVPS_SM_ID_EXPO_END,xray_num);
//        }

//        if (ctrl_data.enable[xray_num] && (ctrl_data.expo[xray_num] == 0)) {
//            xray_data.isCheckAvailable[xray_num] = 0;
//            config_xrayOn_signal(0);
//            set_hv_state(HVPS_SM_ID_READY,xray_num);
//            user_pid.Kp = 20;
//						user_pid.Ti = 1;
//            tube_current_piControl(0);
//            debug_tx3("闭环值: %f, %d, %f, %f\n",
//                param_pid.Out_pid, param_pid.config_ref, user_pid.currValue, param_pid.integral);
//        }

//        if (ctrl_data.enable[xray_num] && ctrl_data.expo[xray_num]) {
//            /* 曝光计数和超时判断 */
//            config_data.expo_count_total[xray_num]++;
//            config_data.expo_count[xray_num]++;
//        }
//				//
//				break;
//			

//       case HVPS_SM_ID_EXPO_END: 
//        ctrl_data.filament_on[xray_num] = 0;
//        xray_CT_disable(xray_num);
//        cali_data.para_save_flag[xray_num] = 1;

//        set_hv_state(HVPS_SM_ID_IDLE,xray_num);
//			 default:
//				   set_hv_state(HVPS_SM_ID_EXPOSURING,xray_num);
//		 }	

//    (get_hv_state(xray_num) == HVPS_SM_ID_EXPOSURING) ? xray_on_led(1) : xray_on_led(0);

//    if (get_filament_pin(xray_num) && (get_hv_state(xray_num) != HVPS_SM_ID_EXPOSURING)) {
//        config_data.fila_protect_cnt[xray_num]++;
//    } else {
//        config_data.fila_protect_cnt[xray_num] = 0;
//    }

//    return;
//}
