#include "protect.h"
#include "xray.h"
#include "temperature.h"
#include "calibrate.h"

HVPS_FAULT_REGS mHVPS_Fault;

float oilerr=0;
void Protect_Check_Slow()
{
    /* 灯丝开启，长时间不曝光 */
    if (config_data.fila_protect_cnt[0] > para_range.fila_protect_cnt_max)
    {
        mHVPS_Fault.FAULT_REG4.bit.lamp_wait_overtime = 1;
    }
    if (config_data.fila_protect_cnt[1] > para_range.fila_protect_cnt_max)
    {
        mHVPS_Fault.FAULT_REG4.bit.lamp_wait_overtime2 = 1;
    }

//    if (ctrl_data.interlock == 0)
//    {
//        xray_data.interLock_count++;
//        if (xray_data.interLock_count > OVER_RANGE_TIME_LIMIT)
//        {
//            if (ctrl_data.xray_current == 1)
//                mHVPS_Fault.FAULT_REG1.bit.INTERLOCK1 = 1;
//            else
//                mHVPS_Fault.FAULT_REG4.bit.INTERLOCK2 = 1;
//        }
//        /* 计时，超限报警 */
//    }
//    else
//    {
//        xray_data.interLock_count = 0;
//    }

//    if (sampled_data.power_24v_value > para_range.power_24v_max_protected)
//    {
//        xray_data.pwr_24_overCount++;
//        if (xray_data.pwr_24_overCount > 20 * OVER_RANGE_TIME_LIMIT)
//            mHVPS_Fault.FAULT_REG2.bit.V24_OV = 1;
//    }
//    else
//    {
//        xray_data.pwr_24_overCount = 0;
//    }

//    if (sampled_data.power_24v_value < para_range.power_24v_min_protected)
//    {
//        xray_data.pwr_24_underCount++;
//        if (xray_data.pwr_24_underCount > 20 * OVER_RANGE_TIME_LIMIT)
//            mHVPS_Fault.FAULT_REG2.bit.V24_UV = 1;
//    }
//    else
//    {
//        xray_data.pwr_24_underCount = 0;
//    }

//    float temp_oil = get_temp(sampled_data.temp_oil_value);
//    sampled_data.oil_temp = temp_oil;
//    if (temp_oil > para_range.temp_oil_max_protected || temp_oil < para_range.temp_oil_min_protected)
//    {
//				oilerr= temp_oil;
//        mHVPS_Fault.FAULT_REG1.bit.OIL_TEMP1 = 1;
//    }

//    if (temp_oil > para_range.temp_oil_warning)
//    {
//        mHVPS_Fault.FAULT_REG1.bit.OIL_OT1 = 1;
//        // xray_data.oil_temp_errCount++;
//        // if (xray_data.oil_temp_errCount > OVER_RANGE_TIME_LIMIT) mHVPS_Fault.FAULT_REG1.bit.OIL_OT1 = 1;
//    }

//    if (mHVPS_Fault.FAULT_REG1.bit.OIL_OT1 == 1)
//    {
//        if (temp_oil < para_range.temp_oil_recovery)
//        {
//            mHVPS_Fault.FAULT_REG1.bit.OIL_OT1 = 0;
//        }
//        else
//        {
//            mHVPS_Fault.FAULT_REG1.bit.OIL_OT1 = 1;
//        }
//    }
    return;
}

void xray_system_fault_check()
{
    if (!Is_System_Without_Fault())
    {
        set_hv_state(HVPS_SM_ID_FAULT, 0);
        set_hv_state(HVPS_SM_ID_FAULT, 1);
        xray_system_disable();
        config_fault_signal(1);

        falut_led(1);
    }
    else
    {
        falut_led(0);
    }

    return;
}

		float err_curr=0;
		float err_tub=0;
void xray_fast_protect()
{
    if ((!Is_Exposing() )|| (!xray_data.isCheckAvailable))//[0] || !xray_data.isCheckAvailable[1]
    {
        return;
    } 
		/*长时间曝光*/
    if (config_data.expo_count[0] > para_range.expo_time_limit)
    {
        mHVPS_Fault.FAULT_REG3.bit.EXPO1_OVERTIME = 1;
    }
    if (config_data.expo_count[1] > para_range.expo_time_limit)
    {
        mHVPS_Fault.FAULT_REG3.bit.EXPO2_OVERTIME = 1;
    }



    /*油箱直接报的故障*/
    if (ctrl_data.hv_vol_fault || ctrl_data.hv_curr_fault)
    {
        xray_data.hv_hardware_count++;
        if (xray_data.hv_hardware_count > FAST_PROTECT_TIME_RANGE) mHVPS_Fault.FAULT_REG1.bit.HV_HARDW_FAULT = 1;
    }
    else
    {
        xray_data.hv_hardware_count = 0;
    }

    /*管电压*/
    float tube_vol_p = sampled_data.tube_vol_p_value * 2;
    float tube_vol_n = sampled_data.tube_vol_n_value * 2;

    /*过压判断 区分AB源*/
    if ((tube_vol_p > para_range.tube_vol_max_protected) ||
            (tube_vol_n > para_range.tube_vol_max_protected))
    {
        xray_data.tube_kv_overCount++;
        if (xray_data.tube_kv_overCount > FAST_PROTECT_TIME_RANGE)
        {
            if (ctrl_data.xray_current == 1)
                mHVPS_Fault.FAULT_REG2.bit.KV1_OVER = 1;
            else
                mHVPS_Fault.FAULT_REG2.bit.KV2_OVER = 1;
        }

    }
    else
    {
        xray_data.tube_kv_overCount = 0;
    }

    float tube_vol_target = Is_CalibrateMode() ? CALI_HV_REF : (float)(float)config_data.tube_vol[ctrl_data.xray_current - 1];

    /*欠压 持续1ms 区分AB源*/
    if ((tube_vol_p < para_range.tube_vol_min_protected) ||
            (tube_vol_n < para_range.tube_vol_min_protected) || (tube_vol_p < tube_vol_target - 40) || (tube_vol_n < tube_vol_target - 40)
           )
    {
        xray_data.tube_kv_underCount++;
        if (xray_data.tube_kv_underCount > FAST_PROTECT_TIME_RANGE)
        {
            if (ctrl_data.xray_current == 1)
                mHVPS_Fault.FAULT_REG2.bit.KV1_UNDER = 1;
            else
                mHVPS_Fault.FAULT_REG3.bit.KV2_UNDER = 1;
        }
    }
    else
    {
        xray_data.tube_kv_underCount = 0;
    }


    /*拉弧，跌到了20kv以下，持续1ms，持续4次*/
    if ((tube_vol_p < tube_vol_target - 20) || (tube_vol_n < tube_vol_target - 20))
    {
        xray_data.oil_strike_count++;
        if (xray_data.oil_strike_count > FAST_PROTECT_ONE_TIME_RANGE)
        {
            xray_data.oil_strike_count = 0;
            xray_data.oil_strike_times++;
        }

        if (xray_data.oil_strike_times >= STRIKE_TIEMS_RANGE)
        {
            if (ctrl_data.xray_current == 1)
						{
							err_tub=tube_vol_p;
                mHVPS_Fault.FAULT_REG4.bit.ARC1 = 1;
						}
            else
                mHVPS_Fault.FAULT_REG4.bit.ARC2 = 1;
        }
    }
    else
    {
        xray_data.oil_strike_count = 0;
    }

    /*双边打火*/
    if ((tube_vol_p < tube_vol_target - 40) && (tube_vol_n < tube_vol_target - 40))
    {
        if (xray_data.tube_strike_count > FAST_PROTECT_ONE_TIME_RANGE)
        {
            if (ctrl_data.xray_current == 1)
                mHVPS_Fault.FAULT_REG4.bit.spark1 = 1;
            else
                mHVPS_Fault.FAULT_REG4.bit.spark2 = 1;
        }
    }
    else
    {
        xray_data.tube_strike_count = 0;
    }
		
    /*电压断线 区分AB源*/
    if (tube_vol_p < 1 || tube_vol_n < 1)
    {
        xray_data.tube_vol_broken_count++;
        if (xray_data.tube_vol_broken_count > FAST_PROTECT_TIME_RANGE)
        {
            if (ctrl_data.xray_current == 1)
                mHVPS_Fault.FAULT_REG4.bit.HV_broken1 = 1;
            else
                mHVPS_Fault.FAULT_REG4.bit.HV_broken2 = 1;
        }
    }
    else
    {
        xray_data.tube_vol_broken_count = 0;
    }

    /*管电流*/
    float tube_curr = sampled_data.tube_curr_value * 10; /*比较时单位0.1mA */

    /*过流 区分AB源*/
    if (tube_curr > para_range.tube_curr_max_protected)
    {
        xray_data.tube_mA_overCount++;
        if (xray_data.tube_mA_overCount > FAST_PROTECT_TIME_RANGE)
        {
            if (ctrl_data.xray_current == 1)
						{ mHVPS_Fault.FAULT_REG2.bit.MA1_OVER = 1;
							err_curr = tube_curr;
						}
            else
                mHVPS_Fault.FAULT_REG3.bit.MA2_OVER = 1;
        }
    }
    else
    {
        xray_data.tube_mA_overCount = 0;
    }

    /*欠流 区分AB源*/
    if (tube_curr < para_range.tube_curr_min_protected)
    {
        xray_data.tube_mA_underCount++;
        if (xray_data.tube_mA_underCount > FAST_PROTECT_TIME_RANGE)
        {
            if (ctrl_data.xray_current == 1)
						{
							err_curr = tube_curr;
							mHVPS_Fault.FAULT_REG2.bit.MA1_UNDER = 1;
						}
            else
                mHVPS_Fault.FAULT_REG3.bit.MA2_UNDER = 1;
        }
    }
    else
    {
        xray_data.tube_mA_underCount = 0;
    }

    /*电流采样断线*/
    if (tube_curr < 1)
    {
        xray_data.tube_curr_broken_count++;
        if (xray_data.tube_curr_broken_count > FAST_PROTECT_TIME_RANGE)
        {
            if (ctrl_data.xray_current == 1)
                mHVPS_Fault.FAULT_REG4.bit.current_broken1 = 1;
            else
                mHVPS_Fault.FAULT_REG4.bit.current_broken2 = 1  ;
        }
    }
    else
    {
        xray_data.tube_curr_broken_count = 0;
    }

//    float fila_vol = sampled_data.filament_vol_value;
//    /*灯丝电压过压*/
//    if (fila_vol > para_range.filament_vol_max_protected)
//    {
//        xray_data.fila_vol_overCount++;
//        if (xray_data.fila_vol_overCount > FAST_PROTECT_TIME_RANGE)
//        {
//            if (ctrl_data.xray_current == 1)
//                mHVPS_Fault.FAULT_REG4.bit.LAMP1_OV = 1;
//            else
//                mHVPS_Fault.FAULT_REG4.bit.LAMP2_OV = 1;
//        }
//    }
//    else
//    {
//        xray_data.fila_vol_overCount = 0;
//    }
//    /*灯丝电压欠压*/
//    if (fila_vol < para_range.filament_vol_min_protected)
//    {
//        xray_data.fila_vol_underCount++;
//        if (xray_data.fila_vol_underCount > FAST_PROTECT_TIME_RANGE)
//        {
//            if (ctrl_data.xray_current == 1)
//                mHVPS_Fault.FAULT_REG1.bit.LAMP1_UV = 1;
//            else
//                mHVPS_Fault.FAULT_REG4.bit.LAMP2_UV = 1;
//        }
//    }
//    else
//    {
//        xray_data.fila_vol_underCount = 0;
//    }

    return;
}

void transform_adc_values()
{
    sampled_data.power_24v_value        = 0.01209f * ((float)(adc_buffer2[0]));
    // sampled_data.temp_sink_value        = -0.04747f * ((float)(adc_buffer2[1])) + 122.59205f;

    sampled_data.filament_vol_value     = 0.00806f * ((float)(adc_buffer2[2]));
    sampled_data.filament_curr_value    = 0.00806f * ((float)(adc_buffer2[3]));
    sampled_data.tube_vol_p_value       = 0.02579f * ((float)(adc_buffer3[0]));
    sampled_data.tube_vol_n_value       = 0.02579f * ((float)(adc_buffer3[1]));
    sampled_data.tube_curr_value        = 0.00645f * ((float)(adc_buffer3[2]));
    sampled_data.temp_oil_value         = ((float)(adc_buffer3[3]));


}
