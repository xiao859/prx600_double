///*
// *  calibrate.c
// *
// *  Created on: Mar 04, 2025
// *  Author: Administrator
// *
// */

#include "calibrate.h"
#include "ct_exposure.h"
#include "adc.h"
#include "time.h"
#include "xray.h"
#include "delay.h"
#include "app_spi.h"
#include "app_uart.h"
#include "pi_ctl.h"

volatile xray_calibrate_data cali_data;
extern TIM_HandleTypeDef htim5;


/*校准参数
 *单电流校准脉冲40个  校准时间3.2s
 *脉冲：35ms校准周期，15ms曝光
 * */
void calibrate_mode_config()
{
    ctrl_data.xrayMode = XRAY_MODE_D_PULSE;
    cali_data.expoCycle_perCurrent[0] = CALI_PULSE_COUNT;
    cali_data.expoCycle_perCurrent[1] = CALI_PULSE_COUNT;
    cali_data.expoTime_expect      = CALI_PULSE_CURR_EXPO_TIME * 20;
    cali_data.coolTime_expect      = (CALI_PULSE_SIGLE_CURR_PERIOD - CALI_PULSE_CURR_EXPO_TIME) * 20;

    config_data.tube_vol[0] = CALI_HV_REF;
    config_data.tube_vol[1] = CALI_HV_REF;

    config_data.tube_vol_realtime[1] = 0;
    config_data.tube_vol_step[1] = (float)(config_data.tube_vol[1] - IDLE_HV_REF) / (20 * parm_table[1].rising_time);

    config_data.tube_vol_realtime[0] = 0;
    config_data.tube_vol_step[0] = (float)(config_data.tube_vol[0] - IDLE_HV_REF) / (20 * parm_table[0].rising_time);

    config_disable_sw_safe(1);
    config_enable_sw_safe(0);


    return;
}
uint32_t arr[8] = {0};
void config_filamentRef_cali(uint8_t curr_index, uint16_t n)
{
    uint32_t fila_vol_ref = parm_table[n].currRef_c[curr_index] ;

    if (n == 0)
        HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, fila_vol_ref);
    else //PWM
    {
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, fila_vol_ref);
    }
    return;
}

void config_hvref_cali(uint16_t n)
{
    if (get_hv_state(n) == HPVS_SM_ID_CAL_EXPOSURING)
    {
        if (cali_data.tube_vol_realtime < CALI_HV_REF) cali_data.tube_vol_realtime += cali_data.tube_vol_step;
    }
    else
    {
        if (cali_data.tube_vol_realtime > IDLE_HV_REF) cali_data.tube_vol_realtime -= cali_data.tube_vol_step;
    }

    cali_data.tube_vol_realtime = MAX(MIN(cali_data.tube_vol_realtime, CALI_HV_REF), IDLE_HV_REF);

    // uint32_t tube_vol_ref = (uint32_t)((cali_data.tube_vol_realtime / 64) / ADDA_FULL_SCALE_VIL_VALUE * 4095);  /* 0~2.5V¦ 0~160kV */
    int32_t tube_vol_ref = (int32_t)(cali_data.tube_vol_realtime * 19.389f);

    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, tube_vol_ref);

    return;
}

void calibrate_para_init()
{
    cali_data.mode = XRAY_MODE_D_PULSE;
    cali_data.curr_index    = 0;
    cali_data.cycle_count   = 0;
    cali_data.timmer_count  = 1;
    ctrl_data.filament_on[0]   = 1;
    cali_data.para_save_flag = 0;

    cali_data.tube_vol_step = ((float)(CALI_HV_REF - IDLE_HV_REF) / 100);

    config_data.fila_ref_realtime[0] = 0;
    config_data.fila_ref_step[0] = IDLE_FILAMENT1_REF / (20 * 20);          /* 20ms*/

    config_data.fila_ref_realtime[1] = 0;
    config_data.fila_ref_step[1] = IDLE_FILAMENT2_REF / (20 * 20);          /* 20ms*/

    return;
}

/*使能高压信号*/
void xray_HV_enable(uint16_t value)
{
    config_HVEn_signal(value);      /*高电平开*/
    config_xrayOn_signal(value);    /*低电平开*/

    return;
}

/*更新校准结果到表中*/
void filament_ref_update(uint8_t curr_idx, uint16_t n)
{
	if((cali_data.curr_index>=5)&&(n == 1))
    parm_table[n].currRef_c[curr_idx]   = user_pid_2.config_ref[n]*0.96;//
	else if((cali_data.curr_index>=5)&&(n == 0))
		parm_table[n].currRef_c[curr_idx]   = user_pid_2.config_ref[n]*0.96;//;
	else 
		parm_table[n].currRef_c[curr_idx]   = user_pid_2.config_ref[n];
    return;
}



//故障指示灯  高压
void calibrate_task()
{
    static uint8_t cali_source = 0; // 当前射源通道，0表示射源1，1表示射源2
    static uint32_t last_expo_end_time[2] = {0, 0}, last_expo_count = 0;
    static uint32_t sw_count = 0;
    static uint8_t sw_state = 0;
    static uint32_t currRef[2] = {0};
    ctrl_data.xray_current =  cali_source + 1 ;
    static uint8_t cycle = 1;

    if (cali_data.timmer_count >= 1)
        cali_data.timmer_count++;
    else
        return;

    last_expo_count ++;

    switch (get_hv_state(cali_source))
    {
    case HPVS_SM_ID_CAL_PREPARE:
        config_mcuLock_signal(1);                          // 启用互锁
        calibrate_mode_config();                           // 初始化参数

        config_filamentOn_signal(1, 0);                     // 打开灯丝0
        config_filament_ref_slop(0);                        // 控制灯丝DA软启动
        config_filamentOn_signal(1, 1);                     // 打开灯丝1
        config_filament_ref_slop(1);                        // 控制灯丝PWM软启动

        if (cali_data.timmer_count > TIMER6_2P5_SECOND_CYCLES)
        {
            cali_data.timmer_count = 1;
            set_hv_state(HPVS_SM_ID_CAL_RUN, cali_source);
        }
        break;

    case HPVS_SM_ID_CAL_RUN:
        currRef[0] =  parm_table[0].currRef_c[cali_data.curr_index];
        currRef[1] =  parm_table[1].currRef_c[cali_data.curr_index];
        pid_Init_2(parm_table[0].currValue[cali_data.curr_index], currRef[0], Is_PulseMode_CT(), 0);
        pid_Init_2(parm_table[1].currValue[cali_data.curr_index], currRef[1], Is_PulseMode_CT(), 1);
        config_hvref_slope(cali_source);                               // 设置高压参考
        config_filamentRef_cali(cali_data.curr_index, 0);    // 设置灯丝DA/PWM输出
        config_filamentRef_cali(cali_data.curr_index, 1);

        if (cali_data.timmer_count >= TIMER6_3_SECOND_CYCLES)//每个电流间两秒的时钟间隔
        {
            set_hv_state(HPVS_SM_ID_CAL_EXPOSURING, cali_source);
            cali_data.timmer_count = 1;
            // 确保间隔大于10ms
        }
        break;

    case HPVS_SM_ID_CAL_EXPOSURING:
        config_hvref_slope(cali_source);
        if ((last_expo_count - last_expo_end_time[1 - cali_source]) >= 200)
        {

            // 曝光后 4ms 开始允许采样检查
            xray_data.isCheckAvailable = (cali_data.timmer_count > 80) ? 1 : 0;

            xray_HV_enable(1);
            user_pid_2.currValue[cali_source] = 0.00645f * ((float)(adc_buffer3[2]));
            config_data.expo_count_total[cali_source]++;
            if (cali_data.timmer_count >= cali_data.expoTime_expect)
            {
                xray_HV_enable(0);
                xray_data.isCheckAvailable = 0;
                last_expo_end_time[cali_source] = last_expo_count;            // 记录曝光结束时间戳
                set_hv_state(HPVS_SM_ID_CAL_COOLING, cali_source);
                cali_data.timmer_count = 1;
            }
        }

        // 如果关闭使能，则强制回到IDLE
        if (ctrl_data.enable[cali_source] == 0)
        {
            xray_system_disable();
            config_disable_sw_safe(1);
            config_enable_sw_safe(0);
            set_hv_state(HVPS_SM_ID_IDLE, 0);
            set_hv_state(HVPS_SM_ID_IDLE, 1);
            return;
        }
        break;

    case HPVS_SM_ID_CAL_COOLING:
        config_hvref_slope(cali_source);

        if (cali_data.timmer_count == (uint32_t)(cali_data.coolTime_expect / 2))
        {
            user_pid_2.Kp[1] = 50;
            user_pid_2.Ki[1] = 30;
            user_pid_2.Kp[0] = 50;
            user_pid_2.Ki[0] = 40;

            param_pid.pulse_count ++;
            tube_current_piControl_v2(cali_source);
            param_pid.config_ref = user_pid_2.config_ref[cali_source];

            //debug_tx3("pi:%d, %d, %d, %f\n", cali_source, oldref[cali_source], user_pid_2.config_ref[cali_source], user_pid_2.currValue[cali_source]);
        }
        if (last_expo_count - last_expo_end_time[cali_source] < 240)//延时12ms
            return;


        if (cali_data.timmer_count > cali_data.coolTime_expect)
        {
            if (cycle == 1)
            {
                cali_data.cycle_count++;
                cycle = 0;
            }

            if (cali_data.cycle_count >= (cali_data.expoCycle_perCurrent[cali_source] * 2 - 2))
            {
                filament_ref_update(cali_data.curr_index, cali_source);    // 更新灯丝查表
            }


            //B源 电流12ma 循环次数expoCycle_perCurrent，直接跳到结束
            if ((cali_data.curr_index >= 11) && (cali_source == 1) && (cali_data.cycle_count >= cali_data.expoCycle_perCurrent[cali_source] * 2))
            {

                sw_count++;

                switch (sw_state)
                {
                case 0:  // 准备关闭当前采样开关
                    config_disable_sw_safe(cali_source);
                    sw_state = 1;
                    sw_count = 1;
                    break;

                case 1:  // 延时后打开下一个采样开关
                    if (sw_count >= 20)
                    {
                        config_enable_sw_safe(1 - cali_source);
                        sw_state = 2;
                        sw_count = 1;
                    }
                    break;

                case 2:  // 延时后切换射源
                    if (sw_count >= 20)
                    {
                        parm_table[0].expo_count_total++;
                        parm_table[1].expo_count_total++;
                        // 重置状态
                        cali_data.cycle_count = 0;
                        sw_state = 0;
                        sw_count = 0;
                        cycle = 1;
                        set_hv_state(HPVS_SM_ID_CAL_END, cali_source);
                        param_pid.pulse_count = 0;
                    }

                }
            }
            //B源 循环次数expoCycle_perCurrent 下一个电流
            else if ((cali_data.cycle_count >= cali_data.expoCycle_perCurrent[cali_source] * 2) && (cali_source == 1))
            {
                sw_count++;

                switch (sw_state)
                {
                case 0:  // 准备关闭当前采样开关
                    config_disable_sw_safe(cali_source);
                    sw_state = 1;
                    sw_count = 1;
                    break;

                case 1:  // 延时后打开下一个采样开关
                    if (sw_count >= 20)
                    {
                        config_enable_sw_safe(1 - cali_source);
                        sw_state = 2;
                        sw_count = 1;
                    }
                    break;

                case 2:  // 延时后切换射源
                    if (sw_count >= 20)
                    {
                        parm_table[0].expo_count_total++;
                        parm_table[1].expo_count_total++;
                        cali_data.cycle_count = 0;
                        // 重置状态
                        sw_state = 0;
                        sw_count = 0;
                        cali_source = 1 - cali_source;
                        cali_data.curr_index++;
                        set_hv_state(HPVS_SM_ID_CAL_RUN, cali_source);
                        cali_data.timmer_count = 1;
                        cycle = 1;
                        param_pid.pulse_count = 0;
                    }
                }
            }
            //A B交替切换，准备下个射源
            else
            {
                sw_count++;
                switch (sw_state)
                {
                case 0:  // 准备关闭当前采样开关

                    config_disable_sw_safe(cali_source);
                    sw_state = 1;
                    sw_count = 1;

                    break;

                case 1:  // 延时后打开下一个采样开关
                    if (sw_count >= 20)
                    {
                        config_enable_sw_safe(1 - cali_source);
                        sw_state = 2;
                        sw_count = 1;
                    }
                    break;

                case 2:  // 延时后切换射源
                    if (sw_count >= 20)
                    {
                        // 重置状态
                        sw_state = 0;
                        sw_count = 0;
                        cali_source = 1 - cali_source;

                        config_hvref_slope(cali_source);                                // 设置高压参考

                        set_hv_state(HPVS_SM_ID_CAL_EXPOSURING, cali_source);
                        cali_data.timmer_count = 1;
                        cycle = 1;
                    }
                }
            }
        }
        break;
    case HPVS_SM_ID_CAL_END:
        exp_count[0] += config_data.expo_count_total[0];
        exp_count[1] += config_data.expo_count_total[1];
        parm_table[0].expo_times_total +=  exp_count[0] / 1200000;
        exp_count[0] = exp_count[1] % 1200000;
        parm_table[1].expo_times_total += exp_count[1] / 1200000;
        exp_count[1] = exp_count[1] % 1200000;
        xray_system_disable();
        cali_data.timmer_count = 0;
        set_hv_state(HVPS_SM_ID_IDLE, 0);
        set_hv_state(HVPS_SM_ID_IDLE, 1);
        cali_data.para_save_flag = 1;
        break;

    default:
        break;
    }
    // 指示灯
    xray_on_led((get_hv_state(cali_source) == HPVS_SM_ID_CAL_EXPOSURING) ? 1 : 0);
}


