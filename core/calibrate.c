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

#define LAMP_PREHEAT_COUNT  640000//灯丝预热计数
#define ENABLE_CYCLE_COUNT  656000//使能周期计数
#define ENABLE_EFFECT_COUNT  256000
#define EXPO_DELAY_COUNT     128000 //曝光延时计数
#define EXPO_CYCLE_COUNT     9600//曝光周期计数
#define EXPO_EFFECT_COUNT    1200//曝光有效计数
#define EXPO_DEAD_COUNT    5400//曝光死区计数
#define EXPO2_EFFECT_COUNT   6600
#define STORAGE_TIME_COUNT   80000
volatile xray_calibrate_data cali_data;
extern TIM_HandleTypeDef htim5;


/*校准参数
 *单电流校准时间3.2s
 *脉冲：120ms校准周期，15ms曝光
 * */
void calibrate_mode_config(uint16_t n)
{
    if (Is_PulseMode())
    {
        cali_data.expoCycle_perCurrent[n] = (uint32_t)((CALI_SINGLE_CURR_TIME * 1000) / CALI_PULSE_SIGLE_CURR_PERIOD);
        cali_data.expoTime_expect      = CALI_PULSE_CURR_EXPO_TIME * 50;
        cali_data.coolTime_expect      = (CALI_PULSE_SIGLE_CURR_PERIOD - CALI_PULSE_CURR_EXPO_TIME) * 50;
    }
    else
    {
        cali_data.expoCycle_perCurrent[n] = 1;
        cali_data.expoTime_expect      = CALI_SINGLE_CURR_TIME * 1000 * 50;
        cali_data.coolTime_expect      = 0;
    }

    return;
}

void config_filamentRef_cali(uint8_t curr_index, uint16_t n)
{
    uint32_t fila_vol_ref = (Is_PulseMode()) ?
                            parm_table[n].currRef[curr_index] : parm_table[n].currRef_c[curr_index];

    /*测试代码为1.4V，¦1.4/3.3*4095 = 1737 */
    // fila_vol_ref = 1900;
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

    uint32_t tube_vol_ref = (uint32_t)((cali_data.tube_vol_realtime / 64) / ADDA_FULL_SCALE_VIL_VALUE * 4095);  /* 0~2.5V¦ 0~160kV */


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
    config_data.fila_ref_step[0] = IDLE_FILAMENT_REF / (20 * 50);          /* 20msÉÏÉýÊ±¼ä */

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
    if (Is_PulseMode())
    {
        parm_table[n].currRef[curr_idx]   = param_pid.config_ref;
    }
    else
    {
        parm_table[n].currRef_c[curr_idx] = param_pid.config_ref;
    }


    return;
}

void calibrate_task()
{

    static uint8_t cali_source = 0; // 当前射源通道，0表示射源1，1表示射源2
    static uint32_t last_expo_end_time = 0;
    static uint32_t last_sw_end_time = 0;

    if (cali_data.timmer_count >= 1)
        cali_data.timmer_count++;

    // 如果关闭使能，则强制回到IDLE
    if (ctrl_data.enable[cali_source] == 0)
    {
        xray_system_disable(cali_source);
        disable_hvref(cali_source);
        disable_filamentref(cali_source);
        set_hv_state(HVPS_SM_ID_IDLE, cali_source);
        return;
    }

    switch (get_hv_state(cali_source))
    {
    case HPVS_SM_ID_CAL_PREPARE:
        config_mcuLock_signal(1);                          // 启用互锁
        calibrate_mode_config(cali_source);                           // 初始化参数
        config_filamentOn_signal(1, cali_source);                      // 打开灯丝
        config_filament_ref_slop(cali_source);                        // 控制灯丝DA软启动
        config_enable_sw(cali_source);
        config_disable_sw((cali_source == 0) ? 1 : 0);

        if (cali_data.timmer_count > TIMER6_2P5_SECOND_CYCLES)
        {
            set_hv_state(HPVS_SM_ID_CAL_RUN, cali_source);
            cali_data.timmer_count = 1;
            uint32_t currRef = (Is_PulseMode()) ?
                               parm_table[cali_source].currRef[cali_data.curr_index] : parm_table[cali_source].currRef_c[cali_data.curr_index];
            pid_Init(parm_table[cali_source].currValue[cali_data.curr_index], currRef, Is_PulseMode());
        }
        break;

    case HPVS_SM_ID_CAL_RUN:
        config_hvref_cali(cali_source);                                // 设置高压参考
        config_filamentRef_cali(cali_data.curr_index, cali_source);    // 设置灯丝DA/PWM输出

        if (cali_data.timmer_count >= TIMER6_5_SECOND_CYCLES)
        {
            set_hv_state(HPVS_SM_ID_CAL_EXPOSURING, cali_source);
            cali_data.timmer_count = 1;
            // 确保间隔大于10ms
        }
        break;

    case HPVS_SM_ID_CAL_EXPOSURING:

        if ((get_tick_ms() - last_expo_end_time) >= 10)
        {
            xray_HV_enable(1);
            config_hvref_cali(cali_source);
        }
        else
            return;

        config_data.expo_count_total[cali_source]++;

        if (cali_data.timmer_count >= cali_data.expoTime_expect)
        {
            last_expo_end_time = get_tick_ms();            // 记录曝光结束时间戳
            set_hv_state(HPVS_SM_ID_CAL_COOLING, cali_source);
            cali_data.timmer_count = 1;
        }

        // 曝光期间调节 PI（如连续模式）
        if (Is_ContinuousMode() &&
                (cali_data.timmer_count > TIMER6_4_MILSECOND_CYCLES) &&
                (cali_data.timmer_count % TIMER6_10_MILSECOND_CYCLES == 0))
        {
            user_pid.currValue = sampled_data.tube_curr_value;
            user_pid.Kp = 3.7;
            user_pid.Ti = 0.0009;
            tube_current_piControl(cali_source,cali_source);
        }

        break;

    case HPVS_SM_ID_CAL_COOLING:
        last_sw_end_time = get_tick_ms();
        config_hvref_cali(cali_source);

        if ((Is_PulseMode()) && (cali_data.timmer_count == (uint32_t)(cali_data.coolTime_expect / 2)))
        {
            user_pid.Kp = 20;
            user_pid.Ti = 1;
            tube_current_piControl(cali_source,cali_source);
        }

        if (cali_data.timmer_count < cali_data.coolTime_expect)
            return;

        cali_data.cycle_count++;

        if ((get_tick_ms() - last_sw_end_time) >= 8)
        {
            config_disable_sw(cali_source);
            config_enable_sw((cali_source == 0) ? 1 : 0);
        }


        if (cali_data.cycle_count >= cali_data.expoCycle_perCurrent[cali_source])
        {
            filament_ref_update(cali_data.curr_index, cali_source);    // 更新灯丝查表

            cali_data.cycle_count = 0;
            parm_table[cali_source].expo_count_total++;
            parm_table[cali_source].expo_times_total += config_data.expo_count_total[cali_source] / 3000000;

            cali_data.curr_index++;
            if ((cali_data.curr_index >= FILAMENT_CURRENT_TABLE_ORDER) & (cali_source))
            {
                uint32_t last_idx = cali_data.curr_index - 1;
                debug_tx3("闭环结果: %f, %f, %d, %d\n",
                          user_pid.currValue, parm_table[cali_source].currValue[last_idx], parm_table[cali_source].currRef[last_idx], param_pid.config_ref);
                cali_data.curr_index = 0;

                /*该模式下所有电流校准完毕*/
                if (Is_PulseMode())
                {
                    // 切换下一个射源交替运行
                    cali_source ^= 1;
                    // 重新开始 RUN 状态
                    set_hv_state(HPVS_SM_ID_CAL_RUN, cali_source);
                    cali_data.timmer_count = 1;

                    cali_data.mode = XRAY_MODE_D_CONTINUOUS;
                    calibrate_mode_config(cali_source);
                    uint32_t currRef = (Is_PulseMode()) ?
                                       parm_table[cali_source].currRef[cali_data.curr_index] : parm_table[cali_source].currRef_c[cali_data.curr_index];
                    pid_Init(parm_table[cali_source].currValue[cali_data.curr_index], currRef, Is_PulseMode());
                }
                else
                {
                    set_hv_state(HPVS_SM_ID_CAL_RUN, cali_source);
                }
                if ((cali_data.mode == XRAY_MODE_D_CONTINUOUS) & (cali_source == 1))
                {
                    cali_data.para_save_flag = 1;              // 标记保存参数
                    set_hv_state(HPVS_SM_ID_CAL_END, cali_source);
                }
                break;
            }
            else if (cali_source == 1) //准备下个电流
            {
                /*上报本次校准结果*/
                uint32_t last_idx = cali_data.curr_index - 1;
                debug_tx3("闭环结果: %d.%f, %f, %d, %d\n",
                          cali_source, user_pid.currValue, parm_table[cali_source].currValue[last_idx], parm_table[cali_source].currRef[last_idx], param_pid.config_ref);

                uint32_t currRef = (Is_PulseMode()) ?
                                   parm_table[cali_source].currRef[cali_data.curr_index] : parm_table[cali_source].currRef_c[cali_data.curr_index];
                // 切换下一个射源交替运行
                cali_source ^= 1;
                set_hv_state(HPVS_SM_ID_CAL_RUN, cali_source);
                pid_Init(parm_table[cali_source].currValue[cali_data.curr_index], currRef, Is_PulseMode());//PID系数重置
            }
            else//准备下个射源
            {
                // 切换下一个射源交替运行
                cali_source ^= 1;

                // 重新开始 RUN 状态
                set_hv_state(HPVS_SM_ID_CAL_RUN, cali_source);
                cali_data.timmer_count = 1;
                // 初始化下一个点的PID
                uint32_t currRef = (Is_PulseMode()) ?
                                   parm_table[cali_source].currRef[cali_data.curr_index] : parm_table[cali_source].currRef_c[cali_data.curr_index];
                pid_Init(parm_table[cali_source].currValue[cali_data.curr_index], currRef, Is_PulseMode());
            }
        }
        break;

    case HPVS_SM_ID_CAL_END:
        xray_system_disable(0);
        xray_system_disable(1);
        cali_data.timmer_count = 0;
        set_hv_state(HVPS_SM_ID_IDLE, 0);
        set_hv_state(HVPS_SM_ID_IDLE, 1);
        break;

    default:
        break;
    }

    // 指示灯
    xray_on_led((get_hv_state(cali_source) == HPVS_SM_ID_CAL_EXPOSURING) ? 1 : 0);
}




