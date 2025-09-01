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

    config_disable_sw_safe(1);
    config_enable_sw_safe(0);


    return;
}

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
    parm_table[n].currRef[curr_idx]   = param_pid.config_ref;
    return;
}


//故障指示灯  高压
void calibrate_task()
{
    static uint8_t cali_source = 0; // 当前射源通道，0表示射源1，1表示射源2
    static uint32_t last_expo_end_time[2] = {0, 0}, last_expo_count = 0;
    static uint32_t sw_count = 0;
    static uint8_t sw_state = 0;
    static uint32_t currRef = 0, oldref[2] = {0};
    ctrl_data.xray_current =  cali_source + 1 ;
    static uint8_t cycle = 1;

    // 如果关闭使能，则强制回到IDLE
    if (ctrl_data.enable[cali_source] == 0)
    {
        xray_system_disable();
        set_hv_state(HVPS_SM_ID_IDLE, 0);
        set_hv_state(HVPS_SM_ID_IDLE, 1);
        return;
    }

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

//        config_filamentOn_signal(1, 0);                     // 打开灯丝0
//        config_filament_ref_slop(0);                        // 控制灯丝DA软启动
//        config_filamentOn_signal(1, 1);                     // 打开灯丝1
//        config_filament_ref_slop(1);                        // 控制灯丝PWM软启动
        config_filamentOn_signal(0, 0);
        config_filamentOn_signal(0, 1);

        if (cali_data.timmer_count > TIMER6_2P5_SECOND_CYCLES)
        {
            cali_data.timmer_count = 1;
            set_hv_state(HPVS_SM_ID_CAL_RUN, cali_source);
        }
        break;

    case HPVS_SM_ID_CAL_RUN:
        currRef =  parm_table[cali_source].currRef_c[cali_data.curr_index];
        pid_Init_2(parm_table[cali_source].currValue[cali_data.curr_index], currRef, Is_PulseMode_CT(), cali_source);

        config_hvref_cali(cali_source);                                // 设置高压参考
        config_filamentRef_cali(cali_data.curr_index, cali_source);    // 设置灯丝DA/PWM输出

        if (cali_data.timmer_count >= TIMER6_3_SECOND_CYCLES)//每个电流间两秒的时钟间隔
        {
            set_hv_state(HPVS_SM_ID_CAL_EXPOSURING, cali_source);
            cali_data.timmer_count = 1;
            // 确保间隔大于10ms
        }
        break;

    case HPVS_SM_ID_CAL_EXPOSURING:
        if ((last_expo_count - last_expo_end_time[1 - cali_source]) >= 200)
        {

            // 曝光后 4ms 开始允许采样检查
            xray_data.isCheckAvailable = (cali_data.timmer_count > 80) ? 1 : 0;

            xray_HV_enable(1);
            config_hvref_cali(cali_source);
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
        break;

    case HPVS_SM_ID_CAL_COOLING:
        config_hvref_cali(cali_source);

        if (cali_data.timmer_count == (uint32_t)(cali_data.coolTime_expect / 2))
        {
            user_pid_2.Kp[1] = 50;
            user_pid_2.Ki[1] = 30;
            user_pid_2.Kp[0] = 40;
            user_pid_2.Ki[0] = 40;

            user_pid_2.currValue[cali_source] = 0.00645f * ((float)(adc_buffer3[2]));
            oldref[cali_source] = user_pid_2.config_ref[cali_source];
            param_pid.pulse_count = cali_data.cycle_count;
            // tube_current_piControl_v2(cali_source);
            param_pid.config_ref = user_pid_2.config_ref[cali_source];

            // debug_tx3("pi:%d, %d, %d, %f\n", cali_source, oldref[cali_source], user_pid_2.config_ref[cali_source], user_pid_2.currValue[cali_source]);
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

            if (cali_data.cycle_count >= cali_data.expoCycle_perCurrent[cali_source] * 2)
            {
                filament_ref_update(cali_data.curr_index, cali_source);    // 更新灯丝查表
                filament_ref_update(cali_data.curr_index, 1 - cali_source);  // 更新灯丝查表
                //cali_data.cycle_count = 0;
                parm_table[cali_source].expo_count_total++;
                parm_table[cali_source].expo_times_total += config_data.expo_count_total[cali_source] / 1200000;
                parm_table[1 - cali_source].expo_count_total++;
                parm_table[1 - cali_source].expo_times_total += config_data.expo_count_total[cali_source] / 1200000;
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
                        // 重置状态
                        cali_data.cycle_count = 0;
                        sw_state = 0;
                        sw_count = 0;
                        cycle = 1;
                        set_hv_state(HPVS_SM_ID_CAL_END, cali_source);
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
                        cali_data.cycle_count = 0;
                        // 重置状态
                        sw_state = 0;
                        sw_count = 0;
                        cali_source = 1 - cali_source;
                        cali_data.curr_index++;
                        set_hv_state(HPVS_SM_ID_CAL_RUN, cali_source);
                        cali_data.timmer_count = 1;
                        cycle = 1;
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

                        currRef =  parm_table[cali_source].currRef_c[cali_data.curr_index];
                        pid_Init_2(parm_table[cali_source].currValue[cali_data.curr_index], currRef, Is_PulseMode_CT(), cali_source);

                        config_hvref_cali(cali_source);                                // 设置高压参考
                        config_filamentRef_cali(cali_data.curr_index, cali_source);    // 设置灯丝DA/PWM输出

                        set_hv_state(HPVS_SM_ID_CAL_EXPOSURING, cali_source);
                        cali_data.timmer_count = 1;
                        cycle = 1;
                    }
                }
            }
        }
        break;
    case HPVS_SM_ID_CAL_END:
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





// ---------- 分离的状态处理函数实现 ----------

void cal_finish()
{
    xray_system_disable();
    cali_data.timmer_count = 0;
    set_hv_state(HVPS_SM_ID_IDLE, 0);
    set_hv_state(HVPS_SM_ID_IDLE, 1);
}

uint32_t get_last_expo_time(uint8_t src)
{
    static uint32_t last_expo_end_time[2] = {0};
    return last_expo_end_time[src];
}

void handle_channel_switching()
{
    static uint8_t sw_state = 0;
    static uint32_t sw_count = 0;
    static uint8_t cali_source = 0;

    if (cali_data.curr_index >= FILAMENT_CURRENT_TABLE_ORDER && cali_source)
    {
        if (HAL_GetTick() - get_last_expo_time(cali_source) < 12)
            return;
    }

    sw_count++;

    switch (sw_state)
    {
    case 0:
        config_disable_sw(cali_source);
        sw_state = 1;
        sw_count = 1;
        break;

    case 1:
        if (sw_count >= 20)
        {
            config_enable_sw(1 - cali_source);
            sw_state = 2;
            sw_count = 1;
        }
        break;

    case 2:
        if (sw_count >= 20)
        {
            sw_state = 0;
            sw_count = 0;
            cali_source = 1 - cali_source;
            cali_data.curr_index++;

            if (cali_data.curr_index >= FILAMENT_CURRENT_TABLE_ORDER && cali_source == 1)
                set_hv_state(HPVS_SM_ID_CAL_END, cali_source);
            else
                set_hv_state(HPVS_SM_ID_CAL_RUN, cali_source);

            cali_data.timmer_count = 1;
        }
        break;
    }
}


void cal_prepare()
{
    config_mcuLock_signal(1);
    calibrate_mode_config();

    config_filamentOn_signal(1, 0);
    config_filament_ref_slop(0);
    config_filamentOn_signal(1, 1);
    config_filament_ref_slop(1);

    if (cali_data.timmer_count > TIMER6_2P5_SECOND_CYCLES)
    {
        cali_data.timmer_count = 1;
        set_hv_state(HPVS_SM_ID_CAL_RUN, 0);
    }
}

void cal_run(uint8_t src)
{
//    uint32_t currRef = parm_table[src].currRef[cali_data.curr_index];
//    pid_Init_2(parm_table[src].currValue[cali_data.curr_index], currRef, Is_PulseMode_CT(), src);

//    config_hvref_cali(src);
//    config_filamentRef_cali(cali_data.curr_index, src);

//    if (cali_data.timmer_count >= TIMER6_2_SECOND_CYCLES)
//    {
//        set_hv_state(HPVS_SM_ID_CAL_EXPOSURING, src);
//        cali_data.timmer_count = 1;
//    }
}

void cal_exposing(uint8_t src)
{
    static uint32_t last_expo_end_time[2] = {0};

    if ((get_tick_ms() - last_expo_end_time[1 - src]) >= 10)
    {
        xray_HV_enable(1);
        config_hvref_cali(src);
        config_data.expo_count_total[src]++;

        if (cali_data.timmer_count >= cali_data.expoTime_expect)
        {
            xray_HV_enable(0);
            last_expo_end_time[src] = get_tick_ms();
            set_hv_state(HPVS_SM_ID_CAL_COOLING, src);
            cali_data.timmer_count = 1;
        }
    }
}

void cal_cooling(uint8_t src)
{
    config_hvref_cali(src);

    if (cali_data.timmer_count == (uint32_t)(cali_data.coolTime_expect / 2))
    {
//        user_pid_2.Kp = 100;
//        user_pid_2.Ki = 30;
        tube_current_piControl_v2(src);
        param_pid.config_ref = user_pid_2.config_ref[src];
    }

    if (cali_data.timmer_count < cali_data.coolTime_expect)
        return;

    cali_data.cycle_count++;

    if (cali_data.cycle_count >= cali_data.expoCycle_perCurrent[src])
    {
        filament_ref_update(cali_data.curr_index, src);
        parm_table[src].expo_count_total++;
        parm_table[src].expo_times_total += config_data.expo_count_total[src] / 1200000;

        cali_data.cycle_count = 0;

        handle_channel_switching();
    }
    else
    {
        // 同电流重复多次曝光
        set_hv_state(HPVS_SM_ID_CAL_RUN, src);
        cali_data.timmer_count = 1;
    }
}



// 重构后的校准任务函数框架
void calibrate_task1()
{
    static uint8_t cali_source = 0; // 射源通道：0 或 1

    // 如果当前通道关闭，强制进入空闲状态
    if (ctrl_data.enable[cali_source] == 0)
    {
        xray_system_disable();
        set_hv_state(HVPS_SM_ID_IDLE, 0);
        set_hv_state(HVPS_SM_ID_IDLE, 1);
        return;
    }

    // 时间更新
    if (cali_data.timmer_count >= 1)
        cali_data.timmer_count++;

    // 状态调度器
    switch (get_hv_state(cali_source))
    {
    case HPVS_SM_ID_CAL_PREPARE:
        cal_prepare();
        break;

    case HPVS_SM_ID_CAL_RUN:
        cal_run(cali_source);
        break;

    case HPVS_SM_ID_CAL_EXPOSURING:
        cal_exposing(cali_source);
        break;

    case HPVS_SM_ID_CAL_COOLING:
        cal_cooling(cali_source);
        break;

    case HPVS_SM_ID_CAL_END:
        cal_finish();
        break;

    default:
        break;
    }

    // 指示灯状态更新
    xray_on_led((get_hv_state(cali_source) == HPVS_SM_ID_CAL_EXPOSURING) ? 1 : 0);
}




