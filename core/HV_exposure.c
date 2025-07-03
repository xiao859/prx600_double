#include "HV_exposure.h"
#include "adc.h"
#include <math.h>
#include "string.h"
#include "app_spi.h"
//#include "calibrate.h"
#include "delay.h"
#include "xray.h"


#define IDLE_FILAMENT_REF               1000

hvps_sm_state volatile hv_state[XRAY_NUMS];

volatile xray_config_data config_data;
volatile xray_parament_table parm_table[XRAY_NUMS] =
{
    {
        0, 0, 1,
        {1,    2,    3,    4,    5,    6,    7,    8,    9,    10,  11,   12},
        {1766, 2012, 2148, 2232, 2304, 2373, 2434, 2491, 2546, 2598,  2246, 2282},
        {1500, 1660, 1770, 1850, 1920, 1980, 2030, 2080, 2120, 2160, 2200, 2250},
    },
    {
        0, 0, 1,
        {1,    2,    3,    4,    5,    6,    7,    8,    9,    10,  11,   12},
        {1766, 2012, 2148, 2232, 2304, 2373, 2434, 2491, 2546, 2598,  2246, 2282},
        {1500, 1660, 1770, 1850, 1920, 1980, 2030, 2080, 2120, 2160, 2200, 2250},
    },
};
volatile xray_parament_range para_range =
{
    140, 50,            /*管电压保护值*/
    130, 5, 750,        /*管电流保护值*/

    70, -25, 60, 50,    /* ÓÍÏäÎÂ¶È */

    28, 20,           	/* 24V供电*/
    100, 100,           /*散热器温度*/
    30, 8,           		/*灯丝电压*/
    100, 100,           /*灯丝电流*/

    125, 60,            /*管电压配置门限*/
    120, 10,            /*管电流配置门限*/

    1000, 1000,         /*曝光时间保护*/

    4500000,            /*曝光时间1.5min */

    6000000             /*灯丝开启未曝光最大时间*/
};
volatile xray_running_data xray_data;
volatile adc_sampled_value sampled_data= {
    30, 30, 10, 1000, 24, 0, 24, 0, 20};
volatile adc_sampled_value sampled_data_last = {0};
volatile cmd_control_data ctrl_data;

uint16_t adc_buffer2[ADC_2_CHANNEL_NUM * ADC_SAMPLE_CYCLE_NUM];
uint16_t adc_buffer3[ADC_3_CHANNEL_NUM * ADC_SAMPLE_CYCLE_NUM];

/* CT关闭放线*/
void xray_CT_disable()
{
    /*基准清零*/
    disable_hvref(0);
    disable_hvref(1);
    disable_filamentref(0);
    disable_filamentref(1);
    /*给射源*/
    config_HVEn_signal(0);
    config_mcuLock_signal(0);

    /*给灯丝*/
    config_filamentOn_signal(0, 0);
    config_filamentOn_signal(0, 1);

    /*给主控*/
    config_ready_signal(0);
    config_xrayOn_signal(0);

    return;
}

void xray_system_disable(uint16_t n)
{
    xray_CT_disable();

    ctrl_data.enable[n] = 0;
    ctrl_data.expo[n]  = 0;
    ctrl_data.filament_on[n]  = 0;
}

hvps_sm_state get_hv_state(uint16_t n)
{
    if (n < 2)
        return hv_state[n];
}

void set_hv_state(hvps_sm_state state, uint16_t n)
{
    hv_state[n] = state;

    return;
}

User_PID   user_pid;
PARAM_PID  param_pid;

void pid_Init(float target, uint32_t ref_init, uint8_t isPulseMode)
{
    if (isPulseMode)
    {
        user_pid.Kp = 25;
        user_pid.Ti = 1;
    }
    else
    {
        user_pid.Kp = 4;
        user_pid.Ti = 0.001;
    }

    user_pid.currTarget = target;

    param_pid.Error     = 0;
    param_pid.lastError     = 0;
    param_pid.integral  = 0;
    param_pid.Out_pid   = 0;
    param_pid.coeff     = 1;
    param_pid.config_ref = ref_init;

    return;
}

uint32_t test_flag = 0;
void tube_current_piControl(uint8_t conflag)
{

    param_pid.Error = user_pid.currTarget - user_pid.currValue;

    param_pid.integral  += param_pid.Error;

    param_pid.integral = MAX(MIN(param_pid.integral, 26), -26);


    param_pid.Out_pid = user_pid.Kp * param_pid.Error;


    // param_pid.Out_pid = MAX(MIN(param_pid.Out_pid, 30.0), 0);
    if (param_pid.integral == 26 || param_pid.integral == -26)
    {
        param_pid.integral = 0;
    }



    float pid_value;
    pid_value = param_pid.Out_pid;
    if (conflag == 1)
    {
        if (Is_ContinuousMode_CT())
        {
            pid_value = MAX(MIN(param_pid.Out_pid, 1), -1);
            // if (param_pid.Error < 0.05 && param_pid.Error > -0.05) pid_value = 0;
        }
    }
    else
    {
        pid_value = param_pid.Out_pid;
        if ((pid_value > -1) && (pid_value < 0)) pid_value = 0;
    }

    // if (pi_fast_flag == 1 && param_pid.Out_pid > 0) pid_value = 1;
    param_pid.config_ref = (uint32_t)(param_pid.config_ref + pid_value);

    param_pid.config_ref = MAX(MIN(param_pid.config_ref, 2750), 1000);

    // HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 2050);
    if (ctrl_data.xray_current == 1)
        HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, param_pid.config_ref);
    else {} //pwm

    return;
}

void tube_current_piControl_2(uint8_t conflag)
{
    // user_pid.currTarget = 5.0;
    param_pid.Error = user_pid.currTarget - user_pid.currValue;



    param_pid.Out_pid = user_pid.Kp * (param_pid.Error - param_pid.lastError) + user_pid.Ti * param_pid.Error;


    float pid_value;
    pid_value = param_pid.Out_pid;
    if (conflag == 1)
    {
        if (Is_ContinuousMode_CT()) pid_value = MAX(MIN(param_pid.Out_pid, 1), -1);
    }
    else
    {
        pid_value = param_pid.Out_pid;
    }
    if (param_pid.Error < 0.1f && param_pid.Error > -0.1f)
    {
        pid_value = 0;
    }

    param_pid.config_ref = (uint32_t)(param_pid.config_ref + pid_value);
    // debug_tx3("±ջ·ֵ: %f, %d, %f, %f\n",
    //     param_pid.Out_pid, param_pid.config_ref, user_pid.currValue, param_pid.integral);


    param_pid.config_ref = MAX(MIN(param_pid.config_ref, 2450), 1000);

    param_pid.lastError = param_pid.Error;

    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, param_pid.config_ref);

    return;
}


void save_parament_to_flash()
{
    wirte_flash_parament((uint8_t *)&parm_table, sizeof(xray_parament_table) * 2);

    return;
}

void config_filament0_ref_slop()
{
    if (ctrl_data.filament_on[0] == 1)
    {
        if (config_data.fila_ref_realtime[0] < IDLE_FILAMENT_REF) config_data.fila_ref_realtime[0] += config_data.fila_ref_step[0];
    }
    else
    {
        if (config_data.fila_ref_realtime > 0) config_data.fila_ref_realtime[0] -= config_data.fila_ref_step[0];
    }

    config_data.fila_ref_realtime[0] = MAX(MIN(config_data.fila_ref_realtime[0], IDLE_FILAMENT_REF), 0);

    uint32_t filament_ref = (uint32_t)floor(((config_data.fila_ref_realtime[0] / ADDA_FULL_SCALE_VIL_VALUE) * 4095));

    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, filament_ref);
}

void flash_table_init()
{
    get_flash_parament((uint8_t *)&parm_table, sizeof(xray_parament_table) * 2);

    uint32_t currRef[FILAMENT_CURRENT_TABLE_ORDER] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
    int i;

    if (memcmp((uint8_t *)&parm_table[0].currValue[0], (uint8_t *)&currRef[0], FILAMENT_CURRENT_TABLE_ORDER * sizeof(uint32_t)) != 0)
    {
        //
    }
    for (i = 0; i < FILAMENT_CURRENT_TABLE_ORDER; i++)
    {
        parm_table[0].currRef[i]   = MIN(MAX(parm_table[0].currRef[i], 1000), 2450);
        parm_table[0].currRef_c[i] = MIN(MAX(parm_table[0].currRef_c[i], 1000), 2450);
    }
    config_data.expo_count_total[0] = 0;     //


    if (memcmp((uint8_t *)&parm_table[1].currValue[0], (uint8_t *)&currRef[0], FILAMENT_CURRENT_TABLE_ORDER * sizeof(uint32_t)) != 0)
    {
        //
    }
    for (i = 0; i < FILAMENT_CURRENT_TABLE_ORDER; i++)
    {
        parm_table[1].currRef[i]   = MIN(MAX(parm_table[0].currRef[i], 1000), 2450);
        parm_table[1].currRef_c[i] = MIN(MAX(parm_table[0].currRef_c[i], 1000), 2450);
    }
    config_data.expo_count_total[1] = 0;     //
}

void hvState_ilde_init(uint16_t n)
{
    xray_data.isCheckAvailable[n] = 0;
    /*把曝光中的计数清零 */
//    debug_data.timmer_count = 0;
//    cali_data.timmer_count = 0;
}

void config_hvref_slope(uint16_t n)
{
    if ((get_hv_state(n) == HVPS_SM_ID_EXPOSURING) ||
            (get_hv_state(n) == HPVS_SM_ID_CAL_EXPOSURING) ||
            (get_hv_state(n) == HVPS_SM_ID_TRAIN_EXPOSURING))
    {
        if (config_data.tube_vol_realtime[n] < config_data.tube_vol[n]) config_data.tube_vol_realtime[n] += config_data.tube_vol_step[n];
    }
    else
    {
        if (config_data.tube_vol_realtime[n] > IDLE_HV_REF) config_data.tube_vol_realtime[n] -= config_data.tube_vol_step[n];
    }

    config_data.tube_vol_realtime[n] = MAX(MIN(config_data.tube_vol_realtime[n], config_data.tube_vol[n]), IDLE_HV_REF);

    int32_t tube_vol_ref = (int32_t)((config_data.tube_vol_realtime[n] / 64) / ADDA_FULL_SCALE_VIL_VALUE * 4095);  /* 0~2.5V  =  0~160kV */
    int32_t offset = (int32_t)(0.22405f * config_data.tube_vol_realtime[n] - 6.89364f);

    tube_vol_ref = tube_vol_ref + offset;
    tube_vol_ref = MAX(tube_vol_ref, 0);

    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)tube_vol_ref);
}

void config_filament_ref_slop(uint16_t n)
{
    if (n == 0)
    {
        if (ctrl_data.filament_on[n] == 1)
        {
            if (config_data.fila_ref_realtime[n] < IDLE_FILAMENT_REF)
                config_data.fila_ref_realtime[n] += config_data.fila_ref_step[n];
        }
        else
        {
            if (config_data.fila_ref_realtime[n] > 0)
                config_data.fila_ref_realtime[n] -= config_data.fila_ref_step[n];
        }
        config_data.fila_ref_realtime[n] = MAX(MIN(config_data.fila_ref_realtime[n], IDLE_FILAMENT_REF), 0);
        uint32_t filament_ref = (uint32_t)floor(((config_data.fila_ref_realtime[n] / ADDA_FULL_SCALE_VIL_VALUE) * 4095));
        HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (uint32_t)config_data.fila_ref_realtime[n]);
    }
    else
    {

    }
}

uint32_t get_filamentRef(float tube_current, uint16_t n)
{
    int index = 0;

    for (index = 0; index < FILAMENT_CURRENT_TABLE_ORDER; index++)
    {
        if (tube_current >= parm_table[n].currValue[FILAMENT_CURRENT_TABLE_ORDER - 1])
        {
            config_data.tube_curr_index[n] = FILAMENT_CURRENT_TABLE_ORDER - 1;
            break;
        }

        if ((tube_current >= parm_table[n].currValue[index]) && (tube_current < parm_table[n].currValue[index + 1]))
        {
            config_data.tube_curr_index[n] = index;
            break;
        }
    }

    if (tube_current >= parm_table[n].currValue[FILAMENT_CURRENT_TABLE_ORDER - 1])
    {
        return parm_table[n].currRef[config_data.tube_curr_index[n]];
    }

    uint32_t currRef_uplimit;
    uint32_t currRef_downlimit;

    currRef_uplimit   = parm_table[n].currRef[config_data.tube_curr_index[n] + 1];
    currRef_downlimit = parm_table[n].currRef[config_data.tube_curr_index[n]];

    return (currRef_downlimit + (tube_current - parm_table[n].currValue[index]) * (currRef_uplimit - currRef_downlimit));

}

void config_filamentRef(uint16_t n)
{
    config_data.fila_ref_realtime[n] = get_filamentRef(config_data.tube_curr[n], n);
    if (n == 0)
        HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, config_data.fila_ref_realtime[n]);
    else
    {
        //pwm
    }
    return;
}
void disable_hvref(uint16_t n)
{
    config_data.tube_vol_realtime[n] = 0;
    config_data.tube_vol[n] = 0;
    config_data.tube_vol_step[n] = 0;

    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
}

void disable_filamentref(uint16_t n)
{
    if (n == 0)
        HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
    else
    {}
    //PWM DOWN
}

float kp_test = 3.5;
float ki_test = 0.0009;
void  ct_task()
{
//    ctrl_data.xray_current = 0;
    for (uint8_t i = 0; i < XRAY_NUMS; i++)
    {
        if (xray_data.timmer_count[i] == 1)
            xray_data.timmer_count[i]++;
        switch (hv_state[i])
        {
        case HVPS_SM_ID_IDLE:

            hvState_ilde_init(i);
            config_ready_signal(1);
            config_xrayOn_signal(1);
            //开启灯丝，预热2.5s
            if ((ctrl_data.enable[i] == 1) && (ctrl_data.interlock == 1) &&
                    (ctrl_data.filament_on[i] == 1) && (xray_data.timmer_count[i] > TIMER6_2P5_SECOND_CYCLES))
            {
                config_mcuLock_signal(1);
                set_hv_state(HVPS_SM_ID_PREPARE, i);
                xray_data.timmer_count[i] = 1;
                config_data.expo_count[i] = 0;
            }
            if (ctrl_data.enable[i] == 1)
            {
                config_enable_sw(i);

            }
            if (ctrl_data.filament_on[i] == 1)
            {
                config_filament_ref_slop(i);
            }
            break;

        case HVPS_SM_ID_PREPARE:
            config_filamentRef(i);//灯丝基准拉到预期值

            /*通知MCU准备好*/
            config_ready_signal(1);
            set_hv_state(HVPS_SM_ID_READY, i);
            xray_data.timmer_count[i] = 0;
            uint32_t currRef = config_data.fila_ref_realtime[i];

            pid_Init(config_data.tube_curr[i], currRef, Is_PulseMode_CT());
            break;

        case HVPS_SM_ID_READY:
            config_hvref_slope(i);
            if (ctrl_data.enable[i] && ctrl_data.expo[i])
            {
                config_HVEn_signal(i);

                //通知MCU正在曝光
                config_xrayOn_signal(i);

                set_hv_state(HVPS_SM_ID_EXPOSURING, i);
                xray_data.timmer_count[i] = 1;
            }
            if (ctrl_data.enable[i] == 0)//exp先关，enable后关
            {
                if (config_data.expo_count_total[i] > 1)
                {
                    parm_table[i].expo_count_total++;
                    parm_table[i].expo_times_total += config_data.expo_count_total[i] / 3000000;//曝光60s加一次
                }
                config_xrayOn_signal(0);
                set_hv_state(HVPS_SM_ID_EXPO_END, i);
            }

        case HVPS_SM_ID_EXPOSURING:
            if (xray_data.timmer_count[i] > 185)
            {
                xray_data.isCheckAvailable[i] = 1;
            }
            else
            {
                xray_data.isCheckAvailable[i] = 0;
            }
            config_hvref_slope(i);

            if (xray_data.timmer_count[i] == TIMER6_8_MILSECOND_CYCLES)//等待8ms稳定
            {
                user_pid.currValue = sampled_data.tube_curr_value;
            }

            if ((xray_data.timmer_count[i] > TIMER6_8_MILSECOND_CYCLES))
            {
                if (xray_data.timmer_count[i] % TIMER6_5_MILSECOND_CYCLES == 0)
                {
                    user_pid.currValue = sampled_data.tube_curr_value;
                    user_pid.Kp = kp_test;
                    user_pid.Ti = ki_test;

                    tube_current_piControl(1);
                }
            }
            else
            {
                xray_data.isCheckAvailable[i] = 0;
            }

            if (ctrl_data.enable[i] == 0)
            {
                xray_data.isCheckAvailable[i] = 0;
                parm_table[i].expo_count_total++;

                parm_table[i].expo_times_total += config_data.expo_count_total[i] / 3000000;
                config_xrayOn_signal(0);
                set_hv_state(HVPS_SM_ID_EXPO_END, i);
            }

            if (ctrl_data.enable[i] && (ctrl_data.expo[i] == 0))
            {
                xray_data.isCheckAvailable[i] = 0;
                config_xrayOn_signal(0);
                set_hv_state(HVPS_SM_ID_READY, i);
                user_pid.Kp = 20;
                user_pid.Ti = 1;
                tube_current_piControl(0);
                debug_tx3("闭环值: %f, %d, %f, %f\n",
                          param_pid.Out_pid, param_pid.config_ref, user_pid.currValue, param_pid.integral);
            }

            if (ctrl_data.enable[i] && ctrl_data.expo[i])
            {
                config_data.expo_count_total[i]++;
                config_data.expo_count[i]++;
            }
            //
            break;


        case HVPS_SM_ID_EXPO_END:
            ctrl_data.filament_on[i] = 0;
            xray_CT_disable();

            set_hv_state(HVPS_SM_ID_IDLE, i);
            if ((ctrl_data.xrayMode == XRAY_MODE_D_CONTINUOUS) | (ctrl_data.xrayMode == XRAY_MODE_D_PULSE))
            {
                if (i == 1)
                {
                    i = 0;
                    ctrl_data.xray_current = 1;
                }
                else
                    ctrl_data.xray_current = 2;
            }
            else
                i = 2;
            break;
        default:
            set_hv_state(HVPS_SM_ID_IDLE, i);
        }
        if (i < 2)
        {
            if (get_filament_pin(i) && (get_hv_state(i) != HVPS_SM_ID_EXPOSURING))
            {
                config_data.fila_protect_cnt[i]++;
            }
            else
            {
                config_data.fila_protect_cnt[i] = 0;
            }
        }
    }

}





