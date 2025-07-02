#include "HV_exposure.h"
#include "adc.h"
#include <math.h>
#include "string.h"
#include "app_spi.h"
#include "calibrate.h"
#include "delay.h"
#include "xray.h"


#define IDLE_FILAMENT_REF               1000

hvps_sm_state volatile hv_state[XRAY_NUMS];
volatile cmd_control_data ctrl_data;
volatile xray_config_data config_data;
volatile xray_parament_table parm_table[XRAY_NUMS];
volatile xray_parament_range para_range;
volatile xray_running_data xray_data;
volatile adc_sampled_value sampled_data;
volatile adc_sampled_value sampled_data_last;

uint16_t adc_buffer2[ADC_2_CHANNEL_NUM * ADC_SAMPLE_CYCLE_NUM];
uint16_t adc_buffer3[ADC_3_CHANNEL_NUM * ADC_SAMPLE_CYCLE_NUM];

extern volatile ctrl_calibr ctrl_calibr_data;

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
    // user_pid.currTarget = 5.0;
    param_pid.Error = user_pid.currTarget - user_pid.currValue;
    // if (param_pid.Error < 0.05 && param_pid.Error > -0.05) {
    //     test_flag = 1;
    //     return;
    // } else {
    //     test_flag = 0;
    // }
    // if (fabsf(param_pid.Error) < 0.1) {
    //     param_pid.integral  += param_pid.Error;
    // } else {
    //     param_pid.integral  += 0;
    // }

    // param_pid.integral  += param_pid.Error;


    // param_pid.integral = MAX(MIN(param_pid.integral, 26), -26);

    param_pid.Out_pid = user_pid.Kp * param_pid.Error + user_pid.Ti * param_pid.integral;

    // param_pid.Out_pid = MAX(MIN(param_pid.Out_pid, 30.0), 0);



    // uint32_t fila_vol_ref = parm_table.currRef[config_data.tube_curr_index];
    // float fila_vol_ref = 1737;
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
    // if (param_pid.Error < 0.1 && param_pid.Error > -0.1) {
    //     pid_value = 0;
    //     param_pid.integral = 0;
    // }

    if (param_pid.Out_pid > 0.5f && param_pid.Out_pid < 1.0f) pid_value = 1;
    if (param_pid.Out_pid < -0.5f && param_pid.Out_pid > -1.0f) pid_value = -1;
    param_pid.config_ref = (uint32_t)(param_pid.config_ref + pid_value);


    param_pid.config_ref = MAX(MIN(param_pid.config_ref, 2450), 1000);

    if (param_pid.Error < 0.05f && param_pid.Error > -0.05f) param_pid.config_ref = param_pid.config_ref + 1;

    // HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 2050);
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, param_pid.config_ref);

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
    xray_data.isCheckAvailable = 0;
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

    int32_t tube_vol_ref = (int32_t)((config_data.tube_vol_realtime[n] / 64) / ADDA_FULL_SCALE_VIL_VALUE * 4095);  /* 0~2.5V ¶ԓ¦ 0~160kV */
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

        /* ϲςȡջ */
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
void disable_hvref()
{
    // config_data.tube_vol_realtime = 0;
    // config_data.tube_vol = 0;
    // config_data.tube_vol_step = 0;

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

void xray_CT_disable(uint16_t n)
{

    disable_hvref();
    disable_filamentref(n);

    config_HVEn_signal(0);      /* ¸ߵ熽¿ª */
    config_mcuLock_signal(0);

    /* ¸øµƋ¿ */
    config_filamentOn_signal(0, n);

    /* ¸øַ¿ؠ*/
    config_ready_signal(0);
    config_xrayOn_signal(0);

    return;
}
float kp_test = 3.5;
float ki_test = 0.0009;
void HVPS_SM_Control()
{
//    ctrl_data.xray_current = 0;
    uint16_t xray_num = 0;
    switch (hv_state[xray_num])
    {
    case HVPS_SM_ID_IDLE:
        if (xray_data.timmer_count[xray_num] == 1)
            xray_data.timmer_count[xray_num]++;
        hvState_ilde_init(xray_num);
        config_ready_signal(1);
        config_xrayOn_signal(1);

        if ((ctrl_data.enable[xray_num] == 1) && (ctrl_data.interlock == 1) &&
                (ctrl_data.filament_on[xray_num] == 1) && (xray_data.timmer_count[xray_num] > TIMER6_2P5_SECOND_CYCLES))
        {
            config_mcuLock_signal(1);
            set_hv_state(HVPS_SM_ID_PREPARE, xray_num);
            xray_data.timmer_count[xray_num] = 1;
            config_data.expo_count[xray_num] = 0;
        }

        if (ctrl_data.filament_on[xray_num] == 1)
        {
            config_HV_sw(1, xray_num);
            config_filament_ref_slop(xray_num);
        }
        break;

    case HVPS_SM_ID_PREPARE:
        config_filamentRef(xray_num);

        /* ֪ͨMCUґ׼±¸ºà*/
        config_ready_signal(1);
        set_hv_state(HVPS_SM_ID_READY, xray_num);
        xray_data.timmer_count[xray_num] = 0;
        uint32_t currRef = config_data.fila_ref_realtime[xray_num];

        pid_Init(config_data.tube_curr[xray_num], currRef, Is_PulseMode_CT());
        break;

    case HVPS_SM_ID_READY:
        config_hvref_slope(xray_num);
        if (ctrl_data.enable[xray_num] && ctrl_data.expo[xray_num])
        {
            /**/
            config_HVEn_signal(1);

            //
            config_xrayOn_signal(1);

            set_hv_state(HVPS_SM_ID_EXPOSURING, xray_num);
            xray_data.timmer_count[xray_num] = 1;
        }
        else
            config_filamentOn_signal(0, 1);

        if (ctrl_data.enable[xray_num] == 0)
        {
            if (config_data.expo_count_total[xray_num] > 1)
            {
                parm_table[xray_num].expo_count_total++;
                parm_table[xray_num].expo_times_total += config_data.expo_count_total[xray_num] / 3000000;
            }
            config_xrayOn_signal(0);
            set_hv_state(HVPS_SM_ID_EXPO_END, xray_num);
        }

    case HVPS_SM_ID_EXPOSURING:
        config_hvref_slope(xray_num);

        if (xray_data.timmer_count[xray_num] == TIMER6_8_MILSECOND_CYCLES)
        {
            user_pid.currValue = sampled_data.tube_curr_value;
        }

        // if (Is_ContinuousMode_CT() && (xray_data.timmer_count > TIMER6_8_MILSECOND_CYCLES)) {
        if ((xray_data.timmer_count[xray_num] > TIMER6_5_MILSECOND_CYCLES))
        {
            xray_data.isCheckAvailable = 1;
            if (xray_data.timmer_count[xray_num] % TIMER6_10_MILSECOND_CYCLES == 0)
            {
                user_pid.currValue = sampled_data.tube_curr_value;
                user_pid.Kp = kp_test;
                // user_pid.Ti = 0.005;
                user_pid.Ti = ki_test;

                tube_current_piControl(1);
            }
        }
        else
        {
            xray_data.isCheckAvailable = 0;
        }

        if (ctrl_data.enable[xray_num] == 0)
        {
            xray_data.isCheckAvailable = 0;
            parm_table[xray_num].expo_count_total++;

            parm_table[xray_num].expo_times_total += config_data.expo_count_total[xray_num] / 3000000;
            config_xrayOn_signal(0);
            set_hv_state(HVPS_SM_ID_EXPO_END, xray_num);
        }

        if (ctrl_data.enable[xray_num] && (ctrl_data.expo[xray_num] == 0))
        {
            xray_data.isCheckAvailable = 0;
            config_xrayOn_signal(0);
            set_hv_state(HVPS_SM_ID_READY, xray_num);
            user_pid.Kp = 20;
            user_pid.Ti = 1;
            tube_current_piControl(0);
            debug_tx3("闭环值: %f, %d, %f, %f\n",
                      param_pid.Out_pid, param_pid.config_ref, user_pid.currValue, param_pid.integral);
        }

        if (ctrl_data.enable[xray_num] && ctrl_data.expo[xray_num])
        {
            config_data.expo_count_total[xray_num]++;
            config_data.expo_count[xray_num]++;
        }
        //
        break;


    case HVPS_SM_ID_EXPO_END:
        ctrl_data.filament_on[xray_num] = 0;
        xray_CT_disable(xray_num);
        ctrl_calibr_data.calibraflag = 1;

        set_hv_state(HVPS_SM_ID_IDLE, xray_num);
    default:
        set_hv_state(HVPS_SM_ID_EXPOSURING, xray_num);
    }
    xray_num = 1;
    switch (hv_state[xray_num])
    {
    case HVPS_SM_ID_IDLE:
        if (xray_data.timmer_count[xray_num] == 1)
            xray_data.timmer_count[xray_num]++;
        hvState_ilde_init(xray_num);
        config_ready_signal(1);
        config_xrayOn_signal(1);

        if ((ctrl_data.enable[xray_num] == 1) && (ctrl_data.interlock == 1) &&
                (ctrl_data.filament_on[xray_num] == 1) && (xray_data.timmer_count[xray_num] > TIMER6_2P5_SECOND_CYCLES))
        {
            config_mcuLock_signal(1);
            set_hv_state(HVPS_SM_ID_PREPARE, xray_num);
            xray_data.timmer_count[xray_num] = 1;
            config_data.expo_count[xray_num] = 0;
        }

        if (ctrl_data.filament_on[xray_num] == 1)
        {
            config_filament_ref_slop(xray_num);
        }
        break;

    case HVPS_SM_ID_PREPARE:
        config_filamentRef(xray_num);


        config_ready_signal(1);
        set_hv_state(HVPS_SM_ID_READY, xray_num);
        xray_data.timmer_count[xray_num] = 0;
        uint32_t currRef = config_data.fila_ref_realtime[xray_num];

        pid_Init(config_data.tube_curr[xray_num], currRef, Is_PulseMode_CT());
        break;

    case HVPS_SM_ID_READY:
        config_hvref_slope(xray_num);
        if (ctrl_data.enable[xray_num] && ctrl_data.expo[xray_num])
        {
            /**/
            config_HVEn_signal(1);

            //
            config_xrayOn_signal(1);

            set_hv_state(HVPS_SM_ID_EXPOSURING, xray_num);
            xray_data.timmer_count[xray_num] = 1;
        }
        else
            config_filamentOn_signal(0, xray_num);


        if (ctrl_data.enable[xray_num] == 0)
        {
            if (config_data.expo_count_total[xray_num] > 1)
            {
                parm_table[xray_num].expo_count_total++;

                parm_table[xray_num].expo_times_total += config_data.expo_count_total[xray_num] / 3000000;
            }
            config_xrayOn_signal(0);
            set_hv_state(HVPS_SM_ID_EXPO_END, xray_num);
        }

    case HVPS_SM_ID_EXPOSURING:
        config_hvref_slope(xray_num);

        if (xray_data.timmer_count[xray_num] == TIMER6_8_MILSECOND_CYCLES)
        {
            user_pid.currValue = sampled_data.tube_curr_value;
        }

        // if (Is_ContinuousMode_CT() && (xray_data.timmer_count > TIMER6_8_MILSECOND_CYCLES)) {
        if ((xray_data.timmer_count[xray_num] > TIMER6_5_MILSECOND_CYCLES))
        {
            xray_data.isCheckAvailable = 1;
            if (xray_data.timmer_count[xray_num] % TIMER6_10_MILSECOND_CYCLES == 0)
            {
                user_pid.currValue = sampled_data.tube_curr_value;
                user_pid.Kp = kp_test;
                // user_pid.Ti = 0.005;
                user_pid.Ti = ki_test;

                tube_current_piControl(1);
            }
        }
        else
        {
            xray_data.isCheckAvailable = 0;
        }


        if (ctrl_data.enable[xray_num] == 0)
        {
            xray_data.isCheckAvailable = 0;
            parm_table[xray_num].expo_count_total++;

            parm_table[xray_num].expo_times_total += config_data.expo_count_total[xray_num] / 3000000;
            config_xrayOn_signal(0);
            set_hv_state(HVPS_SM_ID_EXPO_END, xray_num);
        }

        if (ctrl_data.enable[xray_num] && (ctrl_data.expo[xray_num] == 0))
        {
            xray_data.isCheckAvailable = 0;
            config_xrayOn_signal(0);
            set_hv_state(HVPS_SM_ID_READY, xray_num);
            user_pid.Kp = 20;
            user_pid.Ti = 1;
            tube_current_piControl(0);
            debug_tx3("闭环值: %f, %d, %f, %f\n",
                      param_pid.Out_pid, param_pid.config_ref, user_pid.currValue, param_pid.integral);
        }

        if (ctrl_data.enable[xray_num] && ctrl_data.expo[xray_num])
        {
            config_data.expo_count_total[xray_num]++;
            config_data.expo_count[xray_num]++;
        }
        //
        break;


    case HVPS_SM_ID_EXPO_END:
        ctrl_data.filament_on[xray_num] = 0;
        xray_CT_disable(xray_num);
        ctrl_calibr_data.calibraflag = 1;

        set_hv_state(HVPS_SM_ID_IDLE, xray_num);
    default:
        set_hv_state(HVPS_SM_ID_EXPOSURING, xray_num);
    }

    (get_hv_state(xray_num) == HVPS_SM_ID_EXPOSURING) ? xray_on_led(1) : xray_on_led(0);

    if (get_filament_pin(xray_num) && (get_hv_state(xray_num) != HVPS_SM_ID_EXPOSURING))
    {
        config_data.fila_protect_cnt[xray_num]++;
    }
    else
    {
        config_data.fila_protect_cnt[xray_num] = 0;
    }
}

//void PI_Control_Update()
//{
//      float HV_Err;

//    // 计算电压、电流和参考值
//    switch(ctrl_data.xray_current)
//    {
//        case 1:
//            CalculateSlopeRef(0, &HVPS_Ref_Regs);
//            HVPS_Regs.hv_out_v_ref = HVPS_Ref_Regs.Ref_Output;
//            HVPS_Regs.hv_out_v = (float)ADC_HV_OUT_VOLTAGE * HVPS_Regs.hv_out_v_coef[0];
//            HVPS_Regs.hv_out_c = (float)ADC_HV_OUT_CURRENT * HVPS_Regs.hv_out_c_coef[0];
//            HV_Err = HVPS_Regs.hv_out_v_ref - HVPS_Regs.hv_out_v;
//            if(HVPS_Regs.hv_out_v > 25.0f)
//            {
//                mStatistics.isr_count[0]++;
//                if(mStatistics.isr_count[0] > 4800000)
//                {
//                    mStatistics.isr_count[0] -= 4800000;
//                    mStatistics.expo_time[0]++;
//                }
//            }
//            break;

//        case 2:
//            CalculateSlopeRef(1, &HVPS_Ref_Regs);
//            HVPS_Regs.hv_out_v_ref = HVPS_Ref_Regs.Ref_Output;
//            HVPS_Regs.hv_out_v = (float)ADC_HV_OUT_VOLTAGE * HVPS_Regs.hv_out_v_coef[1];
//            HVPS_Regs.hv_out_c = (float)ADC_HV_OUT_CURRENT * HVPS_Regs.hv_out_c_coef[1];
//            HV_Err = HVPS_Regs.hv_out_v_ref - HVPS_Regs.hv_out_v;
//            if(HVPS_Regs.hv_out_v > 25.0f)
//            {
//                mStatistics.isr_count[1]++;
//                if(mStatistics.isr_count[1] > 4800000)
//                {
//                    mStatistics.isr_count[1] -= 4800000;
//                    mStatistics.expo_time[1]++;
//                }
//            }
//            break;

//        default:
//            HVPS_Ref_Regs.Stage = 0;
//            CalculateSlopeRef(0, &HVPS_Ref_Regs);
//            HVPS_Regs.hv_out_v_ref = 0.0f;
//            HVPS_Regs.hv_out_v = (float)ADC_HV_OUT_VOLTAGE * HVPS_Regs.hv_out_v_coef[0];
//            HVPS_Regs.hv_out_c = (float)ADC_HV_OUT_CURRENT * HVPS_Regs.hv_out_c_coef[0];
//            HV_Err = 0.0f;
//            break;
//    }

//}





