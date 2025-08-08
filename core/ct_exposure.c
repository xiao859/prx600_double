#include "ct_exposure.h"
#include "adc.h"
#include <math.h>
#include "string.h"
#include "app_spi.h"
#include "app_uart.h"
#include "delay.h"
#include "xray.h"
#include "tim.h"
#include "stdbool.h"
#include "pi_ctl.h"


hvps_sm_state volatile hv_state[XRAY_NUMS];

volatile xray_config_data config_data;
volatile xray_parament_table parm_table[XRAY_NUMS] =
{
    {
        0, 0, 1,
        {1,    2,    3,    4,    5,    6,    7,    8,    9,    10,  11,   12},
        {1700, 1850, 1980, 2050, 2140, 2200, 2270, 2320, 2370, 2400, 2440, 2480},
        {1600, 1800, 1890, 2000, 2050, 2120, 2190, 2230, 2290, 2330, 2370, 2430},
    },
    {
        0, 0, 1,
        {1,    2,    3,    4,    5,    6,    7,    8,    9,    10,  11,   12},
        {1100, 1200, 1280, 1330, 1380, 1420, 1460, 1500, 1530, 1550, 1580, 1600},
        {1030, 1150, 1210, 1270, 1320, 1360, 1400, 1430, 1460, 1480, 1500, 1530},
    },
};
volatile xray_parament_range para_range =
{
    140, 20,            /*管电压保护值140*/
    130, 5, 750,        /*管电流保护值*/

    70, -25, 60, 50,    /*油温*/

    28, 20,             /* 24V供电*/
    100, 100,           /*散热器温度*/
    30, 8,              /*灯丝电压*/
    100, 100,           /*灯丝电流*/

    125, 60,            /*管电压配置门限*/
    120, 10,            /*管电流配置门限*/

    1000, 1000,         /*曝光时间保护*/

    4500000,            /*曝光时间1.5min */

    6000000             /*灯丝开启未曝光最大时间*/
};
volatile xray_running_data xray_data;
volatile adc_sampled_value sampled_data =
{
    30, 30, 10, 1000, 24, 0, 24, 0, 20
};
volatile adc_sampled_value sampled_data_last = {0};
volatile cmd_control_data ctrl_data;

uint16_t adc_buffer2[ADC_2_CHANNEL_NUM * ADC_SAMPLE_CYCLE_NUM];
uint16_t adc_buffer3[ADC_3_CHANNEL_NUM * ADC_SAMPLE_CYCLE_NUM];

/* CT关闭放线*/
void xray_CT_disable()
{
    /*基准清零*/
    disable_hvref();
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

void xray_system_disable()
{
    xray_CT_disable();

    ctrl_data.enable[0] = 0;
    ctrl_data.expo[0]  = 0;
    ctrl_data.filament_on[0]  = 0;

    ctrl_data.enable[1] = 0;
    ctrl_data.expo[1]  = 0;
    ctrl_data.filament_on[1]  = 0;
}

hvps_sm_state get_hv_state(uint16_t n)
{
    if (n < 2)
        return hv_state[n];
    return HVPS_SM_ID_FAULT;
}

void set_hv_state(hvps_sm_state state, uint16_t n)
{
    hv_state[n] = state;

    return;
}

void save_parament_to_flash()
{
    bsp_erase_sector(0x80000000);
    wirte_flash_parament((uint8_t *)&parm_table, sizeof(xray_parament_table));

    return;
}

void config_filament0_ref_slop()
{
    if (ctrl_data.filament_on[0] == 1)
    {
        if (config_data.fila_ref_realtime[0] < IDLE_FILAMENT1_REF) config_data.fila_ref_realtime[0] += config_data.fila_ref_step[0];
    }
    else
    {
        if (config_data.fila_ref_realtime > 0) config_data.fila_ref_realtime[0] -= config_data.fila_ref_step[0];
    }

    config_data.fila_ref_realtime[0] = MAX(MIN(config_data.fila_ref_realtime[0], IDLE_FILAMENT1_REF), 0);

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
        parm_table[0].currRef[i]   = MIN(MAX(parm_table[0].currRef[i], 1000), 2500);
        parm_table[0].currRef_c[i] = MIN(MAX(parm_table[0].currRef_c[i], 1000), 2500);
    }
    config_data.expo_count_total[0] = 0;     //


    if (memcmp((uint8_t *)&parm_table[1].currValue[0], (uint8_t *)&currRef[0], FILAMENT_CURRENT_TABLE_ORDER * sizeof(uint32_t)) != 0)
    {
        //
    }
    for (i = 0; i < FILAMENT_CURRENT_TABLE_ORDER; i++)
    {
        parm_table[1].currRef[i]   = MIN(MAX(parm_table[1].currRef[i], 1000), 1650);
        parm_table[1].currRef_c[i] = MIN(MAX(parm_table[1].currRef_c[i], 1000), 1650);
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

//    int32_t tube_vol_ref = (int32_t)((config_data.tube_vol_realtime[n] / 64) / ADDA_FULL_SCALE_VIL_VALUE * 4095);  /* 0~2.5V  =  0~160kV */
    int32_t tube_vol_ref = (int32_t)(config_data.tube_vol_realtime[n] * 19.389f);
    int32_t offset = (int32_t)(0.22405f * config_data.tube_vol_realtime[n] - 6.89364f);

    tube_vol_ref = tube_vol_ref + offset;
    tube_vol_ref = MAX(tube_vol_ref, 0);

    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)tube_vol_ref);
}

void config_filament_ref_slop(uint16_t n)
{

    if (ctrl_data.filament_on[n] == 1)
    {
        if (config_data.fila_ref_realtime[n] < (n == 0 ? IDLE_FILAMENT1_REF : IDLE_FILAMENT2_REF))
            config_data.fila_ref_realtime[n] += config_data.fila_ref_step[n];
    }
    else
    {
        if (config_data.fila_ref_realtime[n] > 0)
            config_data.fila_ref_realtime[n] -= config_data.fila_ref_step[n];
    }
    uint32_t fila_ref_target;
    if (n == 0)
    {
        fila_ref_target = (ctrl_data.filament_on[n] == 1) ? IDLE_FILAMENT1_REF : 0;
        config_data.fila_ref_realtime[n] = MAX(MIN(config_data.fila_ref_realtime[n], IDLE_FILAMENT1_REF), 0);   
				if (config_data.fila_ref_realtime[n] == fila_ref_target) return;
       // uint32_t filament_ref = (uint32_t)floor(((config_data.fila_ref_realtime[n] / ADDA_FULL_SCALE_VIL_VALUE) * 4095));
        HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (uint32_t)config_data.fila_ref_realtime[n]);
    }
    else
    {
        fila_ref_target = (ctrl_data.filament_on[n] == 1) ? IDLE_FILAMENT2_REF : 0;
        config_data.fila_ref_realtime[n] = MAX(MIN(config_data.fila_ref_realtime[n], IDLE_FILAMENT2_REF), 0); 
				if (config_data.fila_ref_realtime[n] == fila_ref_target) return;
        //uint32_t filament_ref = (uint32_t)floor(((config_data.fila_ref_realtime[n] / ADDA_FULL_SCALE_VIL_VALUE) * 2999));
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, config_data.fila_ref_realtime[n]);//1360

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
//		return parm_table[n].currRef[config_data.tube_curr_index[n]];
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
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4,config_data.fila_ref_realtime[n] );//1360
    }
    return;
}
void disable_hvref()
{
//    config_data.tube_vol_realtime[n] = 0;
//    config_data.tube_vol[n] = 0;
//    config_data.tube_vol_step[n] = 0;

    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
}

void disable_filamentref(uint16_t n)
{
    if (n == 0)
        HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
    else
        //PWM DOWN
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
}

void check_dual_filament_preheat(void)
{
    if ((ctrl_data.xrayMode == XRAY_MODE_D_CONTINUOUS) || (ctrl_data.xrayMode == XRAY_MODE_D_PULSE))
    {
        ctrl_data.filament_on[0] = 1;
        ctrl_data.filament_on[1] = 1;
    }
}


//float kp_test = 3.5;
//float ki_test = 0.0009;
uint32_t pulse_time_base_count = 0;
float pulse_kp = 100;
float pulse_ki = 1;
uint32_t oldref[2] = {0};
void ct_task()
{
    static uint32_t last_expo_end_tick[XRAY_NUMS] = {0};
    static uint8_t sw_state = 0;  // 采样开关状态：0-SW1，1-SW2
    static uint16_t ct_source = 0;
    static uint16_t sw_count = 0;
    bool expo_end = false;
    bool is_dual_source = false;

    ctrl_data.xray_current = ct_source + 1;
    hvps_sm_state ct_source_state = get_hv_state(ct_source);

    // Tick自增
    if (xray_data.timmer_count[ct_source] >= 1)
        xray_data.timmer_count[ct_source]++;
    else
        return;

    switch (ct_source_state)
    {
    case HVPS_SM_ID_IDLE:
        hvState_ilde_init(ct_source);
        config_ready_signal(0);
        config_xrayOn_signal(0);


        // 先判断是否双源开启，提前进行预热参考输出
        if (ctrl_data.filament_on[0] && ctrl_data.filament_on[1])
        {
            config_filament_ref_slop(0);
            config_filament_ref_slop(1);
        }
        else if (ctrl_data.filament_on[ct_source])
        {
            config_filament_ref_slop(ct_source);
        }
        else if (ctrl_data.filament_on[0] == 0)
            break;

        // 仅当满足时间、interlock后才允许进入下一状态
        if (ctrl_data.interlock && ctrl_data.filament_on[ct_source] &&
                xray_data.timmer_count[ct_source] > TIMER6_2P5_SECOND_CYCLES)
        {
            config_mcuLock_signal(1);
            set_hv_state(HVPS_SM_ID_PREPARE, ct_source);
            xray_data.timmer_count[ct_source] = 1;
            config_data.expo_count[ct_source] = 0;
        }
        break;
        
    case HVPS_SM_ID_PREPARE:
				config_filamentRef(ct_source); /*灯丝基准值拉到预期*/
				HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, parm_table[ct_source].currRef[config_data.tube_curr_index[ct_source]]);

				__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4,parm_table[ct_source].currRef[config_data.tube_curr_index[ct_source]]);// 1360
				//	uint32_t rtt= parm_table[ct_source].currRef[config_data.tube_curr_index[ct_source]];
				
        config_ready_signal(1);
        set_hv_state(HVPS_SM_ID_READY, ct_source);
        xray_data.timmer_count[ct_source] = 1;
			  pid_Init(config_data.tube_curr[ct_source], config_data.fila_ref_realtime[ct_source], Is_PulseMode_CT());
        param_pid.pulse_count = 0;
        pid_Init_2(config_data.tube_curr[0], config_data.fila_ref_realtime[0], Is_PulseMode_CT(), 0);
				pid_Init_2(config_data.tube_curr[1], config_data.fila_ref_realtime[1], Is_PulseMode_CT(), 1);
        break;

    case HVPS_SM_ID_READY:

		
        config_hvref_slope(ct_source);
        if (ctrl_data.enable[ct_source] && ctrl_data.expo[ct_source])
        {
            // 曝光允许前需检查对方曝光是否间隔超过10ms
            uint8_t other = (ct_source == 0) ? 1 : 0;
            if (HAL_GetTick() - last_expo_end_tick[other] < 10)
                break;  // 距离对方曝光过短，等待
            set_hv_state(HVPS_SM_ID_EXPOSURING, ct_source);
            xray_data.timmer_count[ct_source] = 1;
        }

        if (!ctrl_data.enable[ct_source])
        {
            if (config_data.expo_count_total[ct_source] > 1)
            {
                parm_table[ct_source].expo_count_total++;
                /*曝光60s加一次*/
                parm_table[ct_source].expo_times_total += config_data.expo_count_total[ct_source] / 3000000;
            }
            config_xrayOn_signal(0);
            set_hv_state(HVPS_SM_ID_EXPO_END, ct_source);
        }
        break;

    case HVPS_SM_ID_EXPOSURING:

        // 曝光后 3.7ms 开始允许采样检查
        xray_data.isCheckAvailable[ct_source] = (xray_data.timmer_count[ct_source] > 185) ? 1 : 0;

        // 持续输出高压与准备信号
        config_hvref_slope(ct_source);
        config_mcuLock_signal(1);
        config_HVEn_signal(1);
        config_xrayOn_signal(1);

        // 曝光控制：延时 PI 初始化
        user_pid_2.currValue[ct_source] = 0.00645f * ((float)(adc_buffer3[2]));
        user_pid.currValue = 0.00645f * ((float)(adc_buffer3[2]));        

        /*连续模式PI调节*/
        if ((param_pid.pulse_count >= 5) && Is_ContinuousMode_CT() && (xray_data.timmer_count[ct_source] > TIMER6_5_MILSECOND_CYCLES))
        {

            param_pid.ti_CycleCount++;

            user_pid.Kp = 0;
            user_pid.Ti = 0.2;

            if (param_pid.ti_CycleCount == TIMER6_10_MILSECOND_CYCLES)
            {
                param_pid.ki_flag = 1;
                param_pid.ti_CycleCount = 0;
            }
            else
            {
                param_pid.ki_flag = 0;
            }

            user_pid_2.Kp = pulse_kp;
            user_pid_2.Ki = pulse_ki;

            tube_current_piControl(1, ct_source);
        }

        // ---- 曝光结束条件判断 ----
        if (!ctrl_data.enable[ct_source])
        {
            // 主使能关闭
            expo_end = true;
        }
        else if (!ctrl_data.expo[ct_source])
        {
            // 曝光信号关闭
            expo_end = true;

            if (ctrl_data.enable[1 - ct_source])
            {
                // 双源脉冲模式：准备切换通道
                set_hv_state(HVPS_SM_ID_EXPO_END, ct_source);
            }
            else
            {
                // 单源脉冲：留在 READY 等待下次曝光
                set_hv_state(HVPS_SM_ID_READY, ct_source);
                param_pid.pulse_count++;
            }
        }

        if (expo_end)
        {
            // 所有模式通用关闭操作
            xray_data.isCheckAvailable[ct_source] = 0;
            config_mcuLock_signal(0);
            config_HVEn_signal(0);
            config_xrayOn_signal(0);
            if (!Is_ContinuousMode_CT())
            {
                user_pid_2.Kp = 100;
                user_pid_2.Ki = 30;
                oldref[ct_source] = user_pid_2.config_ref[ct_source];
                tube_current_piControl_v2(ct_source);
                param_pid.config_ref = user_pid_2.config_ref[ct_source];

								debug_tx3("pi:%d, %d, %d, %f\n", ct_source, oldref[ct_source], user_pid_2.config_ref[ct_source], user_pid_2.currValue[ct_source]);
            }
            // 曝光计数
            config_data.expo_count[ct_source]++;
            config_data.expo_count_total[ct_source]++;
            parm_table[ct_source].expo_count_total++;
            parm_table[ct_source].expo_times_total += config_data.expo_count_total[ct_source] / 3000000;

            // 若未在上面进入 EXPO_END / READY，则此处兜底
            if (get_hv_state(ct_source) == HVPS_SM_ID_EXPOSURING)
            {
                set_hv_state(HVPS_SM_ID_EXPO_END, ct_source);
            }
        }

        // 曝光完成时间记录
        last_expo_end_tick[ct_source] = HAL_GetTick();
        break;

    case HVPS_SM_ID_EXPO_END:

//        // 高压基准维持
//        config_hvref_slope(ct_source);

        // ----------- 灯丝保护计数 -----------
        for (uint8_t i = 0; i < XRAY_NUMS; i++)
        {
            if (get_filament_pin(i) && get_hv_state(i) != HVPS_SM_ID_EXPOSURING)
                config_data.fila_protect_cnt[i]++;
            else
                config_data.fila_protect_cnt[i] = 0;
        }

        // ----------- 检查是否为双源交替切换 -----------
        is_dual_source = (ctrl_data.enable[0] && ctrl_data.enable[1] &&
                          ctrl_data.filament_on[0] && ctrl_data.filament_on[1] &&
                          ctrl_data.interlock);

        if (is_dual_source)
        {
            uint32_t elapsed = HAL_GetTick() - last_expo_end_tick[ct_source];

            if (elapsed >= 12)
            {
                sw_count++;

                switch (sw_state)
                {
                case 0:  // 准备关闭当前采样开关
                    config_disable_sw(ct_source);
                    sw_state = 1;
                    sw_count = 1;
                    break;

                case 1:  // 延时后打开下一个采样开关
                    if (sw_count >= 20)
                    {
                        config_enable_sw(1 - ct_source);
                        sw_state = 2;
                        sw_count = 1;
                    }
                    break;

                case 2:  // 延时后切换射源
                    if (sw_count >= 20)
                    {
                        ctrl_data.xray_current = 2;
                        set_hv_state(HVPS_SM_ID_READY, 1 - ct_source);

                        // 曝光次数统计
                        parm_table[ct_source].expo_count_total++;
                        parm_table[ct_source].expo_times_total += config_data.expo_count_total[ct_source] / 3000000;
                        config_data.expo_count_total[ct_source] = 0;

                        // 重置状态
                        sw_state = 0;
                        sw_count = 0;
                        ct_source = 1 - ct_source;
                        xray_data.timmer_count[ct_source] = 1;
                    }
                    break;

                default:
                    sw_state = 0;
                    sw_count = 0;
                    break;
                }
            }
        }
        else if (!ctrl_data.enable[ct_source])
        {
            config_disable_sw(1);
            config_enable_sw(0);
            // 单源或失能，直接退出
            xray_CT_disable();
            ctrl_data.filament_on[0] = 0;
            ctrl_data.filament_on[1] = 0;
            set_hv_state(HVPS_SM_ID_IDLE, 0);
            set_hv_state(HVPS_SM_ID_IDLE, 1);
        }
        break;
    default:
        //set_hv_state(HVPS_SM_ID_IDLE, ct_source);
        break;
    }

    // LED指示
    xray_on_led((get_hv_state(0) == HVPS_SM_ID_EXPOSURING) || (get_hv_state(1) == HVPS_SM_ID_EXPOSURING));
}




//void ct_task(void)
//{
//    static uint8_t ct_source = 0;                       // 当前射源：0 或 1
//    static uint8_t sw_state = 0;                        // 采样开关切换状态机
//    static uint32_t last_expo_end_tick[XRAY_NUMS] = {0};
//    static uint32_t sw_count = 0;

//    // 当前状态
//    hvps_sm_state state = get_hv_state(ct_source);

//    // 计时器自增
//    if (xray_data.timmer_count[ct_source] >= 1)
//        xray_data.timmer_count[ct_source]++;
//    else
//        return;

//    switch (state)
//    {
//    case HVPS_SM_ID_IDLE:
//        preheat_filament(ct_source); // 优化后的灯丝预热
//        break;

//    case HVPS_SM_ID_PREPARE:
//        config_filamentRef(ct_source); // 拉到设定电压
//        config_ready_signal(1);        // 准备完成
//        set_hv_state(HVPS_SM_ID_READY, ct_source);
//        xray_data.timmer_count[ct_source] = 1;
//        break;

//    case HVPS_SM_ID_READY:
//        config_hvref_slope(ct_source);
//        if (ctrl_data.enable[ct_source] && ctrl_data.expo[ct_source])
//        {
//            // 曝光前确保间隔满足条件
//            uint8_t other = 1 - ct_source;
//            if ((HAL_GetTick() - last_expo_end_tick[other]) >= 10)
//            {
//                set_hv_state(HVPS_SM_ID_EXPOSURING, ct_source);
//                xray_data.timmer_count[ct_source] = 1;
//            }
//        }
//        break;

//    case HVPS_SM_ID_EXPOSURING:
//        // 曝光检查使能
//        xray_data.isCheckAvailable[ct_source] = (xray_data.timmer_count[ct_source] > 185) ? 1 : 0;

//        config_hvref_slope(ct_source);
//        config_mcuLock_signal(1);
//        config_HVEn_signal(1);
//        config_xrayOn_signal(1);

//        if (xray_data.timmer_count[ct_source] == TIMER6_8_MILSECOND_CYCLES)
//            user_pid.currValue = sampled_data.tube_curr_value;

//        if (xray_data.timmer_count[ct_source] > TIMER6_8_MILSECOND_CYCLES &&
//            xray_data.timmer_count[ct_source] % TIMER6_10_MILSECOND_CYCLES == 0)
//        {
//            user_pid.currValue = sampled_data.tube_curr_value;
//            user_pid.Kp = kp_test;
//            user_pid.Ti = ki_test;
//            //tube_current_piControl(1, ct_source);
//        }

//        // 曝光结束判断
//        if (!ctrl_data.enable[ct_source] || !ctrl_data.expo[ct_source])
//        {
//            xray_data.isCheckAvailable[ct_source] = 0;
//            config_xrayOn_signal(0);
//            config_mcuLock_signal(0);
//            config_HVEn_signal(0);

//            // 统计数据
//            config_data.expo_count[ct_source]++;
//            config_data.expo_count_total[ct_source]++;

//            if (!ctrl_data.expo[ct_source]) {
//                if (ctrl_data.enable[1 - ct_source]) {
//                    set_hv_state(HVPS_SM_ID_EXPO_END, ct_source); // 双源脉冲
//                } else {
//                    set_hv_state(HVPS_SM_ID_READY, ct_source);   // 单源脉冲
//                }
//            } else {
//                set_hv_state(HVPS_SM_ID_EXPO_END, ct_source);     // 连续模式，仍需确认
//            }

//            last_expo_end_tick[ct_source] = HAL_GetTick();
//        }
//        break;

//    case HVPS_SM_ID_EXPO_END:
//        config_hvref_slope(ct_source);

//        // 灯丝保护
//        for (uint8_t i = 0; i < XRAY_NUMS; i++) {
//            if (get_filament_pin(i) && get_hv_state(i) != HVPS_SM_ID_EXPOSURING)
//                config_data.fila_protect_cnt[i]++;
//            else
//                config_data.fila_protect_cnt[i] = 0;
//        }

//        // 双源模式判断
//        if (ctrl_data.enable[0] && ctrl_data.enable[1] &&
//            ctrl_data.filament_on[0] && ctrl_data.filament_on[1] &&
//            ctrl_data.interlock)
//        {
//            uint32_t delta = HAL_GetTick() - last_expo_end_tick[ct_source];

//            if (delta >= 8)
//            {
//                sw_count++;

//                switch (sw_state)
//                {
//                case 0:
//                    config_disable_sw(ct_source);
//                    sw_state = 1;
//                    sw_count = 1;
//                    break;
//                case 1:
//                    if (sw_count >= 50) {
//                        config_enable_sw(1 - ct_source);
//                        sw_state = 2;
//                        sw_count = 1;
//                    }
//                    break;
//                case 2:
//                    if (sw_count >= 50) {
//                        ctrl_data.xray_current = 2;
//                        set_hv_state(HVPS_SM_ID_READY, 1 - ct_source);

//                        // 清除当前统计
//                        parm_table[ct_source].expo_count_total++;
//                        parm_table[ct_source].expo_times_total += config_data.expo_count_total[ct_source] / 3000000;
//                        config_data.expo_count_total[ct_source] = 0;

//                        ct_source = 1 - ct_source;
//                        sw_state = 0;
//                        sw_count = 0;
//                        xray_data.timmer_count[ct_source] = 1;
//                    }
//                    break;
//                }
//            }
//        }
//        else
//        {
//            // 关闭所有
//            xray_CT_disable();
//            set_hv_state(HVPS_SM_ID_IDLE, 0);
//            set_hv_state(HVPS_SM_ID_IDLE, 1);
//        }
//        break;

//    default:
//        break;
//    }

//    // LED指示
//    xray_on_led((get_hv_state(0) == HVPS_SM_ID_EXPOSURING) || (get_hv_state(1) == HVPS_SM_ID_EXPOSURING));
//}



