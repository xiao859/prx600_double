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
#include "protect.h"
#include "calibrate.h"
#include "app_fun.h"

uint32_t exp_count[2] = {0};

uint8_t B_pulse_KP = 30;
uint8_t B_pulse_KI = 30;

hvps_sm_state volatile hv_state[XRAY_NUMS];

Exposure_Parameters exp_para[XRAY_NUMS] = {0};

volatile xray_config_data config_data = {
    0,        //准备就绪
    0,        //射源出信号
    0,        //故障

    /* output to HV*/
    0,        //高压使能
    0,        //高压互锁
    0,        //故障复位
		{60,60},  //串口配置的管电压
    {2,2},    //串口配置的管电流
    {0,0}, 		//管电流查表索引

    {0,0}, 		//管电压基准实时配置值
    {3,3},    //管电压基准上升步长

    {0,0},    //灯丝基准目标值
    {0,0},  	//灯丝基准实时配置
    {0,0},    //灯丝基准上升步长

    /* output to filament*/
    {0,0},    //灯丝使能

    /* other data */
    {0,0},   //射源单次曝光计时
    {0,0},   //射源总曝光计时
    {0,0},   //曝光时间设置
    {0,0},   //灯丝开启未曝光计数

};


xray_type xray_tube_table[XRAY_TUBE_TYPES] =
{
    // ---------- KL181球管 ----------
    {
        .pluse_kp = 40,
        .pluse_ki = 40,
        .currRef1 = {1720, 1735, 1770, 1810, 1880, 1920, 1950, 1980, 2000, 2020, 2040, 2060},
        .currRef2 = {1031, 1051, 1111, 1171, 1221, 1241, 1261, 1281, 1301, 1311, 1331, 1356},
        .fila_ref_offset =
        {
            {10, 12, 14, 15, 16, 18, 20, 21, 22, 23, 24, 25},  // 射源0 偏置
            {5,  7,  8,  9, 10, 11, 12, 13, 14, 15, 16, 17},   // 射源1 偏置
        }
    },
		
    // ---------- KL3球管 ----------
    {
			0,
    },
		
    // ---------- 万森球管 ----------
    {
        .pluse_kp = 30,
        .pluse_ki = 25,
				.currRef1 = {1720, 1735, 1770, 1810, 1880, 1920, 1950, 1980, 2000, 2020, 2040, 2060},
        .currRef2 = {1030, 1050, 1110, 1170, 1220, 1240, 1260, 1280, 1300, 1310, 1330, 1355},
        .fila_ref_offset =
        {
            {2,  5,  5,  8,  10,  15, 15, 15, 16, 17,  18,  18},  // 射源0
            {2,  3,  3,  3,   3,   4,  5,  5,  6,  6,   8,  9},  // 射源1
        }
    },

    // ---------- KL29球管 ----------
    {
			0,
    },
};


volatile xray_parament_table parm_table[XRAY_NUMS] =
{
    {
        2,0, 0, 1,
        {1,    2,    3,    4,    5,    6,    7,    8,    9,    10,  11,   12},
        {1720, 1875, 1920, 2050, 2100, 2110, 2120, 2130, 2195, 2215, 2240, 2260},
        {1720, 1735, 1770, 1810, 1880, 1920, 1950, 1980, 2000, 2020, 2040, 2060},
    },
    {
        2,0, 0, 1,
        {1,    2,    3,    4,    5,    6,    7,    8,    9,    10,  11,   12},
        {1030, 1050, 1110, 1170, 1220, 1260, 1320, 1360, 1400, 1410, 1430, 1460},
        {1030, 1050, 1110, 1170, 1220, 1240, 1260, 1280, 1300, 1310, 1330, 1355},
    },
};
volatile xray_parament_range para_range =
{
    120, 50,            /*管电压保护值120 50*/
    140, 5, 750,        /*管电流保护值130 5 */

    70, -25, 60, 50,    /*油温*/

    28, 20,             /* 24V供电*/
    100, 100,           /*散热器温度*/
    30, 8,              /*灯丝电压*/
    100, 100,           /*灯丝电流*/

    120, 60,            /*管电压配置门限*/
    120, 10,            /*管电流配置门限*/

    1000, 1000,         /*曝光时间保护*/

    1400000,            /*曝光时间1.5min 1800000 4500000*/

    2400000             /*灯丝开启未曝光最大时间 6000000*/
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

    config_data.fila_ref_realtime[0] = 0;
    config_data.fila_ref_realtime[1] = 0;
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

    config_data.fila_ref_realtime[0] = 0;
    config_data.fila_ref_realtime[1] = 0;
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
    wirte_flash_parament((uint8_t *)&parm_table, sizeof(xray_parament_table) * 2);

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


static uint32_t calc_crc32(const uint8_t *data, uint32_t length)
{
    uint32_t crc = 0xFFFFFFFF;
    for (uint32_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 1) crc = (crc >> 1) ^ 0xEDB88320;
            else crc >>= 1;
        }
    }
    return crc ^ 0xFFFFFFFF;
}


bool load_from_flash(volatile xray_parament_table *parm_table)
{

    xray_parament_flash_t flash_data;

    // 1. 读取主区
    bsp_read_buffer((uint8_t*)&flash_data, FLASH_PRIMARY_ADDR, sizeof(flash_data));

    // 2. 校验 CRC
    uint32_t CRC1 = calc_crc32((const uint8_t*)flash_data.data, sizeof(flash_data.data));
    if (CRC1 == flash_data.crc32)
    {
        memcpy((void*)parm_table, flash_data.data, sizeof(flash_data.data));
				if(parm_table[0].xray_type < 4)
				{
					B_pulse_KP = xray_tube_table[parm_table[0].xray_type].pluse_kp;
					B_pulse_KI = xray_tube_table[parm_table[0].xray_type].pluse_ki;
					memcpy((void*)fila_ref_offset,xray_tube_table[parm_table[0].xray_type].fila_ref_offset,24);
					if(parm_table[0].xray_type == 0)
					{
						version.tube_ver_high = 0x11;
						version.tube_ver_low = 0x13;
					}
					else if(parm_table[0].xray_type == 1)
					{
						version.tube_ver_high = 0x12;
						version.tube_ver_low = 0x11;
					}
					else if(parm_table[0].xray_type == 2)
					{
						version.tube_ver_high = 0x11;
						version.tube_ver_low = 0x11;
					}
					else if(parm_table[0].xray_type == 3)
					{
						version.tube_ver_high = 0x11;
						version.tube_ver_low = 0x12;
					}
				}

        parm_table[0].rising_time = 1;
        parm_table[1].rising_time = 1;
        return true;
    }
    else
    {
        // 尝试读取备用区
        bsp_read_buffer((uint8_t*)&flash_data, FLASH_BACKUP_ADDR,  sizeof(flash_data));

        if (calc_crc32((const uint8_t *)flash_data.data, sizeof(flash_data.data)) == flash_data.crc32)
        {
            memcpy((void*)parm_table, flash_data.data, sizeof(flash_data.data));
						if(parm_table[0].xray_type == 0)
						{
							B_pulse_KP = xray_tube_table[0].pluse_kp;
							B_pulse_KI = xray_tube_table[0].pluse_ki;
							memcpy((void*)fila_ref_offset,xray_tube_table[0].fila_ref_offset,24);
						}
						else if(parm_table[0].xray_type == 1)
						{
							B_pulse_KP = xray_tube_table[1].pluse_kp;
							B_pulse_KI = xray_tube_table[1].pluse_ki;
							memcpy((void*)fila_ref_offset,xray_tube_table[0].fila_ref_offset,24);
						}
					  parm_table[0].rising_time = 1;
						parm_table[1].rising_time = 1;
            return true;
        }
    }

    return false; // 两个区都损坏
}
bool save_to_flash(volatile xray_parament_table *parm_table)
{

    xray_parament_flash_t flash_data;

    // 1. 拷贝数据到临时结构（非 volatile）
    memcpy(flash_data.data, (const void*)parm_table, sizeof(flash_data.data));

    // 2. 计算 CRC
    flash_data.crc32 = calc_crc32((const uint8_t*)flash_data.data, sizeof(flash_data.data));
    flash_data.crc32 = calc_crc32((const uint8_t*)flash_data.data, sizeof(flash_data.data));

    // 3. 擦除备用区扇区
    bsp_erase_sector(FLASH_BACKUP_ADDR);

    // 4. 写入备用区
    bsp_write_buffer((uint8_t*)&flash_data, FLASH_BACKUP_ADDR,  sizeof(flash_data));


    // 5. 读取回读并校验
    xray_parament_flash_t read_back;
    bsp_read_buffer((uint8_t*)&read_back, FLASH_BACKUP_ADDR, sizeof(read_back));

    if (read_back.crc32 != flash_data.crc32) return false; // 写入失败

    // 6. 擦除主区并写入
    bsp_erase_sector(FLASH_PRIMARY_ADDR);
    bsp_write_buffer((uint8_t*)&flash_data, FLASH_PRIMARY_ADDR,  sizeof(flash_data));

    return true;
}


void flash_table_init()
{
    //  load_from_flash(parm_table);

    xray_parament_table parm_table_temp[XRAY_NUMS];

    get_flash_parament((uint8_t *)&parm_table_temp, sizeof(xray_parament_table) * 2);

    if (parm_table_temp[0].rising_time > 5)
        return;

//    uint32_t currRef[FILAMENT_CURRENT_TABLE_ORDER] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};

    int i;

    for (i = 0; i < FILAMENT_CURRENT_TABLE_ORDER; i++)
    {
        parm_table[0].currRef[i]   = MIN(MAX(parm_table[0].currRef[i], 1000), 3000);
        parm_table[0].currRef_c[i] = MIN(MAX(parm_table[0].currRef_c[i], 1000), 3000);
    }
    config_data.expo_count_total[0] = 0;     //

    for (i = 0; i < FILAMENT_CURRENT_TABLE_ORDER; i++)
    {
        parm_table[1].currRef[i]   = MIN(MAX(parm_table[1].currRef[i], 1000), 1900);
        parm_table[1].currRef_c[i] = MIN(MAX(parm_table[1].currRef_c[i], 1000), 1900);
    }
    config_data.expo_count_total[1] = 0;     //
}

void hvState_ilde_init(uint16_t n)
{
    xray_data.isCheckAvailable = 0;
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

    if (tube_current >= parm_table[n].currValue[FILAMENT_CURRENT_TABLE_ORDER - 1])
    {

        return parm_table[n].currRef_c[config_data.tube_curr_index[n]];

    }

    uint32_t currRef_uplimit;
    uint32_t currRef_downlimit;

    currRef_uplimit   = parm_table[n].currRef_c[config_data.tube_curr_index[n] + 1];
    currRef_downlimit = parm_table[n].currRef_c[config_data.tube_curr_index[n]];

    debug_tx3("ref:%d,%d,%d,%f,%d\n", n, currRef_uplimit, currRef_downlimit, tube_current, index);

    return (currRef_downlimit + (tube_current - parm_table[n].currValue[index]) * (currRef_uplimit - currRef_downlimit));

}

void config_filamentRef(uint16_t n)
{
    config_data.fila_ref_realtime[n] = get_filamentRef(config_data.tube_curr[n], n);
    if (n == 0)
    {

        HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, config_data.fila_ref_realtime[n]);
    }
    else
    {
        //pwm
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, config_data.fila_ref_realtime[n]);//1360

    }
    debug_tx3("ref1:%d,%f\n", n, config_data.fila_ref_realtime[n]);
    return;
}
void disable_hvref()
{
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


// =================== 互斥采样开关控制 ===================
static uint8_t sw_status[2] = {0, 0}; // 软件记录开关状态

void config_enable_sw_safe(uint8_t sw)
{
    if (sw > 1) return;

    // 打开前，强制关闭另一只
    if (sw == 0)
        config_disable_sw(1);
    else
        config_disable_sw(0);

    config_enable_sw(sw);
    sw_status[sw] = 1;
    sw_status[1 - sw] = 0; // 确保互斥
}

void config_disable_sw_safe(uint8_t sw)
{
    if (sw > 1) return;
    config_disable_sw(sw);
    sw_status[sw] = 0;
}


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
    static uint32_t last_expo_count = 0;
    bool is_xrayB_mode = false;

    //------------ 检查是否为单独B源模式 ---------------//B源在最开始就需要切换采样开关
    if ((ctrl_data.xray_current == 2) &&
            (ctrl_data.filament_on[0] == 0) && (ctrl_data.filament_on[1] == 1) &&
            (ctrl_data.interlock == 1))
        is_xrayB_mode = 1;



    ct_source = ctrl_data.xray_current - 1;

    hvps_sm_state ct_source_state = get_hv_state(ct_source);

    // Tick自增
    if (xray_data.timmer_count[ct_source] >= 1)
        xray_data.timmer_count[ct_source]++;
    else
        return;
    last_expo_count++;

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

        // 仅当满足时间、interlock后才允许进入下一状态
        if (ctrl_data.interlock && ctrl_data.filament_on[ct_source] &&
                xray_data.timmer_count[ct_source] > TIMER6_1P5_SECOND_CYCLES)//1.5s
        {
            config_mcuLock_signal(1);
            set_hv_state(HVPS_SM_ID_PREPARE, ct_source);
            xray_data.timmer_count[ct_source] = 1;
            config_data.expo_count[ct_source] = 0;
        }
        break;

    case HVPS_SM_ID_PREPARE:
        if (ctrl_data.xrayMode == XRAY_MODE_D_PULSE)
        {
            config_filamentRef(ct_source); /*灯丝基准值拉到预期*/
            config_filamentRef(1 - ct_source); /*灯丝基准值拉到预期*/
        }
        else
            config_filamentRef(ct_source); /*灯丝基准值拉到预期*/
				
        if (xray_data.timmer_count[ct_source] > TIMER6_1_SECOND_CYCLES)//1s
        {
					config_ready_signal(1);
					set_hv_state(HVPS_SM_ID_READY, ct_source);
					xray_data.timmer_count[ct_source] = 1;
					pid_Init(config_data.tube_curr[ct_source], config_data.fila_ref_realtime[ct_source], Is_PulseMode_CT());
					param_pid.pulse_count = 0;
					pid_Init_2(config_data.tube_curr[0], config_data.fila_ref_realtime[0], Is_PulseMode_CT(), 0);
					pid_Init_2(config_data.tube_curr[1], config_data.fila_ref_realtime[1], Is_PulseMode_CT(), 1);
				}
        break;

    case HVPS_SM_ID_READY:
        //ctrl_data.xray_current = ct_source + 1;

        config_hvref_slope(ct_source);
        if (ctrl_data.enable[ct_source] && ctrl_data.expo[ct_source])
        {
            // 曝光允许前需检查对方曝光是否间隔超过10ms
            //uint8_t other = (ct_source == 0) ? 1 : 0;
            // if (HAL_GetTick() - last_expo_end_tick[other] < 10)
            //    break;  // 距离对方曝光过短，等待
            set_hv_state(HVPS_SM_ID_EXPOSURING, ct_source);
            xray_data.timmer_count[ct_source] = 1;
        }

        if (!ctrl_data.enable[ct_source])
        {
            config_xrayOn_signal(0);
            set_hv_state(HVPS_SM_ID_EXPO_END, ct_source);
        }
        break;

    case HVPS_SM_ID_EXPOSURING:

        // 曝光后 3ms 开始允许采样检查
        xray_data.isCheckAvailable = (xray_data.timmer_count[ct_source] > 70) ? 1 : 0;

        // 持续输出高压与准备信号
        config_hvref_slope(ct_source);
        config_mcuLock_signal(1);
        config_HVEn_signal(1);
        config_xrayOn_signal(1);

        // 曝光计数
        config_data.expo_count[ct_source]++;

        // 曝光控制：延时 PI 初始化
        user_pid_2.currValue[ct_source] = 0.00645f * ((float)(adc_buffer3[2]));//0.00645*1.01(校准系数)
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
                //debug_tx3("pi:%d,%f\n", param_pid.config_ref, user_pid.currValue);
            }
            else
            {
                param_pid.ki_flag = 0;
            }

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
                param_pid.pulse_count++;
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
            xray_data.isCheckAvailable = 0;

            config_mcuLock_signal(0);
            config_HVEn_signal(0);
            // 曝光完成时间记录
            last_expo_end_tick[ct_source] = last_expo_count;
            config_xrayOn_signal(0);

            if (ctrl_data.enable[ct_source] == 1)
            {
                user_pid_2.Kp[1] = B_pulse_KP;
                user_pid_2.Ki[1] = B_pulse_KI;
                user_pid_2.Kp[0] = 40;
                user_pid_2.Ki[0] = 40;
                oldref[ct_source] = user_pid_2.config_ref[ct_source];
                tube_current_piControl_v2(ct_source);
                param_pid.config_ref = user_pid_2.config_ref[ct_source];

                // debug_tx3("pi:%d,%d,%f\n", ct_source, oldref[ct_source], user_pid_2.currValue[ct_source]);
            }

            parm_table[ct_source].expo_count_total++;

            // 若未在上面进入 EXPO_END / READY，则此处兜底
            if (get_hv_state(ct_source) == HVPS_SM_ID_EXPOSURING)
            {
                set_hv_state(HVPS_SM_ID_EXPO_END, ct_source);
            }
        }

        break;

    case HVPS_SM_ID_EXPO_END:

        // ----------- 灯丝保护计数 -----------
        for (uint8_t i = 0; i < XRAY_NUMS; i++)
        {
            if (get_filament_pin(i) && get_hv_state(i) != HVPS_SM_ID_EXPOSURING)
                config_data.fila_protect_cnt[i]++;
            else
                config_data.fila_protect_cnt[i] = 0;
        }

        // ----------- 检查是否为双源交替切换 -----------
        is_dual_source = (ctrl_data.filament_on[0] && ctrl_data.filament_on[1] &&
                          ctrl_data.interlock);



        if ((is_dual_source) && (ctrl_data.enable[ct_source]))
        {
            uint32_t elapsed = last_expo_count - last_expo_end_tick[ct_source];

            if (elapsed >= 200)
            {
                sw_count++;

                switch (sw_state)
                {
                case 0:  // 准备关闭当前采样开关
                    config_disable_sw_safe(ct_source);// config_disable_sw(ct_source);
                    sw_state = 1;
                    sw_count = 1;
                    break;

                case 1:  // 延时后打开下一个采样开关
                    if (sw_count >= 20)
                    {
                        config_enable_sw_safe(1 - ct_source);// config_enable_sw(1 - ct_source);
                        sw_state = 2;
                        sw_count = 1;
                    }
                    break;

                case 2:  // 延时后切换射源
                    if (sw_count >= 20)
                    {
                        set_hv_state(HVPS_SM_ID_READY, 1 - ct_source);

                        // 曝光次数统计
                        parm_table[ct_source].expo_count_total++;

                        // 重置状态
                        sw_state = 0;
                        sw_count = 0;
                        ct_source = 1 - ct_source;
                        ctrl_data.xray_current = ct_source + 1;
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
            if (is_xrayB_mode == 1)
            {
                uint32_t elapsed = last_expo_count - last_expo_end_tick[ct_source];

                if (elapsed >= 200)
                {
                    sw_count++;

                    switch (sw_state)
                    {
                    case 0:  // 准备关闭当前采样开关
                        config_disable_sw_safe(1);//config_disable_sw(1);
                        sw_state = 1;
                        sw_count = 1;
                        break;

                    case 1:  // 延时后打开下一个采样开关,回A
                        if (sw_count >= 20)
                        {
                            config_enable_sw_safe(0);//config_enable_sw(0);
                            sw_state = 2;
                            sw_count = 1;
                        }
                        break;
                    default:
                        sw_state = 0;
                        sw_count = 0;
                        break;
                    }
                }
            }
            if (is_dual_source)
            {
                parm_table[0].expo_count_total++;
                parm_table[1].expo_count_total++;
                exp_count[0] += config_data.expo_count_total[0];
                exp_count[1] += config_data.expo_count_total[1];
                parm_table[0].expo_times_total +=  exp_count[0] / 1200000;
                exp_count[0] = exp_count[1] % 1200000;
                parm_table[1].expo_times_total += exp_count[1] / 1200000;
                exp_count[1] = exp_count[1] % 1200000;

            }
            else
            {
                parm_table[ct_source].expo_count_total++;
                exp_count[ct_source] += config_data.expo_count_total[ct_source];
                parm_table[ct_source].expo_times_total += exp_count[ct_source] / 1200000;
                exp_count[ct_source] = exp_count[ct_source] % 1200000;
            }
            config_disable_sw_safe(1);//config_disable_sw(1);
            config_enable_sw_safe(0);//config_enable_sw(0);
            // 单源或失能，直接退出
            xray_CT_disable();
            ctrl_data.filament_on[0] = 0;
            ctrl_data.filament_on[1] = 0;
            set_hv_state(HVPS_SM_ID_IDLE, 0);
            set_hv_state(HVPS_SM_ID_IDLE, 1);
            config_data.expo_count_total[0] = 0;
            config_data.expo_count_total[1] = 0;
            last_expo_count = 0;
            ct_source = 0;
            ctrl_data.xray_current = 1;
            cali_data.para_save_flag =1;
        }
        break;
    default:
        set_hv_state(HVPS_SM_ID_IDLE, ct_source);
        break;
    }
    // --------- 安全检查：禁止SW1和SW2同时开启 ----------
    if (sw_status[0] && sw_status[1])
    {
        // 出现异常，紧急关闭所有采样开关
        config_disable_sw_safe(0);
        config_disable_sw_safe(1);

        // 记录故障标志，便于调试
        mHVPS_Fault.FAULT_REG4.bit.current_broken1 = 1;
    }

    // LED指示
    xray_on_led((get_hv_state(0) == HVPS_SM_ID_EXPOSURING) || (get_hv_state(1) == HVPS_SM_ID_EXPOSURING));
}


