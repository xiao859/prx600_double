#include "comm_string.h"
#include "string.h"
#include "math.h"
#include "stdlib.h"
#include "app_uart.h"
#include "stm32g4xx_hal.h"
#include "app_spi.h"
#include "debug_mode.h"
#include "xray.h"
#include "tim.h"
#include "app_fun.h"
#include "ct_exposure.h"
#include "calibrate.h"



LC_Command_string cmd_pack[LC_CMD_NUM];

void unpackCmd_savePara(volatile uint8_t *buff, uint8_t *cmdHead, volatile uint32_t *targetAddr, uint8_t offset)
{
    if (strstr((const char *)buff, (const char *)cmdHead) == NULL)
    {
        debug_tx3("格式错误!!!\r\n");
        return;
    }

    uint32_t table_temp[FILAMENT_CURRENT_TABLE_ORDER];

    char temp_buf[128];  // 临时缓冲区，根据串口数据长度调整
    strncpy(temp_buf, (char *)(buff + offset), sizeof(temp_buf) - 1);
    temp_buf[sizeof(temp_buf) - 1] = '\0';  // 保证结尾有 \0
    uint8_t para_count = 0;

    char *token = strtok(temp_buf, " ");

    while (token != NULL && para_count < FILAMENT_CURRENT_TABLE_ORDER)
    {
        int num = atoi(token);  // 将字符串转为整数
        table_temp[para_count++] = (uint32_t)num;
        token = strtok(NULL, " ");
    }

    if (para_count < FILAMENT_CURRENT_TABLE_ORDER)
    {
        debug_tx3("参数不够: %d!!!\n", para_count);
    }
    else
    {
        // save_parament_to_flash();

        /* 读 */
        get_flash_parament((uint8_t *)&parm_table, sizeof(xray_parament_table));


        memcpy((uint8_t *)targetAddr, (uint8_t *)&table_temp[0], sizeof(table_temp));

        /*写 */
        save_parament_to_flash();
    }

    return;
}

int func_setCurrvalue1(volatile uint8_t *buff, char *p)
{
    // uint8_t cmdHead[] = "set currValue1 ";

    //unpackCmd_savePara(buff, cmdHead, &parm_table[0].currValue, 14);

    return 0;
}

int func_setCurrvalue2(volatile uint8_t *buff, char *p)
{
//    uint8_t cmdHead[] = "set currValue2 ";

    // unpackCmd_savePara(buff, cmdHead, &parm_table[1].currValue, 14);

    return 0;
}


int func_setCurrRef1(volatile uint8_t *buff, char *p)
{
    uint8_t cmdHead[] = "set currRef1 ";

    unpackCmd_savePara(buff, cmdHead, &parm_table[0].currRef[0], 13);

    return 0;
}

int func_setCurrRef2(volatile uint8_t *buff, char *p)
{
    uint8_t cmdHead[] = "set currRef2 ";

    unpackCmd_savePara(buff, cmdHead, &parm_table[1].currRef[0], 13);

    return 0;
}

int func_setCurrvalue_c1(volatile uint8_t *buff, char *p)
{
//    uint8_t cmdHead[] = "set currValue_c1 ";

//    unpackCmd_savePara(buff, cmdHead, &parm_table[0].currValue[0], 17);

    return 0;
}

int func_setCurrvalue_c2(volatile uint8_t *buff, char *p)
{
//    uint8_t cmdHead[] = "set currValue_c2 ";

//    unpackCmd_savePara(buff, cmdHead, &parm_table[1].currValue[0], 17);

    return 0;
}

int func_setCurrRef_c1(volatile uint8_t *buff, char *p)
{
    uint8_t cmdHead[] = "set currRef_c1 ";

    unpackCmd_savePara(buff, cmdHead, &parm_table[0].currRef_c[0], 15);

    return 0;
}

int func_setCurrRef_c2(volatile uint8_t *buff, char *p)
{
    uint8_t cmdHead[] = "set currRef_c2 ";

    unpackCmd_savePara(buff, cmdHead, &parm_table[1].currRef_c[0], 15);

    return 0;
}

int func_getTable1(volatile uint8_t *buff, char *p)
{
    debug_tx3("A曝光次数: %d\n", parm_table[0].expo_count_total);
    debug_tx3("A曝光时间: %d\n", parm_table[0].expo_times_total);

    for (int i = 0; i < FILAMENT_CURRENT_TABLE_ORDER; i++)
    {
        debug_tx3("AcurrValue:%d\n", parm_table[0].currValue[i]);
    }

    debug_tx3("A currRef:");
    for (int i = 0; i < FILAMENT_CURRENT_TABLE_ORDER; i++)
    {
        debug_tx3(" %d", parm_table[0].currRef[i]);
    }
    debug_tx3("\n");

    debug_tx3("A currRef_c:");
    for (int i = 0; i < FILAMENT_CURRENT_TABLE_ORDER; i++)
    {
        debug_tx3(" %d", parm_table[0].currRef_c[i]);
    }
    debug_tx3("\n");

    return 0;
}

int func_getTable2(volatile uint8_t *buff, char *p)
{
    debug_tx3("B曝光次数: %d\n", parm_table[1].expo_count_total);
    debug_tx3("B曝光时间: %d\n", parm_table[1].expo_times_total);

    for (int i = 0; i < FILAMENT_CURRENT_TABLE_ORDER; i++)
    {
        debug_tx3("B currValue: %d\n", parm_table[0].currValue[i]);
    }

    debug_tx3("B currRef:");
    for (int i = 0; i < FILAMENT_CURRENT_TABLE_ORDER; i++)
    {
        debug_tx3(" %d", parm_table[0].currRef[i]);
    }
    debug_tx3("\n");

    debug_tx3("B currRef_c:");
    for (int i = 0; i < FILAMENT_CURRENT_TABLE_ORDER; i++)
    {
        debug_tx3(" %d", parm_table[0].currRef_c[i]);
    }
    debug_tx3("\n");
    return 0;
}


/* 设置管电压set tubeVC 60 20*/
int set_tube_vol_curr1(volatile uint8_t *buff, char *p)
{
    volatile uint8_t *p_p = buff + 11;
    uint8_t para_count = 0;
    int data[2];

    char *token = strtok((char *)p_p, " ");
    while (token != NULL && para_count <= 2)
    {
        int num = atoi(token);
        data[para_count] = num;
        para_count++;
        token = strtok(NULL, " ");
    }

    if (para_count == 2)
    {
        config_data.tube_curr[0] = ((float)data[1]) / 10;
        config_data.tube_vol[0]  = data[0];
        config_data.tube_vol_step[0] = (float)(config_data.tube_vol[0]) / (50 * 1);
        debug_tx3("A tube_vol:%d,tube_curr:%d, tube_vol_step:%f\n", config_data.tube_vol[0], config_data.tube_curr[0], config_data.tube_vol_step[0]);
    }
    else
    {
        debug_tx3("A参数格式错误\n");
    }

    return 0;
}

/* 设置管电压*/
int set_tube_vol_curr2(volatile uint8_t *buff, char *p)
{
    volatile uint8_t *p_p = buff + 11;
    uint8_t para_count = 0;
    int data[2];

    char *token = strtok((char *)p_p, " ");
    while (token != NULL && para_count <= 2)
    {
        int num = atoi(token);
        data[para_count] = num;
        para_count++;
        token = strtok(NULL, " ");
    }


    if (para_count == 2)
    {
        config_data.tube_curr[1] = ((float)data[1]) / 10;
        config_data.tube_vol[1]  = data[0];
        config_data.tube_vol_step[1] = (float)(config_data.tube_vol[1]) / (50 * 1);
        debug_tx3("B tube_vol:%d,tube_curr:%d, tube_vol_step:%f\n", config_data.tube_vol[1], config_data.tube_curr[1], config_data.tube_vol_step[1]);
    }
    else
    {
        debug_tx3("B参数格式错误\n");
    }

    return 0;
}

/* 设置曝光时间，必须先设置曝光模式
 * 25ms周期中，8ms；脉冲次数：x秒*40
 * 50kHz=0.02ms 曝光周期400，冷却周期850
 */
int set_expo_time1(volatile uint8_t *buff, char *p)
{
    volatile uint8_t *p_p = buff + 13;
    int num = 0;

    char *token = strtok((char *)p_p, " ");
    while (token != NULL)
    {
        num = atoi(token);
        token = strtok(NULL, " ");
    }

    if (num == 0)
    {
        debug_tx3("A参数格式错误");
        return 0;
    }

    if ((ctrl_data.xrayMode == XRAY_MODE_S_PULSE)||(ctrl_data.xrayMode == XRAY_MODE_D_PULSE))
    {
        debug_data.expoCycle_perCurrent[0] = num * 40;
        debug_data.expoTime_expect[0] = 400;
        debug_data.coolTime_expect[0] = 850;
    }
    else
    {
        debug_data.expoCycle_perCurrent[0] = 1;
        debug_data.expoTime_expect[0] = num * 50000;
    }

    debug_tx3("A pusre count:%d, exp time:%d, cooltime:%d\n",
              debug_data.expoCycle_perCurrent[0], debug_data.expoTime_expect[0], debug_data.coolTime_expect[0]);

    return 0;
}

/* 设置曝光时间，必须先设置曝光模式
 * 25ms周期中，8ms；脉冲次数：x秒*40
 * 50kHz=0.02ms 曝光周期400，冷却周期850
 */
int set_expo_time2(volatile uint8_t *buff, char *p)
{
    volatile uint8_t *p_p = buff + 13;
    int num = 0;

    char *token = strtok((char *)p_p, " ");
    while (token != NULL)
    {
        num = atoi(token);
        token = strtok(NULL, " ");
    }

    if (num == 0)
    {
        debug_tx3("B参数格式错误");
        return 0;
    }

    if ((ctrl_data.xrayMode == XRAY_MODE_S_PULSE)||(ctrl_data.xrayMode == XRAY_MODE_D_PULSE))
    {
        debug_data.expoCycle_perCurrent[1] = num * 40;
        debug_data.expoTime_expect[1] = 400;
        debug_data.coolTime_expect[1] = 850;
    }
    else
    {
        debug_data.expoCycle_perCurrent[1] = 1;
        debug_data.expoTime_expect[1] = num * 50000;
    }

    debug_tx3("B pusre count:%d, exp time:%d, cooltime:%d\n",
              debug_data.expoCycle_perCurrent[1], debug_data.expoTime_expect[1], debug_data.coolTime_expect[1]);

    return 0;
}

/* 设置曝光模式 */
int set_expo_mode(volatile uint8_t *buff, char *p)
{
    volatile uint8_t *p_p = buff + 13;
    int num = 0;

    config_reset_signal(1);

    char *token = strtok((char *)p_p, " ");
    while (token != NULL)
    {
        num = atoi(token);
        token = strtok(NULL, " ");
    }
    switch (num)
    {
    case HVPS_MODE_S_CONTINUOUS:
        ctrl_data.xrayMode = XRAY_MODE_S_CONTINUOUS;
        break;
    case HVPS_MODE_S_PULSE:
        ctrl_data.xrayMode = XRAY_MODE_S_PULSE;
        break;
    case HVPS_MODE_D_CONTINUOUS:
        ctrl_data.xrayMode = XRAY_MODE_D_CONTINUOUS;
        break;
    case HVPS_MODE_D_PULSE:
        ctrl_data.xrayMode = XRAY_MODE_D_PULSE;
        break;
    }
    return 0;
}


/* 设置enable信号 */
int set_enable(volatile uint8_t *buff, char *p)
{
    volatile uint8_t *p_p = buff + 11;
    int num = 0;

    char *token = strtok((char *)p_p, " ");
    while (token != NULL)
    {
        num = atoi(token);
        token = strtok(NULL, " ");
    }

    if (num == 1)
    {
        set_hv_state(HVPS_SM_ID_TRAIN_PREPARE, 0);
        set_hv_state(HVPS_SM_ID_IDLE, 1);
        ctrl_data.interlock = 1;
        ctrl_data.enable[0] = 1;
        ctrl_data.enable[1] = 0;
        debug_data.timmer_count = 1;
				config_enable_sw(0); // 选取射源0采样
				config_disable_sw(1); // 关闭射源1采样
        HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
        debug_tx3("ʹ使能开始ʼ\n");
    }
    else if (num == 2)
    {
        set_hv_state(HVPS_SM_ID_TRAIN_PREPARE, 1);
        set_hv_state(HVPS_SM_ID_IDLE, 0);
        ctrl_data.interlock = 1;
        ctrl_data.enable[1] = 1;
        ctrl_data.enable[0] = 0;
        debug_data.timmer_count = 1;
				config_enable_sw(1); // 选取射源1采样
				config_disable_sw(0); // 关闭射源0采样
//      PWM
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
        debug_tx3("ʹ使能结束ʼ\n");
    }
    else if (num == 3)
    {
        set_hv_state(HVPS_SM_ID_TRAIN_PREPARE, 0);
        set_hv_state(HVPS_SM_ID_TRAIN_PREPARE, 1);
        ctrl_data.interlock = 1;
        ctrl_data.enable[0] = 1;
        ctrl_data.enable[1] = 1;
        debug_data.timmer_count = 1;
				config_enable_sw(0); // 选取射源0采样
				config_disable_sw(1); // 关闭射源1采样
        HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
        //PWM
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
        debug_tx3("ʹ使能开始ʼ\n");

    }
    else
    {
        set_hv_state(HVPS_SM_ID_IDLE, 0);
        set_hv_state(HVPS_SM_ID_IDLE, 1);
        ctrl_data.interlock = 0;
        ctrl_data.enable[0] = 0;
        ctrl_data.enable[0] = 0;
        debug_tx3("ʹ使能结束\n");
    }
    return 0;
}


int set_ref_onoff(volatile uint8_t *buff, char *p)
{
    set_hv_state(HVPS_SM_ID_TRAIN_EXPOSURING, 0);
    // debug_data.timmer_count = 0;
//    xray_HV_enable_debug(1);
    ctrl_data.enable[0]    = 1;
    debug_data.timmer_count = 1;

    debug_tx3("开始给高压基准\n");

    return 0;
}


int set_ref_test(volatile uint8_t *buff, char *p)
{
    volatile uint8_t *p_p = buff + 8;
    uint8_t para_count = 0;
    int data[2];
    set_hv_state(HVPS_SM_ID_TRAIN_DEBUG, 0);

    char *token = strtok((char *)p_p, " ");
    while (token != NULL && para_count <= 2)
    {
        int num = atoi(token);
        data[para_count] = num;
        para_count++;
        token = strtok(NULL, " ");
    }

    if (para_count == 2)
    {
        config_data.tube_curr[0] = data[1];
        config_data.tube_vol[0]  = data[0];
        uint32_t tube_vol_ref = (uint32_t)(((float)config_data.tube_vol[0] / 64) / ADDA_FULL_SCALE_VIL_VALUE * 4095);  /* 0~2.5V 0~160kV */
        // uint32_t tube_curr_ref = (uint32_t)((config_data.tube_curr / 64) / ADDA_FULL_SCALE_VIL_VALUE * 4095);

        HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, tube_vol_ref);
        // HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, tube_vol_ref);
        debug_tx3("Atube_vol/tube_curr:%d, %d\n", tube_vol_ref, config_data.tube_curr[0]);
    }
    else
    {
        debug_tx3("A参数格式错误\n");
    }

    return 0;
}

/* 灯丝复位信号 */
int set_reset(volatile uint8_t *buff, char *p)
{
    config_reset_signal(1);

    debug_tx3("AB灯丝复位λ\n");

    return 0;
}


/* 灯丝开始 */
int set_filament_onoff(volatile uint8_t *buff, char *p)
{
    volatile uint8_t *p_p = buff + 16;
    int num = 0;

    char *token = strtok((char *)p_p, " ");
    while (token != NULL)
    {
        num = atoi(token);
        token = strtok(NULL, " ");
    }

    if (num == 0)
    {
        config_filamentOn_signal(1, 0);
        ctrl_data.filament_on[0] = 0;
        HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);
        debug_tx3("A lam on");
    }
    else if (num == 1)
    {
        config_filamentOn_signal(1, 1);
        ctrl_data.filament_on[1] = 0;
        //PWM
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, 0);
        debug_tx3("B lam on");
    }

    return 0;
}

/* A灯丝基准 */
int set_filament_ref_onoff1(volatile uint8_t *buff, char *p)
{
    volatile uint8_t *p_p = buff + 17;
    int num = 0;

    char *token = strtok((char *)p_p, " ");
    while (token != NULL)
    {
        num = atoi(token);
        token = strtok(NULL, " ");
    }

    ctrl_data.filament_on[0] = 1;
    set_hv_state(HVPS_SM_ID_TRAIN_IDLE, 0);
    debug_data.timmer_count = 0;

    // uint32_t a = (uint32_t)(num * 1.2409);  /* ((num / 1000) / 3.3) * 4095 */

    config_data.fila_ref_target[0] = (float)num / 1000;
    config_data.fila_ref_realtime[0] = IDLE_FILAMENT_REF_DEBUG;
    config_data.fila_ref_step[0] = (config_data.fila_ref_target[0] - IDLE_FILAMENT_REF_DEBUG) / (50 * 1);

    debug_tx3("A lam ref:%f,%f\n", config_data.fila_ref_target[0], config_data.fila_ref_step[0]);

    // HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, a);

    return 0;
}

/*B灯丝基准*/
int set_filament_ref_onoff2(volatile uint8_t *buff, char *p)
{
    volatile uint8_t *p_p = buff + 17;
    int num = 0;

    char *token = strtok((char *)p_p, " ");
    while (token != NULL)
    {
        num = atoi(token);
        token = strtok(NULL, " ");
    }

    ctrl_data.filament_on[1] = 1;
    set_hv_state(HVPS_SM_ID_TRAIN_IDLE, 1);
    debug_data.timmer_count = 0;

    // uint32_t a = (uint32_t)(num * 1.2409);  /* ((num / 1000) / 3.3) * 4095 */

    config_data.fila_ref_target[1] = (float)num / 1000;
    config_data.fila_ref_realtime[1] = IDLE_FILAMENT_REF_DEBUG;
    config_data.fila_ref_step[1] = (config_data.fila_ref_target[1] - IDLE_FILAMENT_REF_DEBUG) / (50 * 1);

    debug_tx3("B lam ref:%f,%f\n", config_data.fila_ref_target[1], config_data.fila_ref_step[1]);

    // HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, a);

    return 0;
}

/* 开始自动校准*/
int start_calibrate(volatile uint8_t *buff, char *p)
{
    volatile uint8_t *p_p = buff + 16;
    int num = 0;

    char *token = strtok((char *)p_p, " ");
    while (token != NULL)
    {
        num = atoi(token);
        token = strtok(NULL, " ");
    }

    if (num == 1)
    {
        calibrate_para_init();
        set_hv_state(HPVS_SM_ID_CAL_PREPARE, 0);
        set_hv_state(HPVS_SM_ID_CAL_PREPARE, 1);
        ctrl_data.enable[0] = 1;
        ctrl_data.enable[1] = 1;
        // pid_Init(1.0);
        debug_tx3("");
    }
    else
    {
        ctrl_data.enable[0] = 0;
        ctrl_data.enable[1] = 0;
        debug_tx3("");
    }

    return 0;
}

/* 开始自动校准*/
int set_hv_on(volatile uint8_t *buff, char *p)
{
    debug_tx3("CALIBRATE START");

    xray_HV_enable_debug(1);
    debug_data.timmer_count = 1;

    return 0;
}

/* 初始化flash中参数 */
int init_para_table(volatile uint8_t *buff, char *p)
{
    xray_parament_table parm_table_temp[2] =
    {
        {
            0,
            0,
            1,
            {1,    2,    3,    4,    5,    6,    7,    8,    9,    10,   11,   12},
            {1606, 1734, 1830, 1911, 1980, 2039, 2091, 2134, 2174, 2210, 2246, 2282},
            {1500, 1660, 1770, 1850, 1920, 1980, 2030, 2080, 2120, 2160, 2200, 2250},
        },
        {
            0,
            0,
            1,
            {1,    2,    3,    4,    5,    6,    7,    8,    9,    10,   11,   12},
            {1606, 1734, 1830, 1911, 1980, 2039, 2091, 2134, 2174, 2210, 2246, 2282},
            {1500, 1660, 1770, 1850, 1920, 1980, 2030, 2080, 2120, 2160, 2200, 2250},
        }
    };

    wirte_flash_parament((uint8_t *)&parm_table_temp, sizeof(xray_parament_table) * 2);

    get_flash_parament((uint8_t *)&parm_table, sizeof(xray_parament_table) * 2);

    debug_tx3("已刷新FLASH");

    return 0;
}

int ray_source(volatile uint8_t *buff, char *p)
{
    volatile uint8_t *p_p = buff + 16;
    int num = 0;

    char *token = strtok((char *)p_p, " ");
    while (token != NULL)
    {
        num = atoi(token);
        token = strtok(NULL, " ");
    }

    if (num == 1)
    {
        ctrl_data.xray_current = 1;
        // pid_Init(1.0);
        debug_tx3("A select");
    }
    else
    {
        ctrl_data.xray_current = 2;
        debug_tx3("B select");
    }

    return 0;
}

// lc oven sttemp 31.255
void cmd_parser_string()
{
    if (uart3.recv_complete != 1) return;

    volatile uint8_t *buff = uart3.uart_rx_buf;

    int i;

    char *ret;

    /* 判断指令是否在注册表中*/
    for (i = 0; i < LC_CMD_NUM; i++)
    {
        ret = strstr((const char *)buff, (const char *)cmd_pack[i].cmdstr);
        if (ret != NULL) break;
    }

    if (ret == NULL)
    {
        debug_tx3("没有该指令！");
        restart_usart_receive(USART3);
        return;
    }

    /* 2调用对应的函数*/
    cmd_pack[i].func_ptr(buff, ret); /* ret为指令初始位置*/

    restart_usart_receive(USART3);

    return;
}

/* 注册表*/
void registerFunc_init()
{
    memcpy(cmd_pack[0].cmdstr, "set currValue1", sizeof("set currValue1"));
    cmd_pack[0].func_ptr = &func_setCurrvalue1;

    memcpy(cmd_pack[1].cmdstr, "set currValue2", sizeof("set currValue2"));
    cmd_pack[1].func_ptr = &func_setCurrvalue2;

    memcpy(cmd_pack[2].cmdstr, "set currRef1", sizeof("set currRef1"));
    cmd_pack[2].func_ptr = &func_setCurrRef1;

    memcpy(cmd_pack[3].cmdstr, "set currRef2", sizeof("set currRef2"));
    cmd_pack[3].func_ptr = &func_setCurrRef2;

    memcpy(cmd_pack[4].cmdstr, "set currValue_c1", sizeof("set currValue_c1"));
    cmd_pack[4].func_ptr = &func_setCurrvalue_c1;

    memcpy(cmd_pack[5].cmdstr, "set currValue_c2", sizeof("set currValue_c2"));
    cmd_pack[5].func_ptr = &func_setCurrvalue_c2;

    memcpy(cmd_pack[6].cmdstr, "set currRef_c1", sizeof("set currRef_c1"));
    cmd_pack[6].func_ptr = &func_setCurrRef_c1;

    memcpy(cmd_pack[7].cmdstr, "set currRef_c2", sizeof("set currRef_c2"));
    cmd_pack[7].func_ptr = &func_setCurrRef_c2;

    memcpy(cmd_pack[8].cmdstr, "get currTable1", sizeof("get currTable1"));
    cmd_pack[8].func_ptr = &func_getTable1;

    memcpy(cmd_pack[9].cmdstr, "get currTable2", sizeof("get currTable2"));
    cmd_pack[9].func_ptr = &func_getTable2;

    memcpy(cmd_pack[10].cmdstr, "set tubeVC1", sizeof("set tubeVC1"));
    cmd_pack[10].func_ptr = &set_tube_vol_curr1;

    memcpy(cmd_pack[11].cmdstr, "set tubeVC2", sizeof("set tubeVC2"));
    cmd_pack[11].func_ptr = &set_tube_vol_curr2;

    memcpy(cmd_pack[12].cmdstr, "set expotime1", sizeof("set expotime1"));
    cmd_pack[12].func_ptr = &set_expo_time1;

    memcpy(cmd_pack[13].cmdstr, "set expotime2", sizeof("set expotime2"));
    cmd_pack[13].func_ptr = &set_expo_time2;

    memcpy(cmd_pack[14].cmdstr, "set expomode", sizeof("set expomode"));
    cmd_pack[14].func_ptr = &set_expo_mode;

    memcpy(cmd_pack[15].cmdstr, "set enable", sizeof("set enable"));
    cmd_pack[15].func_ptr = &set_enable;

    memcpy(cmd_pack[16].cmdstr, "tube reset", sizeof("tube reset"));
    cmd_pack[16].func_ptr = &set_reset;

    memcpy(cmd_pack[17].cmdstr, "start expo", sizeof("start expo"));
    cmd_pack[17].func_ptr = &set_ref_onoff;

    memcpy(cmd_pack[18].cmdstr, "set filament on", sizeof("set filament on"));
    cmd_pack[18].func_ptr = &set_filament_onoff;

    memcpy(cmd_pack[19].cmdstr, "set filament ref1", sizeof("set filament ref1"));
    cmd_pack[19].func_ptr = &set_filament_ref_onoff1;

    memcpy(cmd_pack[20].cmdstr, "set filament ref2", sizeof("set filament ref2"));
    cmd_pack[20].func_ptr = &set_filament_ref_onoff2;

    memcpy(cmd_pack[21].cmdstr, "start calibrate", sizeof("start calibrate"));
    cmd_pack[21].func_ptr = &start_calibrate;

    memcpy(cmd_pack[22].cmdstr, "set ref", sizeof("set ref"));
    cmd_pack[22].func_ptr = &set_ref_test;

    memcpy(cmd_pack[23].cmdstr, "set HVon", sizeof("set HVon"));
    cmd_pack[23].func_ptr = &set_hv_on;

    memcpy(cmd_pack[24].cmdstr, "init table", sizeof("init table"));
    cmd_pack[24].func_ptr = &init_para_table;

    memcpy(cmd_pack[25].cmdstr, "ray source", sizeof("ray source"));
    cmd_pack[25].func_ptr = &ray_source;

    return;
}



