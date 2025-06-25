#include "comm_string.h"
#include "string.h"
#include "comm_protocol.h"
#include "math.h"
#include "stdlib.h"
#include "app_uart.h"
#include "stm32g4xx_hal.h"
//#include "stmflash.h"
#include "app_spi.h"
//#include "debug_mode.h"
#include "xray.h"
#include "calibrate.h"
#include "pi_control.h"

LC_Command_string cmd_pack[LC_CMD_NUM];

void unpackCmd_savePara(volatile uint8_t *buff, uint8_t *cmdHead, volatile uint32_t *targetAddr, uint8_t offset)
{
    if (strstr((const char *)buff, (const char *)cmdHead) == NULL) {
        debug_tx3("格式错误!!!\r\n");
        return;
    }

    volatile uint8_t *p_p = buff + offset;
    uint32_t table_temp[FILAMENT_CURRENT_TABLE_ORDER];
    uint8_t para_count = 0;

    char *token = strtok((char *)p_p, " ");
    while (token != NULL) {
        int num = atoi(token);
        table_temp[para_count] = num;
        para_count++;
        token = strtok(NULL, " ");
    }

    if (para_count < FILAMENT_CURRENT_TABLE_ORDER) {
        debug_tx3("参数不够: %d!!!\n", para_count);
    } else {
        // save_parament_to_flash();

        /* 读 */
        get_flash_parament((uint8_t *)&parm_table, sizeof(xray_parament_table));

        /* 赋 */
        memcpy((uint8_t *)targetAddr, (uint8_t *)&table_temp[0], sizeof(table_temp));

        /* 擦 */
        save_parament_to_flash();
    }

    return;
}

int func_setCurrvalue(volatile uint8_t *buff, char *p)
{
//    uint8_t cmdHead[] = "set currValue ";

    // unpackCmd_savePara(buff, cmdHead, &parm_table.currValue[0], 14);

    return 0;
}

int func_setCurrRef(volatile uint8_t *buff, char *p)
{
    uint8_t cmdHead[] = "set currRef ";

    unpackCmd_savePara(buff, cmdHead, &parm_table.currRef[0], 12);

    return 0;
}

int func_setCurrvalue_c(volatile uint8_t *buff, char *p)
{
//    uint8_t cmdHead[] = "set currValue_c ";

    // unpackCmd_savePara(buff, cmdHead, &parm_table.currValue_c[0], 16);

    return 0;
}

int func_setCurrRef_c(volatile uint8_t *buff, char *p)
{
    uint8_t cmdHead[] = "set currRef_c ";

    unpackCmd_savePara(buff, cmdHead, &parm_table.currRef_c[0], 14);

    return 0;
}

int func_getTable(volatile uint8_t *buff, char *p)
{
    debug_tx3("曝光次数: %d\n", parm_table.expo_count_total);
    debug_tx3("曝光时间: %d\n", parm_table.expo_times_total);

    debug_tx3("脉冲电流值:");
    for (int i = 0; i < FILAMENT_CURRENT_TABLE_ORDER; i++) {
        debug_tx3(" %d", parm_table.currValue[i]);
    }
    debug_tx3("\n");

    debug_tx3("脉冲电流表:");
    for (int i = 0; i < FILAMENT_CURRENT_TABLE_ORDER; i++) {
        debug_tx3(" %d", parm_table.currRef[i]);
    }
    debug_tx3("\n");

    debug_tx3("连续电流表:");
    for (int i = 0; i < FILAMENT_CURRENT_TABLE_ORDER; i++) {
        debug_tx3(" %d", parm_table.currRef_c[i]);
    }
    debug_tx3("\n");

    return 0;
}

/* 设置管电压：单位是kv。格式set tubeVC 60 20。只有60kv生效 */
int set_tube_vol_curr(volatile uint8_t *buff, char *p)
{
    volatile uint8_t *p_p = buff + 11;
    uint8_t para_count = 0;
    int data[2];

    char *token = strtok((char *)p_p, " ");
    while (token != NULL && para_count <=2) {
        int num = atoi(token);
        data[para_count] = num;
        para_count++;
        token = strtok(NULL, " ");
    }

    if (para_count == 2) {
        config_data.tube_curr = ((float)data[1])/10;
        config_data.tube_vol  = data[0];
        config_data.tube_vol_step = (float)(config_data.tube_vol) / (50 * 1);
        debug_tx3("管电压/电流：%d, %d, %f\n", config_data.tube_vol, config_data.tube_curr, config_data.tube_vol_step);
    } else {
        debug_tx3("参数格式错误\n");
    }

    return 0;
}

/* 设置曝光时间（必须先设置曝光模式）。
 * 25ms周期中，8ms曝光；脉冲次数：x秒 * 40。
 * 任务50kHz对应0.02ms；则曝光周期：400，冷却周期850
 */
int set_expo_time(volatile uint8_t *buff, char *p)
{
    volatile uint8_t *p_p = buff + 13;
    int num = 0;

    char *token = strtok((char *)p_p, " ");
    while (token != NULL) {
        num = atoi(token);
        token = strtok(NULL, " ");
    }

    if (num == 0) {
        debug_tx3("参数格式错误");
        return 0;
    }

    if (ctrl_data.xrayMode == XRAY_MODE_PULSE) {
        debug_data.expoCycle_perCurrent = num * 40;
        debug_data.expoTime_expect = 400;
        debug_data.coolTime_expect = 850;
    } else {
        debug_data.expoCycle_perCurrent = 1;
        debug_data.expoTime_expect = num * 50000;
    }

    debug_tx3("脉冲个数：%d, 曝光：%d, 冷却：%d\n",
            debug_data.expoCycle_perCurrent, debug_data.expoTime_expect, debug_data.coolTime_expect);

    return 0;
}

/* 设置曝光模式，0：脉冲；1：连续 */
int set_expo_mode(volatile uint8_t *buff, char *p)
{
    volatile uint8_t *p_p = buff + 13;
    int num = 0;

    config_reset_signal(1);

    char *token = strtok((char *)p_p, " ");
    while (token != NULL) {
        num = atoi(token);
        token = strtok(NULL, " ");
    }

    if (num == 0) {
        ctrl_data.xrayMode = XRAY_MODE_PULSE;
        debug_tx3("脉冲模式\n");
    } else {
        ctrl_data.xrayMode = XRAY_MODE_CONTINUOUS;
        debug_tx3("连续模式\n");
    }

    return 0;
}

/* 设置enable信号 */
int set_enable(volatile uint8_t *buff, char *p)
{
    volatile uint8_t *p_p = buff + 11;
    int num = 0;

    char *token = strtok((char *)p_p, " ");
    while (token != NULL) {
        num = atoi(token);
        token = strtok(NULL, " ");
    }

    if (num == 1) {
        set_hv_state(HVPS_SM_ID_TRAIN_PREPARE);
        ctrl_data.interlock = 1;
        ctrl_data.enable    = 1;
        debug_data.timmer_count = 1;
        HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
        debug_tx3("使能开始\n");
    } else {
        set_hv_state(HVPS_SM_ID_IDLE);
        ctrl_data.interlock = 0;
        ctrl_data.enable = 0;
        debug_tx3("使能结束\n");
    }

    return 0;
}

/* 模拟曝光信号的开启，后面的周期自己控制 */
int set_ref_onoff(volatile uint8_t *buff, char *p)
{
    set_hv_state(HVPS_SM_ID_TRAIN_EXPOSURING);
    // debug_data.timmer_count = 0;
    xray_HV_enable_debug(1);
    ctrl_data.enable    = 1;
    debug_data.timmer_count = 1;

    debug_tx3("开始给高压基准\n");

    return 0;
}

/* 模拟曝光信号的开启，后面的周期自己控制 */
int set_ref_test(volatile uint8_t *buff, char *p)
{
    volatile uint8_t *p_p = buff + 8;
    uint8_t para_count = 0;
    int data[2];
    set_hv_state(HVPS_SM_ID_TRAIN_DEBUG);

    char *token = strtok((char *)p_p, " ");
    while (token != NULL && para_count <=2) {
        int num = atoi(token);
        data[para_count] = num;
        para_count++;
        token = strtok(NULL, " ");
    }

    if (para_count == 2) {
        config_data.tube_curr = data[1];
        config_data.tube_vol  = data[0];
        uint32_t tube_vol_ref = (uint32_t)(((float)config_data.tube_vol / 64) / ADDA_FULL_SCALE_VIL_VALUE * 4095);  /* 0~2.5V 对应 0~160kV */
        // uint32_t tube_curr_ref = (uint32_t)((config_data.tube_curr / 64) / ADDA_FULL_SCALE_VIL_VALUE * 4095);

        HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, tube_vol_ref);
        // HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, tube_vol_ref);
        debug_tx3("管电压/电流：%d, %d, %f\n", tube_vol_ref, config_data.tube_curr);
    } else {
        debug_tx3("参数格式错误\n");
    }

    return 0;
}

/* 灯丝复位信号 */
int set_reset(volatile uint8_t *buff, char *p)
{
    config_reset_signal(1);

    debug_tx3("灯丝复位\n");

    return 0;
}


/* 灯丝开启 */
int set_filament_onoff(volatile uint8_t *buff, char *p)
{
    volatile uint8_t *p_p = buff + 16;
    int num = 0;

    char *token = strtok((char *)p_p, " ");
    while (token != NULL) {
        num = atoi(token);
        token = strtok(NULL, " ");
    }

    if (num == 1) {
        config_filamentOn_signal(1);
    } else {
        config_filamentOn_signal(0);
        ctrl_data.filament_on = 0;
    }

    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 0);

    debug_tx3("灯丝使能：%d\n", num);

    return 0;
}

/* 给灯丝基准，单位是引脚电平扩大1000倍 */
int set_filament_ref_onoff(volatile uint8_t *buff, char *p)
{
    volatile uint8_t *p_p = buff + 17;
    int num = 0;

    char *token = strtok((char *)p_p, " ");
    while (token != NULL) {
        num = atoi(token);
        token = strtok(NULL, " ");
    }

    ctrl_data.filament_on = 1;
    set_hv_state(HVPS_SM_ID_TRAIN_IDLE);
    debug_data.timmer_count = 0;

    // uint32_t a = (uint32_t)(num * 1.2409);  /* ((num / 1000) / 3.3) * 4095 */

    config_data.fila_ref_target = (float)num / 1000;
    config_data.fila_ref_realtime = IDLE_FILAMENT_REF_DEBUG;
    config_data.fila_ref_step = (config_data.fila_ref_target - IDLE_FILAMENT_REF_DEBUG) / (50 * 1);

    debug_tx3("灯丝基准：%f, %f\n", config_data.fila_ref_target, config_data.fila_ref_step);

    // HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, a);

    return 0;
}

/* 开始自动校准 */
int start_calibrate(volatile uint8_t *buff, char *p)
{
    volatile uint8_t *p_p = buff + 16;
    int num = 0;

    char *token = strtok((char *)p_p, " ");
    while (token != NULL) {
        num = atoi(token);
        token = strtok(NULL, " ");
    }

    if (num == 1) {
        calibrate_para_init();
        set_hv_state(HPVS_SM_ID_CAL_PREPARE);
        ctrl_data.enable = 1;
        // pid_Init(1.0);
        debug_tx3("开始校准");
    } else {
        ctrl_data.enable = 0;
        debug_tx3("结束校准");
    }

    return 0;
}

/* 开始自动校准 */
int set_hv_on(volatile uint8_t *buff, char *p)
{
    debug_tx3("高压使能");
    xray_HV_enable_debug(1);
    debug_data.timmer_count = 1;

    return 0;
}

/* 初始化FLASH的参数 */
int init_para_table(volatile uint8_t *buff, char *p)
{
    xray_parament_table parm_table_temp = {
        0, 0, 1,
        {1,    2,    3,    4,    5,    6,    7,    8,    9,    10,   11,   12},
        {1606, 1734, 1830, 1911, 1980, 2039, 2091, 2134, 2174, 2210, 2246, 2282},
        {1500, 1660, 1770, 1850, 1920, 1980, 2030, 2080, 2120, 2160, 2200, 2250},
    };

    wirte_flash_parament((uint8_t *)&parm_table_temp, sizeof(xray_parament_table));

    get_flash_parament((uint8_t *)&parm_table, sizeof(xray_parament_table));

    debug_tx3("已刷新FLASH");

    return 0;
}

// lc oven sttemp 31.255
void cmd_parser_string()
{
    if (uart3.recv_complete != 1) return;

    volatile uint8_t *buff = uart3.uart_rx_buf;

    int i;

    char *ret;

    /* 1、判断功能指令是否在注册表中 */
    for (i = 0; i < LC_CMD_NUM; i++) {
        ret = strstr((const char *)buff, (const char *)cmd_pack[i].cmdstr);
        if (ret != NULL) break;
    }

    if (ret == NULL) {
        debug_tx3("没有该指令！！！");
        restart_usart_receive(USART3);
        return;
    }

    /* 2、调用对应的函数 */
    cmd_pack[i].func_ptr(buff, ret); /* ret为指令起始位置 */

    restart_usart_receive(USART3);

    return;
}

/* 注册表 */
void registerFunc_init()
{
    memcpy(cmd_pack[0].cmdstr, "set currValue", sizeof("set currValue"));
    cmd_pack[0].func_ptr = &func_setCurrvalue;

    memcpy(cmd_pack[1].cmdstr, "set currRef", sizeof("set currRef"));
    cmd_pack[1].func_ptr = &func_setCurrRef;

    memcpy(cmd_pack[2].cmdstr, "set currValue_c", sizeof("set currValue_c"));
    cmd_pack[2].func_ptr = &func_setCurrvalue_c;

    memcpy(cmd_pack[3].cmdstr, "set currRef_c", sizeof("set currRef_c"));
    cmd_pack[3].func_ptr = &func_setCurrRef_c;

    memcpy(cmd_pack[4].cmdstr, "get currTable", sizeof("get currTable"));
    cmd_pack[4].func_ptr = &func_getTable;

    memcpy(cmd_pack[5].cmdstr, "set tubeVC", sizeof("set tubeVC"));
    cmd_pack[5].func_ptr = &set_tube_vol_curr;

    memcpy(cmd_pack[6].cmdstr, "set expotime", sizeof("set expotime"));
    cmd_pack[6].func_ptr = &set_expo_time;

    memcpy(cmd_pack[7].cmdstr, "set expomode", sizeof("set expomode"));
    cmd_pack[7].func_ptr = &set_expo_mode;

    memcpy(cmd_pack[8].cmdstr, "set enable", sizeof("set enable"));
    cmd_pack[8].func_ptr = &set_enable;

    memcpy(cmd_pack[9].cmdstr, "tube reset", sizeof("tube reset"));
    cmd_pack[9].func_ptr = &set_reset;

    memcpy(cmd_pack[10].cmdstr, "start expo", sizeof("start expo"));
    cmd_pack[10].func_ptr = &set_ref_onoff;

    memcpy(cmd_pack[11].cmdstr, "set filament on", sizeof("set filament on"));
    cmd_pack[11].func_ptr = &set_filament_onoff;

    memcpy(cmd_pack[12].cmdstr, "set filament ref", sizeof("set filament ref"));
    cmd_pack[12].func_ptr = &set_filament_ref_onoff;

    memcpy(cmd_pack[13].cmdstr, "start calibrate", sizeof("start calibrate"));
    cmd_pack[13].func_ptr = &start_calibrate;

    memcpy(cmd_pack[14].cmdstr, "set ref", sizeof("set ref"));
    cmd_pack[14].func_ptr = &set_ref_test;

    memcpy(cmd_pack[15].cmdstr, "set HVon", sizeof("set HVon"));
    cmd_pack[15].func_ptr = &set_hv_on;

    memcpy(cmd_pack[16].cmdstr, "init table", sizeof("init table"));
    cmd_pack[16].func_ptr = &init_para_table;

    return;
}
