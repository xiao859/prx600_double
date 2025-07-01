///*
// *  app_comm.c
// *
// *  Created on: Feb 24, 2025
// *  Author: Administrator
// *
// */

#include "comm_protocol.h"
//#include <stdio.h>
//#include "main.h"
//#include "xray.h"
//#include "app_uart.h"
//#include "app_fun.h"
////#include "log.h"
//#include <string.h>
//#include "math.h"
////#include "stmflash.h"
//#include "app_spi.h"
//#include "exposure.h"
//#include "debug_mode.h"
//#include "calibrate.h"
//#include "protect.h"

//uint16_t adc_buffer2[ADC_2_CHANNEL_NUM * ADC_SAMPLE_CYCLE_NUM];
//uint16_t adc_buffer3[ADC_3_CHANNEL_NUM * ADC_SAMPLE_CYCLE_NUM];

//volatile hvps_sm_state hv_state[XRAY_NUMS];
////hv_state[0]	= HVPS_SM_ID_IDLE;
//volatile xray_mode xrayMode = XRAY_MODE_S_PULSE;
//volatile cmd_control_data ctrl_data;
//volatile xray_config_data config_data;
//volatile adc_sampled_value sampled_data = {
//    30, 30, 10, 1000, 24, 0, 24, 0, 20};
//volatile adc_sampled_value sampled_data_last = {0};

//volatile xray_parament_range para_range[XRAY_NUMS] = {
//    {140, 50,            /* 管电压保护值 */
//    145, 5, 750,        /* 管电流保护值：最大最小和瞬时 */

//    70, -25, 60, 57,     /* 油箱温度 */

//    28, 20,           /* 24V供电 */
//    100, 100,           /* 散热器温度 */
//    30, 8,           /* 灯丝电压保护 */
//    100, 100,           /* 灯丝电流保护 */

//    100, 60,            /* 管电压配置门限 */
//    120, 10,             /* 管电流配置门限 */

//    1000, 1000,         /* 曝光时间保护 */

//    4500000,            /* 最大曝光时间，1.5min */

//    6000000             /* 灯丝开启未曝光时间保护 */
//		},
//		{
//			0,
//		
//		}
//};

//volatile xray_parament_table parm_table[XRAY_NUMS] = {
//	{
//    0, 0, 1,
//    {1,    2,    3,    4,    5,    6,    7,    8,    9,    10,   11,   12},
//    {1606, 1734, 1830, 1911, 1980, 2039, 2091, 2134, 2174, 2210, 2246, 2282},
//    {1500, 1660, 1770, 1850, 1920, 1980, 2030, 2080, 2120, 2160, 2200, 2250},
//	},
//	{
//	        0, 0, 0,
//        {0}, {0}, {0}
//	}
//};

//volatile xray_version version = {
//    1, 1,
//    1, 0, 2,
//    2, 1, 19, 0,
//    17, 23,
//    3, 6, 0
//};







//void func_test(uint8_t *buff)
//{
//    return;
//}



////void parament_init(void)
////{
////    set_hv_state(HVPS_SM_ID_IDLE);

////    config_data.tube_vol  = 60;
////    config_data.tube_curr = 10;

////    /* FLASH中参数是否擦写：
////       判断到valid_flag == 0，说明从boot擦写过来的：需要把参数重新擦写到FLASH中。（为了满足用户手动改参数）
////       否则，说明是正常的上电动作，直接把FLASH更新到参数中
////    */
////   if (stmflash_read_word(PARAM_CODE_VALID_ADDR) != PARAM_CODE_VALID_FLAG) {
////        /* 1、设置标志位 */
////        update_paraFlash();

////        /* 2、把参数擦写到FLASH中 */
////        xray_parament_table table_local;
////        /* 2.1、读取全部FLASH */
////        get_flash_parament((uint8_t *)&table_local, sizeof(xray_parament_table));

////        /* 2.2、赋值需要的参数，曝光次数和时间不需要刷新 */
////        parm_table.expo_times_total = table_local.expo_times_total;
////        parm_table.expo_count_total = table_local.expo_count_total;

////        memcpy((uint8_t *)&table_local.currValue[0], (uint8_t *)&parm_table.currValue[0], sizeof(parm_table.currValue));
////        memcpy((uint8_t *)&table_local.currRef[0], (uint8_t *)&parm_table.currRef[0], sizeof(parm_table.currRef));
////        memcpy((uint8_t *)&table_local.currRef_c[0], (uint8_t *)&parm_table.currRef_c[0], sizeof(parm_table.currRef_c));

////        /* 2.3、写入到FLASH中 */
////        wirte_flash_parament((uint8_t *)&table_local, sizeof(xray_parament_table));

////   } else {
////        /* 把校准过的值赋值给表 */
////        // xray_parament_table table_local;
////        get_flash_parament((uint8_t *)&parm_table, sizeof(xray_parament_table));
////        // memcpy((uint8_t *)&parm_table, (uint8_t *)&table_local, sizeof(xray_parament_table));
////   }

////   return;
////}









///* 进行一些开始前的初始化， */


///* 由实际值，找到索引；由索引找到REF */




///* 控制管电压基准上升曲线  offset = 0.22405*config_data.tube_vol_realtime - 6.89364*/






///* 非曝光状态，各种信号初始化 */
//void xray_system_disable()
//{
//    xray_CT_disable(0);
//	  xray_CT_disable(1);

//    ctrl_data.enable[0] = 0;
//	  ctrl_data.enable[1] = 0;
//    ctrl_data.expo[0] = 0;
//	  ctrl_data.expo[1] = 0;
//    ctrl_data.filament_on[0] = 0;
//    ctrl_data.filament_on[1] = 0;	

//    return;
//}

//void xray_system_fault_check()
//{
//    if (!Is_System_Without_Fault()) {
//        set_hv_state(HVPS_SM_ID_FAULT,0);
//			  set_hv_state(HVPS_SM_ID_FAULT,1);
//        xray_system_disable();
//        config_fault_signal(1);

//        falut_led(1);
//    } else {
//        falut_led(0);
//    }

//    return;
//}
