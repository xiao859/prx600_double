//#ifndef COMM_PROTOCOL_H_
//#define COMM_PROTOCOL_H_

///* 1、头文件包含 */
//#include <stdint.h>
//#include "main.h"
////#include "debug_mode.h"
///* 2、宏定义 */
//#define XRAY_NUMS                       2

///* ADC通道多次采样取平均 */
//#define ADC_SAMPLE_CYCLE_NUM            16

//#define ADC_2_CHANNEL_NUM               4
//#define ADC_3_CHANNEL_NUM               4

//#define FILAMENT_CURRENT_TABLE_ORDER    12

//#define ADDA_FULL_SCALE_VIL_VALUE       (3.3f)




///* 开灯丝时的预热基准，先给的电压值 */
//#define IDLE_FILAMENT_REF               1000

//#define OVER_RANGE_TIME_LIMIT           5000

///* 10kHz定时器，对应4ms */
//#define FAST_PROTECT_TIME_RANGE         40

///* 连续打火5次 */
//#define STRIKE_TIEMS_RANGE              5

//#define MAX(a, b) ((a) > (b) ? (a) : (b))
//#define MIN(a, b) ((a) < (b) ? (a) : (b))

///* 3、数据类型定义 */
//typedef enum {
//    SCI_MSG_INQ_MODE=0x00,
//    SCI_MSG_INQ_TUBE_VSET,
//    SCI_MSG_INQ_TUBE_ISET,
//    SCI_MSG_INQ_MAX_TIME,
//    SCI_MSG_INQ_TEMP,
//    SCI_MSG_INQ_FAULT,
//    SCI_MSG_INQ_STATE,
//    SCI_MSG_INQ_SW,
//    SCI_MSG_INQ_HW,
//    SCI_MSG_INQ_TUBE_LAST_V,
//    SCI_MSG_INQ_TUBE_LAST_I,
//    SCI_MSG_INQ_TUBE_LAST_EXPOTIME,
//    SCI_MSG_INQ_LAMP_SW,
//    SCI_MSG_INQ_LAMP_HW,
//    SCI_MSG_INQ_EXPO_TIME1,
//	  SCI_MSG_INQ_EXPO_TIME2,
//    SCI_MSG_INQ_EXPO_COUNT1,
//	  SCI_MSG_INQ_EXPO_COUNT2,
//    SCI_MSG_INQ_AUTOCALIBRA,
//    SCI_MSG_INQ_XSOURCE_SW,

//    SCI_MSG_SET_MODE=0x20,
//    SCI_MSG_SET_TUBE_V,
//    SCI_MSG_SET_TUBE_I,
//    SCI_MSG_SET_MAX_TIME,
//    SCI_MSG_SET_EXP1_COUNTCLR,
//		SCI_MSG_SET_EXP2_COUNTCLR,
//    SCI_MSG_SET_EXP1_TIMECLR,
//		SCI_MSG_SET_EXP2_TIMECLR,
//		SCI_MSG_SET_ENABLE,

//    SCI_MSG_CTRL_RST=0x30,
//    SCI_MSG_CTRL_CAL,
//    SCI_MSG_CTRL_TRAIN,
//    SCI_MSG_CTRL_UPDATE,
//    SCI_MSG_CTRL_STORE_TABLE,
//    SCI_MSG_CTRL_STORE_TABLE_INQ,
//    SCI_MSG_CTRL_STORE_STATISTICS,
//    SCI_MSG_LAMP_CONTROL,

//    SCI_MSG_SET_PFCTHRESHOLD=0x40,
//    SCI_MSG_SET_24VTHRESHOLD,
//    SCI_MSG_SET_KVMATHRESHOLD,
//    SCI_MSG_SET_BUCKLLCTHRESHOLD,

//    SCI_MSG_DEBUG_EXPO_CTRL=0x50,

//    SCI_MSG_DEBUG_LAMP_I_SET=0xA0,
//    SCI_MSG_DEBUG_TUBE_VIDLE_SET,
//    SCI_MSG_DEBUG_TUBE_VRISE_TIME_SET,
//    SCI_MSG_DEBUG_ONLINE_PI,

//    SCI_MSG_TEST_1=0xB0,
//    SCI_MSG_TEST_2,
//    SCI_MSG_TEST_3,
//    SCI_MSG_TEST_4,
//} SCI_MSG_ID;

///*  射源系统高压状态机定义 */


///* 射源模式，脉冲或者连续 */
//typedef enum
//{
//    XRAY_MODE_S_CONTINUOUS = 0x00,           /* 连续模式 */
//		XRAY_MODE_S_PULSE      = 0x01,           /* 脉冲模式 */
//	  XRAY_MODE_D_CONTINUOUS = 0x02,           /* 连续模式 */
//		XRAY_MODE_D_PULSE      = 0x03,           /* 脉冲模式 */
//} xray_mode;
//extern volatile xray_mode xrayMode;
///* MCU指令控制数据 */
//typedef struct
//{
//    /* input from MCU: EXIT IO */
//    uint8_t     enable[XRAY_NUMS];         /* 使能信号，射源从空闲转为工作状态 */
//    uint8_t     expo[XRAY_NUMS];           /* 曝光信号 */
//    uint8_t     interlock;      /* 互锁开信号 */
//    uint8_t     hv_vol_fault;   /* 高压电压过压故障 */
//    uint8_t     hv_curr_fault;  /* 高压电流过流故障 */

//    uint8_t     filament_on[XRAY_NUMS];    /* 灯丝开启 */

//    xray_mode   xrayMode;
//	
//    uint16_t xray_current; 
//	
//	  uint16_t xray_switch_counter;    

//} cmd_control_data;
//extern volatile cmd_control_data ctrl_data;

///* 射源配置数据 */

//extern volatile xray_config_data config_data;





///* 版本号 */


//extern uint16_t adc_buffer2[ADC_2_CHANNEL_NUM * ADC_SAMPLE_CYCLE_NUM];
//extern uint16_t adc_buffer3[ADC_3_CHANNEL_NUM * ADC_SAMPLE_CYCLE_NUM];

///* 4、函数声明 */
//

//void set_hv_state(hvps_sm_state state,uint16_t n);
//void cmd_parser(void);

//void xray_system_disable(void);
//void xray_system_fault_check(void);
//uint32_t get_filamentRef(float tube_current,uint16_t n);
//void parament_init(void);
//void save_parament_to_flash(void);
//void config_hvref_slope(uint16_t n);
//void config_filament_ref_slop(uint16_t n);
//void config_filamentRef(uint16_t n);
//void disable_hvref(void);
//void disable_filamentref(uint16_t n);

//void hvState_ilde_init(uint16_t n);


//#endif
