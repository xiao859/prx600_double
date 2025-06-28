#ifndef COMM_PROTOCOL_H_
#define COMM_PROTOCOL_H_

/* 1、头文件包含 */
#include <stdint.h>
#include "main.h"
//#include "debug_mode.h"
/* 2、宏定义 */
#define XRAY_NUMS                       2

/* ADC通道多次采样取平均 */
#define ADC_SAMPLE_CYCLE_NUM            16

#define ADC_2_CHANNEL_NUM               4
#define ADC_3_CHANNEL_NUM               4

#define FILAMENT_CURRENT_TABLE_ORDER    12

#define ADDA_FULL_SCALE_VIL_VALUE       (3.3f)

#define Is_Exposing()                   ((hv_state[0] == HVPS_SM_ID_EXPOSURING) || (hv_state[0] == HPVS_SM_ID_CAL_EXPOSURING) || (hv_state[0] == HVPS_SM_ID_TRAIN_EXPOSURING)||(hv_state[1] == HVPS_SM_ID_EXPOSURING) || (hv_state[1] == HPVS_SM_ID_CAL_EXPOSURING) || (hv_state[1] == HVPS_SM_ID_TRAIN_EXPOSURING))

#define Is_FaultState()                 (hv_state[0] == HVPS_SM_ID_FAULT||hv_state[0] == HVPS_SM_ID_FAULT)

#define Is_CalibrateMode()              ((hv_state[0] >= HPVS_SM_ID_CAL_PREPARE) && (hv_state[0] <= HPVS_SM_ID_CAL_END)|| (hv_state[1] >= HPVS_SM_ID_CAL_PREPARE) && (hv_state[1] <= HPVS_SM_ID_CAL_END))

#define Is_CTMode()                     ((hv_state[0] >= HVPS_SM_ID_IDLE) && (hv_state[0] <= HVPS_SM_ID_EXPO_END)||(hv_state[1] >= HVPS_SM_ID_IDLE) && (hv_state[1] <= HVPS_SM_ID_EXPO_END))

#define Is_DebugMode()                  ((hv_state[0] >= HVPS_SM_ID_TRAIN_IDLE) && (hv_state[0] <= HVPS_SM_ID_TRAIN_END)||(hv_state[1] >= HVPS_SM_ID_TRAIN_IDLE) && (hv_state[1] <= HVPS_SM_ID_TRAIN_END))

#define IDLE_HV_REF                     0

/* 开灯丝时的预热基准，先给的电压值 */
#define IDLE_FILAMENT_REF               1000

#define OVER_RANGE_TIME_LIMIT           5000

/* 10kHz定时器，对应4ms */
#define FAST_PROTECT_TIME_RANGE         40

/* 连续打火5次 */
#define STRIKE_TIEMS_RANGE              5

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

/* 3、数据类型定义 */
typedef enum {
    SCI_MSG_INQ_MODE=0x00,
    SCI_MSG_INQ_TUBE_VSET,
    SCI_MSG_INQ_TUBE_ISET,
    SCI_MSG_INQ_MAX_TIME,
    SCI_MSG_INQ_TEMP,
    SCI_MSG_INQ_FAULT,
    SCI_MSG_INQ_STATE,
    SCI_MSG_INQ_SW,
    SCI_MSG_INQ_HW,
    SCI_MSG_INQ_TUBE_LAST_V,
    SCI_MSG_INQ_TUBE_LAST_I,
    SCI_MSG_INQ_TUBE_LAST_EXPOTIME,
    SCI_MSG_INQ_LAMP_SW,
    SCI_MSG_INQ_LAMP_HW,
    SCI_MSG_INQ_EXPO_TIME1,
	  SCI_MSG_INQ_EXPO_TIME2,
    SCI_MSG_INQ_EXPO_COUNT1,
	  SCI_MSG_INQ_EXPO_COUNT2,
    SCI_MSG_INQ_AUTOCALIBRA,
    SCI_MSG_INQ_XSOURCE_SW,

    SCI_MSG_SET_MODE=0x20,
    SCI_MSG_SET_TUBE_V,
    SCI_MSG_SET_TUBE_I,
    SCI_MSG_SET_MAX_TIME,
    SCI_MSG_SET_EXP1_COUNTCLR,
		SCI_MSG_SET_EXP2_COUNTCLR,
    SCI_MSG_SET_EXP1_TIMECLR,
		SCI_MSG_SET_EXP2_TIMECLR,
		SCI_MSG_SET_ENABLE,

    SCI_MSG_CTRL_RST=0x30,
    SCI_MSG_CTRL_CAL,
    SCI_MSG_CTRL_TRAIN,
    SCI_MSG_CTRL_UPDATE,
    SCI_MSG_CTRL_STORE_TABLE,
    SCI_MSG_CTRL_STORE_TABLE_INQ,
    SCI_MSG_CTRL_STORE_STATISTICS,
    SCI_MSG_LAMP_CONTROL,

    SCI_MSG_SET_PFCTHRESHOLD=0x40,
    SCI_MSG_SET_24VTHRESHOLD,
    SCI_MSG_SET_KVMATHRESHOLD,
    SCI_MSG_SET_BUCKLLCTHRESHOLD,

    SCI_MSG_DEBUG_EXPO_CTRL=0x50,

    SCI_MSG_DEBUG_LAMP_I_SET=0xA0,
    SCI_MSG_DEBUG_TUBE_VIDLE_SET,
    SCI_MSG_DEBUG_TUBE_VRISE_TIME_SET,
    SCI_MSG_DEBUG_ONLINE_PI,

    SCI_MSG_TEST_1=0xB0,
    SCI_MSG_TEST_2,
    SCI_MSG_TEST_3,
    SCI_MSG_TEST_4,
} SCI_MSG_ID;

/*  射源系统高压状态机定义 */
typedef enum
{
    HVPS_SM_ID_IDLE = 0x00,             // 空闲 0x00
    HVPS_SM_ID_PREPARE,                 // 正在准备0x01，进入条件Enable置0
    HVPS_SM_ID_READY,                   // 准备完成0x02，进入条件射源做好准备工作
    HVPS_SM_ID_EXPOSURING,             	// 出射线中，Exposure置0
    HVPS_SM_ID_EXPO_END,               	// 出射线结束

    HPVS_SM_ID_CAL_PREPARE = 0x10,      // 校准等待启动，进入条件收到0x31报文
    HPVS_SM_ID_CAL_RUN,                 // 校准运行，进入条件Enable置0
    HPVS_SM_ID_CAL_EXPOSURING,          // 校准运行，进入条件Enable置0
    HPVS_SM_ID_CAL_COOLING,             // 校准运行，进入条件Enable置0
    HPVS_SM_ID_CAL_END,                 // 校准结束，退出条件Enable置1后转至HVPS_SM_ID_IDLE

    HVPS_SM_ID_TRAIN_IDLE = 0x20,    // 训管空闲
    HVPS_SM_ID_TRAIN_PREPARE,        // 训管等待启动，进入条件收到0x32报文
    HVPS_SM_ID_TRAIN_RUN,               // 训管运行中，进入条件Enable置0
    HVPS_SM_ID_TRAIN_EXPOSURING,        // 校准运行，进入条件Enable置0
    HVPS_SM_ID_TRAIN_COOLING,           // 校准运行，进入条件Enable置0
    HVPS_SM_ID_TRAIN_END,               // 训管结束，退出条件Enable置1后转至HVPS_SM_ID_IDLE
    HVPS_SM_ID_TRAIN_DEBUG,             // 测试模式

    HVPS_SM_ID_UPDATE_PREPRE = 0x30,    // 升级准备，进入条件收到0x33报文
    HVPS_SM_ID_UPDATE_RUN,              // 升级运行中，进入条件Enable置0
    HVPS_SM_ID_UPDATE_END,              // 升级完毕，之后重启

    HVPS_SM_ID_FAULT = 0xFF            	// 故障，收到0x30报文清除故障后进入HVPS_SM_ID_IDLE
} hvps_sm_state;
extern volatile hvps_sm_state hv_state[XRAY_NUMS];

/* 射源模式，脉冲或者连续 */
typedef enum
{
    XRAY_MODE_S_CONTINUOUS = 0x00,           /* 连续模式 */
		XRAY_MODE_S_PULSE      = 0x01,           /* 脉冲模式 */
	  XRAY_MODE_D_CONTINUOUS = 0x02,           /* 连续模式 */
		XRAY_MODE_D_PULSE      = 0x03,           /* 脉冲模式 */
} xray_mode;
extern volatile xray_mode xrayMode;
/* MCU指令控制数据 */
typedef struct
{
    /* input from MCU: EXIT IO */
    uint8_t     enable[XRAY_NUMS];         /* 使能信号，射源从空闲转为工作状态 */
    uint8_t     expo[XRAY_NUMS];           /* 曝光信号 */
    uint8_t     interlock;      /* 互锁开信号 */
    uint8_t     hv_vol_fault;   /* 高压电压过压故障 */
    uint8_t     hv_curr_fault;  /* 高压电流过流故障 */

    uint8_t     filament_on[XRAY_NUMS];    /* 灯丝开启 */

    xray_mode   xrayMode;
	
    uint16_t xray_current; 
	
	  uint16_t xray_switch_counter;    

} cmd_control_data;
extern volatile cmd_control_data ctrl_data;

/* 射源配置数据 */
typedef struct
{
    /* output to MCU*/
    uint8_t reay_signal;    /* 准备就绪 */
    uint8_t xray_on;        /* 射源出射线信号 */
    uint8_t fault;          /* 发生了故障 */

    /* output to HV*/
    uint8_t hv_en;          /* 高压使能 */
    uint8_t mcu_lock;       /* 高压互锁 */
    uint8_t reset;          /* 故障复位 */

    int8_t  tube_vol[XRAY_NUMS];        /* 串口配置的管电压和管电流 */
    float   tube_curr[XRAY_NUMS];       /* 管电流，单位1mA */
    uint8_t tube_curr_index[XRAY_NUMS]; /* 管电流查表索引 */

    float   tube_vol_realtime[XRAY_NUMS];  /* 管电压基准实时配置值 */
    float   tube_vol_step[XRAY_NUMS];      /* 管电压基准上升步长 */

    float   fila_ref_target[XRAY_NUMS];    /* 灯丝基准目标值 */
    float   fila_ref_realtime[XRAY_NUMS];  /* 灯丝基准实时配置值 */
    float   fila_ref_step[XRAY_NUMS];      /* 灯丝基准上升步长 */

    /* output to filament*/
    uint8_t filament_en[XRAY_NUMS];    /* 灯丝电源使能 */

    /* other data */
    uint32_t expo_count[XRAY_NUMS];         /* 射源曝光计时，单次的 */
    uint32_t expo_count_total[XRAY_NUMS];   /* 射源总曝光计时，从上电开始计算 */
    uint32_t expo_time_expect[XRAY_NUMS];   /* 曝光时间设置，换算成了周期数 */
    uint32_t fila_protect_cnt[XRAY_NUMS];   /* 灯丝开启未曝光保护计数 */ 

} xray_config_data;
extern volatile xray_config_data config_data;

/* ADC采样值 adc2:4\5\11\12  adc3:14\2\4\6 */
typedef struct {
    /* ADC3 */
    float tube_vol_p_value;      /* 正管电压 */
    float tube_vol_n_value;      /* 负管电压 */
    float tube_curr_value;       /* 管电流 */
    float temp_oil_value;        /* 油箱温度:AD值 */

    /* ADC2 */
    float power_24v_value;       /* 24V供电 */
    float temp_sink_value;       /* 散热器温度 */
    float filament_vol_value;    /* 灯丝电压 */
    float filament_curr_value;   /* 灯丝电流 */

    float oil_temp;              /* 油箱温度：转换值 */

} adc_sampled_value;
extern volatile adc_sampled_value sampled_data;
extern volatile adc_sampled_value sampled_data_last;
//extern volatile xray_debug_data debug_data;
/* 各种保护值，包括是采样的和配置的 */
typedef struct {
    /* 最大值和最小值保护，采样超限报警 */
    float tube_vol_max_protected;        /* 管电压 */
    float tube_vol_min_protected;

    float tube_curr_max_protected;       /* 管电流 */
    float tube_curr_min_protected;
    float tube_curr_peak_protected;      /* 瞬时管电流 */

    float temp_oil_max_protected;        /* 油箱温度 */
    float temp_oil_min_protected;
    float temp_oil_warning;
    float temp_oil_recovery;

    float power_24v_max_protected;       /* 24V供电 */
    float power_24v_min_protected;
    float temp_sink_max_protected;       /* 散热器温度 */
    float temp_sink_min_protected;
    float filament_vol_max_protected;    /* 灯丝电压 */
    float filament_vol_min_protected;
    float filament_curr_max_protected;   /* 灯丝电流 */
    float filament_curr_min_protected;

    /* 配置门限 */
    float tube_vol_max_config;        /* 管电压 */
    float tube_vol_min_config;

    float tube_curr_max_config;       /* 管电流 */
    float tube_curr_min_config;

    int16_t expo_time_max;               /* 设置曝光时间的最大值 */
    int16_t expo_time_min;               /* 设置曝光时间的最小值 */

    uint32_t expo_time_limit;

    uint32_t fila_protect_cnt_max;       /* 灯丝开启未曝光最大计数 */
} xray_parament_range;
extern volatile xray_parament_range para_range[XRAY_NUMS];

/* 电流查表 */
typedef struct {
    uint32_t    expo_times_total;  /* 总的曝光时间 */
    uint32_t    expo_count_total;  /* 总的曝光次数 */
    uint32_t    rising_time;       /* 管电压上升时间 */
    float       currValue[FILAMENT_CURRENT_TABLE_ORDER];      /* 电流值表 */
    uint32_t    currRef[FILAMENT_CURRENT_TABLE_ORDER];        /* 电流基准表 */
    uint32_t    currRef_c[FILAMENT_CURRENT_TABLE_ORDER];        /* 电流基准表 */
} xray_parament_table;
extern volatile xray_parament_table parm_table[XRAY_NUMS];

/* 版本号 */
typedef struct {
    /* 硬件版本号，高位PFC供电板，低位控制板 */
    uint16_t hw_ver_high : 8;
    uint16_t hw_ver_low  : 8;

    /* 固件版本号 */
    uint16_t sw_ver_high : 4;
    uint16_t sw_ver_mid  : 4;
    uint16_t sw_ver_low  : 8;

    /* 灯丝硬件版本号 */
    uint16_t fila_ver_high : 5;
    uint16_t fila_ver_mid1 : 4;
    uint16_t fila_ver_mid2 : 4;
    uint16_t fila_ver_low  : 3;

    /* 球管信息：11:凯龙，13:KL181SBR-0.6-125 */
    uint16_t tube_ver_high : 8;
    uint16_t tube_ver_low  : 8;

    /* 射源类型 */
    uint16_t xsrc_ver_high : 5;
    uint16_t xsrc_ver_mid  : 8;
    uint16_t xsrc_ver_low  : 3;
} xray_version;
extern volatile xray_version version;

extern uint16_t adc_buffer2[ADC_2_CHANNEL_NUM * ADC_SAMPLE_CYCLE_NUM];
extern uint16_t adc_buffer3[ADC_3_CHANNEL_NUM * ADC_SAMPLE_CYCLE_NUM];

/* 4、函数声明 */
hvps_sm_state get_hv_state(uint16_t n);

void set_hv_state(hvps_sm_state state,uint16_t n);
void cmd_parser(void);

void xray_system_disable(void);
void xray_system_fault_check(void);
uint32_t get_filamentRef(float tube_current,uint16_t n);
void parament_init(void);
void save_parament_to_flash(void);
void config_hvref_slope(uint16_t n);
void config_filament_ref_slop(uint16_t n);
void config_filamentRef(uint16_t n);
void disable_hvref(void);
void disable_filamentref(uint16_t n);
void flash_table_init(void);
void hvState_ilde_init(uint16_t n);


#endif
