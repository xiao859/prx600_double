#ifndef __HV_EXPOSURE_H
#define __HV_EXPOSURE_H

#include "stdint.h"

#define IDLE_FILAMENT1_REF               2120
#define IDLE_FILAMENT2_REF               1360

#define XRAY_NUMS 2
#define IDLE_HV_REF                     0
#define FILAMENT_CURRENT_TABLE_ORDER    12

#define ADC_SAMPLE_CYCLE_NUM            1
#define ADC_2_CHANNEL_NUM               4
#define ADC_3_CHANNEL_NUM               4
#define ADDA_FULL_SCALE_VIL_VALUE       (3.3f)

#define     Is_PulseMode_CT()           ((ctrl_data.xrayMode == XRAY_MODE_S_PULSE) || (ctrl_data.xrayMode == XRAY_MODE_D_PULSE))

#define     Is_ContinuousMode_CT()      ((ctrl_data.xrayMode== XRAY_MODE_S_CONTINUOUS) || (ctrl_data.xrayMode== XRAY_MODE_D_CONTINUOUS) )

extern uint16_t adc_buffer2[ADC_2_CHANNEL_NUM * ADC_SAMPLE_CYCLE_NUM];
extern uint16_t adc_buffer3[ADC_3_CHANNEL_NUM * ADC_SAMPLE_CYCLE_NUM];

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

// 状态机ID定义
typedef enum
{
    HVPS_SM_ID_IDLE = 0x00,             // 空闲 0x00
    HVPS_SM_ID_PREPARE,                 // 正在准备0x01，进入条件Enable置0
    HVPS_SM_ID_READY,                  // 准备完成0x02，进入条件射源做好准备工作
    HVPS_SM_ID_EXPOSURING,             // 出射线中，Exposure置0
    HVPS_SM_ID_EXPO_END,               // 出射线结束

    HPVS_SM_ID_CAL_PREPARE = 0x10,      // 校准等待启动，进入条件收到0x31报文
    HPVS_SM_ID_CAL_RUN,                 // 校准运行，进入条件Enable置0
    HPVS_SM_ID_CAL_EXPOSURING,          //
    HPVS_SM_ID_CAL_COOLING,             //
    HPVS_SM_ID_CAL_END,                 // 校准结束，退出条件Enable置1后转至HVPS_SM_ID_IDLE

    HVPS_SM_ID_TRAIN_IDLE = 0x20,    //
    HVPS_SM_ID_TRAIN_PREPARE = 0x20,    // 训管等待启动，进入条件收到0x32报文
    HVPS_SM_ID_TRAIN_RUN,               // 训管运行中，进入条件Enable置0
    HVPS_SM_ID_TRAIN_EXPOSURING,        // Ð£×¼ÔËÐÐ£¬½øÈëÌõ¼þEnableÖÃ0
    HVPS_SM_ID_TRAIN_COOLING,           // Ð£×¼ÔËÐÐ£¬½øÈëÌõ¼þEnableÖÃ0
    HVPS_SM_ID_TRAIN_END,               // 训管结束，退出条件Enable置1后转至HVPS_SM_ID_IDLE
    HVPS_SM_ID_TRAIN_DEBUG,             // ²âÊÔÄ£Ê½

    HVPS_SM_ID_UPDATE_PREPRE = 0x30,    // 升级准备，进入条件收到0x33报文
    HVPS_SM_ID_UPDATE_RUN,              // 升级运行中，进入条件Enable置0
    HVPS_SM_ID_UPDATE_END,              // 升级完毕，之后重启

    HVPS_SM_ID_FAULT = 0xFF            // 故障，收到0x30报文清除故障后进入HVPS_SM_ID_IDLE
} hvps_sm_state;


#define Is_Exposing()                   ((hv_state[0] == HVPS_SM_ID_EXPOSURING) || (hv_state[0] == HPVS_SM_ID_CAL_EXPOSURING) || (hv_state[0] == HVPS_SM_ID_TRAIN_EXPOSURING)||(hv_state[1] == HVPS_SM_ID_EXPOSURING) || (hv_state[1] == HPVS_SM_ID_CAL_EXPOSURING) || (hv_state[1] == HVPS_SM_ID_TRAIN_EXPOSURING))

#define Is_FaultState()                 (hv_state[0] == HVPS_SM_ID_FAULT||hv_state[1] == HVPS_SM_ID_FAULT)

#define Is_CalibrateMode()              (((hv_state[0] >= HPVS_SM_ID_CAL_PREPARE) && (hv_state[0] <= HPVS_SM_ID_CAL_END))|| ((hv_state[1] >= HPVS_SM_ID_CAL_PREPARE) && (hv_state[1] <= HPVS_SM_ID_CAL_END)))

#define Is_CTMode()                     (((int)hv_state[0] >= HVPS_SM_ID_IDLE) && (int)(hv_state[0] <= HVPS_SM_ID_EXPO_END) && ((int)hv_state[1] >= HVPS_SM_ID_IDLE) && (int)(hv_state[1] <= HVPS_SM_ID_EXPO_END))

#define Is_DebugMode()                  (((hv_state[0] >= HVPS_SM_ID_TRAIN_IDLE) && (hv_state[0] <= HVPS_SM_ID_TRAIN_END))||((hv_state[1] >= HVPS_SM_ID_TRAIN_IDLE) && (hv_state[1] <= HVPS_SM_ID_TRAIN_END)))



typedef enum
{
    XRAY_MODE_S_CONTINUOUS = 0x00,
    XRAY_MODE_S_PULSE      = 0x01,
    XRAY_MODE_D_CONTINUOUS = 0x02,
    XRAY_MODE_D_PULSE      = 0x03,
} xray_mode;


typedef struct
{
    /* input from MCU: EXIT IO */
    uint8_t     enable[XRAY_NUMS];         //使能信号
    uint8_t     expo[XRAY_NUMS];           //曝光信号
    uint8_t     interlock;      //互锁信号
    uint8_t     hv_vol_fault;   //高压过压故障
    uint8_t     hv_curr_fault;  //高压过流故障

    uint8_t     filament_on[XRAY_NUMS];    //灯丝开关

    xray_mode   xrayMode;

    uint16_t xray_current;  //当前工作射源

    uint16_t xray_switch_counter;    //切换射源过程计数，需要计数达到后才能稳定工作

} cmd_control_data;


typedef struct
{
    /* output to MCU*/
    uint8_t reay_signal;    //准备就绪
    uint8_t xray_on;        //射源出信号
    uint8_t fault;          //故障

    /* output to HV*/
    uint8_t hv_en;          //高压使能
    uint8_t mcu_lock;       //高压互锁
    uint8_t reset;          //故障复位

    int8_t  tube_vol[XRAY_NUMS];        //串口配置的管电压
    float   tube_curr[XRAY_NUMS];       //串口配置的管电流
    uint8_t tube_curr_index[XRAY_NUMS]; //管电流查表索引

    float   tube_vol_realtime[XRAY_NUMS]; //管电压基准实时配置值
    float   tube_vol_step[XRAY_NUMS];       //管电压基准上升步长

    float   fila_ref_target[XRAY_NUMS];    //灯丝基准目标值
    float   fila_ref_realtime[XRAY_NUMS];  //灯丝基准实时配置
    float   fila_ref_step[XRAY_NUMS];      //灯丝基准上升步长

    /* output to filament*/
    uint8_t filament_en[XRAY_NUMS];    //灯丝使能

    /* other data */
    uint32_t expo_count[XRAY_NUMS];         //射源单次曝光计时
    uint32_t expo_count_total[XRAY_NUMS];   //射源总曝光计时
    uint32_t expo_time_expect[XRAY_NUMS];   //曝光时间设置
    uint32_t fila_protect_cnt[XRAY_NUMS];   //灯丝开启未曝光计数

} xray_config_data;

//电流查表
typedef struct
{
    uint32_t    expo_times_total;  //总曝光时间
    uint32_t    expo_count_total;  //总曝光次数
    uint32_t    rising_time;       //管电压上升时间
    float       currValue[FILAMENT_CURRENT_TABLE_ORDER];      //电流表值
    uint32_t    currRef[FILAMENT_CURRENT_TABLE_ORDER];        //脉冲电流基准表
    uint32_t    currRef_c[FILAMENT_CURRENT_TABLE_ORDER];      //连续电流基准表
} xray_parament_table;

/* ADC2:4\5\11\12  adc3:14\2\4\6 */
typedef struct
{
    /* ADC3 */
    float tube_vol_p_value;      /*正管电压*/
    float tube_vol_n_value;      /*负管电压*/
    float tube_curr_value;       /* 管电流*/
    float temp_oil_value;        /*油箱温度*/

    /* ADC2 */
    float power_24v_value;       /* 24V供电*/
    float temp_sink_value;       /*散热器温度*/
    float filament_vol_value;    /*灯丝电压 */
    float filament_curr_value;   /* 灯丝电流 */

    float oil_temp;              /*油箱温度转换值*/

} adc_sampled_value;

//extern volatile xray_debug_data debug_data;
//各种保护值
typedef struct
{
    /*最大值和最小值保护*/
    float tube_vol_max_protected;        /*管电压*/
    float tube_vol_min_protected;

    float tube_curr_max_protected;       /*管电流*/
    float tube_curr_min_protected;
    float tube_curr_peak_protected;      /*瞬时管电流*/

    float temp_oil_max_protected;        /*油箱温度*/
    float temp_oil_min_protected;
    float temp_oil_warning;
    float temp_oil_recovery;

    float power_24v_max_protected;       //24v供电
    float power_24v_min_protected;
    float temp_sink_max_protected;       /*散热器温度*/
    float temp_sink_min_protected;
    float filament_vol_max_protected;    /*灯丝电压 */
    float filament_vol_min_protected;
    float filament_curr_max_protected;   /*灯丝电流*/
    float filament_curr_min_protected;

    /*配置门限*/
    float tube_vol_max_config;        /*管电压*/
    float tube_vol_min_config;

    float tube_curr_max_config;       /*管电流*/
    float tube_curr_min_config;

    int16_t expo_time_max;               /*设置曝光时间最大值*/
    int16_t expo_time_min;               /*最小值*/

    uint32_t expo_time_limit;

    uint32_t fila_protect_cnt_max;       /*灯丝开启未曝光计数*/
} xray_parament_range;

typedef struct
{
    uint32_t timmer_count[XRAY_NUMS];    /* y2.5s */

    /*各种告警限时*/
    uint32_t interLock_count;
    uint32_t pwr_24_overCount;
    uint32_t pwr_24_underCount;
    uint32_t tube_kv_overCount;
    uint32_t tube_kv_underCount;
    uint32_t tube_mA_overCount;
    uint32_t tube_mA_peak_overCount;
    uint32_t tube_mA_underCount;
    uint32_t fila_vol_overCount;
    uint32_t fila_vol_underCount;
    uint32_t fila_curr_overCount;
    uint32_t fila_curr_underCount;
    uint32_t sink_temp_errCount;
    uint32_t oil_temp_errCount;
    uint32_t oil_temp_warnCount;

    uint32_t tube_vol_broken_count;
    uint32_t tube_curr_broken_count;

    uint32_t tube_strike_count;     /*打火定时器周期计数*/
    uint32_t tube_strike_times;     /*打火次数，5次为打火故障 */
    uint32_t oil_strike_count;      /*油箱周期计数 */
    uint32_t oil_strike_times;      /*油箱次数*/

    uint32_t isCheckAvailable;   /*管电压和管电流达到稳定后可以开始检测，稳定时间根据上升时间确定*/
		
		uint32_t hv_hardware_count;
} xray_running_data;

extern volatile xray_running_data xray_data;

extern volatile xray_parament_range para_range;

extern volatile xray_parament_table parm_table[XRAY_NUMS];

extern volatile xray_config_data config_data;

extern volatile cmd_control_data ctrl_data;

extern volatile hvps_sm_state hv_state[XRAY_NUMS];

extern volatile adc_sampled_value sampled_data;

extern volatile adc_sampled_value sampled_data_last;

;

hvps_sm_state get_hv_state(uint16_t n);
void set_hv_state(hvps_sm_state state, uint16_t n);

void config_filament0_ref_slop(void);
void flash_table_init(void);
void save_parament_to_flash(void);
void Set_PWM_CMP(void);
void ct_task(void);

void xray_system_disable(void);

void config_filamentRef(uint16_t n);
void disable_hvref(void);
void disable_filamentref(uint16_t n);
void config_filament_ref_slop(uint16_t n);
void config_hvref_slope(uint16_t n);

#endif

