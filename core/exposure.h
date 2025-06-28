#ifndef __EXPOSURE_H
#define __EXPOSURE_H
#include "stdint.h"
#include "comm_protocol.h"


#include <stdint.h>


#define EXPO_MAXTIME_PROTECT        1000  

#define     Is_PulseMode_CT()           ((ctrl_data.xrayMode == XRAY_MODE_S_PULSE) || (ctrl_data.xrayMode == XRAY_MODE_D_PULSE))

#define     Is_ContinuousMode_CT()      ((ctrl_data.xrayMode== XRAY_MODE_D_CONTINUOUS) || (ctrl_data.xrayMode== XRAY_MODE_D_CONTINUOUS) )

//struct CONVERTER_VAR_STRUCT {
//    float hv_buck_in_v;
//    float hv_buck_out_v;
//    float hv_buck_out_c;

//    float hv_buck_in_v_coef;
//    float hv_buck_out_v_coef;
//    float hv_buck_out_c_coef;

//    float hv_buck_in_v_offset;
//    float hv_buck_out_v_offset;
//    float hv_buck_out_c_offset;

//    // ??LLC????(2?1)????1?LLC????
//    float hv_llc_pk;
//    float hv_llc_pk_coef;
//    float hv_llc_pk_offset;

//    // ????Buck????
//    float lv_buck_in_v;
//    float lv_buck_out_v[XRAY_NUMS];
//    float lv_buck_out_c[XRAY_NUMS];

//    float lv_buck_in_v_coef;
//    float lv_buck_out_v_coef[XRAY_NUMS];
//    float lv_buck_out_c_coef[XRAY_NUMS];

//    float lv_buck_in_v_offset;
//    float lv_buck_out_v_offset[XRAY_NUMS];
//    float lv_buck_out_c_offset[XRAY_NUMS];

//    // KV?????????????????
//    float hv_ipk[XRAY_NUMS];
//    float hv_iav_n[XRAY_NUMS];
//    float hv_hv_n[XRAY_NUMS];

//    float hv_ipk_coef[XRAY_NUMS];
//    float hv_iav_n_coef[XRAY_NUMS];
//    float hv_hv_n_coef[XRAY_NUMS];

//    float hv_ipk_offset[XRAY_NUMS];
//    float hv_iav_n_offset[XRAY_NUMS];
//    float hv_hv_n_offset[XRAY_NUMS];
//    // NTC??
//    float ntc[4];
//    float ntc_last[4];

//    // ????
//    float hv_out_v;       
//    float hv_out_c;

//    float hv_out_v_last[XRAY_NUMS];    
//    float hv_out_c_last[XRAY_NUMS];
//    float hv_out_v_check[XRAY_NUMS];
//    float hv_out_c_check[XRAY_NUMS];
//    float STEAY_last[4];

//    float hv_out_v_coef[XRAY_NUMS];
//    float hv_out_c_coef[XRAY_NUMS];

//    float hv_out_v_offset[XRAY_NUMS];
//    float hv_out_c_offset[XRAY_NUMS];


// // Converter PI coefficient, two groups
//    float KP1;
//    float KI1;
//    float KP2;
//    float KI2;
//    // Output Voltage and current reference
//    float hv_out_v_ref;
//    float hv_out_c_ref[XRAY_NUMS];

//    // Converter PWM duty ratio related
//    float Duty;
//    float Duty_Integral;
//    float DUTY_MAX;
//    float DUTY_MIN;

// //   XRAY_SET_RANGE hv_range;
//    // ????
//    uint16_t hv_arc_count[XRAY_NUMS];
//    uint32_t hv_arc_count_decrease[XRAY_NUMS];
//    uint16_t update_flag;   // ??????,??????,??????1~3,??????0,????0
//    uint16_t store_flag; // ??????,???????,??????1~3,????,??0,????0
//    uint16_t calibraflag;//????
//    uint16_t calibr_enble_flag;//????
//    uint16_t calibr_expo1_flag;
//    uint16_t calibr_expo2_flag;
//    uint32_t calibr_enble_count;
//    uint16_t calibr_expo1_count;
//    uint16_t calibr_expo2_count;
//    uint32_t calibr_lampctrl_count;
//    uint32_t calibr_wait_count;



//    uint32_t idle_ready_counter[XRAY_NUMS];  // ?IDLE?READY??????
//    uint32_t new_pulse_counter[XRAY_NUMS];     // ??????

//    uint16_t xray_mode;     // ????,0x00?0x01??????,0x02?0x03??????
//    uint16_t xray_current;  // 
//                            // 
//    uint16_t xray_switch_counter;    //
//};

typedef struct
{
    uint32_t timmer_count;   


    uint32_t interLock_count[XRAY_NUMS];
    uint32_t pwr_24_overCount[XRAY_NUMS];
    uint32_t pwr_24_underCount[XRAY_NUMS];
    uint32_t tube_kv_overCount[XRAY_NUMS];
    uint32_t tube_kv_underCount[XRAY_NUMS];
    uint32_t tube_mA_overCount[XRAY_NUMS];
    uint32_t tube_mA_peak_overCount[XRAY_NUMS];
    uint32_t tube_mA_underCount[XRAY_NUMS];
    uint32_t fila_vol_overCount[XRAY_NUMS];
    uint32_t fila_vol_underCount[XRAY_NUMS];
    uint32_t fila_curr_overCount[XRAY_NUMS];
    uint32_t fila_curr_underCount[XRAY_NUMS];
    uint32_t sink_temp_errCount[XRAY_NUMS];
    uint32_t oil_temp_errCount[XRAY_NUMS];
    uint32_t oil_temp_warnCount[XRAY_NUMS];

    uint32_t tube_vol_broken_count[XRAY_NUMS];
    uint32_t tube_curr_broken_count[XRAY_NUMS];

    uint32_t tube_strike_count[XRAY_NUMS];    
    uint32_t tube_strike_times[XRAY_NUMS];     
    uint32_t oil_strike_count;      
    uint32_t oil_strike_times;      

    uint32_t isCheckAvailable[XRAY_NUMS];

} xray_running_data;
extern volatile xray_running_data xray_data;

void ct_task(void);
void xray_CT_disable(uint16_t n);


typedef enum {
    SOURCE_A = 0,
    SOURCE_B
} ExposureSource;

void Exposure_Start(ExposureSource src);         //启动曝光流程
void Exposure_Stop(ExposureSource src);          //停止曝光
uint8_t Exposure_IsDone(uint32_t dummy);         //曝光是否完成
void Exposure_Tick_Handler(void);                //1ms定时器调度函数

#endif
