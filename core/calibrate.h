#ifndef _CALIBRATE_H
#define _CALIBRATE_H

#include "stdint.h"
#include "ct_exposure.h"


#define     CALI_HV_REF                     90

#define     CALI_SINGLE_CURR_TIME           2.4 //2.04


#define     CALI_PULSE_CURR_EXPO_TIME       15

#define     CALI_PULSE_SIGLE_CURR_PERIOD    120

#define     Is_PulseMode()                  ((cali_data.mode) == XRAY_MODE_D_PULSE)

#define     Is_ContinuousMode()             ((cali_data.mode) == XRAY_MODE_D_CONTINUOUS)


typedef struct {
    xray_mode   mode;                   /*校准模式*/
    uint8_t     curr_index;             /*校准阶段 */
    uint8_t     finished_flag;          /*结束标记*/

    float       tube_vol_realtime;
    float       tube_vol_step;

    uint32_t    timmer_count;           /*计时，用于冷却或控制曝光*/
    uint32_t    cycle_count;            /*计时，单毫安循环*/

    uint32_t    expoTime_expect;        /*设置的曝光时间，单个电流下，每个脉冲的持续时间*/
    uint32_t    coolTime_expect;        /*设置的冷却时间，单个电流下，每个脉冲的冷却时间*/
    uint32_t    expoCycle_perCurrent[XRAY_NUMS];   /*校准，单毫安曝光次数*/

    uint32_t    para_save_flag;
} xray_calibrate_data;
extern volatile xray_calibrate_data cali_data;


void calibrate_para_init(void);
void calibrate_task(void);


#endif


