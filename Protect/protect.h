#ifndef __PROTECT_H
#define __PROTECT_H

#include "ct_exposure.h"
#include "app_uart.h"
#include "comm_string.h"

#define OVER_RANGE_TIME_LIMIT           5000
#define FAST_PROTECT_TIME_RANGE         40  //10khz 4ms
#define STRIKE_TIEMS_RANGE              5

#define Is_System_Without_Fault()       ((mHVPS_Fault.FAULT_REG1.value == 0) && \
                                         (mHVPS_Fault.FAULT_REG2.value == 0) && \
                                         (mHVPS_Fault.FAULT_REG3.value == 0) && \
                                         (mHVPS_Fault.FAULT_REG4.value == 0) && \
                                         (mHVPS_Fault.FAULT_REG5.value == 0) && \
                                         (mHVPS_Fault.FAULT_REG6.value == 0))

enum HVPS_FAULT_ID
{
    HVPS_FAULT_ID_OK = 0x00,            //无故障
    HVPS_FAULT_ID_INIT_FAIL,            //设备初始化失败
    HVPS_FAULT_ID_ARM_INIT_FAIL,        //ARM初始化失败

    HVPS_FAULT_ID_KV_BEFORE = 0x20,         //曝光前采样到KV
    HVPS_FAULT_ID_KV_NONE,                  //曝光后未采样KV
    HVPS_FAULT_ID_MA_NONE,                  //曝光后未采样到MA
    HVPS_FAULT_ID_KV_OVER,                  // KV过高
    HVPS_FAULT_ID_KV_UNDER,                 // KV过低
    HVPS_FAULT_ID_MA_OVER,                  // MA过高
    HVPS_FAULT_ID_MA_UNDER,                 // MA过低
    HVPS_FAULT_ID_LAMP_OC,                  // 灯丝电流过高
    HVPS_FAULT_ID_LAMP_UC,                  // 灯丝电流过低
    HVPS_FAULT_ID_V24_OV,                   // 24V电压过高
    HVPS_FAULT_ID_V24_UV,                   // 24V电压过低
    HVPS_FAULT_ID_MA_PEAK_OVER,             // MA峰值瞬间过流

    HVPS_FAULT_ID_READY_OVERTIME = 0x30,        //准备超时，enable置位后10s没有检测到Exposure
    HVPS_FAULT_ID_EXPO_OVERTIME,                //曝光超时
    HVPS_FAULT_ID_BUCK_OPC,                     // Buck峰值电流过流
    HVPS_FAULT_ID_BUCK_OV,                      // Buck峰值电压过压
    HVPS_FAULT_ID_LLC_OV,                       // LLC峰值电压过压
    HVPS_FAULT_ID_LLC_OC,                       // LLC峰值电流过流
    HVPS_FAULT_ID_CLOSELOOP_MA,                 // 闭环管电流控制失效
    HVPS_FAULT_ID_CLOSELOOP_KV,                 // 闭环管电压控制失效

    HVPS_FAULT_ID_ARC1 = 0xA0,
    HVPS_FAULT_ID_ARC2,                        // 闭环管电压控制失效
    HVPS_FAULT_ID_LAMP2_UV,
    HVPS_FAULT_ID_current_broken1,
    HVPS_FAULT_ID_HV_broken1,
    HVPS_FAULT_ID_current_broken2,
    HVPS_FAULT_ID_HV_broken2,                  //校准数据丢失
    HVPS_FAULT_ID_LAMP1_OV,
    HVPS_FAULT_ID_LAMP2_OV,
    HVPS_FAULT_ID_lamp_wait_overtime,
    HVPS_FAULT_ID_TRAIN = 0xB0,                 //训管异常
    HVPS_FAULT_ID_UPDATE = 0xC0                 //升级异常
};

typedef union
{
    uint16_t    value;
    struct  HVPS_FAULT_GROUP1_BITS
    {
        uint16_t NO_FAULT: 1;
        uint16_t HV_HARDW_FAULT: 1;
        uint16_t DSP_INIT_FAIL: 1;
        uint16_t PFC_OV: 1;
        uint16_t PFC_UV: 1;
        uint16_t OIL_OT1: 1;
        uint16_t INTERLOCK1: 1;
        uint16_t SCI_LENGTH: 1;
        uint16_t SCI_CRC: 1;
        uint16_t OIL_TEMP1: 1;
        uint16_t HEATSINK_TEMP: 1;
        uint16_t PWR_24V_OV: 1;
        uint16_t PWR_24V_UV: 1;
        uint16_t OIL_TEMP2: 1;
        uint16_t CONFLICT: 1;
        uint16_t LAMP1_UV: 1;
    } bit;
} HVPS_FAULT_GROUP1_REG;

typedef union
{
    uint16_t value;
    struct HVPS_FAULT_GROUP2_BITS
    {
        uint16_t KV1_BEFORE: 1;
        uint16_t KV1_NONE: 1;
        uint16_t MA1_NONE: 1;
        uint16_t KV1_OVER: 1;
        uint16_t KV1_UNDER: 1;
        uint16_t MA1_OVER: 1;
        uint16_t MA1_UNDER: 1;
        uint16_t LAMP1_OC: 1;
        uint16_t LAMP1_UC: 1;
        uint16_t V24_OV: 1;
        uint16_t V24_UV: 1;
        uint16_t MA_PEAK_OVER: 1;
        uint16_t KV2_BEFORE: 1;
        uint16_t KV2_NONE: 1;
        uint16_t MA2_NONE: 1;
        uint16_t KV2_OVER: 1;
    } bit;
} HVPS_FAULT_GROUP2_REG;

typedef union
{
    uint16_t value;
    struct HVPS_FAULT_GROUP3_BITS
    {
        uint16_t READY1_OVERTIME: 1;
        uint16_t EXPO1_OVERTIME: 1;
        uint16_t BUCK_OPC_A: 1;
        uint16_t BUCK_OV: 1;
        uint16_t LLC_OV: 1;
        uint16_t LLC1_OC: 1;
        uint16_t CLOSELOOP1: 1;
        uint16_t CLOSELOOP2: 1;
        uint16_t KV2_UNDER: 1;
        uint16_t MA2_OVER: 1;
        uint16_t MA2_UNDER: 1;
        uint16_t LAMP2_OC: 1;
        uint16_t LAMP2_UC: 1;
        uint16_t READY2_OVERTIME: 1;
        uint16_t EXPO2_OVERTIME: 1;
        uint16_t LLC2_OC: 1;
    } bit;
} HVPS_FAULT_GROUP3_REG;

typedef union
{
    uint16_t value;
    struct HVPS_FAULT_GROUP4_BITS
    {
        uint16_t ARC1: 1;
        uint16_t ARC2: 1;
        uint16_t LAMP2_UV: 1;
        uint16_t current_broken1: 1;
        uint16_t HV_broken1: 1;
        uint16_t current_broken2: 1;
        uint16_t HV_broken2: 1;
        uint16_t LAMP1_OV: 1;
        uint16_t LAMP2_OV: 1;
        uint16_t lamp_wait_overtime: 1;
        uint16_t VOL_CURR_OV: 1;
        uint16_t rsv: 5;
    }  bit;
} HVPS_FAULT_GROUP4_REG;

typedef union
{
    uint16_t value;
    struct HVPS_FAULT_GROUP5_BITS
    {
        uint16_t TRAIN1: 1;
        uint16_t TRAIN2: 1;
				uint16_t EXPO2_OVERTIME: 1;
        uint16_t rsv: 13;
    }  bit;
} HVPS_FAULT_GROUP5_REG;

typedef union
{
    uint16_t    value;
    struct HVPS_FAULT_GROUP6_BITS
    {
        uint16_t UPDATE: 1;
        uint16_t rsvd1: 15;
    } bit;
} HVPS_FAULT_GROUP6_REG;


typedef struct
{
    HVPS_FAULT_GROUP1_REG FAULT_REG1;
    HVPS_FAULT_GROUP2_REG FAULT_REG2;
    HVPS_FAULT_GROUP3_REG FAULT_REG3;
    HVPS_FAULT_GROUP4_REG FAULT_REG4;
    HVPS_FAULT_GROUP5_REG FAULT_REG5;
    HVPS_FAULT_GROUP6_REG FAULT_REG6;
} HVPS_FAULT_REGS;
extern HVPS_FAULT_REGS mHVPS_Fault;

void InqHVPSFault(message_protocol *msg);
void UpdateVar_CheckFaultFast(void);
void Protect_Check_Slow(void);

void transform_adc_values(void);
#endif

