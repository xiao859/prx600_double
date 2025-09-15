#ifndef APP_FUN_H_
#define APP_FUN_H_


#include <stdint.h>
#include "app_uart.h"
#include "ct_exposure.h"

#define SETUP_SUCCESS               0x00
#define SETUP_SM_ERROR              0x01
#define SETUP_OUT_LIMIT             0x02

#define HVPS_MODE_S_CONTINUOUS     0x00
#define HVPS_MODE_S_PULSE          0x01
#define HVPS_MODE_D_CONTINUOUS     0x02
#define HVPS_MODE_D_PULSE          0x03

#define APP_FUNC_NUM                    55


typedef enum
{
    SCI_MSG_INQ_MODE = 0x00,
    SCI_MSG_INQ_XRAY1,
    SCI_MSG_INQ_XRAY2,
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
    SCI_MSG_INQ_XSOURCE_SW,//20

    SCI_MSG_SET_MODE = 0x20,
    SCI_MSG_SET_TUBE_V,
    SCI_MSG_SET_TUBE_I,
    SCI_MSG_SET_MAX_TIME,
    SCI_MSG_SET_EXP1_COUNTCLR,
    SCI_MSG_SET_EXP2_COUNTCLR,
    SCI_MSG_SET_EXP1_TIMECLR,
    SCI_MSG_SET_EXP2_TIMECLR,
		SCI_MSG_SET_NULL,
    SCI_MSG_SET_ENABLE ,//30

    SCI_MSG_CTRL_RST = 0x30,
    SCI_MSG_CTRL_CAL,
    SCI_MSG_CTRL_TRAIN,
    SCI_MSG_CTRL_UPDATE,
    SCI_MSG_CTRL_STORE_TABLE,
    SCI_MSG_CTRL_STORE_TABLE_INQ,
    SCI_MSG_CTRL_STORE_STATISTICS,
    SCI_MSG_LAMP_CONTROL,//38

    SCI_MSG_SET_PFCTHRESHOLD = 0x40,
    SCI_MSG_SET_24VTHRESHOLD,
    SCI_MSG_SET_KVMATHRESHOLD,
    SCI_MSG_SET_BUCKLLCTHRESHOLD,//42

    SCI_MSG_DEBUG_LAMP_I_SET = 0xA0,
    SCI_MSG_DEBUG_TUBE_VIDLE_SET,
    SCI_MSG_DEBUG_TUBE_VRISE_TIME_SET,
    SCI_MSG_DEBUG_ONLINE_PI,
    SCI_MSG_DEBUG_EXPO_CTRL,
		
    SCI_MSG_TEST_1 = 0xB0,
    SCI_MSG_TEST_2,
    SCI_MSG_TEST_3,
    SCI_MSG_TEST_4,//52
} SCI_MSG_ID;

typedef struct
{
    SCI_MSG_ID msgId;
    void  (*func_ptr)(message_protocol *msg);
} controler_cmd_funcs;
extern controler_cmd_funcs funcs[APP_FUNC_NUM];

typedef struct
{
    //硬件版本号
    uint16_t hw_ver_high : 8;
    uint16_t hw_ver_low  : 8;

    //固件版本号
    uint16_t sw_ver_high : 4;
    uint16_t sw_ver_mid  : 4;
    uint16_t sw_ver_low  : 8;

    //灯丝硬件版本号
    uint16_t fila_ver_high : 5;
    uint16_t fila_ver_mid1 : 4;
    uint16_t fila_ver_mid2 : 4;
    uint16_t fila_ver_low  : 3;

    //球管信息
    uint16_t tube_ver_high : 8;
    uint16_t tube_ver_low  : 8;

    //射源类型
    uint16_t xsrc_ver_high : 5;
    uint16_t xsrc_ver_mid  : 8;
    uint16_t xsrc_ver_low  : 3;
} xray_version;
extern volatile xray_version version;

void InqHVPS1LastVandC(message_protocol *msg);

void HVswversion(message_protocol *msg);

void HVhwversion(message_protocol *msg);

void lampswversion(message_protocol *msg);

void lamphwversion(message_protocol *msg);

void InqHVPSTemp(message_protocol *msg);

void InqHVPSExpo_Time1(message_protocol *msg);

void InqHVPSExpo_Time2(message_protocol *msg);

void InqHVPSExpo_Count1(message_protocol *msg);

void InqHVPSExpo_Count2(message_protocol *msg);

void SetHV1TubeVoltageandcurrent(message_protocol *msg);

void SetHV2TubeVoltageandcurrent(message_protocol *msg);

void SetHVTubeIdleVoltage(message_protocol *msg);

void SetHVLampCurrent(message_protocol *msg);

void SetHVTubeRisingTime(message_protocol *msg);

void SetOnlinePI(message_protocol *msg);

void FaultReset(message_protocol *msg);

void StoreLookupTable(message_protocol *msg);

void StoreLookupTableINQ(message_protocol *msg);

void SetHVPSMode(message_protocol *msg);

void StoreStatistics(message_protocol *msg);

void Setmaxexpotime(message_protocol *msg);

void Inqixay1HVPSCurrentset(message_protocol *msg);

void Inqixay2HVPSCurrentset(message_protocol *msg);

void Inqmaxtimeset(message_protocol *msg);

void Lampcontrol(message_protocol *msg);

void Autocalibra(message_protocol *msg);

void xsourcetype(message_protocol *msg);

void Inqautocalibra(message_protocol *msg);

void exp1countclr(message_protocol *msg);

void exp2countclr(message_protocol *msg);

void exp1timeclr(message_protocol *msg);

void exp2timeclr(message_protocol *msg);

void Setenable(message_protocol *msg);

void setpfcthreshold(message_protocol *msg);

void set24vthreshold(message_protocol *msg);

void setkvmathreshold(message_protocol *msg);

void setbuckllcthreshold(message_protocol *msg);

void test_func1(message_protocol *msg);

void test_func2(message_protocol *msg);

void test_func3(message_protocol *msg);

void test_func4(message_protocol *msg);

void invalid_cmd_reply(void);

void debug_expo_ctrl(message_protocol *msg);

void fun_null(message_protocol *msg);

void firmUpgrade(message_protocol *msg);

void cmd_parser(void);

#endif
