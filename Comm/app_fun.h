#ifndef APP_FUN_H_
#define APP_FUN_H_

/* 1、头文件包含 */
#include "comm_protocol.h"
#include <stdint.h>
#include "app_uart.h"
//#include "test.h"

/* 2、宏定义 */
#define SETUP_SUCCESS               0x00
#define SETUP_SM_ERROR              0x01
#define SETUP_OUT_LIMIT             0x02

#define HVPS_MODE_S_CONTINUOUS     0x00
#define HVPS_MODE_S_PULSE          0x01
#define HVPS_MODE_D_CONTINUOUS     0x02
#define HVPS_MODE_D_PULSE          0x03

#define APP_FUNC_NUM                    50
/* 3、数据类型定义 */
/* 串口指令消息处理函数注册 */
typedef struct
{
    SCI_MSG_ID msgId;
    void  (*func_ptr)(message_protocol *msg);
} controler_cmd_funcs;
extern controler_cmd_funcs funcs[APP_FUNC_NUM];

/* 4、函数声明 */

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

void invalid_cmd_reply(void);

void debug_expo_ctrl(message_protocol *msg);

void fun_null(message_protocol *msg);

#endif
