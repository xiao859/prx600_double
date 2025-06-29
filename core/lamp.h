#ifndef __LAMP_H__
#define __LAMP_H__
#include "stdint.h"
#include "HV_exposure.h"

// 24V控制电采样系数
#define V24V_COEF       0.0073        // 24V控制电压采样系数3/4096/10
#define MIN_INTE        0.01
#define MAX_INTE        0.535

#define LAMP_DUTY_STEP  0.0001f           // 占空比变化步长
#define LAMP_DUTY_MIN   0.02f          // 最小占空比，必须比LAMP_DUTY_STEP大
#define LAMP_DUTY_INIT  0.455f


//                      灯丝控制部分
//
#define LAMP_PREHEAT_COUNT   640000//灯丝预热计数
#define ENABLE_CYCLE_COUNT   656000//使能周期计数
#define ENABLE_EFFECT_COUNT  256000
#define EXPO_DELAY_COUNT     128000 //曝光延时计数
#define EXPO_CYCLE_COUNT     9600//曝光周期计数
#define EXPO_EFFECT_COUNT    1200//曝光有效计数
#define EXPO_DEAD_COUNT      5400//曝光死区计数
#define EXPO2_EFFECT_COUNT   6600
#define STORAGE_TIME_COUNT   80000

#define LAMP_IDLE_CURRENT   2800        // 灯丝待机电流
#define IDLE_READY_COUNT    96000      // 灯丝从待机到稳态所需周期数
// 灯丝输出电压电流采样系数
#define VLAMPBUCK1_COEF   0.0073f        // 1号灯丝buck电压采样系数3/4096*10
#define ILAMPBUCK1_COEF   0.00029297f    // 1号灯丝buck电流采样系数3/2.5/4096

#define VLAMPBUCK2_COEF   0.0073f        // 2号灯丝buck电压采样系数3/4096*10
#define ILAMPBUCK2_COEF   0.00029297f    // 2号灯丝buck电流采样系数3/2.5/4096


#define CYCLES_PER_MS   80.0f           // 每毫秒控制周期数
#define LAMP_BAUDRATE   1200.0f         // 灯丝波特率

#define LAMP_HIGH_DUTY  0.94f           // 高占空比
#define LAMP_LOW_DUTY   0.88f            // 低占空比
#define LAMP_DUTY_STEP  0.0001f           // 占空比变化步长
#define LAMP_DUTY_MIN   0.02f          // 最小占空比，必须比LAMP_DUTY_STEP大
// 占空比切所需的控制周期数
#define LAMP_CYCLES_PER_SWITCH  ((LAMP_HIGH_DUTY-LAMP_LOW_DUTY)/LAMP_DUTY_STEP)
// 占空比切换完成后，每BIT持续的控制周期数
#define LAMP_CYCLES_PER_BIT     ((CYCLES_PER_MS*1000.0)/LAMP_BAUDRATE-(LAMP_HIGH_DUTY-LAMP_LOW_DUTY)/LAMP_DUTY_STEP)
// 启动到稳定所需的周期数
#define LAMP_CYCLES_TO_STEADY   LAMP_CYCLES_PER_BIT*12.0f

#define LAMP_SCI_TYPE1_BITS   18        // 灯丝通信设定帧位数，1个起始位，14个数据位，2个效验位，1个结束位
#define LAMP_SCI_TYPE2_BITS    7        // 灯丝通信控制帧位数，1个起始位，3个数据位，2个效验位，1个结束位

#define LAMP_SCI_VOL_SET    0
#define LAMP_SCI_CUR_SET    1

#define LAMP_SCI_ON    0b10
#define LAMP_SCI_OFF   0b00

//lamp state ID
enum HVPS_LAMP_SM_ID {
    HVPS_LAMP_SM_ID_INIT = 0x00,        //lamp init
    HVPS_LAMP_SM_ID_PREPARE ,        		//lamp start LAMP_DUTY_STEP,???LAMP_HIGH_DUTY
    HVPS_LAMP_SM_ID_TRANSIENT,          // 
    HVPS_LAMP_SM_ID_WAIT,               // 
};

#define LAMP_CURRENT_CALC_ORDER 10
struct LAMP_CONTROL_REGS {
    enum HVPS_LAMP_SM_ID mHVPS_Lamp_State;          //lamp state
    uint16_t Buck_On_flag;
    uint16_t Buck_flag;
    uint16_t steady_flag;
    float LoopOut;
    float Lamp_Current;
    float Err;
    float Buck_duty;                                // Buck
    float counter;                                  
    float coef1[LAMP_CURRENT_CALC_ORDER];           //mA,kV
    float coef2;                                    //
    float mLamp_Current;                            // mA,kV
    float mLamp_Kp;
    float mLamp_Ki;

};

extern struct LAMP_CONTROL_REGS mLamp_Control_Regs[XRAY_NUMS];

void Lamp_Control_Regs_Init(void);
void Lamp_Voltage_Set(int16_t v,uint16_t n);           //灯丝电压设置
void Lamp_Current_Set(float D,uint16_t n);           //灯丝电流设置 
void Lamp_Buck_On(uint16_t n);            // 灯丝状态机如果在init状态，转到下一个状态
void Lamp_Buck_Off(uint16_t n);           // 灯丝状态机回到HVPS_LAMP_SM_ID_INIT
void Lamp_SM_Control(uint16_t n);                 //灯丝状态机控制
void Lamp_Current_Cal(uint16_t n,float v,float i);       //根据管电压和管电流，开环计算灯丝电流
uint32_t get_filamentRef(float tube_current,uint16_t n); //根据管电压和管电流，查找灯丝电流
void Lamp_Current_Lookup_Update(uint16_t n, float c, float c_l); //根据管电压和管电流，更新查找表

#endif

