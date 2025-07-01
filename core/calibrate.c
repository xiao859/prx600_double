///*
// *  calibrate.c
// *
// *  Created on: Mar 04, 2025
// *  Author: Administrator
// *
// */

#include "calibrate.h"
#include "lamp.h"
#include "HV_exposure.h"
#include "adc.h"
#include "time.h"
//#include "comm_protocol.h"
//#include "xray.h"
//#include "pi_control.h"
//#include "delay.h"
//#include "app_spi.h"
//#include "app_uart.h"

#define LAMP_PREHEAT_COUNT  640000//灯丝预热计数
#define ENABLE_CYCLE_COUNT  656000//使能周期计数
#define ENABLE_EFFECT_COUNT  256000
#define EXPO_DELAY_COUNT     128000 //曝光延时计数
#define EXPO_CYCLE_COUNT     9600//曝光周期计数
#define EXPO_EFFECT_COUNT    1200//曝光有效计数
#define EXPO_DEAD_COUNT    5400//曝光死区计数
#define EXPO2_EFFECT_COUNT   6600
#define STORAGE_TIME_COUNT   80000
volatile ctrl_calibr ctrl_calibr_data;
extern TIM_HandleTypeDef htim5;
void Autocalibrationcount()
{
    static uint16_t idex_c = 0 ;

    if (ctrl_calibr_data.calibraflag == 1)
    {
        // 打开两个灯丝的 Buck（灯丝1 仍为 PWM 控制）
        Lamp_Buck_On(0);
        Lamp_Buck_On(1);

        ctrl_calibr_data.calibr_lampctrl_count++;
        ctrl_calibr_data.calibr_expo1_count++;

        if (ctrl_calibr_data.calibr_lampctrl_count >= LAMP_PREHEAT_COUNT)
        {
            ctrl_calibr_data.calibr_lampctrl_count = LAMP_PREHEAT_COUNT;
            ctrl_calibr_data.calibr_enble_count++;

            if (ctrl_calibr_data.calibr_enble_count > ENABLE_CYCLE_COUNT)
            {
                ctrl_calibr_data.calibr_enble_count = 0;
                ctrl_calibr_data.calibr_enble_flag = 1;
                ctrl_calibr_data.calibr_expo1_count = 0;
            }
            else if (ctrl_calibr_data.calibr_enble_count > ENABLE_EFFECT_COUNT)
                ctrl_calibr_data.calibr_enble_flag = 0;
            else
                ctrl_calibr_data.calibr_enble_flag = 1;

            if (ctrl_calibr_data.calibr_expo1_count > EXPO_CYCLE_COUNT)
            {
                ctrl_calibr_data.calibr_expo1_flag = 1;
                ctrl_calibr_data.calibr_expo1_count = 0;
            }
            else if (ctrl_calibr_data.calibr_expo1_count > EXPO_EFFECT_COUNT)
                ctrl_calibr_data.calibr_expo1_flag = 0;
            else
                ctrl_calibr_data.calibr_expo1_flag = 1;

            if (ctrl_calibr_data.calibr_expo1_flag == 0)
            {
                ctrl_calibr_data.calibr_expo2_count++;
                if (ctrl_calibr_data.calibr_expo2_count > EXPO2_EFFECT_COUNT)
                {
                    ctrl_calibr_data.calibr_expo2_count = 0;
                    ctrl_calibr_data.calibr_expo2_flag = 0;
                }
                else if (ctrl_calibr_data.calibr_expo2_count > EXPO_DEAD_COUNT)
                    ctrl_calibr_data.calibr_expo2_flag = 1;
                else
                    ctrl_calibr_data.calibr_expo2_flag = 0;
            }
            else
            {
                ctrl_calibr_data.calibr_expo2_flag = 0;
                ctrl_calibr_data.calibr_expo2_count = 0;
            }

            if (ctrl_calibr_data.calibr_enble_flag == 1)
                ctrl_calibr_data.calibr_wait_count = 0;
            else
            {
                ctrl_calibr_data.calibr_wait_count++;
                if (ctrl_calibr_data.calibr_wait_count == STORAGE_TIME_COUNT && ctrl_calibr_data.calibr_enble_flag == 0)
                {
                    ctrl_calibr_data.store_flag = 1;
                    idex_c++;

                    if (idex_c > 9)
                    {
                        Lamp_Buck_Off(0);
                        Lamp_Buck_Off(1);
                        ctrl_calibr_data.calibraflag = 0;
                        ctrl_calibr_data.store_flag = 1;
                        idex_c = 0;
                    }

                    // 电流参考值更新

                    mLamp_Control_Regs[0].mLamp_Current = parm_table[0].currValue[idex_c];
                    mLamp_Control_Regs[1].mLamp_Current = parm_table[1].currValue[idex_c];
                }

                // 独立控制灯丝
                if (mLamp_Control_Regs[0].mHVPS_Lamp_State >= HVPS_LAMP_SM_ID_PREPARE && mLamp_Control_Regs[0].steady_flag == 1)
                    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, mLamp_Control_Regs[0].mLamp_Current);; // 灯丝0：DA输出

                if (mLamp_Control_Regs[1].mHVPS_Lamp_State >= HVPS_LAMP_SM_ID_PREPARE && mLamp_Control_Regs[1].steady_flag == 1)
                    Lamp_Current_Set(mLamp_Control_Regs[1].mLamp_Current, 1); // 灯丝1：PWM输出
            }
        }
    }
    else
    {
        idex_c = 0;
        ctrl_calibr_data.calibr_enble_flag = 0;
        ctrl_calibr_data.calibr_expo1_flag = 0;
        ctrl_calibr_data.calibr_expo2_flag = 0;
        ctrl_calibr_data.calibr_enble_count = 0;
        ctrl_calibr_data.calibr_expo1_count = 0;
        ctrl_calibr_data.calibr_expo2_count = 0;
        ctrl_calibr_data.calibr_lampctrl_count = 0;
    }

}

void Set_PWM_CMP()
{
    // 灯丝0：使用 DA 控制，跳过 PWM 设置
    if (ctrl_data.xray_current == 1)
        return;


    // 灯丝1：PWM 控制
    mLamp_Control_Regs[1].Buck_duty = mLamp_Control_Regs[1].Buck_duty > 0.9 ? 0.9 : mLamp_Control_Regs[1].Buck_duty;
    mLamp_Control_Regs[1].Buck_duty = mLamp_Control_Regs[1].Buck_duty < 0 ? 0 : mLamp_Control_Regs[1].Buck_duty;
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, (uint32_t)(mLamp_Control_Regs[1].Buck_duty * (float)500));//50k

}


