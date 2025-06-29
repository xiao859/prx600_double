#include "lamp.h"

struct LAMP_CONTROL_REGS mLamp_Control_Regs[XRAY_NUMS];
void Lamp_Control_Regs_Init()
{
    uint16_t i/*,j*/;
    for(i=0;i<XRAY_NUMS;i++)
    {
        // 设置灯丝状态机为初始化状态
        mLamp_Control_Regs[i].mHVPS_Lamp_State = HVPS_LAMP_SM_ID_INIT;
        mLamp_Control_Regs[i].Buck_On_flag = 0;
        mLamp_Control_Regs[i].steady_flag = 0;
        mLamp_Control_Regs[i].Buck_duty = LAMP_DUTY_MIN;
        mLamp_Control_Regs[i].counter = 0.0f;
    }


}

void Lamp_Current_Set(float D,uint16_t n)
{
        mLamp_Control_Regs[n].LoopOut=D>MAX_INTE?MAX_INTE:D;
        mLamp_Control_Regs[n].LoopOut=D<MIN_INTE?MIN_INTE:D;
}

// 关灯丝Buck
void Lamp_Buck_Off(uint16_t n)
{
    if(n<XRAY_NUMS)  // 灯丝设定未超界
    {
    mLamp_Control_Regs[n].mHVPS_Lamp_State = HVPS_LAMP_SM_ID_INIT;   // 设置成初始状态
    mLamp_Control_Regs[n].Buck_flag =0;
    mLamp_Control_Regs[n].Buck_On_flag = 0;
    }
}

// 开灯丝Buck
void Lamp_Buck_On(uint16_t n)
{
    if(n<XRAY_NUMS && mLamp_Control_Regs[n].mHVPS_Lamp_State == HVPS_LAMP_SM_ID_INIT)  // 灯丝设定未超界，并且灯丝Buck未开启
    {
        mLamp_Control_Regs[n].Buck_On_flag = 1;
        mLamp_Control_Regs[n].Buck_flag =1;
    }
}

void Lamp_SM_Control(uint16_t n)
{
    switch(mLamp_Control_Regs[n].mHVPS_Lamp_State)
    {
        case HVPS_LAMP_SM_ID_INIT:
            if(mLamp_Control_Regs[n].Buck_duty > LAMP_DUTY_MIN)
                mLamp_Control_Regs[n].Buck_duty -= LAMP_DUTY_STEP;
            else if(mLamp_Control_Regs[n].Buck_On_flag == 1)
            {
                mLamp_Control_Regs[n].mHVPS_Lamp_State = HVPS_LAMP_SM_ID_PREPARE;
                mLamp_Control_Regs[n].counter = 0.0;
                mLamp_Control_Regs[n].steady_flag = 0;
            }
            break;

        case HVPS_LAMP_SM_ID_PREPARE:
            if(mLamp_Control_Regs[n].Buck_On_flag == 0)
                mLamp_Control_Regs[n].mHVPS_Lamp_State = HVPS_LAMP_SM_ID_INIT;
            else
            {
                // 根据通道选择控制方式
                if(n == 0) {
                    // 灯丝0：DA 输出灯丝电流
										config_filament0_ref_slop();
                } else {
                    // 灯丝1：PWM控制
                    if(mLamp_Control_Regs[n].Buck_duty < LAMP_DUTY_INIT)
                        mLamp_Control_Regs[n].Buck_duty += LAMP_DUTY_STEP;
                    if(mLamp_Control_Regs[n].Buck_duty > LAMP_DUTY_INIT)
                        mLamp_Control_Regs[n].Buck_duty -= LAMP_DUTY_STEP;
                }

                if(mLamp_Control_Regs[n].counter < 5)
                    mLamp_Control_Regs[n].counter++;
                else
                {
                    mLamp_Control_Regs[n].counter = 5;
                    mLamp_Control_Regs[n].steady_flag = 1;

                    if(get_hv_state(n) > HVPS_SM_ID_IDLE)
                    {
                        mLamp_Control_Regs[n].counter = 0;
                        mLamp_Control_Regs[n].mHVPS_Lamp_State = HVPS_LAMP_SM_ID_TRANSIENT;
                    }
                }
            }
            break;

        case HVPS_LAMP_SM_ID_TRANSIENT:
            if(mLamp_Control_Regs[n].Buck_On_flag == 0)
                mLamp_Control_Regs[n].mHVPS_Lamp_State = HVPS_LAMP_SM_ID_INIT;
            else
            {
                if(n == 1) {
                    // 灯丝1仍使用PWM控制
                    if(mLamp_Control_Regs[n].Buck_duty < mLamp_Control_Regs[n].LoopOut)
                        mLamp_Control_Regs[n].Buck_duty += LAMP_DUTY_STEP;
                    if(mLamp_Control_Regs[n].Buck_duty > mLamp_Control_Regs[n].LoopOut)
                        mLamp_Control_Regs[n].Buck_duty -= LAMP_DUTY_STEP;
                }
            }
            break;

        default:
            mLamp_Control_Regs[n].mHVPS_Lamp_State = HVPS_LAMP_SM_ID_INIT;
            break;
    }
}






