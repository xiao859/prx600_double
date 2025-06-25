
/*
 * 文件名：state_machine.c
 * 功能：PRX600 状态机控制逻辑
 * 编码：GB2312
 */

#include "state_machine.h"
#include "exposure.h"
#include "protect.h"

SystemState g_state = SYS_IDLE;

void StateMachine_Init(void)
{
    g_state = SYS_IDLE;
}

void StateMachine_Run(void)
{
    switch (g_state)
    {
//        case SYS_IDLE:
//            if (Protect_GetTrigger(SOURCE_A)) {
//                Exposure_Start(SOURCE_A);
//                g_state = SYS_EXPOSURE_A;
//            }
//            else if (Protect_GetTrigger(SOURCE_B)) {
//                Exposure_Start(SOURCE_B);
//                g_state = SYS_EXPOSURE_B;
//            }
//            break;

//        case SYS_EXPOSURE_A:
//            if (Exposure_IsDone(0)) {
//                g_state = SYS_FINISHED;
//            }
//            break;

//        case SYS_EXPOSURE_B:
//            if (Exposure_IsDone(0)) {
//                g_state = SYS_FINISHED;
//            }
//            break;

//        case SYS_FINISHED:
//            g_state = SYS_IDLE;
//            break;

//        case SYS_FAULT:
//            // 故障状态保持，等待系统复位或上位机清除
//            break;

        default:
            g_state = SYS_IDLE;
            break;
    }
}


