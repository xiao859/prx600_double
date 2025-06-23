#ifndef __STATE_MACHINE_H
#define __STATE_MACHINE_H

typedef enum {
    SYS_IDLE = 0,       //空闲
    SYS_EXPOSURE_A,     // A源曝光中
    SYS_EXPOSURE_B,     // B源曝光中
    SYS_FINISHED,       // 曝光完成
    SYS_FAULT           // 故障状态
} SystemState;

extern SystemState g_state;

void StateMachine_Init(void);   // 状态机初始化
void StateMachine_Run(void);    // 状态机运行
#endif

