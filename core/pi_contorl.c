
#include "pi_control.h"
#include "main.h"
#include "comm_string.h"
#include "comm_protocol.h"
#include "math.h"
//#include "test.h"
#include "calibrate.h"
#include "exposure.h"

User_PID   user_pid;
PARAM_PID  param_pid;

void pid_Init(float target, uint32_t ref_init, uint8_t isPulseMode)
{
    if (isPulseMode) {
	    user_pid.Kp = 25;
	    user_pid.Ti = 1;
    } else {
        user_pid.Kp = 4;
	    user_pid.Ti = 0.001;
    }

	user_pid.currTarget = target;

	param_pid.Error     = 0;
    param_pid.lastError     = 0;
	param_pid.integral  = 0;
	param_pid.Out_pid   = 0;
    param_pid.coeff     = 1;
    param_pid.config_ref = ref_init;

    return;
}

uint32_t test_flag = 0;
/* 管电流闭环调节：单位0.1mA
 * 管电流-灯丝电源基准闭环功能，软启功能
 */
void tube_current_piControl(uint8_t conflag)
{
    // user_pid.currTarget = 5.0;
    param_pid.Error = user_pid.currTarget - user_pid.currValue;
    // if (param_pid.Error < 0.05 && param_pid.Error > -0.05) {
    //     test_flag = 1;
    //     return;
    // } else {
    //     test_flag = 0;
    // }
    // if (fabsf(param_pid.Error) < 0.1) {
    //     param_pid.integral  += param_pid.Error;
    // } else {
    //     param_pid.integral  += 0;
    // }
    /* 积分限幅 */
    // param_pid.integral  += param_pid.Error;


    // param_pid.integral = MAX(MIN(param_pid.integral, 26), -26);

    param_pid.Out_pid = user_pid.Kp * param_pid.Error + user_pid.Ti * param_pid.integral;

    // param_pid.Out_pid = MAX(MIN(param_pid.Out_pid, 30.0), 0);

    /* 输出限幅 */

    // uint32_t fila_vol_ref = parm_table.currRef[config_data.tube_curr_index];
    // float fila_vol_ref = 1737;
    float pid_value;
    pid_value = param_pid.Out_pid;
    if (conflag == 1) {
        if (Is_ContinuousMode_CT()) pid_value = MAX(MIN(param_pid.Out_pid, 1), -1);
    } else {
        pid_value = param_pid.Out_pid;
    }
    // if (param_pid.Error < 0.1 && param_pid.Error > -0.1) {
    //     pid_value = 0;
    //     param_pid.integral = 0;
    // }

    if (param_pid.Out_pid > 0.5f && param_pid.Out_pid < 1.0f) pid_value = 1;
    if (param_pid.Out_pid < -0.5f && param_pid.Out_pid > -1.0f) pid_value = -1;
    param_pid.config_ref = (uint32_t)(param_pid.config_ref + pid_value);

    /* 限幅 */
    param_pid.config_ref = MAX(MIN(param_pid.config_ref, 2450), 1000);

    if (param_pid.Error < 0.05f && param_pid.Error > -0.05f) param_pid.config_ref = param_pid.config_ref + 1;

    // HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 2050);
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, param_pid.config_ref);

    return;
}

void tube_current_piControl_2(uint8_t conflag)
{
    // user_pid.currTarget = 5.0;
    param_pid.Error = user_pid.currTarget - user_pid.currValue;

    /* 积分限幅 */

    param_pid.Out_pid = user_pid.Kp * (param_pid.Error - param_pid.lastError) + user_pid.Ti * param_pid.Error;

    /* 输出限幅 */
    float pid_value;
    pid_value = param_pid.Out_pid;
    if (conflag == 1) {
        if (Is_ContinuousMode_CT()) pid_value = MAX(MIN(param_pid.Out_pid, 1), -1);
    } else {
        pid_value = param_pid.Out_pid;
    }
    if (param_pid.Error < 0.1f && param_pid.Error > -0.1f) {
        pid_value = 0;
    }

    param_pid.config_ref = (uint32_t)(param_pid.config_ref + pid_value);
    // debug_tx3("闭环值: %f, %d, %f, %f\n",
    //     param_pid.Out_pid, param_pid.config_ref, user_pid.currValue, param_pid.integral);

    /* 限幅 */
    param_pid.config_ref = MAX(MIN(param_pid.config_ref, 2450), 1000);

    param_pid.lastError = param_pid.Error;

    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, param_pid.config_ref);

    return;
}

