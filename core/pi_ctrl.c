#include "pi_ctl.h"
#include "ct_exposure.h"
#include "dac.h"
#include "tim.h"
#include "app_uart.h"

User_PID   user_pid;
PARAM_PID  param_pid;
User_PID_2   user_pid_2;

//    else  //pwm
//    {
//        param_pid.config_ref = (uint32_t)(param_pid.config_ref + pid_value);

//        param_pid.config_ref = MAX(MIN(param_pid.config_ref, 1550), 1000);
//        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, config_data.fila_ref_realtime[ct_source]);//max 1999
//    }

uint32_t fila_ref_offset[FILAMENT_CURRENT_TABLE_ORDER] = {
    /* 1   2   3   4   5   6   7    8   9   10   11   12 */
       2,  5,  5,  8,  10,  15, 15, 15, 20, 20,  20,  23
};		
		
		
		
void pid_Init(float target, uint32_t ref_init, uint8_t isPulseMode)
{
    if (isPulseMode)
    {
        user_pid.Kp = 25;
        user_pid.Ti = 1;
    }
    else
    {
        user_pid.Kp = 4;
        user_pid.Ti = 0.001;
    }

    user_pid.currTarget = target;

    param_pid.Error     = 0;
    param_pid.integral  = 0;
    param_pid.Out_pid   = 0;
    param_pid.coeff     = 1;
    param_pid.config_ref = ref_init;

    param_pid.pulse_count = 0;
    param_pid.ti_CycleCount = 0;
    param_pid.pi_flag = 0;
    param_pid.pulse_pi_flag = 0;
    param_pid.pi_fast_flag = 0;
    param_pid.threshold_offset = 0;
    param_pid.conu_start_ref = 0;

    return;
}

void pid_Init_2(float target, uint32_t ref_init, uint8_t isPulseMode,uint8_t n)
{
    user_pid_2.config_ref[n] = ref_init;
    user_pid_2.currTarget[n]  = target;
    user_pid_2.err[n]  = 0;
    user_pid_2.last_err[n]  = 0;

    user_pid_2.Kp[n] = 1;
    user_pid_2.Ki[n] = 1;

}

/* 管电流闭环调节：单位0.1mA
 * 管电流-灯丝电源基准闭环功能，软启功能
 */
void tube_current_piControl(uint8_t conflag,uint8_t n)
{
    param_pid.Error = user_pid.currTarget - user_pid.currValue;
    /* 积分限幅 */
    param_pid.integral  += param_pid.Error;

    param_pid.integral = MAX(MIN(param_pid.integral, 26), -26);

    param_pid.Out_pid = 0;
    if (param_pid.ki_flag == 1)
    {
        param_pid.Out_pid = user_pid.Kp * param_pid.Error + 0.1f * param_pid.integral;
        if ((param_pid.integral == 26 || param_pid.integral == -26))
        {
            param_pid.integral = 0;
            if (param_pid.config_ref == (param_pid.conu_start_ref + param_pid.threshold_offset))
            {
                param_pid.thresh_achieve_count++;
            }

            if (param_pid.thresh_achieve_count >= 10)
            {
                param_pid.threshold_offset++;
                param_pid.thresh_achieve_count = 0;
            }
        }
        else
        {
            param_pid.thresh_achieve_count = 0;
        }
    }

    float pid_value;
    pid_value = param_pid.Out_pid;
    if (pid_value < 0 && pid_value > -1) pid_value = 0;

    pid_value = MAX(MIN(param_pid.Out_pid, 1), -1);

    param_pid.config_ref = (uint32_t)(param_pid.config_ref + pid_value);

    if (param_pid.pulse_count == 5)
    {
        param_pid.config_ref = param_pid.config_ref + fila_ref_offset[config_data.tube_curr_index[n]];
        param_pid.pulse_count = 6;
        param_pid.conu_start_ref = param_pid.config_ref;
    }
		
		uint32_t max_ref = param_pid.conu_start_ref + param_pid.threshold_offset;

    /* 限幅 */
    if (Is_CalibrateMode())
    {
        param_pid.config_ref = MAX(MIN(param_pid.config_ref, 2850), 1000);
    }
    else
    {
        param_pid.config_ref = MAX(MIN(param_pid.config_ref, max_ref), max_ref - 2);
    }

    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, param_pid.config_ref);
//				HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, param_pid.conu_start_ref);
    return;
}

void tube_current_pid_pulseInit(uint8_t n)
{
    uint32_t fila_offset_pulse[FILAMENT_CURRENT_TABLE_ORDER] =
    {
        /* 1   2   3   4   5   6   7    8   9   10   11   12 */
        0,  0,  0,  0,  0,  0,  10,  15,  15, 15//,  20,  30
    };

    param_pid.Error    = 0;
    param_pid.integral = 0;
    param_pid.config_ref += fila_offset_pulse[config_data.tube_curr_index[n]];

    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, param_pid.config_ref);
}

void tube_current_piControl_v2(uint8_t n)
{
    user_pid_2.err[n] = user_pid_2.currTarget[n] - user_pid_2.currValue[n];
		float output;
//		if(config_data.tube_curr_index[n]<=10)
				output = user_pid_2.Kp[n] * (user_pid_2.err[n] - user_pid_2.last_err[n]) + user_pid_2.Ki[n] * user_pid_2.err[n];
//		else
//			output = (user_pid_2.Kp[n]+20) * (user_pid_2.err[n] - user_pid_2.last_err[n]) + user_pid_2.Ki[n] * user_pid_2.err[n];

    if (param_pid.pulse_count >= 10)
    {
        output = MAX(MIN(output, 1), -1);
    }

    user_pid_2.last_err[n] = user_pid_2.err[n];

    user_pid_2.config_ref[n] = (uint32_t)(user_pid_2.config_ref[n] + output);
//		if(n==0)
//		debug_tx3("curt:%f,%f,%f,%d\n", user_pid_2.currValue[n],output, user_pid_2.last_err[n],user_pid_2.config_ref[n]);
		if(n==0)
		{
    user_pid_2.config_ref[n] = MAX(MIN(user_pid_2.config_ref[n], 2200), 1000);
    HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, user_pid_2.config_ref[n]);
		}
		else
		{
		user_pid_2.config_ref[n] = MAX(MIN(user_pid_2.config_ref[n], 1430), 1000);
		 __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4,user_pid_2.config_ref[n] );//1360
		}
//		debug_tx3("pi_p:%d, %f, %d, %f\n", n, output, user_pid_2.config_ref[n],user_pid_2.err[n]);
}
