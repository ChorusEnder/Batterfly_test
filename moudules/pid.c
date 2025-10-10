#include "pid.h"
#include "dwt.h"

static void OutputLimit(PID_Instance_s *pid)
{
    if (pid->output > pid->maxout) {
        pid->output = pid->maxout;
    } else if (pid->output < -pid->maxout) {
        pid->output = -pid->maxout;
    }
}

float PIDCalculate(PID_Instance_s *pid, float measure, float target)
{
    //离散式pid
    // //更新误差
    // pid->err[2] = pid->err[1];
    // pid->err[1] = pid->err[0];
    // pid->err[0] = target - measure;

    // //计算PID,采用增量式PID
    // pid->delta_output = pid->kp * (pid->err[0] - pid->err[1]) + pid->ki * pid->err[0] + pid->kd * (pid->err[0] - 2 * pid->err[1] + pid->err[2]);
    // pid->output += pid->delta_output;

    // OutputLimit(pid);

    //获取两次计算的时间间隔
    pid->dt = DWT_GetDeltaT_s(&pid->DWT_CNT);

    pid->err[1] = pid->err[0];
    pid->err[0] = target - measure;

    // 基本的pid计算,使用位置式
    pid->pout = pid->kp * pid->err[0];
    pid->iterm = pid->ki * pid->err[0] * pid->dt;
    pid->dout = pid->kd * (pid->err[0] - pid->err[1]) / pid->dt;

    pid->iout += pid->iterm;
    pid->output = pid->pout + pid->iout + pid->dout;

    OutputLimit(pid);

    return pid->output;
}