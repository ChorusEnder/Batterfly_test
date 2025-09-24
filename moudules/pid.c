#include "pid.h"

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
    //更新误差
    pid->err[2] = pid->err[1];
    pid->err[1] = pid->err[0];
    pid->err[0] = target - measure;

    //计算PID,采用增量式PID
    pid->delta_output = pid->kp * (pid->err[0] - pid->err[1]) + pid->ki * pid->err[0] + pid->kd * (pid->err[0] - 2 * pid->err[1] + pid->err[2]);
    pid->output += pid->delta_output;

    OutputLimit(pid);

    return pid->output;
}