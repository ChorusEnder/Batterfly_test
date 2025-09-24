#ifndef PID_H
#define PID_H

#include "main.h"

typedef struct {
    float kp;       // 比例系数
    float ki;       // 积分系数
    float kd;       // 微分系数
    float err[3];   // 误差数组

    float output;   // 输出值
    float maxout; // 输出限幅
    float delta_output;
} PID_Instance_s;

float PIDCalculate(PID_Instance_s *pid, float measure, float target);

#endif