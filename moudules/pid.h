#ifndef PID_H
#define PID_H

#include "main.h"

typedef struct {
    float kp;       // 比例系数
    float ki;       // 积分系数
    float kd;       // 微分系数
    float err[3];   // 误差数组

    float pout;
    float iout;
    float iterm;// 积分项
    float dout;
    float output;   // 输出值
    float maxout; // 输出限幅
    //用于增量式pid计算
    float delta_output;

    //用于积分
    float dt;
    //用于给时钟计数
    uint32_t DWT_CNT;
} PID_Instance_s;

/**
 * @brief PID计算函数
 * @param pid PID实例
 * @param measure 测量值
 * @param target 目标值
 */
float PIDCalculate(PID_Instance_s *pid, float measure, float target);

#endif