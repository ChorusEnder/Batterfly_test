#include "motor.h"

/**
 * @brief 电机初始化函数,由于一个电机需要两个PWM信号操作,后面考虑采用电机实例
 * 
 */
void Motor_Init(TIM_HandleTypeDef *htim, uint32_t channel)
{
    HAL_TIM_PWM_Start(htim, channel);
}

/**
 * @brief 目前没有速度反馈方案,暂时采用开环控制
 * @param speed 范围0~100
 */
void Motor_SetSpeed(int16_t speed)
{
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
}