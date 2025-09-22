#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

extern TIM_HandleTypeDef htim3;

void Motor_Init(TIM_HandleTypeDef *htim, uint32_t channel);
void Motor_SetSpeed(int16_t speed);

#endif