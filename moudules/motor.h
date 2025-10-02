#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include "pid.h"

//比较寄存器的值,根据实际设置更改
#define VALUE_COMPARE 100
#define MOTOR_COUNT 4

extern TIM_HandleTypeDef htim3;

typedef enum {
    MOTOR_DIR_NORMAL = 0,
    MOTOR_DIR_REVERSE = 1,
} Motor_Reverse_Flag_e;

typedef enum {
    OPEN_LOOP = 0,
    CLOSE_LOOP = 1,
} Loop_Type_e;

//电机设置
typedef struct {
    Motor_Reverse_Flag_e reverse; // 反转标志
} Motor_Setting_s;

//pwm时钟,通道配置
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel1;
    uint32_t channel2;
} Motor_PWM_Config_s;

//电机测量值
typedef struct {
    float angle;
    float speed;
} Motor_Measures_s;

//电机控制器
typedef struct {
    Loop_Type_e loop_type;
    float ref;
    PID_Instance_s angle_pid;
} Motor_Controller_s;

typedef struct {
    Motor_PWM_Config_s pwm_config;  //对应的时钟句柄指针和通道
    Motor_Setting_s setting;
    Motor_Measures_s measures;
    Motor_Controller_s controller;
} Motor_Instance_s;

typedef struct {
    Motor_PWM_Config_s pwm_config;
    Motor_Controller_s controller;
    Motor_Setting_s setting;
} Motor_Init_Config_s;


Motor_Instance_s* Motor_Init(Motor_Init_Config_s *config);
void MotorControl();

void MotorSetRef(Motor_Instance_s *motor, float ref);

#endif