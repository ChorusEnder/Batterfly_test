#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include "pid.h"
#include <stdlib.h>
#include <string.h>

//比较寄存器的值,根据实际设置更改
#define VALUE_COMPARE 1000 -1
#define MOTOR_COUNT 4


typedef enum {
    MOTOR_DIR_NORMAL = 0,
    MOTOR_DIR_REVERSE = 1,
} Motor_Reverse_Flag_e;

typedef enum {
    MOTOR_STOP = 0,
    MOTOR_ENABLE,
} Motor_State_e;

typedef enum {
    OPEN_LOOP = 0b0000,
    SPEED_LOOP = 0b0001,
    ANGLE_LOOP = 0b0010,

    SPEED_AND_ANGLE_LOOP = 0b0011,
} Loop_Type_e;

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
    float angle_last;
} Motor_Measures_s;

//电机控制器
typedef struct {
    float pid_ref;
    float set;
    Loop_Type_e loop_type;
    PID_Instance_s angle_pid;
    PID_Instance_s speed_pid;

} Motor_Controller_s;

//电机设置
typedef struct {
    I2C_HandleTypeDef *hi2c;//iic句柄指针
    Motor_PWM_Config_s pwm_config;
    Motor_Reverse_Flag_e reverse; // 反转标志
    Motor_State_e motor_state;
} Motor_Setting_s;


typedef struct {
    Motor_Setting_s setting;
    Motor_Measures_s measures;
    Motor_Controller_s controller;
} Motor_Instance_s;

typedef struct {
    Motor_Controller_s controller;
    Motor_Setting_s setting;
} Motor_Init_Config_s;


Motor_Instance_s* Motor_Init(Motor_Init_Config_s *config);
void MotorControl();

void MotorSetRef(Motor_Instance_s *motor, float ref);
void MotorEnable(Motor_Instance_s *motor);
void MotorStop(Motor_Instance_s *motor);



#endif