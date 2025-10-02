
#include "motor.h"

static uint8_t idx = 0;
//模块'motor'的私有变量,用于保存电机实例的地址
static Motor_Instance_s *motor_instance[MOTOR_COUNT] = {0};

/**
 * @brief 电机初始化函数
 * 
 */
Motor_Instance_s* Motor_Init(Motor_Init_Config_s *config)
{
    Motor_Instance_s *instance = (Motor_Instance_s *)malloc(sizeof(Motor_Instance_s));
    memset(instance, 0, sizeof(Motor_Instance_s));

    instance->controller.angle_pid = config->controller.angle_pid;
    instance->setting = config->setting;
    instance->pwm_config = config->pwm_config;
    HAL_TIM_PWM_Start(instance->pwm_config.htim, instance->pwm_config.channel1);
    HAL_TIM_PWM_Start(instance->pwm_config.htim, instance->pwm_config.channel2);

    motor_instance[idx++] = instance;
    return instance;
}

/**
 * @brief 电机驱动函数,根据输入值控制电机正反转和速度
 * @param value 范围为正负比较寄存器的值,
 */
void MotorDrive(int16_t value, Motor_PWM_Config_s *pwm_config)
{
    if(value > VALUE_COMPARE) value = VALUE_COMPARE;
    if(value < -VALUE_COMPARE) value = -VALUE_COMPARE;

    if(value >= 0)
    {
        __HAL_TIM_SET_COMPARE(pwm_config->htim, pwm_config->channel1, value);
        __HAL_TIM_SET_COMPARE(pwm_config->htim, pwm_config->channel2, 0);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(pwm_config->htim, pwm_config->channel1, 0);
        __HAL_TIM_SET_COMPARE(pwm_config->htim, pwm_config->channel2, -value);
    }
}


/**
 * @brief 由于用两个PWM控制一个电机,所以这里的速度是-100~100,根据正负号改变PWM信号使能,从而改变方向
 * @brief 目前只有角度单环控制,后面考虑加速度环
 * @param speed 范围-100~100
 */
void MotorControl()
{
    Motor_Instance_s *motor;
    Motor_PWM_Config_s *pwm_config;
    Motor_Setting_s *setting;
    Motor_Controller_s *controller;
    Motor_Measures_s *measures;
    float pid_ref;
    float measure;

    for(int i = 0; i < MOTOR_COUNT; i++)
    {
        if(motor_instance[i] == NULL) break;
        
        motor = motor_instance[i];
        setting = &motor->setting;
        controller = &motor->controller;
        measures = &motor->measures;
        pwm_config = &motor->pwm_config;
        pid_ref = controller->ref;

        //角度环计算
        if (controller->loop_type == OPEN_LOOP) {
        }
        else if (controller->loop_type == CLOSE_LOOP) {
            measure = measures->angle;
            pid_ref = PIDCalculate(&controller->angle_pid, measure, pid_ref);
        }

        if (setting->reverse == MOTOR_DIR_REVERSE) {
            pid_ref *= -1;
        }

        MotorDrive((int16_t)pid_ref, pwm_config);

    }
}

void MotorSetRef(Motor_Instance_s *motor, float ref)
{
    motor->controller.ref = ref;
}