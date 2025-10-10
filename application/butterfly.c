
#include "butterfly.h"
#include "butterfly_task.h"


static Motor_Instance_s* motor1;
static float angle1;


void Butterfly_Init()
{
    OSTask_Init();
    DWT_Init(72);

    Motor_Init_Config_s motorConfig = {
        
        .controller = {
            // .loop_type = ANGLE_LOOP | SPEED_LOOP,
            .loop_type = SPEED_LOOP,
            .angle_pid = {
                .kp = 10.0f,
                .ki = 0.0f,
                .kd = 0.0f,
                .maxout = VALUE_COMPARE,
            },
            .speed_pid = {
                .kp = 0.5f,
                .ki = 0.0f,
                .kd = 0.0f,
                .maxout = VALUE_COMPARE,
            },
            .pid_ref = 0.0f,
        },
        .setting = {
            .hi2c = &hi2c1,
            .pwm_config = {
                .htim = &htim3,
                .channel1 = TIM_CHANNEL_1,
                .channel2 = TIM_CHANNEL_2,
            },
            .reverse = MOTOR_DIR_NORMAL,
        }
    };
    motor1 = Motor_Init(&motorConfig);
}

void Butterfly_Task()
{

    // MotorSetRef(motor1, angle1);

}