
#include "butterfly.h"
#include "butterfly_task.h"

static Motor_Instance_s* motor1;
static float speed1;
static float angle1;

void Butterfly_Init()
{
    OSTask_Init();

    Motor_Init_Config_s motorConfig = {
        .pwm_config = {
            .htim = &htim3,
            .channel1 = TIM_CHANNEL_1,
            .channel2 = TIM_CHANNEL_2,
        },
        .controller = {
            .loop_type = OPEN_LOOP,
            .angle_pid = {
                .kp = 10.0f,
                .ki = 0.0f,
                .kd = 0.0f,
                .maxout = VALUE_COMPARE,
            },
        },
        .setting = {
            .reverse = MOTOR_DIR_NORMAL,
        }
    };
    motor1 = Motor_Init(&motorConfig);
}

void Butterfly_Task()
{

    speed1 = 0;
    MotorSetRef(motor1, speed1);
    angle1 = AS5600_GetAngle(&hi2c1);

}