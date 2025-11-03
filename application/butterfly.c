
#include "butterfly.h"
#include "butterfly_task.h"
#include "tim.h"
#include "ibus.h"
#include "motor.h"
#include "i2c.h"

static Motor_Instance_s* motor1;
static Ibus_Data_Fs_i6x *rc_fs_i6x;
static float angle_l;
static float angle_r;
static float time;

void Butterfly_Init()
{
    OSTask_Init();
    DWT_Init(72); // 初始化DWT，参数为CPU主频，单位MHz

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

    rc_fs_i6x = Ibus_Init(&huart1);
}

void RemoteControl()
{
    if (sw_is_up(rc_fs_i6x->switch_l1 && sw_is_up(rc_fs_i6x->switch_l2))){

    }

}


void Butterfly_Task()
{
    time = DWT_GetTimeLine_s();
    // MotorSetRef(motor1, angle1);

}