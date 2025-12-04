
#include "butterfly.h"
#include "butterfly_task.h"
#include "tim.h"
#include "ibus.h"
#include "motor.h"
#include "as5600.h"
#include "arm_math.h"

static Motor_Instance_s* motor1;
static Ibus_Data_Fs_i6x *rc_fs_i6x;
// static float angle_l;
// static float angle_r;
static float time;

static float ref = 100;

static uint16_t reg_add_r = 0x19;
static uint8_t data_r[2];
static uint16_t reg_add_w = 0x00;
static uint8_t data_w = 0b00001000;

static float dt;
static uint32_t last_t;



void Butterfly_Init()
{
    OSTask_Init();
    DWT_Init(72);

    Motor_Init_Config_s motorConfig = {
        .controller = {
            // .loop_type = ANGLE_LOOP | SPEED_LOOP,
            .loop_type = OPEN_LOOP,
            .pid_ref = 0.0f,
            .angle_pid = {
                .kp = 0.5f,
                .ki = 0.1f,
                .kd = 0.0f,
                .deadband = 0.5,
                .maxout = VALUE_COMPARE,
                .Improve = PID_T_Intergral | PID_I_limit | PID_D_Filter | PID_OutputFilter | PID_Changing_I,
                .core_a = 100,
                .core_b = 50,
                .derivative_LPF_RC = 0.01f,
                .output_LPF_RC = 0.05f,
                .i_limit = 20.0f,
            },
            .speed_pid = {
                .kp = 0.0f,
                .ki = 0.1f,
                .kd = 0.0f,
                .deadband = 1,
                .maxout = VALUE_COMPARE,
                // .Improve = PID_T_Intergral | PID_I_limit | PID_D_On_Measurement | PID_D_Filter | PID_OutputFilter,
                .Improve = 0b00000000,
                .core_a = 100,
                .core_b = 50,
                .derivative_LPF_RC = 0.01f,
                .output_LPF_RC = 0.05f,
                .i_limit = 500.0f,  
            }
        },
        .setting = {
            .hi2c = &hi2c2,
            .pwm_config = {
                .htim = &htim3,
                .channel1 = TIM_CHANNEL_3,
                .channel2 = TIM_CHANNEL_4,
            },
            .reverse = MOTOR_DIR_NORMAL,
            .motor_state = MOTOR_ENABLE,
        }
    };
    motor1 = Motor_Init(&motorConfig);
    rc_fs_i6x = Ibus_Init(&huart1);

    TMAG5273_Init(&hi2c2);
}

void RemoteControl()
{
    if (sw_is_up(rc_fs_i6x->switch_l1 && sw_is_up(rc_fs_i6x->switch_l2))){

    }

}


void Butterfly_Task()
{
    time = DWT_GetTimeLine_s();

    dt = DWT_GetDeltaT_s(&last_t);

    // ref = 200*arm_sin_f32(20*time);
    // MotorSetRef(motor1, ref);

    TMAG5273_ReadReg(&hi2c2, &reg_add_r, data_r);
    TMAG5273_WriteReg(&hi2c2, &reg_add_w, &data_w);
}