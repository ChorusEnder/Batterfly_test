
#include "butterfly.h"
#include "butterfly_task.h"
#include "tim.h"
#include "ibus.h"
#include "motor.h"
#include "as5600.h"
#include "arm_math.h"
#include "remote_fs.h"


static Motor_Instance_s* motor_l;
static Motor_Instance_s* motor_r;
static RC_Fs_Ctrl_s *rc_fs_i6x;


/*-------------------以下是Asin(wt)+B有关的参数---------*/
static float angle_l;
static float angle_r;
static float time;
static float ref = 100;
static float A1 = 100;
static float w1 = 20;
/*----------------------------------------------------*/

/*---------iic读取寄存器的相关变量-----------*/
static uint16_t reg_add_r = 0x19;
static uint8_t data_r[2];
static uint16_t reg_add_w = 0x00;
static uint8_t data_w = 0b00001000;
/*----------------------------------------*/

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
            .feedward  = 0.0f,
            .angle_pid = {
                .kp = 1.0f,
                .ki = 0.1f,
                .kd = 0.0f,
                .deadband = 1.0f,
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
            .hi2c = &hi2c1,
            .pwm_config = {
                .htim = &htim1,
                .channel1 = TIM_CHANNEL_1,
                .channel2 = TIM_CHANNEL_2,
            },
            .flag_motor_reverse = MOTOR_DIR_NORMAL,
            .flag_feedback_reverse = FEEDBACK_DIR_NORMAL,
            .motor_state = MOTOR_ENABLE,
        }
    };
    motor_l = Motor_Init(&motorConfig);

    motorConfig.setting.hi2c = &hi2c1;
    motorConfig.setting.pwm_config.htim = &htim1;
    motorConfig.setting.pwm_config.channel1 = TIM_CHANNEL_3;
    motorConfig.setting.pwm_config.channel2 = TIM_CHANNEL_4;
    motorConfig.setting.flag_motor_reverse = MOTOR_DIR_NORMAL;
    motorConfig.setting.flag_feedback_reverse = FEEDBACK_DIR_NORMAL;
    motor_r = Motor_Init(&motorConfig);//正面


    // rc_fs_i6x = Remote_Fs_Init(&huart1);
    rc_fs_i6x = Ibus_Init(&huart2);


}

void RemoteControl()
{

    if (fs_switch_is_down(rc_fs_i6x->swd)){
        MotorStop(motor_l);
        MotorStop(motor_r);
    }
    else if (fs_switch_is_up(rc_fs_i6x->swd)){
        MotorEnable(motor_l);
        MotorEnable(motor_r);
    }

    if (fs_switch_is_up(rc_fs_i6x->swa) && fs_switch_is_up(rc_fs_i6x->swb)){
        angle_l = (500 + rc_fs_i6x->rocker_l1) / 1000.0f * VALUE_COMPARE;
        angle_r = angle_l;
    }
    else if(fs_switch_is_up(rc_fs_i6x->swa) && fs_switch_is_mid(rc_fs_i6x->swb)){
        angle_l += 0.01f * (rc_fs_i6x->rocker_l1);
        angle_r += 0.01f * (rc_fs_i6x->rocker_l1);
    }


    
}



void Butterfly_Task()
{
    time = DWT_GetTimeLine_s();
    dt = DWT_GetDeltaT_s(&last_t);

    RemoteControl();
    MotorControl();

    
    if (angle_l < 0)
        angle_l = 0;
    if (angle_l > 999)
        angle_l = 999;
    if (angle_r < 0)
        angle_r = 0;
    if (angle_r > 999)
        angle_r = 999;

    MotorSetRef(motor_l, angle_l);
    MotorSetRef(motor_r, angle_r);

    // TMAG5273_ReadReg(&hi2c2, &reg_add_r, data_r);
    // TMAG5273_WriteReg(&hi2c2, &reg_add_w, &data_w);

    RemoteControl();
    MotorControl();
}