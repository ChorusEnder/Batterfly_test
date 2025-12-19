
#include "butterfly.h"
#include "butterfly_task.h"
#include "tim.h"
#include "ibus.h"
#include "motor.h"
#include "as5600.h"
#include "arm_math.h"
#include "remote_fs.h"

static butterfly_mode_e butterfly_mode;
static Motor_Instance_s* motor_l;
static Motor_Instance_s* motor_r;
float feedforward_l;
float feedforward_r;
static RC_Fs_Ctrl_s *rc_fs;


/*-------------------以下是Asin(wt)+B有关的参数---------*/
static float angle_l;
static float angle_r;
static float time;
static float w = 8;

static float Al = 100;
static float bl;
static float Ar = 100;
static float br;
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
            .loop_type = ANGLE_LOOP,
            .pid_ref = 0.0f,
            .feedward  = 0.0f,
            .angle_pid = {
                .kp = 5.0f,
                .ki = 0.0f,
                .kd = 0.0f,
                .deadband = 1.0f,
                .maxout = 500,
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
            .motor_offset = 35.0f,
        }
    };
    motor_l = Motor_Init(&motorConfig);

    motorConfig.setting.hi2c = &hi2c1;
    motorConfig.setting.pwm_config.htim = &htim1;
    motorConfig.setting.pwm_config.channel1 = TIM_CHANNEL_3;
    motorConfig.setting.pwm_config.channel2 = TIM_CHANNEL_4;
    motorConfig.setting.flag_motor_reverse = MOTOR_DIR_REVERSE;
    motorConfig.setting.flag_feedback_reverse = FEEDBACK_DIR_REVERSE;
    motorConfig.setting.motor_offset = 218.0f;
    motor_r = Motor_Init(&motorConfig);//正面


    // rc_fs_i6x = Remote_Fs_Init(&huart1);
    rc_fs = Ibus_Init(&huart2);


}

static void Control_Rise_Fall()
{

}

static void Control_Direction()
{

}

static void RemoteControl()
{
    if (fs_switch_is_down(rc_fs->swd)){
        butterfly_mode = BUTTERFLY_MODE_STOP;
        return;
    }

    if (fs_switch_is_up(rc_fs->swa) && fs_switch_is_up(rc_fs->swb)){
        butterfly_mode = BUTTERFLY_MODE_POSITION;

        angle_l = (rc_fs->rocker_l1) / 1000.0f * 360.0f;
        angle_r = (rc_fs->rocker_l1) / 1000.0f * 360.0f;

        if (angle_l < -80) angle_l = 80;
        if (angle_r < -80) angle_r = 80;
        if (angle_l > 90) angle_l = 90;
        if (angle_r > 90) angle_r = 90;

    }
    else if(fs_switch_is_up(rc_fs->swa) && fs_switch_is_mid(rc_fs->swb)){
        butterfly_mode = BUTTERFLY_MODE_FLYING;

        angle_l = Al * arm_sin_f32(w * time) + bl;
        angle_r = Ar * arm_sin_f32(w * time) + br;
    }
}


static void MotorControl()
{
    switch (butterfly_mode)
    {
        case BUTTERFLY_MODE_POSITION:
            //定点模式,通过遥控器控制翅膀位置           
            MotorEnable(motor_l);
            MotorEnable(motor_r);
            

            MotorSetRef(motor_l, angle_l);
            MotorSetRef(motor_r, angle_r);
            
            break;
        case BUTTERFLY_MODE_FLYING:
            //飞行模式,通过遥控器控制翅膀速度,转向,升降...
            MotorEnable(motor_l);
            MotorEnable(motor_r);

            Control_Rise_Fall();
            Control_Direction();

            MotorSetRef(motor_l, angle_l);
            MotorSetRef(motor_r, angle_r);
            
            break;
        case BUTTERFLY_MODE_STOP:
            //急停模式
            MotorStop(motor_l);
            MotorStop(motor_r);
            break;
    }
}


void Butterfly_Task()
{
    time = DWT_GetTimeLine_s();
    dt = DWT_GetDeltaT_s(&last_t);

    RemoteControl();
    // MotorControl();

    // TMAG5273_ReadReg(&hi2c2, &reg_add_r, data_r);
    // TMAG5273_WriteReg(&hi2c2, &reg_add_w, &data_w);

    feedforward_l = 50 * arm_cos_f32(MotorGetAngle(motor_l) * 3.14f / 180.0f);
    feedforward_r = -100 * arm_cos_f32(MotorGetAngle(motor_r) * 3.14f / 180.0f);
    MotorSetFeedforward(motor_l, feedforward_l);
    MotorSetFeedforward(motor_r, feedforward_r);

    angle_l = Al * arm_sin_f32(w * time) + 20;
    angle_r = Ar * arm_sin_f32(w * time) + 20;

    MotorSetRef(motor_l, angle_l);
    MotorSetRef(motor_r, angle_r);

}