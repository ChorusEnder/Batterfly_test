
#include "butterfly.h"
#include "butterfly_task.h"
#include "tim.h"
#include "ibus.h"
#include "motor.h"
#include "as5600.h"
#include "arm_math.h"
#include "remote_fs.h"

static Motor_Instance_s* motor1;
static Motor_Instance_s* motor2;
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
            .loop_type = ANGLE_LOOP,
            .pid_ref = 0.0f,
            .feedward  = 20.0f,
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
            .reverse = MOTOR_DIR_NORMAL,
            .motor_state = MOTOR_ENABLE,
        }
    };
    motor1 = Motor_Init(&motorConfig);

    motorConfig.setting.hi2c = &hi2c1;
    motorConfig.setting.pwm_config.htim = &htim1;
    motorConfig.setting.pwm_config.channel1 = TIM_CHANNEL_3;
    motorConfig.setting.pwm_config.channel2 = TIM_CHANNEL_4;
    motorConfig.setting.reverse = MOTOR_DIR_NORMAL;
    motor2 = Motor_Init(&motorConfig);//正面


    // rc_fs_i6x = Remote_Fs_Init(&huart1);
    rc_fs_i6x = Ibus_Init(&huart2);

    


}

void RemoteControl()
{

    if (fs_switch_is_down(rc_fs_i6x->swd)){
        motor1->setting.motor_state = MOTOR_STOP;
    }
    else if (fs_switch_is_up(rc_fs_i6x->swd)){
        motor1->setting.motor_state = MOTOR_ENABLE;
    }
    
}


void Butterfly_Task()
{
    time = DWT_GetTimeLine_s();
    dt = DWT_GetDeltaT_s(&last_t);

    // ref = 100*arm_sin_f32(20*time) + 150;
    // MotorSetRef(motor1, ref);
    // ref = 100*arm_sin_f32(20*time) + 150;
    // MotorSetRef(motor1, ref);

    // TMAG5273_ReadReg(&hi2c2, &reg_add_r, data_r);
    // TMAG5273_WriteReg(&hi2c2, &reg_add_w, &data_w);
    // TMAG5273_ReadReg(&hi2c2, &reg_add_r, data_r);
    // TMAG5273_WriteReg(&hi2c2, &reg_add_w, &data_w);

    RemoteControl();
}