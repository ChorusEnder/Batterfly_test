#include "remote_fs.h"
#include "bsp_uart.h"
// #include "daemon.h"
#include <stdlib.h>

#define RC_FS_RXBUFF_SIZE 25 // 遥控器接收的buffer大小
#define SBUS_HEADER 0x0F
#define SBUS_MID 1024
#define SBUS_SW_DIV 500//直接`/`通道值得到开关状态

#define rocker_deadband(x) ((abs(x) < 10) ? 0 : x) // 摇杆死区处理,处理垃圾富斯遥感零偏

static RC_Fs_Ctrl_s rc_ctrl[1]; // 遥控器数据
static UART_Instance *rc_uart_instance;
// static DaemonInstance *rc_daemon_instance;

static void Channel_to_Ctrl_Fs()
{
    int16_t *ch = rc_ctrl->channel;

    //摇杆部分基本不用改
    rc_ctrl->rocker_r_ = ch[0];
    rc_ctrl->rocker_r1 = ch[1];
    rc_ctrl->rocker_l1 = ch[2];
    rc_ctrl->rocker_l_ = ch[3];
    rc_ctrl->rocker_l1 = rocker_deadband(rc_ctrl->rocker_l1);
    rc_ctrl->rocker_l_ = rocker_deadband(rc_ctrl->rocker_l_);
    rc_ctrl->rocker_r1 = rocker_deadband(rc_ctrl->rocker_r1);
    rc_ctrl->rocker_r_ = rocker_deadband(rc_ctrl->rocker_r_);

    //通道匹配遥控器设置
    rc_ctrl->swa = ch[4] / SBUS_SW_DIV;
    rc_ctrl->swb = ch[5] / SBUS_SW_DIV;
    rc_ctrl->swc = ch[6] / SBUS_SW_DIV;
    rc_ctrl->swd = ch[7] / SBUS_SW_DIV;

    rc_ctrl->vra = ch[8];
    rc_ctrl->vrb = ch[9];
}

static void Subs_Decode_Fs()
{
    if (rc_uart_instance->rx_buffer[0] != SBUS_HEADER) return;

    int16_t *ch = rc_ctrl->channel;
    const uint8_t * buf = rc_uart_instance->rx_buffer;

    ch[0]  = ((buf[1]     | buf[2]<<8) & 0x07FF) - SBUS_MID;
    ch[1]  = ((buf[2]>>3  | buf[3]<<5) & 0x07FF) - SBUS_MID;
    ch[2]  = ((buf[3]>>6  | buf[4]<<2 | buf[5]<<10) & 0x07FF) - SBUS_MID;
    ch[3]  = ((buf[5]>>1  | buf[6]<<7) & 0x07FF) - SBUS_MID;
    ch[4]  = ((buf[6]>>4  | buf[7]<<4) & 0x07FF) - SBUS_MID;
    ch[5]  = ((buf[7]>>7  | buf[8]<<1 | buf[9]<<9) & 0x07FF) - SBUS_MID;
    ch[6]  = ((buf[9]>>2  | buf[10]<<6) & 0x07FF) - SBUS_MID;
    ch[7]  = ((buf[10]>>5 | buf[11]<<3) & 0x07FF) - SBUS_MID;
    ch[8]  = ((buf[12]    | buf[13]<<8) & 0x07FF) - SBUS_MID;
    ch[9]  = ((buf[13]>>3 | buf[14]<<5) & 0x07FF) - SBUS_MID;
    ch[10] = ((buf[14]>>6 | buf[15]<<2 | buf[16]<<10) & 0x07FF) - SBUS_MID;
    ch[11] = ((buf[16]>>1 | buf[17]<<7) & 0x07FF) - SBUS_MID;
    ch[12] = ((buf[17]>>4 | buf[18]<<4) & 0x07FF) - SBUS_MID;
    ch[13] = ((buf[18]>>7 | buf[19]<<1 | buf[20]<<9) & 0x07FF) - SBUS_MID;
    ch[14] = ((buf[20]>>2 | buf[21]<<6) & 0x07FF) - SBUS_MID;
    ch[15] = ((buf[21]>>5 | buf[22]<<3) & 0x07FF) - SBUS_MID;
}


static void Remote_fs_RxCallback()
{
    // DaemonReload(rc_daemon_instance); // 先喂狗
    Subs_Decode_Fs();//协议解析
    Channel_to_Ctrl_Fs();//通道值映射到遥控器控的实际按键控制
}

// static void RCLostCallback()
// {
//     memset(rc_ctrl, 0, sizeof(RC_Fs_Ctrl_s));
//     USARTServiceInit(rc_usart_instance);
// }

RC_Fs_Ctrl_s* Remote_Fs_Init(UART_HandleTypeDef *usart_handle)
{
    UART_Init_Config conf;
    conf.callback = Remote_fs_RxCallback;
    conf.huart_ptr = usart_handle;
    conf.rx_buffer_size = RC_FS_RXBUFF_SIZE;
    rc_uart_instance = UART_Init(&conf);

    // 进行守护进程的注册,用于定时检查遥控器是否正常工作
    // Daemon_Init_Config_s daemon_conf = {
    //     .reload_count = 10, 
    //     .callback = RCLostCallback,
    //     .owner_id = NULL, // 只有1个遥控器,不需要owner_id
    // };
    // rc_daemon_instance = DaemonRegister(&daemon_conf);

    return rc_ctrl;
}