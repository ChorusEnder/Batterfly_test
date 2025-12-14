#ifndef REMOTE_FS_H
#define REMOTE_FS_H

#include "main.h"
#include "usart.h"

#define FS_RC_SW_UP ((int8_t)-1)   // 开关向上时的值
#define FS_RC_SW_MID ((int8_t)0)  // 开关中间时的值
#define FS_RC_SW_DOWN ((int8_t)1) // 开关向下时的值
// 三个判断开关状态的宏
#define fs_switch_is_down(s) (s == FS_RC_SW_DOWN)
#define fs_switch_is_mid(s) (s == FS_RC_SW_MID)
#define fs_switch_is_up(s) (s == FS_RC_SW_UP)

typedef struct
{
    //通道数据,Fs只有10个通道
    int16_t channel[16];

    int16_t rocker_l_; // 左水平
    int16_t rocker_l1; // 左竖直
    int16_t rocker_r_; // 右水平
    int16_t rocker_r1; // 右竖直

    int16_t vra;
    int16_t vrb;      

    int8_t swa;
    int8_t swb;
    int8_t swc;
    int8_t swd;

} RC_Fs_Ctrl_s;

/**
 * @param usart_handle 遥控器使用的串口句柄,固定huart3
 * @return RC_Fs_Ctrl_s* 遥控器数据指针
 * @attention 使用前需要确定自己使用的Fs遥控器通道配置,并在`Channel_to_Ctrl_Fs`中进行确认何修改
 */
RC_Fs_Ctrl_s* Remote_Fs_Init(UART_HandleTypeDef *usart_handle);




#endif