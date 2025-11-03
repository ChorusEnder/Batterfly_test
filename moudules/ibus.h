#ifndef IBUS_H
#define IBUS_H

#include "bsp_uart.h"

#define RC_FS_I6X_OFFSET 1500 //遥控器通道偏置
#define RC_SW_DIV 400   //利用C语言中`/`的性质匹配开关状态

#define RC_SW_UP ((int16_t)1)
#define RC_SW_MID ((int16_t)0)
#define RC_SW_DOWN ((int16_t)-1)

#define sw_is_up(s) ((s) == RC_SW_UP)
#define sw_is_mid(s) ((s) == RC_SW_MID)
#define sw_is_down(s) ((s) == RC_SW_DOWN)

typedef struct{
    int16_t channel[16];

    int16_t rocker_l_;//左摇杆水平
    int16_t rocker_l1;//左摇杆垂直
    int16_t rocker_r_;//右摇杆水平
    int16_t rocker_r1;//右摇杆垂直

    int16_t switch_l1;//左开关1
    int16_t switch_l2;//左开关2
    int16_t switch_r1;//右开关1
    int16_t switch_r2;//右开关2
    int16_t dial_l;//左拨轮
    int16_t dial_r;//右拨轮

} Ibus_Data_Fs_i6x;


Ibus_Data_Fs_i6x *Ibus_Init(UART_HandleTypeDef *huart);

#endif