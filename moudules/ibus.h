#ifndef IBUS_H
#define IBUS_H

#include "bsp_uart.h"
#include "remote_fs.h"

#define RC_FS_I6X_OFFSET 1500 //遥控器通道偏置
#define RC_SW_DIV 400   //利用C语言中`/`的性质匹配开关状态

#define RC_SW_UP ((int16_t)1)
#define RC_SW_MID ((int16_t)0)
#define RC_SW_DOWN ((int16_t)-1)

#define sw_is_up(s) ((s) == RC_SW_UP)
#define sw_is_mid(s) ((s) == RC_SW_MID)
#define sw_is_down(s) ((s) == RC_SW_DOWN)




RC_Fs_Ctrl_s *Ibus_Init(UART_HandleTypeDef *huart);

#endif