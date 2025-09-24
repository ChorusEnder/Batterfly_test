#ifndef AS5600_H
#define AS5600_H

#include "main.h"

#define AS5600_I2C_ADDR        (0x36 << 1)  // AS5600 I2C 地址，左移一位以适应 HAL 库
#define AS5600_RAW_ANGLE_REG   0x0C          // RAW ANGLE 高字节寄存器


float AS5600_GetAngle(I2C_HandleTypeDef *hi2c);
#endif