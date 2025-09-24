#include "as5600.h"

float AS5600_GetAngle(I2C_HandleTypeDef *hi2c)
{
    uint8_t rawData[2] = {0};

    // 直接从寄存器地址 0x0C 开始读两个字节（高字节 + 低字节）
    if (HAL_I2C_Mem_Read(hi2c,
                         AS5600_I2C_ADDR,
                         AS5600_RAW_ANGLE_REG,
                         I2C_MEMADD_SIZE_8BIT,
                         rawData,
                         2,
                         HAL_MAX_DELAY) != HAL_OK) {
        return -1.0f;  // 读取失败
    }

    // 合并高低字节
    uint16_t rawAngle = ((uint16_t)rawData[0] << 8) | rawData[1];
    rawAngle &= 0x0FFF;   // 12 位有效

    // 转换为角度 (0 ~ 360°)
    return (rawAngle * 360.0f) / 4096.0f;
}