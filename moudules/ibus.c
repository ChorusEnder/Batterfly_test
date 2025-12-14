
#include "ibus.h"

#define IBUS_HEADER 0x20 // IBUS数据包头
#define IBUS_DATA_LEN 32 // 32字节数据包

static RC_Fs_Ctrl_s Fs_data;//模块'ibus'的私有变量,用于保存接收的数据
static UART_Instance *ibus_uart;//模块'ibus'的私有变量,用于保存UART实例的地址

static uint16_t IBUS_Checksum(uint8_t *buf);
static void Ibus_Decode_Fs_i6x();

RC_Fs_Ctrl_s *Ibus_Init(UART_HandleTypeDef *huart)
{
    UART_Init_Config config;
    config.rx_buffer_size = IBUS_DATA_LEN;
    config.huart_ptr = huart;
    config.callback = Ibus_Decode_Fs_i6x;
    ibus_uart = UART_Init(&config);

    return &Fs_data;
}



/**
 * @brief 'ibus'模块的私有函数,解码
 */
static void Ibus_Decode_Fs_i6x()
{
    uint8_t *rx_buff;//接收缓存
    rx_buff = ibus_uart->rx_buffer;

    //帧头匹配
    if (rx_buff[0] != IBUS_HEADER) return;

    //校验和
    uint16_t sum = IBUS_Checksum(rx_buff);
    uint16_t rxsum = rx_buff[30] | (rx_buff[31] << 8);
    if (sum != rxsum) return;

    //解析通道数据
    for (int i = 0; i < 14; i++)
        Fs_data.channel[i] = (rx_buff[2 + i * 2] | (rx_buff[3 + i * 2] << 8)) - RC_FS_I6X_OFFSET;

    //将通道数据映射到遥控器各个部分
    Fs_data.rocker_r_ = Fs_data.channel[0];
    Fs_data.rocker_r1 = Fs_data.channel[1];
    Fs_data.rocker_l1 = Fs_data.channel[2];
    Fs_data.rocker_l_ = Fs_data.channel[3];
    Fs_data.swa = Fs_data.channel[4] / RC_SW_DIV;
    Fs_data.swb = Fs_data.channel[5] / RC_SW_DIV;
    Fs_data.swc = Fs_data.channel[6] / RC_SW_DIV;
    Fs_data.swd = Fs_data.channel[7] / RC_SW_DIV;
    Fs_data.vra = Fs_data.channel[8];
    Fs_data.vrb = Fs_data.channel[9];

}

static uint16_t IBUS_Checksum(uint8_t *buf)
{
    uint16_t chksum = 0xFFFF;
    for (int i = 0; i < IBUS_DATA_LEN - 2; i++)
        chksum -= buf[i];
    return chksum;
}