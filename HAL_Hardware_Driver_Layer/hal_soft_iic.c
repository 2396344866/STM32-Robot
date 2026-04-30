#include "hal_soft_i2c.h"

// 内部宏：提高可读性，通过句柄操作 GPIO
#define SCL_W(val) HAL_GPIO_WritePin(hI2c->port_scl, hI2c->pin_scl, (val) ? HAL_PIN_SET : HAL_PIN_RESET)
#define SDA_W(val) HAL_GPIO_WritePin(hI2c->port_sda, hI2c->pin_sda, (val) ? HAL_PIN_SET : HAL_PIN_RESET)

/**
 * @brief 初始化软件 I2C 引脚
 * @param hI2c I2C 句柄指针
 */
void HAL_SoftI2C_Init(SoftI2C_Handle_t* hI2c)
{
    // 使用 HAL_GPIO 初始化引脚为开漏输出 (Open-Drain)
    HAL_GPIO_Init(hI2c->port_scl, hI2c->pin_scl, HAL_GPIO_MODE_OUTPUT_OD);
    HAL_GPIO_Init(hI2c->port_sda, hI2c->pin_sda, HAL_GPIO_MODE_OUTPUT_OD);
    
    // 空闲状态拉高
    SCL_W(1);
    SDA_W(1);
}

/**
 * @brief I2C 起始信号
 */
void HAL_SoftI2C_Start(SoftI2C_Handle_t* hI2c)
{
    SDA_W(1);
    SCL_W(1);
    SDA_W(0);
    SCL_W(0);
}

/**
 * @brief I2C 停止信号
 */
void HAL_SoftI2C_Stop(SoftI2C_Handle_t* hI2c)
{
    SDA_W(0);
    SCL_W(1);
    SDA_W(1);
}

/**
 * @brief I2C 发送一个字节
 */
void HAL_SoftI2C_SendByte(SoftI2C_Handle_t* hI2c, uint8_t byte)
{
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        SDA_W(byte & (0x80 >> i));
        SCL_W(1); // 时钟高
        SCL_W(0); // 时钟低
    }
    
    // 处理应答位 (ACK) - OLED 通常不需要读取ACK，但需要提供时钟脉冲
    SCL_W(1);
    SCL_W(0);
}
