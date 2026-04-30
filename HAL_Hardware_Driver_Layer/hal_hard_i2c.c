#include "hal_hard_i2c.h"
#include "hal_delay.h" // 使用HAL_GetTick进行超时计算

#define I2C_TIMEOUT_CNT 100000

// 内部函数：检测标志位并处理超时
static int I2C_WaitEvent(I2C_TypeDef* I2Cx, uint32_t Event, uint32_t Timeout) {
    while (!I2C_CheckEvent(I2Cx, Event)) {
        if (--Timeout == 0) {
            return -1; // 超时
        }
    }
    return 0;
}
void HAL_HardI2C_ResetBus(HardI2C_Handle_t *hI2c) {
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // 禁用I2C
    I2C_Cmd(hI2c->I2Cx, DISABLE);
    
    // 配置引脚为普通推挽输出
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = hI2c->SCL_Pin | hI2c->SDA_Pin;
    GPIO_Init(hI2c->GPIO_Port, &GPIO_InitStructure);
    
    // 强制发送9个时钟脉冲释放SDA
    GPIO_SetBits(hI2c->GPIO_Port, hI2c->SDA_Pin);
    for (int i = 0; i < 9; i++) {
        GPIO_ResetBits(hI2c->GPIO_Port, hI2c->SCL_Pin);
        HAL_Delay_us(5);
        GPIO_SetBits(hI2c->GPIO_Port, hI2c->SCL_Pin);
        HAL_Delay_us(5);
    }
    
    // 重新初始化I2C
    HAL_HardI2C_Init(hI2c);
}

void HAL_HardI2C_Init(HardI2C_Handle_t *hI2c) {
    GPIO_InitTypeDef GPIO_InitStructure;
    I2C_InitTypeDef I2C_InitStructure;

    if (hI2c->I2Cx == I2C1) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    } else if (hI2c->I2Cx == I2C2) {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    }

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Pin = hI2c->SCL_Pin | hI2c->SDA_Pin;
    GPIO_Init(hI2c->GPIO_Port, &GPIO_InitStructure);

    I2C_DeInit(hI2c->I2Cx);
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = hI2c->ClockSpeed;
    I2C_Init(hI2c->I2Cx, &I2C_InitStructure);
    I2C_Cmd(hI2c->I2Cx, ENABLE);
}

int HAL_HardI2C_WriteMem(HardI2C_Handle_t *hI2c, uint8_t DevAddr, uint8_t RegAddr, uint8_t *pData, uint16_t Size) {
    while(I2C_GetFlagStatus(hI2c->I2Cx, I2C_FLAG_BUSY));

    I2C_GenerateSTART(hI2c->I2Cx, ENABLE);
    if(I2C_WaitEvent(hI2c->I2Cx, I2C_EVENT_MASTER_MODE_SELECT, I2C_TIMEOUT_CNT)) goto error;

    I2C_Send7bitAddress(hI2c->I2Cx, DevAddr, I2C_Direction_Transmitter);
    if(I2C_WaitEvent(hI2c->I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, I2C_TIMEOUT_CNT)) goto error;

    I2C_SendData(hI2c->I2Cx, RegAddr);
    if(I2C_WaitEvent(hI2c->I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING, I2C_TIMEOUT_CNT)) goto error;

    for (uint16_t i = 0; i < Size; i++) {
        I2C_SendData(hI2c->I2Cx, pData[i]);
        if(I2C_WaitEvent(hI2c->I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING, I2C_TIMEOUT_CNT)) goto error;
    }

    I2C_GenerateSTOP(hI2c->I2Cx, ENABLE);
    return 0;

error:
    I2C_GenerateSTOP(hI2c->I2Cx, ENABLE);
    HAL_HardI2C_ResetBus(hI2c);
    return -1;
}

int HAL_HardI2C_ReadMem(HardI2C_Handle_t *hI2c, uint8_t DevAddr, uint8_t RegAddr, uint8_t *pData, uint16_t Size) {
    while(I2C_GetFlagStatus(hI2c->I2Cx, I2C_FLAG_BUSY));

    I2C_GenerateSTART(hI2c->I2Cx, ENABLE);
    if(I2C_WaitEvent(hI2c->I2Cx, I2C_EVENT_MASTER_MODE_SELECT, I2C_TIMEOUT_CNT)) goto error;

    I2C_Send7bitAddress(hI2c->I2Cx, DevAddr, I2C_Direction_Transmitter);
    if(I2C_WaitEvent(hI2c->I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, I2C_TIMEOUT_CNT)) goto error;

    I2C_SendData(hI2c->I2Cx, RegAddr);
    if(I2C_WaitEvent(hI2c->I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED, I2C_TIMEOUT_CNT)) goto error;

    I2C_GenerateSTART(hI2c->I2Cx, ENABLE);
    if(I2C_WaitEvent(hI2c->I2Cx, I2C_EVENT_MASTER_MODE_SELECT, I2C_TIMEOUT_CNT)) goto error;

    I2C_Send7bitAddress(hI2c->I2Cx, DevAddr, I2C_Direction_Receiver);
    if(I2C_WaitEvent(hI2c->I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED, I2C_TIMEOUT_CNT)) goto error;

    for (uint16_t i = 0; i < Size; i++) {
        if (i == Size - 1) {
            I2C_AcknowledgeConfig(hI2c->I2Cx, DISABLE);
            I2C_GenerateSTOP(hI2c->I2Cx, ENABLE);
        }
        if(I2C_WaitEvent(hI2c->I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED, I2C_TIMEOUT_CNT)) goto error;
        pData[i] = I2C_ReceiveData(hI2c->I2Cx);
    }
    I2C_AcknowledgeConfig(hI2c->I2Cx, ENABLE);
    return 0;

error:
    I2C_GenerateSTOP(hI2c->I2Cx, ENABLE);
    HAL_HardI2C_ResetBus(hI2c);
    return -1;
}
