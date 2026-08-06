#include "hal_hard_i2c.h"
#include "hal_delay.h" // 使用HAL_GetTick进行超时计算
#include <stdio.h>   // 或 <stdlib.h>，它们内部都包含了 stddef.h
#define I2C_TIMEOUT_CNT 100000


// 保持 WaitEvent 独立封装，添加 static inline 显式提示编译器内联
static inline int I2C_WaitEvent(I2C_TypeDef* I2Cx, uint32_t Event, uint32_t Timeout) {
    while (!I2C_CheckEvent(I2Cx, Event)) {
        if (Timeout == 0 || --Timeout == 0) {
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
		/* 参数合法性检查 */
		if (hI2c == NULL || (Size > 0 && pData == NULL)) {
        return -1;
    }
		uint32_t timeout = I2C_TIMEOUT_CNT;
		
		/* 防死锁：带超时的总线忙状态检测 */
    while (I2C_GetFlagStatus(hI2c->I2Cx, I2C_FLAG_BUSY)) {
        if (--timeout == 0) goto error;
    }
    /* 1. 发送起始信号 (START) */ 
    I2C_GenerateSTART(hI2c->I2Cx, ENABLE);
    if(I2C_WaitEvent(hI2c->I2Cx, I2C_EVENT_MASTER_MODE_SELECT, I2C_TIMEOUT_CNT)) goto error;
		
		/* 2. 发送从机地址 + 写标志 */
    I2C_Send7bitAddress(hI2c->I2Cx, DevAddr, I2C_Direction_Transmitter);
    if(I2C_WaitEvent(hI2c->I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, I2C_TIMEOUT_CNT)) goto error;

		/* 3. 发送目标寄存器地址 (RegAddr) 
			 当数据从 DR 寄存器转移到了底层移位寄存器，即SCL 线上正在串行移位发送该字节 
		   触发TXE (Transmit（TX） Data Register Empty) 表示发送DR数据寄存器空，允许软件写入下一个字节
		*/
    I2C_SendData(hI2c->I2Cx, RegAddr);
    if(I2C_WaitEvent(hI2c->I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING, I2C_TIMEOUT_CNT)) goto error;

		/* 4. 循环发送数据缓冲区 */
    for (uint16_t i = 0; i < Size; i++) {
        I2C_SendData(hI2c->I2Cx, pData[i]);
        if(I2C_WaitEvent(hI2c->I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTING, I2C_TIMEOUT_CNT)) goto error;
    }
		/* 5. 写完最后一个字节后，必须等待物理发送完成及 ACK (BTF=1) */
		if (I2C_WaitEvent(hI2c->I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED, I2C_TIMEOUT_CNT)) goto error;
		
		/* 6. 发送停止信号 (STOP) */
    I2C_GenerateSTOP(hI2c->I2Cx, ENABLE);
    return 0;

error:
    I2C_GenerateSTOP(hI2c->I2Cx, ENABLE);
    HAL_HardI2C_ResetBus(hI2c);
    return -1;
}

int HAL_HardI2C_ReadMem(HardI2C_Handle_t *hI2c, uint8_t DevAddr, uint8_t RegAddr, uint8_t *pData, uint16_t Size) {
    if (hI2c == NULL || pData == NULL || Size == 0) {
        return -1;
    }

    uint32_t timeout = I2C_TIMEOUT_CNT;

    /* 防死锁：带超时的总线忙状态检测 */
    while (I2C_GetFlagStatus(hI2c->I2Cx, I2C_FLAG_BUSY)) {
        if (--timeout == 0) goto error;
    }

    /* 1. 发送写模式起始信号 */
    I2C_GenerateSTART(hI2c->I2Cx, ENABLE);
    if (I2C_WaitEvent(hI2c->I2Cx, I2C_EVENT_MASTER_MODE_SELECT, I2C_TIMEOUT_CNT)) goto error;
		/* 2. 写入目标 DevAddr */
    I2C_Send7bitAddress(hI2c->I2Cx, DevAddr, I2C_Direction_Transmitter);
    if (I2C_WaitEvent(hI2c->I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, I2C_TIMEOUT_CNT)) goto error;
		/* 3. 写入目标 RegAddr 向从机写入要读取的目标寄存器地址
		(正是因为要写入目标寄存器地址所以需要写模式发送起始信号 在写模式下写入目标设备和寄存器地址 也就是主机发送给从机）
		*/
    I2C_SendData(hI2c->I2Cx, RegAddr);
    if (I2C_WaitEvent(hI2c->I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED, I2C_TIMEOUT_CNT)) goto error;

    /* 4. 发送重复起始信号 (Sr)，切换为读模式 */
    I2C_GenerateSTART(hI2c->I2Cx, ENABLE);
    if (I2C_WaitEvent(hI2c->I2Cx, I2C_EVENT_MASTER_MODE_SELECT, I2C_TIMEOUT_CNT)) goto error;

    /* 5. 处理单字节接收的硬件时序约束 */
    if (Size == 1) {
        /* 单字节读取：在清除 ADDR 标志前提前禁用 ACK */
        I2C_AcknowledgeConfig(hI2c->I2Cx, DISABLE);
				/* 发送从机地址 + 读标志 
					发送完毕并且从机响应 ACK 之后，总线控制权已经交交给了从机，从机立刻开始向 SDA 线上输出数据
			
			    也就是地址帧结束后，紧接着就是数据字节的时钟1，中间没有任何空隙，此时ACK已经固定无法更改 
				*/
        I2C_Send7bitAddress(hI2c->I2Cx, DevAddr, I2C_Direction_Receiver);
        if (I2C_WaitEvent(hI2c->I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED, I2C_TIMEOUT_CNT)) goto error;

        /* 清除 ADDR 后，立刻生成 STOP 信号  此时从机已经开始发送数据字节的时钟1！ */
        I2C_GenerateSTOP(hI2c->I2Cx, ENABLE);

        /* 等待数据到达 DR 寄存器  硬件接收数据字节（时钟1~8） 此时无法更改 */
        if (I2C_WaitEvent(hI2c->I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED, I2C_TIMEOUT_CNT)) goto error;
				/* 数据字节的时钟9（主机NACK （就是应答不发就是NACK）      + STOP */
        pData[0] = I2C_ReceiveData(hI2c->I2Cx);
    } 
    else {
        /* 多字节读取 */
        I2C_AcknowledgeConfig(hI2c->I2Cx, ENABLE);

        I2C_Send7bitAddress(hI2c->I2Cx, DevAddr, I2C_Direction_Receiver);
        if (I2C_WaitEvent(hI2c->I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED, I2C_TIMEOUT_CNT)) goto error;

        for (uint16_t i = 0; i < Size; i++) {
            if (i == Size - 2) {
                /* 倒数第二个字节：等待当前字节接收完成 */
                if (I2C_WaitEvent(hI2c->I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED, I2C_TIMEOUT_CNT)) goto error;
                pData[i] = I2C_ReceiveData(hI2c->I2Cx);

                /* 提前拉高 NACK 并生成 STOP，确保最后一个字节接收后发出 NACK  Not Acknowledge

									需要禁ACK的字节是第N个 前面的正常发送ACK 因此需要在倒数第二个接受完毕后 在倒数第一个之前修改 
							*/
                I2C_AcknowledgeConfig(hI2c->I2Cx, DISABLE);
                I2C_GenerateSTOP(hI2c->I2Cx, ENABLE);
            } 
            else if (i == Size - 1) {
                /* 最后一个字节：等待接收完成并读取 */
                if (I2C_WaitEvent(hI2c->I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED, I2C_TIMEOUT_CNT)) goto error;
                pData[i] = I2C_ReceiveData(hI2c->I2Cx);
            } 
            else {
                /* 中间字节：正常等待并读取 */
                if (I2C_WaitEvent(hI2c->I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED, I2C_TIMEOUT_CNT)) goto error;
                pData[i] = I2C_ReceiveData(hI2c->I2Cx);
            }
        }
    }

    /* 恢复 ACK 为默认启用状态，供下次通信使用 */
    I2C_AcknowledgeConfig(hI2c->I2Cx, ENABLE);
    return 0;

error:
    I2C_GenerateSTOP(hI2c->I2Cx, ENABLE);
    I2C_AcknowledgeConfig(hI2c->I2Cx, ENABLE);
    HAL_HardI2C_ResetBus(hI2c);
    return -1;
}
