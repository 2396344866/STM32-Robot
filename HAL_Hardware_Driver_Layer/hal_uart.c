#include "hal_uart.h"
#include "hal_gpio.h" // 引用 GPIO HAL 进行引脚配置


// 内部辅助：自动开启时钟
static void HAL_UART_EnableClock(USART_TypeDef* USARTx)
{
    if (USARTx == USART1) RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    else if (USARTx == USART2) RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    else if (USARTx == USART3) RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
}

void HAL_UART_Init(UART_Handle_t* hUart)
{
    USART_InitTypeDef USART_InitStructure;

    // 1. 开启串口时钟
    HAL_UART_EnableClock(hUart->Instance);

    // 2. 配置参数
    USART_InitStructure.USART_BaudRate = hUart->BaudRate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    // 3. 初始化并使能
    USART_Init(hUart->Instance, &USART_InitStructure);
    USART_Cmd(hUart->Instance, ENABLE);
}

void HAL_UART_SendByte(UART_Handle_t* hUart, uint8_t byte)
{
    USART_SendData(hUart->Instance, byte);
    while (USART_GetFlagStatus(hUart->Instance, USART_FLAG_TXE) == RESET);
}

void HAL_UART_SendString(UART_Handle_t* hUart, const char* str)
{
    while (*str)
    {
        HAL_UART_SendByte(hUart, *str++);
    }
}

// 简单的轮询接收 (阻塞式)
uint8_t HAL_UART_ReceiveByte(UART_Handle_t* hUart)
{
    while (USART_GetFlagStatus(hUart->Instance, USART_FLAG_RXNE) == RESET);
    return (uint8_t)USART_ReceiveData(hUart->Instance);
}

void HAL_UART_EnableIRQ(UART_Handle_t* hUart, uint8_t Priority)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    
    // 使能接收中断
    //USART_ITConfig(hUart->Instance, USART_IT_RXNE, ENABLE);

    if (hUart->Instance == USART1) NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    else if (hUart->Instance == USART2) NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    else if (hUart->Instance == USART3) NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = Priority;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

