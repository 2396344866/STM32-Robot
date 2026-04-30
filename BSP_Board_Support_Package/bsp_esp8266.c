#include "bsp_esp8266.h"
#include "hal_uart.h"
#include "hal_uart_dma.h"
#include "hal_gpio.h"
#include <stdio.h>
uint8_t g_esp8266_rx_buf[ESP8266_RX_MAX];
QueueHandle_t xNetRxQueue = NULL;

// 实例化句柄
static UART_Handle_t hEsp8266Uart = {
    .Instance = USART1,
    .BaudRate = 115200
};

void BSP_ESP8266_Init(uint32_t baudrate)
{
    hEsp8266Uart.BaudRate = baudrate;

    // 创建解析队列 (传递数据长度)
    if (xNetRxQueue == NULL) {
        xNetRxQueue = xQueueCreate(5, sizeof(uint16_t));
    }

    // 1. 绑定引脚 PA9(TX), PA10(RX)
    HAL_GPIO_Init(GPIOA, GPIO_Pin_9, HAL_GPIO_MODE_AF_PP);
    HAL_GPIO_Init(GPIOA, GPIO_Pin_10, HAL_GPIO_MODE_INPUT_PU);

    // 2. 初始化核心外设
    HAL_UART_Init(&hEsp8266Uart);
    HAL_UART_DMA_Rx_Init(&hEsp8266Uart, g_esp8266_rx_buf, ESP8266_RX_MAX);
    
    // 3. 开启中断 (PreemptionPriority = 5)
    HAL_UART_EnableIRQ(&hEsp8266Uart, 5);
}

void BSP_ESP8266_SendString(const char* str){
		printf("[STM32 -> WIFI]: %s", str);
    HAL_UART_SendString(&hEsp8266Uart, str);
}

// 独占 USART1 中断：处理空闲帧
void USART1_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (USART_GetFlagStatus(USART1, USART_FLAG_IDLE) != RESET)
    {
        // 软件清除 IDLE 标志序列
        volatile uint32_t temp = USART1->SR;
        temp = USART1->DR;
        (void)temp;

        // 停止 DMA 以读取接收量
        DMA_Cmd(DMA1_Channel5, DISABLE);
        uint16_t rx_len = ESP8266_RX_MAX - DMA_GetCurrDataCounter(DMA1_Channel5);

        if (rx_len > 0)
        {
            // 补充字符串结束符
            if (rx_len < ESP8266_RX_MAX) g_esp8266_rx_buf[rx_len] = '\0';
            else g_esp8266_rx_buf[ESP8266_RX_MAX - 1] = '\0';
            
            if (xNetRxQueue != NULL) {
                xQueueSendFromISR(xNetRxQueue, &rx_len, &xHigherPriorityTaskWoken);
            }
        }

        // 复位 DMA 准备下一次接收
        DMA_SetCurrDataCounter(DMA1_Channel5, ESP8266_RX_MAX);
        DMA_Cmd(DMA1_Channel5, ENABLE);
    }
		if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET || USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET) {
        volatile uint32_t temp = USART1->SR;
        temp = USART1->DR; // 读数据寄存器可清除 ORE 和 RXNE 标志
        (void)temp;
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
