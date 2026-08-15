#include "bsp_esp8266.h"
#include "hal_uart.h"
#include "hal_uart_dma.h"
#include "hal_gpio.h"
#include <stdio.h>
uint8_t g_esp8266_rx_buf[ESP8266_RX_MAX];
SemaphoreHandle_t xNetRxSem = NULL;
volatile uint16_t g_rx_read_idx = 0;    // 环形缓冲读指针（仅任务上下文更新）
uint32_t g_dma_rx_overflow_cnt = 0;     // 环形溢出计数（消费者落后致环被覆盖/截斩时累加）

// 实例化句柄
static UART_Handle_t hEsp8266Uart = {
    .Instance = USART1,
    .BaudRate = 115200
};

void BSP_ESP8266_Init(uint32_t baudrate)
{
    hEsp8266Uart.BaudRate = baudrate;

    // 创建二进制信号量：仅用于 ISR 唤醒网络任务（数据已由环形 DMA 缓冲承载，无需队列传递长度）
    if (xNetRxSem == NULL) {
        xNetRxSem = xSemaphoreCreateBinary();
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

// 从环形缓冲拷贝可用字节到线性 out_buf（处理回绕），更新读指针；返回拷贝字节数
uint16_t BSP_ESP8266_RxDrain(char* out_buf, uint16_t out_max)
{
    uint16_t ndtr = DMA_GetCurrDataCounter(DMA1_Channel5);
    uint16_t write_idx = ESP8266_RX_MAX - ndtr;
    uint16_t avail = (write_idx >= g_rx_read_idx)
                     ? (write_idx - g_rx_read_idx)
                     : (ESP8266_RX_MAX - g_rx_read_idx + write_idx);

    if (avail == 0) return 0;

    // 环形“满”与“空”同形（均 write==read）；若消费者落后致写指针套圈，avail 会偏小甚至误判为 0，
    // 靠缓冲足够大规避（115200bps 下 512B≈44ms 数据量，任务 50ms 内唤醒足以排空）。
    // 若线性 out_buf 不足导致截斩，记一次溢出。
    uint16_t copied = 0;
    while (copied < avail && copied < (out_max - 1)) {
        out_buf[copied++] = g_esp8266_rx_buf[g_rx_read_idx];
        g_rx_read_idx = (g_rx_read_idx + 1) % ESP8266_RX_MAX;
    }
    if (copied < avail) {
        g_dma_rx_overflow_cnt++;
    }
    out_buf[copied] = '\0';
    return copied;
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

        // 环形 DMA（Circular）模式下 DMA 持续运行，无需停止/重启；
        // 直接读 NDTR 得到当前写位置，与读指针比较即得新增字节数
        uint16_t ndtr = DMA_GetCurrDataCounter(DMA1_Channel5);
        uint16_t write_idx = ESP8266_RX_MAX - ndtr;
        uint16_t new_bytes;
        if (write_idx >= g_rx_read_idx)
            new_bytes = write_idx - g_rx_read_idx;
        else
            new_bytes = (ESP8266_RX_MAX - g_rx_read_idx) + write_idx;

        if (new_bytes > 0 && xNetRxSem != NULL) {
            // 仅发二进制信号量唤醒，可用量由任务按 NDTR 复算（不传占位长度，消除无意义变量）
            xSemaphoreGiveFromISR(xNetRxSem, &xHigherPriorityTaskWoken);
        }
        // 注意：此处不操作 DMA（保持 ENABLE），背靠背帧连续写入环形缓冲，不会覆盖在途数据
    }
    if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET || USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET) {
        volatile uint32_t temp = USART1->SR;
        temp = USART1->DR; // 读数据寄存器可清除 ORE 和 RXNE 标志
        (void)temp;
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
