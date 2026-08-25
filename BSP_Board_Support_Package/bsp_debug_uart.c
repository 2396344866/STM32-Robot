#include "bsp_debug_uart.h"
#include "hal_uart.h"
#include "hal_uart_dma.h"
#include "hal_gpio.h"

#if ENABLE_DEBUG_PRINT

// ===== N=2 双队列资源池化实现 =====
debug_buf_t g_debug_pool[DEBUG_BUF_N];          // 2 块物理独立缓冲
QueueHandle_t xDebugFreeQueue = NULL;           // 空闲池：可写缓冲指针
QueueHandle_t xDebugDataQueue = NULL;           // 数据池：待解析缓冲指针
uint32_t g_debug_rx_overflow_cnt = 0;           // FreeQueue 空时主动丢包计数（硬背压记录）

static TaskHandle_t xDebugTaskToNotify = NULL;  // 唤醒目标（替代调试队列）

// 当前 DMA 正在写入的块索引（ISR 与 DMA 私有，任务不碰）
static uint8_t g_dma_block_idx = 0;
// DMA 暂停标志：FreeQueue 为空时 ISR 不重启 DMA，置位；任务归还空闲块后恢复
// 注意：此处不可加 static —— fsm_network.c 需跨文件读写该标志（与 .h extern 对接）
volatile uint8_t g_dma_suspended = 0;

UART_Handle_t hDebugUart = {
        .Instance = USART2,
        .BaudRate = 115200
};

// 任务上下文：从空闲池取一块并重启 DMA（用于背压解除后恢复）
// 必须在 FreeQueue 有块时调用；关临界区保护 g_dma_block_idx 与 DMA 寄存器
// 注意：不可加 static —— fsm_network.c 需跨文件调用（与 .h extern 对接）
void BSP_DebugUART_ResumeDMA(void)
{
    debug_buf_t* next = NULL;
    if (xQueueReceive(xDebugFreeQueue, &next, 0) == pdPASS) {
        taskENTER_CRITICAL();
        g_dma_block_idx = (uint8_t)(next - g_debug_pool);
        next->len = 0;
        DMA_Cmd(DMA1_Channel6, DISABLE);
        DMA1_Channel6->CMAR = (uint32_t)next->data;
        DMA_SetCurrDataCounter(DMA1_Channel6, DEBUG_RX_MAX);
        DMA_Cmd(DMA1_Channel6, ENABLE);
        g_dma_suspended = 0;
        taskEXIT_CRITICAL();
    }
}

/* ================== 半主机模式禁用（保留，不可删） ================== */
#pragma import(__use_no_semihosting)

struct __FILE {
    int handle;
};
FILE __stdout;

void _sys_exit(int x) {
    x = x;
}

int fputc(int ch, FILE *f)
{
    HAL_UART_SendByte(&hDebugUart, (uint8_t)ch);
    return ch;
}
/* ==================================================================== */

void BSP_DebugUART_Init(uint32_t baudrate, TaskHandle_t notify_task)
{
    hDebugUart.BaudRate = baudrate;
    xDebugTaskToNotify = notify_task;

    // 1. 创建双队列（存指针，非数据体）
    xDebugFreeQueue = xQueueCreate(DEBUG_FREE_Q_LEN, sizeof(debug_buf_t*));
    xDebugDataQueue = xQueueCreate(DEBUG_DATA_Q_LEN, sizeof(debug_buf_t*));
    configASSERT(xDebugFreeQueue && xDebugDataQueue);

    // 2. 初始化资源池：所有块清空，指针全放入空闲池
    for (uint8_t i = 0; i < DEBUG_BUF_N; i++) {
        g_debug_pool[i].len = 0;
        debug_buf_t* p = &g_debug_pool[i];
        xQueueSend(xDebugFreeQueue, &p, 0);
    }

    // 3. 绑定引脚 PA2(TX), PA3(RX)
    HAL_GPIO_Init(GPIOA, GPIO_Pin_2, HAL_GPIO_MODE_AF_PP);
    HAL_GPIO_Init(GPIOA, GPIO_Pin_3, HAL_GPIO_MODE_INPUT_PU);

    // 4. 初始化 UART + DMA（Normal 模式，每帧停，ISR 取空闲块重启）
    HAL_UART_Init(&hDebugUart);
    HAL_UART_DMA_Rx_Init(&hDebugUart, g_debug_pool[0].data, DEBUG_RX_MAX, DMA_Mode_Normal);
    g_dma_block_idx = 0;

    // PreemptionPriority = 12：与 ESP8266 一致，落在 FreeRTOS 管控区间
    HAL_UART_EnableIRQ(&hDebugUart, 12);
}

// 独占 USART2 中断：处理空闲帧（一帧完成）
void USART2_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    BaseType_t xHigherPriorityTaskWoken2 = pdFALSE;

    if (USART_GetFlagStatus(USART2, USART_FLAG_IDLE) != RESET)
    {
        // 软件清除 IDLE 标志序列
        volatile uint32_t temp = USART2->SR;
        temp = USART2->DR;
        (void)temp;

        // DMA 已写满当前块（Normal 模式传输完成后停），计算长度
        uint16_t rx_len = DEBUG_RX_MAX - DMA_GetCurrDataCounter(DMA1_Channel6);
        if (rx_len > 0 && rx_len <= DEBUG_RX_MAX) {
            g_debug_pool[g_dma_block_idx].len = rx_len;
        }

        // 将刚写满的块指针放入数据池（所有权移交任务）
        debug_buf_t* filled = &g_debug_pool[g_dma_block_idx];
        // 数据池满（任务极慢）则丢弃：本应不可能（深度=块数），仍做保护
        if (xQueueIsQueueFullFromISR(xDebugDataQueue) == pdFALSE) {
            xQueueSendFromISR(xDebugDataQueue, &filled, &xHigherPriorityTaskWoken);
        } else {
            g_debug_rx_overflow_cnt++;  // 极端保护，正常不触发
        }

        // 背压核心：从空闲池取一块新缓冲；取不到则主动丢包、DMA 暂停（不践踏）
        debug_buf_t* next = NULL;
        if (xQueueReceiveFromISR(xDebugFreeQueue, &next, &xHigherPriorityTaskWoken2) == pdPASS) {
            g_dma_block_idx = (uint8_t)(next - g_debug_pool);  // 反算索引
            next->len = 0;
            // 重启 DMA 到新缓冲（Normal 模式需重设目标+计数+使能）
            DMA_Cmd(DMA1_Channel6, DISABLE);
            DMA1_Channel6->CMAR = (uint32_t)next->data;
            DMA_SetCurrDataCounter(DMA1_Channel6, DEBUG_RX_MAX);
            DMA_Cmd(DMA1_Channel6, ENABLE);
            // 唤醒任务解析数据池
            if (xDebugTaskToNotify != NULL) {
                xTaskNotifyFromISR(xDebugTaskToNotify, 0, eNoAction, &xHigherPriorityTaskWoken);
            }
        } else {
            // FreeQueue 为空 → 背压生效：主动丢弃本次后续数据，DMA 不重启（暂停）
            // Normal 模式写完即停，不会覆盖任何任务持有块；置位等待任务归还后恢复
            g_dma_suspended = 1;
            g_debug_rx_overflow_cnt++;
        }
    }

    if (USART_GetFlagStatus(USART2, USART_FLAG_ORE) != RESET ||
        USART_GetFlagStatus(USART2, USART_FLAG_RXNE) != RESET)
    {
        volatile uint32_t temp = USART2->SR;
        temp = USART2->DR;
        (void)temp;
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

#endif
