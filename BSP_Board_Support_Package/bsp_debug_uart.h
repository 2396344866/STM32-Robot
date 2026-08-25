#ifndef __BSP_DEBUG_UART_H
#define __BSP_DEBUG_UART_H

#include "sys_config.h"

#if ENABLE_DEBUG_PRINT
    #include <stdint.h>
    #include "FreeRTOS.h"
    #include "task.h"
    #include "queue.h"

    // ===== N=2 双队列资源池化（FreeQueue / DataQueue）=====
    #define DEBUG_RX_MAX      128          // 单块缓冲容量（字节）
    #define DEBUG_BUF_N       2            // 缓冲块数（N=2：空闲池+数据池最小资源池）
    #define DEBUG_FREE_Q_LEN  DEBUG_BUF_N  // 空闲队列深度 = 块数
    #define DEBUG_DATA_Q_LEN  DEBUG_BUF_N  // 数据队列深度 = 块数

    // 单块物理独立缓冲（资源池元素）
    typedef struct {
        uint8_t  data[DEBUG_RX_MAX];
        uint16_t len;                       // 本块有效字节数（ISR 填，任务读）
    } debug_buf_t;

    // 资源池：N 块物理独立缓冲
    extern debug_buf_t g_debug_pool[DEBUG_BUF_N];
    // 空闲池：存放可写缓冲指针（初始全放进去）
    extern QueueHandle_t xDebugFreeQueue;
    // 数据池：存放待解析缓冲指针（初始为空）
    extern QueueHandle_t xDebugDataQueue;
    extern uint32_t g_debug_rx_overflow_cnt; // FreeQueue 为空时主动丢弃计数（硬背压生效记录）
    extern volatile uint8_t g_dma_suspended;  // DMA 因 FreeQueue 空暂停标志（任务归还后恢复）

    // notify_task 为被唤醒的调试处理任务句柄（ISR 用 xTaskNotify 替代队列）
    void BSP_DebugUART_Init(uint32_t baudrate, TaskHandle_t notify_task);
    void BSP_DebugUART_SendString(char* str);
    // 任务上下文：背压解除后从空闲池取块重启 DMA（关临界区保护）
    void BSP_DebugUART_ResumeDMA(void);
#else
    // 调试关闭时，将对外接口宏替换为空，实现无缝裁剪
    #define BSP_DebugUART_Init(baudrate, notify_task)   ((void)0)
    #define BSP_DebugUART_SendString(str)  ((void)0)
#endif

#endif
