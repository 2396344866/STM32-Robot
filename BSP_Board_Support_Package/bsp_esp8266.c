#include "bsp_esp8266.h"
#include "hal_uart.h"
#include "hal_uart_dma.h"
#include "hal_gpio.h"
#include "FreeRTOS.h"
#include "hal_hw_cfg.h"  // 魔法数字宏 + assert_param
#include "task.h"
#include <stdio.h>

// ===== 透传模式（环形）缓冲 =====
uint8_t g_esp8266_rx_buf[ESP8266_RX_MAX];
volatile uint16_t g_rx_read_idx = 0;    // 环形缓冲读指针（仅任务上下文更新）

// ===== AT 模式（双队列）缓冲 =====
esp8266_buf_t g_esp8266_pool[ESP8266_BUF_N];
QueueHandle_t xEspFreeQueue = NULL;    // 空闲池：存放可写缓冲指针
QueueHandle_t xEspDataQueue = NULL;    // 数据池：待解析缓冲指针
uint32_t g_esp_rx_overflow_cnt = 0;    // FreeQueue 为空时主动丢弃计数（硬背压）
volatile uint8_t  g_esp_dma_suspended = 0;      // DMA 暂停标志：FreeQueue 为空时置位，任务归还后清除

// 当前生效模式：0=AT(Normal+双队列)，1=透传(Circular+环形)；默认由宏设定，可运行时切换
volatile uint8_t g_esp_mode = (ESP8266_TRANSPARENT_MODE ? 1 : 0);

// AT 模式：当前 DMA 正在写入的块索引（ISR 与 DMA 私有，任务不碰）
static uint8_t g_esp_dma_block_idx = 0;

// 唤醒目标：网络任务句柄（由 BSP_ESP8266_Init 注册）
static TaskHandle_t xNetTaskToNotify = NULL;

// 实例化句柄
static UART_Handle_t hEsp8266Uart = {
    .Instance = USART1,
    .BaudRate = 115200
};

void BSP_ESP8266_Init(uint32_t baudrate, TaskHandle_t net_task_handle)
{
    assert_param(baudrate >= 1200 && baudrate <= 921600);  // 合法 UART 波特率区间
    assert_param(net_task_handle != NULL);  // 必须传入有效的网络任务句柄用于唤醒
    hEsp8266Uart.BaudRate = baudrate;
    xNetTaskToNotify = net_task_handle;   // 记录唤醒目标，替代二值信号量

    // 1. 绑定引脚 PA9(TX), PA10(RX)
    HAL_GPIO_Init(GPIOA, GPIO_Pin_9, HAL_GPIO_MODE_AF_PP);
    HAL_GPIO_Init(GPIOA, GPIO_Pin_10, HAL_GPIO_MODE_INPUT_PU);

    // 2. 创建双队列（始终创建：AT 模式使用；透传模式不使用但占用极小，以支持运行时切换）
    xEspFreeQueue = xQueueCreate(ESP8266_BUF_N, sizeof(esp8266_buf_t*));
    xEspDataQueue = xQueueCreate(ESP8266_BUF_N, sizeof(esp8266_buf_t*));
    assert_param(xEspFreeQueue != NULL && xEspDataQueue != NULL);  // 队列创建必须成功

    // 3. 初始化核心外设 + 按当前模式配置 DMA
    HAL_UART_Init(&hEsp8266Uart);
    if (g_esp_mode) {
        // 透传：Circular 环形持续写，DMA 永不停止
        g_rx_read_idx = 0;
        HAL_UART_DMA_Rx_Init(&hEsp8266Uart, g_esp8266_rx_buf, ESP8266_RX_MAX, DMA_Mode_Circular);
    } else {
        // AT：所有块放入空闲池，DMA 指向 pool[0]，Normal（每帧停，ISR 切空闲块重启）
        for (uint8_t i = 0; i < ESP8266_BUF_N; i++) {
            g_esp8266_pool[i].len = 0;
            esp8266_buf_t* p = &g_esp8266_pool[i];
            xQueueSend(xEspFreeQueue, &p, 0);
        }
        g_esp_dma_block_idx = 0;
        g_esp_dma_suspended = 0;
        HAL_UART_DMA_Rx_Init(&hEsp8266Uart, g_esp8266_pool[0].data, ESP8266_RX_MAX, DMA_Mode_Normal);
    }

    // 4. 开启中断 (PreemptionPriority = 12)
    // 必须在 FreeRTOS 管控区间内(>= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY=10)，
    // 否则在临界区内仍可抢占并调用 FromISR API 会破坏内核数据。
    HAL_UART_EnableIRQ(&hEsp8266Uart, 12);
}

void BSP_ESP8266_SendString(const char* str){
    printf("[STM32 -> WIFI]: %s", str);
    HAL_UART_SendString(&hEsp8266Uart, str);
}

// 透传模式：二进制安全发送。透传模式下所有字节直接进 TCP socket，
// 裸 MQTT 报文可能含 0x00（剩余长度、长度前缀等），不能用 SendString（遇 '\0' 截断）。
void BSP_ESP8266_SendRaw(const uint8_t* data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        HAL_UART_SendByte(&hEsp8266Uart, data[i]);
    }
}

// 低功耗：ESP8266 modem sleep 开关。自动休眠模式下射频在两次收发之间自动掉电，有数据自动唤醒，
// 不会断 TCP/MQTT 会话，契合 MCU tickless idle 的低功耗策略。
void BSP_ESP8266_SetModemSleep(uint8_t enable)
{
    // 透传模式下无法发送 AT 命令（数据会当成 socket 载荷），因此仅在 AT 模式下生效。
    // 透传模式的低功耗由上层在休眠前 SwitchMode(0) 切回 AT 后启用，或整体依赖 MCU 侧休眠。
    if (g_esp_mode != 0) return;
    if (enable) {
        BSP_ESP8266_SendString("AT+SLEEP=1\r\n");   // 进入 modem sleep（自动）
    } else {
        BSP_ESP8266_SendString("AT+SLEEP=0\r\n");   // 退出 modem sleep，恢复全速
    }
}

// 透传模式：从环形缓冲拷贝可用字节到线性 out_buf（处理回绕），更新读指针；返回拷贝字节数
uint16_t BSP_ESP8266_RxDrain(char* out_buf, uint16_t out_max)
{
    assert_param(out_buf != NULL);
    assert_param(out_max > 1);  // 需至少留 1 字节给字符串终止符
    if (out_buf == NULL || out_max == 0) return 0;
    uint16_t ndtr = DMA_GetCurrDataCounter(DMA1_Channel5);
    uint16_t write_idx = ESP8266_RX_MAX - ndtr;
    uint16_t avail = (write_idx >= g_rx_read_idx)
                     ? (write_idx - g_rx_read_idx)
                     : (ESP8266_RX_MAX - g_rx_read_idx + write_idx);

    if (avail == 0) return 0;

    // 环形“满”与“空”同形（均 write==read）；透传模式由上层按 JSON 括号/超时切包，
    // 此处仅做线性缓冲拷贝，若 out_max 不足则截断（上层可增大 ESP8266_RX_MAX 或分次 drain）。
    uint16_t copied = 0;
    while (copied < avail && copied < (out_max - 1)) {
        out_buf[copied++] = g_esp8266_rx_buf[g_rx_read_idx];
        g_rx_read_idx = (g_rx_read_idx + 1) % ESP8266_RX_MAX;
    }
    out_buf[copied] = '\0';
    return copied;
}

// AT 模式：任务上下文从空闲池取一块并重启 DMA（背压解除后调用）
void BSP_ESP8266_ResumeDMA(void)
{
    if (g_esp_mode) return;  // 透传模式无需此函数
    esp8266_buf_t* next = NULL;
    if (xQueueReceive(xEspFreeQueue, &next, 0) == pdPASS) {
        g_esp_dma_block_idx = (uint8_t)(next - g_esp8266_pool);
        next->len = 0;
        DMA_Cmd(DMA1_Channel5, DISABLE);
        DMA1_Channel5->CMAR = (uint32_t)next->data;
        DMA_SetCurrDataCounter(DMA1_Channel5, ESP8266_RX_MAX);
        DMA_Cmd(DMA1_Channel5, ENABLE);
        g_esp_dma_suspended = 0;
    }
}

// 运行时切换模式：transparent=1 透传(Circular)，=0 AT(Normal+双队列)。返回 1=已切换，0=无需切换。
// 实现“满负荷透传高速上行 / 低功耗 AT 间歇上传”的动态切换。
int BSP_ESP8266_SwitchMode(uint8_t transparent)
{
    uint8_t target = transparent ? 1 : 0;
    if (target == g_esp_mode) return 0;

    // 关闭 USART1 中断，避免切换期间 ISR 访问不一致状态；再停 DMA 重配置
    NVIC_DisableIRQ(USART1_IRQn);
    DMA_Cmd(DMA1_Channel5, DISABLE);
    g_esp_mode = target;

    if (g_esp_mode) {
        // 切到透传：环形复位（DMA Circular 持续写，不暂停）
        g_rx_read_idx = 0;
        HAL_UART_DMA_Rx_Init(&hEsp8266Uart, g_esp8266_rx_buf, ESP8266_RX_MAX, DMA_Mode_Circular);
    } else {
        // 切回 AT：先把 DataQueue 中可能残留的块归还 FreeQueue，再把所有 pool 块补满空闲池，最后复位 pool[0] 重启
        // （透传模式下 pool 块从未入队，切回时必须全部放回，否则 FreeQueue 为空导致 ISR 立即 suspended）
        esp8266_buf_t* p = NULL;
        while (xQueueReceive(xEspDataQueue, &p, 0) == pdPASS) {
            p->len = 0;
            xQueueSend(xEspFreeQueue, &p, 0);
        }
        for (uint8_t i = 0; i < ESP8266_BUF_N; i++) {
            g_esp8266_pool[i].len = 0;
            esp8266_buf_t* q = &g_esp8266_pool[i];
            xQueueSend(xEspFreeQueue, &q, 0);
        }
        g_esp_dma_block_idx = 0;
        g_esp_dma_suspended = 0;
        HAL_UART_DMA_Rx_Init(&hEsp8266Uart, g_esp8266_pool[0].data, ESP8266_RX_MAX, DMA_Mode_Normal);
    }

    NVIC_EnableIRQ(USART1_IRQn);
    return 1;
}

// 独占 USART1 中断：处理空闲帧（按 g_esp_mode 分支：透传=环形通知，AT=双队列）
void USART1_IRQHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if (USART_GetFlagStatus(USART1, USART_FLAG_IDLE) != RESET)
    {
        // 软件清除 IDLE 标志序列
        volatile uint32_t temp = USART1->SR;
        temp = USART1->DR;
        (void)temp;

        if (g_esp_mode) {
            // ===== 透传模式（Circular + 环形）=====
            // DMA 持续运行无需停止/重启；仅通知任务去环形 drain（背靠背帧连续写入环形缓冲，不会覆盖在途数据）
            if (xNetTaskToNotify != NULL) {
                xTaskNotifyFromISR(xNetTaskToNotify, 0, eNoAction, &xHigherPriorityTaskWoken);
            }
        } else {
            // ===== AT 模式（Normal + 双队列资源池化）=====
            // Normal 模式：DMA 写完一帧（IDLE 界定）即停。把当前块入数据池，切空闲块重启。
            esp8266_buf_t* cur = &g_esp8266_pool[g_esp_dma_block_idx];
            cur->len = ESP8266_RX_MAX - DMA_GetCurrDataCounter(DMA1_Channel5);

            // 背压核心：从空闲池取一块新缓冲；取不到则主动丢包、DMA 暂停（不践踏）
            esp8266_buf_t* next = NULL;
            if (xQueueReceiveFromISR(xEspFreeQueue, &next, &xHigherPriorityTaskWoken) == pdPASS) {
                // 刚写满的旧块放入数据池，供任务解析（与 DMA 当前写的位置物理隔离）
                esp8266_buf_t* cur_send = cur;
                xQueueSendFromISR(xEspDataQueue, &cur_send, &xHigherPriorityTaskWoken);

                g_esp_dma_block_idx = (uint8_t)(next - g_esp8266_pool);  // 反算索引
                next->len = 0;
                // 重启 DMA 到新缓冲（Normal 模式需重设目标+计数+使能）
                DMA_Cmd(DMA1_Channel5, DISABLE);
                DMA1_Channel5->CMAR = (uint32_t)next->data;
                DMA_SetCurrDataCounter(DMA1_Channel5, ESP8266_RX_MAX);
                DMA_Cmd(DMA1_Channel5, ENABLE);
                // 唤醒任务解析数据池
                if (xNetTaskToNotify != NULL) {
                    xTaskNotifyFromISR(xNetTaskToNotify, 0, eNoAction, &xHigherPriorityTaskWoken);
                }
            } else {
                // FreeQueue 为空 → 背压生效：主动丢弃本次 DMA 数据，DMA 不重启（暂停）
                // 直到任务解析完某块并归还到 FreeQueue 后，ISR/ResumeDMA 才能重新获取空闲缓冲恢复
                g_esp_rx_overflow_cnt++;
                g_esp_dma_suspended = 1;
                // 注意：此刻 DMA 已停（Normal 模式写完即停），不会覆盖任何任务持有块
            }
        }
    }
    if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET || USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET) {
        volatile uint32_t temp = USART1->SR;
        temp = USART1->DR; // 读数据寄存器可清除 ORE 和 RXNE 标志
        (void)temp;
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
