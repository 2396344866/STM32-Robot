# HAL 层 README

## 定位

硬件抽象层（Hardware Abstraction Layer），直接操作 STM32F103 的寄存器与外设库，向上提供**与单片机型号弱相关**的原子操作接口。
所有对 MCU 外设（GPIO、USART、TIM、I2C、EXTI、DMA、ADC）的访问都必须通过这一层。

## 文件清单

| 文件名 | 说明 |
|--------|------|
| `hal_gpio.c/h` | GPIO 初始化、读/写/翻转，自动使能时钟，模式枚举覆盖推挽、开漏、复用推挽、上拉输入、下拉输入、浮空输入 |
| `hal_delay.c/h` | 微秒/毫秒延时，获取系统 Tick；**支持FreeRTOS与裸机双模式**（由宏 `HAL_USE_FREERTOS` 切换） |
| `hal_uart.c/h` | USART 初始化、字节/字符串发送、单字节接收、中断优先级配置 |
| `hal_uart_dma.c/h` | 为指定 UART 配置 DMA 空闲中断接收（IDLE + DMA），仅负责 DMA 通道初始化 |
| `hal_pwm.c/h` | 定时器时基与四通道 PWM 配置，支持 TIM2/TIM3/TIM4，预设 50Hz 周期（20ms），专为舵机优化 |
| `hal_soft_i2c.c/h` | 软件 I2C 时序模拟（起始、停止、字节发送），用于 OLED 等非标准速率器件 |
| `hal_hard_i2c.c/h` | STM32 硬件 I2C 驱动，含超时检测、总线复位，用于 MPU6050 等标准器件 |
| `hal_exti.c/h` | 外部中断（EXTI）初始化与回调注册，当前仅实现了 PB12（MPU6050 INT 引脚） |
| `hal_tim_ic.c/h` | TIM1 通道4 输入捕获，用于测量外部脉冲宽度（如超声波测距） |
| `hal_adc_dma.c/h` | ADC1 多通道 DMA 循环采集，自动校准，通过句柄 `ADC_Channels[]` 和 `ChannelCount` 灵活配置通道数与序列 |
## 主要设计约束

- **无业务逻辑**：HAL 层不知道 LED 是高亮还是低亮，也不管舵机角度换算。
- **自动时钟使能**：各模块内部根据传入的 `GPIOx/TIMx/USARTx` 自动开启对应 RCC 时钟，上层无需关心。
- **FreeRTOS 可选**：通过 `HAL_USE_FREERTOS` 宏控制，在 `hal_delay` 中实现调度器感知延时（vTaskDelay 或纯循环），全工程只需修改或注释该宏即可切换裸机/RTOS 模式。
- **统一句柄风格**：每个外设模块定义自己的句柄结构体（如 `UART_Handle_t`, `HardI2C_Handle_t`, `ADC_DMA_Handle_t`），方便多实例管理。

## 核心 API 速查

### GPIO
```c
void HAL_GPIO_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, HAL_GpioMode_t Mode);
void HAL_GPIO_WritePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, HAL_PinState_t PinState);
HAL_PinState_t HAL_GPIO_ReadPin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void HAL_GPIO_TogglePin(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
```

### Delay
```c
void HAL_Delay_Init(void);          // 系统时钟配置后调用一次
void HAL_Delay_us(uint32_t us);
void HAL_Delay_ms(uint32_t ms);
uint32_t HAL_GetTick(void);         // RTOS 运行时返回 xTaskGetTickCount()
```

### UART
```c
void HAL_UART_Init(UART_Handle_t* hUart);
void HAL_UART_SendByte(UART_Handle_t* hUart, uint8_t byte);
void HAL_UART_SendString(UART_Handle_t* hUart, const char* str);
uint8_t HAL_UART_ReceiveByte(UART_Handle_t* hUart);
void HAL_UART_EnableIRQ(UART_Handle_t* hUart, uint8_t Priority);
```

### UART DMA
```c
void HAL_UART_DMA_Rx_Init(UART_Handle_t* hUart, uint8_t* rx_buffer, uint16_t buffer_size);
```

### PWM
```c
void HAL_PWM_TimerInit(TIM_TypeDef* TIMx);
void HAL_PWM_ConfigChannel(TIM_TypeDef* TIMx, uint8_t Channel);
void HAL_PWM_SetCompare(TIM_TypeDef* TIMx, uint8_t Channel, uint16_t Compare);
```

### Software I2C
```c
void HAL_SoftI2C_Init(SoftI2C_Handle_t* hI2c);
void HAL_SoftI2C_Start(SoftI2C_Handle_t* hI2c);
void HAL_SoftI2C_Stop(SoftI2C_Handle_t* hI2c);
void HAL_SoftI2C_SendByte(SoftI2C_Handle_t* hI2c, uint8_t byte);
```

### Hardware I2C
```c
void HAL_HardI2C_Init(HardI2C_Handle_t *hI2c);
void HAL_HardI2C_ResetBus(HardI2C_Handle_t *hI2c);
int  HAL_HardI2C_WriteMem(HardI2C_Handle_t *hI2c, uint8_t DevAddr, uint8_t RegAddr, uint8_t *pData, uint16_t Size);
int  HAL_HardI2C_ReadMem(HardI2C_Handle_t *hI2c, uint8_t DevAddr, uint8_t RegAddr, uint8_t *pData, uint16_t Size);
```

### EXTI
```c
void HAL_EXTI_Init_PB12(void);
void HAL_EXTI_RegisterCallback_PB12(EXTI_Callback_t callback);
```

### 定时器输入捕获 (TIM1 CH4)
```c
void HAL_TIM1_CH4_IC_Init(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint32_t HAL_TIM1_CH4_GetPulseWidth(void); // 获取最新测量的脉宽(us)
```
- 内部采用双沿切换（上升沿→下降沿→上升沿）自动测量，结果保存在全局 `tim1_pulse_width`。
- 中断服务函数 `TIM1_CC_IRQHandler` 已实现，无需用户重写。

### ADC DMA 采集
```c
void HAL_ADC_DMA_Init(ADC_DMA_Handle_t* handle);
```
- 固定使用 ADC1，DMA1 通道1，循环模式。
- 句柄中必须指定 GPIO 引脚掩码（GPIO_Pins，支持多引脚或运算）、通道数组指针 ADC_Channels、通道总数 ChannelCount、以及目标内存 DMABuffer（长度需 ≥ ChannelCount）。
- 支持多通道交错阵列采集。当 ChannelCount > 1 时，底层自动开启 ADC 扫描模式与 DMA 内存地址递增。
- 包含完整的 ADC 校准时序，初始化完成后自动触发连续转换，应用层无需额外干预。

## 注意事项
所有 HAL 模块内部均不调用任何 RTOS API（除 `hal_delay` 根据宏选择），保持独立于操作系统。 
- `hal_uart_dma` 仅配置了 USART1 和 USART2 的 DMA 通道，若需 USART3 须补充。
- `hal_soft_i2c` 未实现读操作，仅适用于 OLED 纯写场景。
- `hal_hard_i2c` 的超时退出会触发总线复位，适合强实时系统。
- `hal_delay` 的软件循环延时系数 `fac_us_soft = SystemCoreClock / 1000000 / 5` 为经验值（72MHz 下 ≈ 14），仅在调度器启动前使用。若修改主频或更换 MCU 型号需重新校准该除数，否则延时可能不准。
- `hal_tim_ic` 使用 TIM1 高级定时器，注意中断优先级设置，避免影响其他高实时性任务。
- `hal_adc_dma` 的 `ADC_DMA_Handle_t` 中 `ADC_Channels` 和 `DMABuffer` 由调用方维护生命周期，`ChannelCount` 决定扫描模式与内存递增行为。多通道时 DMABuffer 长度须 ≥ ChannelCount。
- `hal_uart.h` 中遗留了废弃的 `MPL_LOGI`/`MPL_LOGE` 声明，实际日志功能已迁移至 `bsp_mpu6050.c` 中的 `BSP_MPL_LOGI`/`BSP_MPL_LOGE`，建议清理该头文件中的残留声明以避免误导。