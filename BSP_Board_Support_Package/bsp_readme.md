# BSP 层 README

## 定位

板级支持包（Board Support Package），负责**实例化硬件资源**，将具体的 MCU 外设、引脚、时钟配置与 DD 层逻辑绑定，形成可供应用层直接调用的接口。

本层知道的硬件细节包括：
- 哪个 GPIO 连接了 LED
- USART1 用于 WiFi，USART2 用于调试
- PB8/PB9 作软件 I2C 驱动 OLED
- MPU6050 使用硬件 I2C2，INT 脚为 PB12
- 8 路舵机分别使用 TIM2/3/4 的特定通道
- MQ2、HCSR04、DHT11 使用的 ADC、输入捕获、GPIO 等

## 文件清单与资源映射

| 文件 | 对应硬件 | 说明 |
|------|----------|------|
| `bsp_debug_uart.c/h` | USART2, PA2(TX), PA3(RX) | 调试串口，提供 FreeRTOS 队列 `xDebugRxQueue`，可在中断中唤醒解析任务；通过 `ENABLE_DEBUG_PRINT` 宏裁剪 |
| `bsp_esp8266.c/h` | USART1, PA9(TX), PA10(RX) | WiFi 模块，同样使用 DMA+IDLE 接收，数据通过 `xNetRxQueue` 传递 |
| `bsp_led.c/h` | PC13 | 用户 LED1，低电平点亮，初始化后可直接 On/Off/Toggle |
| `bsp_mpu6050.c/h` | I2C2 (PB10, PB11), INT: PB12 | IMU 传感器，若成功初始化则 `g_mpu_is_working=1`，提供数据就绪标志、欧拉角读取及日志接口 `BSP_MPL_LOGI`/`BSP_MPL_LOGE` |
| `bsp_oled.c/h` | 软件 I2C: PB8(SCL), PB9(SDA) | 0.96寸 OLED，地址 0x78，扩展了多种显示接口 |
| `bsp_servo.c/h` | TIM2,3,4 的多路 PWM | 8 个舵机（LT/RT/RB/LB 髋/膝关节），每个对应一个 `Servo_Handle_t` |
| `bsp_sensor.c/h` | ADC1_CH4(PA4), ADC1_CH5(PA5), TIM1_CH4(PA11), PA8(Trig), PA15(DHT11) | 集成烟雾(MQ2)、光照(PA5 ADC1_CH5)、超声波(HCSR04)、温湿度(DHT11)传感器，提供统一初始化与数据读取接口，ADC DMA 双通道采集 |

## 典型初始化流程

```c
// 在 main() 中依次调用：
BSP_DebugUART_Init(115200);   // 若 ENABLE_DEBUG_PRINT=1，初始化调试串口
BSP_LED_Init();               // 初始化 PC13
BSP_OLED_Init();              // 软件 I2C + SSD1306 初始化
BSP_MPU6050_Init();           // 硬件 I2C + MPU6050 DMP 初始化
BSP_ESP8266_Init(115200);     // WiFi 模块初始化
BSP_Servo_Init();             // 8 路舵机 PWM 初始化
BSP_Sensors_Init();           // MQ2/HCSR04/DHT11 初始化
```

## 应用接口风格

所有 BSP 接口统一为 `BSP_<模块>_<动作>`，例如：

### LED
```c
void BSP_LED1_On(void);
void BSP_LED1_Off(void);
void BSP_LED1_Toggle(void);
```

### OLED
```c
void BSP_OLED_Clear(void);
void BSP_OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
// ...其他显示接口
```

### 舵机
```c
void BSP_Servo_Set_Left_Top_Knee(float angle);
void BSP_Servo_Set_Right_Top_Hip(float angle);
// ...其他6个关节
```

### MPU6050
```c
uint8_t BSP_MPU6050_IsDataReady(void);
void BSP_MPU6050_ClearDataReady(void);
int BSP_MPU6050_GetData(MPU6050_Data_t *data);
uint8_t BSP_MPU6050_IsWorking(void);
void BSP_MPL_LOGI(const char* fmt, ...);
void BSP_MPL_LOGE(const char* fmt, ...);
```

### ESP8266
```c
void BSP_ESP8266_SendString(const char* str); // 内部会同时 printf 到调试串口
```

### 传感器集成
```c
void BSP_Sensors_Init(void);
float BSP_Sensor_GetDistance(void);             // 超声波测距 (cm)
float BSP_Sensor_GetSmoke(void);                // MQ2 烟雾 ppm
float BSP_Sensor_GetLight(void);               // 光照强度 (Lux)，ADC1_CH5 (PA5) 采集
uint8_t BSP_Sensor_ReadDHT11(uint8_t *temp, uint8_t *humi);  // 温湿度，返回1表示成功
```

## 关键机制

- **调试输出**：`fputc` 重定向到 USART2，由 `ENABLE_DEBUG_PRINT` 控制。该宏也用来裁剪 `BSP_DebugUART_Init` 和整个调试 ISR。
- **WiFi 与调试队列**：均在各自 UART 中断中利用 DMA+IDLE 接收不定长数据，将包长度放入 FreeRTOS 队列，后续任务可取出长度后处理 `g_xxx_rx_buf`。
- **MPU6050 中断**：PB12 上升沿触发，仅置位 `is_data_ready` 标志，不直接读 FIFO，由任务轮询后调用 `BSP_MPU6050_GetData`。
 **传感器集成**：`bsp_sensor.c` 将 MQ2、光照、HCSR04、DHT11 四个传感器集中管理。其中：
  - MQ2 使用 ADC1 CH4 (PA4)，光照使用 ADC1 CH5 (PA5)，共用 DMA 双通道循环采集，数据分别写入 `adc_dma_buffer[0]` 和 `adc_dma_buffer[1]`。
  - 光照通过线性插值映射为 0~1000 Lux 范围输出。
  - HCSR04 依赖 TIM1 CH4 输入捕获，通过 `HAL_TIM1_CH4_GetPulseWidth` 获取回波脉宽。
  - DHT11 使用 PA15，初始化时需关闭 JTAG（`GPIO_Remap_SWJ_JTAGDisable`），并动态切换引脚方向以满足单总线协议。

## 注意事项

- `bsp_esp8266.c` 中的 `printf("[STM32 -> WIFI]: %s", str)` 会无条件发送到 debug 串口，若串口未初始化可能阻塞（由 `fputc` 内宏控制，关闭调试则为空函数）。
- 舵机映射与物理连线强相关，修改硬件后须同步更新 `TIMx_CHy_Write` 与句柄绑定。
- DHT11 的 PA15 引脚默认是 JTDI 功能，若之前未关闭 JTAG 调试功能会导致引脚初始化失败，程序中通过 `GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE)` 强制释放该引脚。
- 部分 BSP 模块（如 LED、MPU6050、Servo）内部持有 DD 层句柄的静态实例，应用层不需关心句柄，直接调用函数即可。
- 调试串口的 `QueueHandle_t xDebugRxQueue` 和网络串口的 `QueueHandle_t xNetRxQueue` 以及对应的缓冲区均为全局可访问，方便各任务使用。