# DD 层 README

## 定位

设备驱动层（Device Driver），亦称纯软件逻辑层。此层**完全不依赖任何硬件寄存器**，只通过抽象函数指针（V-Table）与外界交互，面向“逻辑操作”实现算法。具体的硬件操作由上层 BSP 在初始化时注入。

## 文件清单

| 文件名 | 说明 |
|--------|------|
| `dev_led.h` | 定义 LED 抽象接口 `LED_IO_Interface_t`（WritePin, ReadPin）及句柄，逻辑层只关心有效电平（高/低） |
| `dev_led.c` | 实现 LED 的开关、翻转逻辑，翻转时优先使用硬件读取状态，降级为软件状态 |
| `dev_mpu6050.h` | 定义 MPU6050 依赖接口（IO 初始化、寄存器读写、延时、时钟获取），姿态数据结构体 |
| `dev_mpu6050.c` | 封装 DMP 库，完成传感器初始化、校准、FIFO 读取与四元数→欧拉角转换 |
| `dev_servo.h` | 定义舵机句柄及 PWM 写入函数指针，保存角度范围 |
| `dev_servo.c` | 实现角度→脉宽线性换算（0°→500us, 180°→2500us，周期 20ms） |
| `dev_ssd1306.h` | OLED 抽象接口（InitIO, WriteCmd, WriteDat, DelayMs） |
| `dev_ssd1306.c` | OLED 初始化时序、清屏、光标设置、字符/字符串/数字显示（含字模引用） |
| `dev_ssd1306_font.h` | 8x16 字模数据，ASCII 可见字符点阵 |
| `dev_hcsr04.h` | 超声波测距模块抽象接口（Trig 控制、延时、脉冲宽度读取） |
| `dev_hcsr04.c` | 实现距离换算（基于声速 340m/s） |
| `dev_mq2.h` | 烟雾传感器抽象接口（ADC 读取函数） |
| `dev_mq2.c` | 实现 ppm 浓度计算（电压→浓度线性映射示例） |
| `dev_dht11.h` | 温湿度传感器抽象接口（引脚方向切换、读写、延时） |
| `dev_dht11.c` | 实现单总线协议读取温湿度原始数据，含校验与超时处理 |

## 设计核心：依赖注入

每个设备驱动定义一个**包含函数指针的结构体**（如 `LED_IO_Interface_t`、`MPU6050_IO_Interface_t`），作为设备句柄的一部分。例如：

```c
// LED
typedef struct {
    void (*WritePin)(uint8_t state);
    uint8_t (*ReadPin)(void);
} LED_IO_Interface_t;

// HCSR04
typedef struct {
    void (*Trig_Write)(uint8_t state);
    void (*Delay_us)(uint32_t us);
    uint32_t (*GetPulseWidth)(void);
} HCSR04_Handle_t;
```

句柄（`LED_Handle_t`、`HCSR04_Handle_t` 等）持有这些接口和逻辑属性。初始化时，由 BSP 层将 HAL 层的具体实现函数注入句柄，完成运行时绑定。这使得 DD 层可以脱离具体硬件仿真测试。

## 主要 API

### LED
```c
void Dev_LED_Init(LED_Handle_t* hLed);
void Dev_LED_On(LED_Handle_t* hLed);
void Dev_LED_Off(LED_Handle_t* hLed);
void Dev_LED_Toggle(LED_Handle_t* hLed);
```

### MPU6050
```c
int  Dev_MPU6050_Init(MPU6050_Handle_t *handle);
void Dev_MPU6050_RunCalibration(void);
int  Dev_MPU6050_Read_DMP(MPU6050_Data_t *out_data);
```
**注意：** DMP 固件加载后强制等待 10 秒收敛，不可删除。

### Servo
```c
void Dev_Servo_Init(Servo_Handle_t* hServo, Servo_WritePWM_Func write_func);
void Dev_Servo_SetAngle(Servo_Handle_t* hServo, float angle);
```

### OLED (SSD1306)
```c
void Dev_SSD1306_Init(SSD1306_Handle_t *handle);
void Dev_SSD1306_Clear(SSD1306_Handle_t *handle);
void Dev_SSD1306_ShowChar(...);
void Dev_SSD1306_ShowString(...);
void Dev_SSD1306_ShowNum(...);
void Dev_SSD1306_ShowSignedNum(...);
void Dev_SSD1306_ShowHexNum(...);
void Dev_SSD1306_ShowBinNum(...);
```

### 超声波测距 (HCSR04)
```c
void Dev_HCSR04_Init(HCSR04_Handle_t* handle,
                     void (*trig_fn)(uint8_t),
                     void (*delay_fn)(uint32_t),
                     uint32_t (*ic_fn)(void));
float Dev_HCSR04_GetDistance(HCSR04_Handle_t* handle); // 返回距离(cm)，失败返回 -1.0
```
**时序逻辑：** 在 `GetDistance` 中自动产生 12us 触发脉冲，然后读取脉冲宽度并换算距离。

### 烟雾传感器 (MQ2)
```c
void Dev_MQ2_Init(MQ2_Handle_t* handle, uint16_t (*read_fn)(void));
float Dev_MQ2_GetPPM(MQ2_Handle_t* handle); // 返回 ppm 浓度值
```
**换算公式：** 将 12 位 ADC 值转换为电压，再线性映射为 ppm（示例公式 `vol * 100.0f`）。

### 温湿度传感器 (DHT11)
```c
void Dev_DHT11_Init(DHT11_Handle_t* handle);
uint8_t Dev_DHT11_Read(DHT11_Handle_t* handle, uint8_t *temp, uint8_t *humi);
```
**注意：** `DHT11_Handle_t` 中需注入引脚方向切换函数 `SetPinOut()`、`SetPinIn()`，读写函数 `WritePin()`、`ReadPin()`，以及微秒/毫秒延时函数。读取函数返回 1 表示成功并通过校验。

## 约束与注意

- 所有设备驱动句柄中的接口函数指针必须在初始化前由 BSP 层正确注入，否则会有空指针风险。
- MPU6050 驱动强依赖 `inv_mpu` 和 `inv_mpu_dmp_motion_driver` 库，通过全局指针 `g_mpu_handle` 桥接抽象接口。
- 字库 `SSD1306_F8x16` 字符索引从空格(0x20)开始，若传入不可打印字符会越界。
- OLED 坐标：Line 1~4, Column 1~16。
- DHT11 的读取协议包含精确的微秒级延时和超时判断，对 `DelayUs` 的准确度要求较高，软件 I2C 场景下需确保不被中断打断。
- MQ2 的 PPM 计算为示例线性映射，实际应用中需根据传感器标定曲线调整。
- HCSR04 依赖输入捕获模块提供精确脉宽，若测量失败（未收到回波）会自动返回 -1.0。