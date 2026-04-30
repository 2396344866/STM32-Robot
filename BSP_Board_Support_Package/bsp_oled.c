#include "bsp_oled.h"
#include "dev_ssd1306.h"  // 引用设备驱动逻辑
#include "hal_soft_i2c.h" // 引用通用软件 I2C HAL
#include "hal_delay.h"
/* ==========================================
 * 1. 硬件资源定义 (Resource Instantiation)
 * ========================================== */

// 定义 OLED 使用的 I2C 总线引脚资源
static SoftI2C_Handle_t hI2c_OLED = {
    .port_scl = GPIOB,
    .pin_scl  = GPIO_Pin_8,
    .port_sda = GPIOB,
    .pin_sda  = GPIO_Pin_9
};

// 定义 OLED 设备句柄
static SSD1306_Handle_t hDev_OLED;

/* ==========================================
 * 2. 接口适配器 (Adapter / Middleware)
 * 将 Device 层的抽象操作映射到 HAL 层的具体实现
 * ========================================== */
// SSD1306 I2C 地址 (0x78) 及控制字节定义
#define OLED_I2C_ADDR   0x78
#define OLED_CTRL_CMD   0x00
#define OLED_CTRL_DATA  0x40
// 适配 IO 初始化
static void Adapter_InitIO(void)
{
    // 调用 HAL 层初始化具体的 I2C 引脚
    HAL_SoftI2C_Init(&hI2c_OLED);
}
// 适配写命令：Start -> Addr -> Ctrl(0x00) -> Cmd -> Stop
static void Adapter_WriteCmd(uint8_t cmd)
{
    HAL_SoftI2C_Start(&hI2c_OLED);
    HAL_SoftI2C_SendByte(&hI2c_OLED, OLED_I2C_ADDR); // 从机地址
    HAL_SoftI2C_SendByte(&hI2c_OLED, OLED_CTRL_CMD); // 控制字节：命令
    HAL_SoftI2C_SendByte(&hI2c_OLED, cmd);           // 发送命令
    HAL_SoftI2C_Stop(&hI2c_OLED);
}

// 适配写数据：Start -> Addr -> Ctrl(0x40) -> Dat -> Stop
static void Adapter_WriteDat(uint8_t dat)
{
    HAL_SoftI2C_Start(&hI2c_OLED);
    HAL_SoftI2C_SendByte(&hI2c_OLED, OLED_I2C_ADDR); // 从机地址
    HAL_SoftI2C_SendByte(&hI2c_OLED, OLED_CTRL_DATA);// 控制字节：数据
    HAL_SoftI2C_SendByte(&hI2c_OLED, dat);           // 发送数据
    HAL_SoftI2C_Stop(&hI2c_OLED);
}
static void Adapter_DelayMs(uint32_t ms)
{
    HAL_Delay_ms(ms);
}

/* ==========================================
 * 3. BSP 初始化 (Dependency Injection)
 * ========================================== */

void BSP_OLED_Init(void)
{
    // 依赖注入：将适配函数填入设备句柄的虚表 (V-Table)
    hDev_OLED.io.InitIO   = Adapter_InitIO;
    hDev_OLED.io.WriteCmd = Adapter_WriteCmd;
    hDev_OLED.io.WriteDat = Adapter_WriteDat;
    hDev_OLED.io.DelayMs  = Adapter_DelayMs;

    // 初始化设备驱动逻辑
    Dev_SSD1306_Init(&hDev_OLED);
}

/* ==========================================
 * 4. 应用接口封装 (Facade / Proxy)
 * 将调用转发给 hDev_OLED 句柄
 * ========================================== */

void BSP_OLED_Clear(void)
{
    Dev_SSD1306_Clear(&hDev_OLED);
}

void BSP_OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
    Dev_SSD1306_ShowChar(&hDev_OLED, Line, Column, Char);
}

void BSP_OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
    Dev_SSD1306_ShowString(&hDev_OLED, Line, Column, String);
}

void BSP_OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    Dev_SSD1306_ShowNum(&hDev_OLED, Line, Column, Number, Length);
}

void BSP_OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
    Dev_SSD1306_ShowSignedNum(&hDev_OLED, Line, Column, Number, Length);
}

void BSP_OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    Dev_SSD1306_ShowHexNum(&hDev_OLED, Line, Column, Number, Length);
}

void BSP_OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    Dev_SSD1306_ShowBinNum(&hDev_OLED, Line, Column, Number, Length);
}
