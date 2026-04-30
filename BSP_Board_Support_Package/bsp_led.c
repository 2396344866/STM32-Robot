#include "bsp_led.h"
#include "dev_led.h"
#include "hal_gpio.h" // 引用物理层

// ==========================================
// 1. 静态胶水函数/ 适配器
// ==========================================
// 即使 Dev 层要求 uint8_t, HAL 层要求枚举，在这里进行类型转换

// 适配 LED1 (物理对应 PC13)
static void LED1_Write(uint8_t state)
{
    // 将 Dev 层的逻辑 1/0 转换为 HAL 层的枚举 SET/RESET
    HAL_PinState_t pin_state = (state == 1) ? HAL_PIN_SET : HAL_PIN_RESET;
    HAL_GPIO_WritePin(GPIOC, GPIO_Pin_13, pin_state);
}

static uint8_t LED1_Read(void)
{
    // 调用 HAL 层接口读取引脚状态 (通常读取 ODR 寄存器)
    HAL_PinState_t state = HAL_GPIO_ReadPin(GPIOC, GPIO_Pin_13);
    
    // 将硬件枚举转换为逻辑值
    return (state == HAL_PIN_SET) ? 1 : 0;
}
//(如果有 LED2，可以再写一个 LED2_Write 绑定不同引脚)

// ==========================================
// 2. 实例化设备对象 (Dependency Injection)
// ==========================================
static LED_Handle_t hLED1 = {
    .io = {
        .WritePin = LED1_Write,
				.ReadPin  = LED1_Read,
    },
    .active_level = LED_ACTIVE_LOW  // PC13 低电平点亮
};

// ==========================================
// 3. 公共接口实现
// ==========================================
void BSP_LED_Init(void)
{
    // A. 硬件初始化 (Hardware Init) - 负责具体的电气特性
    // 注意：Dev 层不管引脚是推挽还是开漏，这里 BSP 必须管
    HAL_GPIO_Init(GPIOC, GPIO_Pin_13, HAL_GPIO_MODE_OUTPUT_PP);

    // B. 逻辑初始化 (Logic Init)
    Dev_LED_Init(&hLED1);
}

// 暴露给应用层的操作接口
void BSP_LED1_On(void)     { Dev_LED_On(&hLED1); }
void BSP_LED1_Off(void)    { Dev_LED_Off(&hLED1); }
void BSP_LED1_Toggle(void) { Dev_LED_Toggle(&hLED1); }
