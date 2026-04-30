#include "bsp_mpu6050.h"
#include "bsp_debug_uart.h"
#include "hal_hard_i2c.h"
#include "hal_exti.h"
#include "hal_delay.h" 
#include "hal_uart.h"
#include <stdarg.h>
#include "sys_config.h" // 确保引入了系统配置
static HardI2C_Handle_t hI2C_MPU;
static MPU6050_Handle_t hDev_MPU;
static volatile uint8_t is_data_ready = 0;
static uint8_t g_mpu_is_working = 0;
// 中断回调
static void MPU_DataReadyCallback(void) {
    is_data_ready = 1; 
    // 如需在此处唤醒RTOS任务（例如使用 xSemaphoreGiveFromISR），可在此拓展
}

// 适配器函数
static void Adapter_InitIO(void) {
    hI2C_MPU.I2Cx = I2C2;
    hI2C_MPU.GPIO_Port = GPIOB;
    hI2C_MPU.SCL_Pin = GPIO_Pin_10;
    hI2C_MPU.SDA_Pin = GPIO_Pin_11;
    hI2C_MPU.ClockSpeed = 100000;
    
    HAL_HardI2C_Init(&hI2C_MPU);
    HAL_EXTI_Init_PB12();
    HAL_EXTI_RegisterCallback_PB12(MPU_DataReadyCallback);
}

static int Adapter_WriteMem(uint8_t DevAddr, uint8_t RegAddr, uint8_t *Data, uint16_t Size) {
    return HAL_HardI2C_WriteMem(&hI2C_MPU, DevAddr, RegAddr, Data, Size);
}

static int Adapter_ReadMem(uint8_t DevAddr, uint8_t RegAddr, uint8_t *Data, uint16_t Size) {
    return HAL_HardI2C_ReadMem(&hI2C_MPU, DevAddr, RegAddr, Data, Size);
}

static void Adapter_DelayMs(uint32_t ms) {
    HAL_Delay_ms(ms);
}

static void Adapter_GetTick(uint32_t *timestamp) {
    if(timestamp) *timestamp = HAL_GetTick(); // 使用系统的统一Tick
}

void BSP_MPU6050_Init(void) {
    hDev_MPU.io.InitIO = Adapter_InitIO;
		//printf("[OK] hDev_MPU.io.InitIO");
    hDev_MPU.io.WriteMem = Adapter_WriteMem;
		//printf("[OK] hDev_MPU.io.WriteMem ");
    hDev_MPU.io.ReadMem = Adapter_ReadMem;
		//printf("[OK] hDev_MPU.io.ReadMem = Adapter_ReadMem ");
    hDev_MPU.io.DelayMs = Adapter_DelayMs;
		//printf("[OK] hDev_MPU.io.ReadMem = Adapter_DelayMs ");
    hDev_MPU.io.GetTick = Adapter_GetTick;
		//printf("[OK] hDev_MPU.io.ReadMem = Adapter_GetTick ");

    if (Dev_MPU6050_Init(&hDev_MPU) == 0) {
        g_mpu_is_working = 1;
        printf("[OK] MPU6050 Ready\n");
    } else {
        g_mpu_is_working = 0;
        printf("[ERROR] MPU6050 Init Failed\n");
    }
	
}

uint8_t BSP_MPU6050_IsDataReady(void) {
    return is_data_ready;
}

void BSP_MPU6050_ClearDataReady(void) {
    is_data_ready = 0;
}

int BSP_MPU6050_GetData(MPU6050_Data_t *data) {
    return Dev_MPU6050_Read_DMP(data);
}


uint8_t BSP_MPU6050_IsWorking(void) {
    return g_mpu_is_working;
}
void BSP_MPL_LOGI(const char* fmt, ...)
{
#if ENABLE_DEBUG_PRINT
    va_list args;
    va_start(args, fmt);
    printf("[INFO] ");
    char buffer[128];
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    printf("%s",buffer);
    printf("\r\n");
    va_end(args);
#else
    (void)fmt; // 消除未使用参数的警告
#endif
}

void BSP_MPL_LOGE(const char* fmt, ...)
{
#if ENABLE_DEBUG_PRINT
    va_list args;
    va_start(args, fmt);
    printf("[ERROR] ");
    char buffer[128];
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    printf("%s",buffer); // 修复了你原代码中未打印 buffer 的潜在问题
    printf("\r\n");
    va_end(args);
#else
    (void)fmt;
#endif
}
