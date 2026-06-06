#include "bsp_mpu6050.h"
#include "hal_hard_i2c.h"
#include "hal_exti.h"
#include "hal_delay.h" 
#include "sys_config.h"

// 物理外设句柄直接在 BSP 层定义并锁死
static HardI2C_Handle_t hI2C_MPU;

// 状态标志
static volatile uint8_t is_data_ready = 0;
static uint8_t g_mpu_is_working = 0;

// ========================================================
// 1. 消除 DD 层，直接提供给 DMP 库调用的底层物理接口
// ========================================================
// DMP 库 (inv_mpu.c) 内部使用宏定义绑定这几个物理函数
// #define i2c_write   BSP_MPU6050_WriteMem
// #define i2c_read    BSP_MPU6050_ReadMem
// #define delay_ms    HAL_Delay_ms
// #define get_ms      HAL_GetTick

int BSP_MPU6050_WriteMem(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char const *data) {
    return HAL_HardI2C_WriteMem(&hI2C_MPU, slave_addr, reg_addr, (uint8_t*)data, length);
}

int BSP_MPU6050_ReadMem(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data) {
    return HAL_HardI2C_ReadMem(&hI2C_MPU, slave_addr, reg_addr, data, length);
}

// ========================================================
// 2. EXTI 中断处理 (硬件层响应)
// ========================================================
static void MPU_DataReadyCallback(void) {
    is_data_ready = 1; 
    // 后续极限优化：可在此处使用 xTaskNotifyFromISR 直接唤醒 MotorTask
}

// ========================================================
// 3. BSP 暴露给 APP 层的直接调用接口
// ========================================================
void BSP_MPU6050_Init(void) {
    // 直接操作 HAL 层配置 I2C 硬件
    hI2C_MPU.I2Cx = I2C2;
    hI2C_MPU.GPIO_Port = GPIOB;
    hI2C_MPU.SCL_Pin = GPIO_Pin_10;
    hI2C_MPU.SDA_Pin = GPIO_Pin_11;
    hI2C_MPU.ClockSpeed = 400000; // 极限提速：MPU6050 支持 400kHz 快速模式
    
    HAL_HardI2C_Init(&hI2C_MPU);
    
    // 初始化外部中断 PB12
    HAL_EXTI_Init_PB12();
    HAL_EXTI_RegisterCallback_PB12(MPU_DataReadyCallback);

    // 直接调用 DMP 库的初始化函数（剥离 dev_mpu6050 的包裹）
    if (mpu_init() == 0 && mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL) == 0 && dmp_load_motion_driver_firmware() == 0) {
        g_mpu_is_working = 1;
        SYS_LOG("MPU", "[OK] MPU6050 DMP Ready\n");
    } else {
        g_mpu_is_working = 0;
        SYS_LOG("MPU", "[ERROR] MPU6050 Init Failed\n");
    }
}

// 供 fsm_motor_task 直接调用的数据读取接口
int BSP_MPU6050_GetData(MPU6050_Data_t *data) {
    if (!g_mpu_is_working) return -1;
    
    short gyro[3], accel[3];
    long quat[4];
    unsigned long timestamp;
    short sensors;
    unsigned char more;
    
    // 直接调用 DMP 底层读取，无间接指针开销
    if (dmp_read_fifo(gyro, accel, quat, &timestamp, &sensors, &more) == 0) {
        // ... 进行四元数到欧拉角的运算 ...
        return 0;
    }
    return -1;
}