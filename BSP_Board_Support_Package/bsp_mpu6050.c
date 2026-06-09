#include "bsp_mpu6050.h"
#include "hal_hard_i2c.h"
#include "hal_exti.h"
#include "hal_delay.h" 
#include "sys_config.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

static HardI2C_Handle_t hI2C_MPU;
static uint8_t g_mpu_is_working = 0;

// 引入全局电机任务句柄，用于中断直接唤醒
extern TaskHandle_t MotorTask_Handler;

/* ========================================================
 * 1. 物理穿透：为 DMP 库提供纯硬件绑定的底层接口
 * ======================================================== */
int Sensors_I2C_WriteRegister(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data) {
    return HAL_HardI2C_WriteMem(&hI2C_MPU, slave_addr << 1, reg_addr, data, length);
}

int Sensors_I2C_ReadRegister(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data) {
    return HAL_HardI2C_ReadMem(&hI2C_MPU, slave_addr << 1, reg_addr, data, length);
}

void Delay_ms(uint32_t ms) {
    HAL_Delay_ms(ms);
}

void get_tick_count(unsigned long *count) {
    *count = (unsigned long)HAL_GetTick();
}

/* ========================================================
 * 2. 中断直达：EXTI 硬件触发 FreeRTOS 上下文切换
 * ======================================================== */
static void MPU_DataReadyCallback(void) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // 极限压榨：直接向电机任务发送硬件就绪通知，替代全局变量轮询
    vTaskNotifyGiveFromISR(MotorTask_Handler, &xHigherPriorityTaskWoken);
    
    // 若电机任务优先级高于当前运行任务，强制触发 PendSV 进行微秒级切换
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* ========================================================
 * 3. 核心解算：四元数转欧拉角 (原属于 DD 层的纯运算移入)
 * ======================================================== */
static void quat_to_euler(long *quat, float *ypr) {
    float q0 = quat[0] / 1073741824.0f;
    float q1 = quat[1] / 1073741824.0f;
    float q2 = quat[2] / 1073741824.0f;
    float q3 = quat[3] / 1073741824.0f;

    ypr[1] = asinf(-2 * q1 * q3 + 2 * q0 * q2) * 57.3f;                                   // Pitch
    ypr[2] = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3f;    // Roll
    ypr[0] = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 57.3f;    // Yaw
}

/* ========================================================
 * 4. BSP 暴露接口
 * ======================================================== */
void BSP_MPU6050_Init(void) {
    hI2C_MPU.I2Cx = I2C2;
    hI2C_MPU.GPIO_Port = GPIOB;
    hI2C_MPU.SCL_Pin = GPIO_Pin_10;
    hI2C_MPU.SDA_Pin = GPIO_Pin_11;
    hI2C_MPU.ClockSpeed = 400000; // 400kHz 满血 I2C 硬件配置
    
    HAL_HardI2C_Init(&hI2C_MPU);
    HAL_EXTI_Init_PB12();
    HAL_EXTI_RegisterCallback_PB12(MPU_DataReadyCallback);

    if (mpu_init(NULL) == 0 && mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL) == 0 && dmp_load_motion_driver_firmware() == 0) {
        
        static signed char gyro_orientation[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
        dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL);
        dmp_set_fifo_rate(100);  // 100Hz = 每 10ms 触发一次 EXTI 中断
        mpu_set_dmp_state(1);   
        
        // 开启 MPU6050 的 INT 引脚物理电平输出
        uint8_t data = 0x02;
        Sensors_I2C_WriteRegister(0x68, 0x37, 1, &data); 
        Sensors_I2C_WriteRegister(0x68, 0x38, 1, &data);
        
        HAL_Delay_ms(10000); // DMP 收敛等待
        g_mpu_is_working = 1;
        SYS_LOG("MPU", "[OK] MPU6050 DMP Ready\n");
    } else {
        g_mpu_is_working = 0;
        SYS_LOG("MPU", "[ERROR] MPU6050 Init Failed\n");
    }
}

int BSP_MPU6050_GetData(MPU6050_Data_t *out_data) {
    if (!g_mpu_is_working) return -1;
    
    short sensors;
    unsigned char more;
    long quat[4];
    unsigned long timestamp;
    int ret;
    uint8_t has_valid_data = 0;
    uint8_t timeout_cnt = 10;
    
    // 原汁原味的 FIFO 排空逻辑，确保提取最新有效帧
    do {
        ret = dmp_read_fifo(out_data->gyro, out_data->accel, quat, &timestamp, &sensors, &more);
        if (ret == 0) {
            if (sensors & INV_WXYZ_QUAT) has_valid_data = 1;
        } 
        else if (ret == -2) {
            mpu_reset_fifo(); 
            return -1;
        }
        else {
            break; 
        }
        timeout_cnt--;
    } while (more > 0 && timeout_cnt > 0);
    
    if (has_valid_data) {
        float ypr[3];
        quat_to_euler(quat, ypr);
        out_data->yaw = ypr[0];
        out_data->pitch = ypr[1];
        out_data->roll = ypr[2];
        return 0; 
    }
    return -1; 
}

uint8_t BSP_MPU6050_IsWorking(void){ 
	return  g_mpu_is_working; 
}

#include <stdarg.h>
#include <stdio.h>

void BSP_MPL_LOGI(const char* fmt, ...) {
#if ENABLE_DEBUG_PRINT
    printf("[MPU-INFO] ");
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
#endif
}

void BSP_MPL_LOGE(const char* fmt, ...) {
#if ENABLE_DEBUG_PRINT
    printf("[MPU-ERR]  ");
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
#endif
}
