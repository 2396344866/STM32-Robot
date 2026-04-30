#include "dev_mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include <math.h>
#include <stddef.h>
#include "hal_uart.h"
// 全局静态指针，专供DMP库底层的宏调用
static MPU6050_Handle_t *g_mpu_handle = NULL;

// 提供给DMP的对接函数
int Sensors_I2C_WriteRegister(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data) {
    if(!g_mpu_handle) return -1;
    return g_mpu_handle->io.WriteMem(slave_addr << 1, reg_addr, data, length);
}

int Sensors_I2C_ReadRegister(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data) {
    if(!g_mpu_handle) return -1;
    return g_mpu_handle->io.ReadMem(slave_addr << 1, reg_addr, data, length);
}

void Delay_ms(uint32_t ms) {
    if(g_mpu_handle) g_mpu_handle->io.DelayMs(ms);
}

void get_tick_count(unsigned long *count) {
    if(g_mpu_handle) g_mpu_handle->io.GetTick((uint32_t*)count);
}

// 内部函数：四元数转欧拉角 (修复坐标轴错位问题)
static void quat_to_euler(long *quat, float *ypr) {
    float q0 = quat[0] / 1073741824.0f;
    float q1 = quat[1] / 1073741824.0f;
    float q2 = quat[2] / 1073741824.0f;
    float q3 = quat[3] / 1073741824.0f;

    // 采用与旧版严格对应的Z-Y-X解算方程
    ypr[1] = asinf(-2 * q1 * q3 + 2 * q0 * q2) * 57.3f;                                   // Pitch
    ypr[2] = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3f;    // Roll
    ypr[0] = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 57.3f;    // Yaw
}
void Dev_MPU6050_RunCalibration(void) {
    long gyro[3];
    long accel[3];
    int result;

    printf("[INFO] 开始硬件校准，请保持模块绝对静止...\r\n");
    Delay_ms(2000); 

    result = mpu_run_self_test(gyro, accel);

    // 0x07: 全通过; 0x03: 基础通过; 0x04: 自检未达标但零偏已获取
    if (result == 0x07 || result == 0x03 || result == 0x04) {
        printf("[INFO] 零偏数据获取完成！(状态码: 0x%X)\r\n", result);
        printf("[INFO] Gyro Bias: %ld, %ld, %ld\r\n", gyro[0], gyro[1], gyro[2]);
        printf("[INFO] Accel Bias: %ld, %ld, %ld\r\n", accel[0], accel[1], accel[2]);
    } else {
        // 返回0代表底层I2C通信彻底失败
        printf("[ERROR] 获取零偏失败，底层I2C错误: 0x%X\r\n", result);
    }
}
int Dev_MPU6050_Init(MPU6050_Handle_t *handle) {
    if(!handle) return -1;
    
    g_mpu_handle = handle;
    handle->io.InitIO();
    
    if(mpu_init(NULL) != 0) return -1;
    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    
    mpu_set_sample_rate(200); 
    mpu_set_lpf(42);
    mpu_set_gyro_fsr(2000); 
    mpu_set_accel_fsr(2);
    
    if (dmp_load_motion_driver_firmware() != 0) return -1;
    
		
    static signed char gyro_orientation[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));
    
    dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL);
		dmp_set_fifo_rate(100);  
    mpu_set_dmp_state(1);   
    
    // 先配置中断寄存器
    uint8_t data = 0x02;
    Sensors_I2C_WriteRegister(0x68, 0x37, 1, &data); 
    Sensors_I2C_WriteRegister(0x68, 0x38, 1, &data);
    
    // 强制阻塞 10 秒，让 DMP 在后台依靠真实物理重力完成四元数收敛
    // 警告：绝不能在此处调用 mpu_reset_fifo()
    handle->io.DelayMs(10000);

    // ===================================================
    return 0;
}

int Dev_MPU6050_Read_DMP(MPU6050_Data_t *out_data) {
    short sensors;
    unsigned char more;
    long quat[4];
    unsigned long timestamp;
    int ret;
    uint8_t has_valid_data = 0;
		uint8_t timeout_cnt = 10;
    
    // 采用循环读取(Drain)策略。若因RTOS调度延迟导致FIFO堆积多帧数据，
    // 持续读取直至FIFO清空，仅提取具备最高时效性的最新一帧姿态数据。
    do {
			ret = dmp_read_fifo(out_data->gyro, out_data->accel, quat, &timestamp, &sensors, &more);
			if (ret == 0) {
            if (sensors & INV_WXYZ_QUAT) {
                has_valid_data = 1; // 标记获取到有效四元数
            }
        } 
        else if (ret == -2) {
						printf("ret == -2  mpu_reset_fifo");
            mpu_reset_fifo(); // 发生严重溢出时恢复系统状态
            return -1;
        }
				else{
					break; // 捕获底层I2C失败，立即跳出循环，交由RTOS进行下一次调度
				}
				timeout_cnt--;
    } while (more > 0 && timeout_cnt > 0);// 持续排空底层缓存
    
    // 执行最终的数据解析与赋值
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
