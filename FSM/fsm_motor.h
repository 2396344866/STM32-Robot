#ifndef FSM_MOTOR_H
#define FSM_MOTOR_H

#include "fsm_core.h"
#include <stdio.h>
#include "bsp_mpu6050.h"
// 状态定义保持不变
typedef enum {
    STATE_MOTOR_STOP,
    STATE_MOTOR_FORWARD,
    STATE_MOTOR_BACKWARD,
    STATE_MOTOR_LEFT,
    STATE_MOTOR_RIGHT,
    STATE_MOTOR_ROT_L,
    STATE_MOTOR_ROT_R,
    STATE_MOTOR_MAX
} motor_state_t;

// --- 新增：数据驱动的核心结构 ---

// 1. 关键帧：一瞬间8个舵机的角度
typedef struct {
    uint8_t angles[8]; // 对应 8 个舵机 ID
} pose_frame_t;

// 2. 动作序列：包含多帧数据和帧数
typedef struct {
    const pose_frame_t* frames;
    uint8_t frame_count;
} gait_sequence_t;

// --- 更新：FSM 上下文 ---
typedef struct {
    const gait_sequence_t* current_seq; // 当前正在播放的动作序列
    int      seq_step;                  // 当前播放到第几帧
    uint32_t last_tick;                 // 上一次执行的时间
} motor_ctx_t;

extern MPU6050_Data_t g_imu_data;

/* --- P0 故障安全：设备级安全响应标志（跨任务共享，由网络/电源任务写，motor 任务读） --- */
/* 网络失联标志：fsm_network 在线态心跳超时后置 1，恢复在线清 0。motor 任务读此标志做"失联即停"。 */
extern volatile uint8_t g_net_link_lost;
/* 故障安全激活标志：motor 任务检测到跌倒/IMU失效/失联后进入安全态时置 1，恢复正常清 0。 */
extern volatile uint8_t g_fault_safe_active;

void Motor_FSM_Setup(fsm_t* fsm);
void Motor_FSM_task(void *pvParameters);
/* P0-① 设备级安全态入口：收腿趴下 + 停走，防翻倒/盲走。供 motor 任务与电源欠压共用。 */
void enter_safe_state(void);
#endif
