#include "fsm_motor.h"
#include "fsm_core.h"
#include <math.h>
#include "FreeRTOS.h"
#include "event_bus.h"
#include "sys_events.h"
#include "task.h"
#include "hal_pwm.h" 
#include "bsp_servo.h"
#include "bsp_mpu6050.h"
#include "state_repo.h"   // 中央状态仓库（ST_IMU_DATA 等）
static fsm_t g_Motor_fsm;
// IMU 数据不再用全局变量：Motor 单写者写入 ST_IMU_DATA，自身 PD 控制与 Network 上报均经 state_read_if_new 读取
static MPU6050_Data_t g_imu_snap;   // Motor 本地 IMU 快照
static uint32_t g_imu_seq = 0;      // ST_IMU_DATA 读取游标
static inline void motor_imu_refresh(void) {
    (void)state_read_if_new(ST_IMU_DATA, &g_imu_snap, &g_imu_seq);
}
// --- 配置宏 ---
#define ACTION_INTERVAL_MS  200   // 动作每一步的间隔时间，加速以产生惯性

#define STAND_INTERVAL_MS   500    // 站立复位的间隔
/* ============================================================
 * 数据结构定义
 * ============================================================ */

// 定义舵机 ID 索引，方便阅读数组 (对应你 8 个舵机的顺序)
enum {
    ID_LT_KNEE = 0, // Left Top Knee
    ID_LT_HIP,      // Left Top Hip
    ID_RT_KNEE,     // Right Top Knee
    ID_RT_HIP,      // Right Top Hip
    ID_RB_KNEE,     // Right Bottom Knee
    ID_RB_HIP,      // Right Bottom Hip
    ID_LB_KNEE,     // Left Bottom Knee
    ID_LB_HIP       // Left Bottom Hip
};
/* ============================================================
 * 1. 步态数据表 (The Gait Data - Flash Storage)
		本文件详细记录了四足机器人六自由度动作的动作序列帧。系统硬件舵机排序映射如下：
		`{ 左前膝(LT_K), 左前髋(LT_H), 右前膝(RT_K), 右前髋(RT_H), 右后膝(RB_K), 右后髋(RB_H), 左后膝(LB_K), 左后髋(LB_H) }`
		极性说明：
		- **左前(LT) & 右后(RB)**：膝关节角度减小（90° -> 55°）为收腿抬高；髋关节角度减小为前摆。
		- **右前(RT) & 左后(LB)**：膝关节角度增大（90° -> 125°）为收腿抬高；髋关节角度增大为前摆。
 * ============================================================ */

// --- A. 前进 (Forward) 数据序列 ---
// 物理机制：标准的对角小跑（Trot步态）。通过交替抬起两组对角腿，并在空中完成髋关节前摆，触地后髋关节向后驱动机体前行。
static const pose_frame_t DATA_FORWARD[] = {
    // === Phase 1: 左前(LT) & 右后(RB) 对角相摆动 ===
    // 步骤 1: 左前与右后腿弯曲抬起，卸载支撑力矩，机体由右前与左后腿支撑
    { {55, 90, 90, 90, 55, 90, 90, 90} },
    // 步骤 2: 抬起腿的髋关节在空中跨步（LT前摆，RB前摆）；同时支撑腿的髋关节保持或微调
    // LT_H=55, RB_H=125, Knees hold 55
    { {55, 55, 90, 90, 55, 125, 90, 90} },
    // 步骤 3: 膝关节恢复至90°伸直触地，重新建立四足支撑态，准备交接转入支撑相
    { {90, 55, 90, 90, 90, 125, 90, 90} },
    // 步骤 4: 髋关节统一复位至中性态（90°），在此过程中产生向后的相对推力驱动机体前进
    { {90, 90, 90, 90, 90, 90, 90, 90} },
    
    // === Phase 2: 右前(RT) & 左后(LB) 对角相摆动 ===
    // 步骤 5: 右前与左后腿弯曲抬起（注意物理对称极性，Knee变为125°），转入空中摆动相
    { {90, 90, 125, 90, 90, 90, 125, 90} },
    // 步骤 6: 抬起腿的髋关节在空中完成跨步（RT前摆=125°，LB前摆=55°）
    // LB_H=55, RT_H=125, Knees hold
    { {90, 90, 125, 125, 90, 90, 125, 55} },
    // 步骤 7: 右前与左后膝关节伸直（恢复至90°）触地，完成当前相的动作下发
    { {90, 90, 90, 125, 90, 90, 90, 55} },
    // 步骤 8: 整体髋关节回摆复位至中性态（90°），推动机体平滑前移，回归初始就绪状态
    { {90, 90, 90, 90, 90, 90, 90, 90} }
};

// --- B. 后退 (Backward) 数据序列 ---
// 物理机制：对角小跑的逆向时序控制。在抬腿后，空中髋关节向后方摆动，触地后通过向前回摆将机体向后推动。
static const pose_frame_t DATA_BACKWARD[] = {
	
    // === Phase 1: 左前(LT) & 右后(RB) 对角相后退 ===
		// 步骤 1: 左前与右后腿膝关节收缩（55°）抬起，脱离地面支撑
    { {55, 90, 90, 90, 55, 90, 90, 90} },
    // 步骤 2: 悬空腿髋关节执行后摆动作（LT后摆=125°，RB后摆=55°），为后退跨步预留空间
    { {55, 125, 90, 90, 55, 55, 90, 90} },
    // 步骤 3: 悬空腿膝关节伸直（90°）触地，重新切换回四足落地状态
    { {90, 125, 90, 90, 90, 55, 90, 90} },
    // 步骤 4: 髋关节由后摆位置强制复位至90°，通过反向摩擦力驱动整个机体向后倒退
    { {90, 90, 90, 90, 90, 90, 90, 90} },
		
    // === Phase 2: 右前(RT) & 左后(LB) 对角相后退 ===
    // 步骤 5: 右前与左后腿膝关节收缩（125°）抬起，切换支撑相
    { {90, 90, 125, 90, 90, 90, 125, 90} },
    // 步骤 6: 悬空腿髋关节执行后摆动作（RT后摆=55°，LB后摆=125°）
    { {90, 90, 125, 55, 90, 90, 125, 125} },
    // 步骤 7: 右前与左后腿膝关节落回支撑位（90°），锚定地面
    { {90, 90, 90, 55, 90, 90, 90, 125} },
    // 步骤 8: 整体髋关节复位至中性态（90°）
    { {90, 90, 90, 90, 90, 90, 90, 90} }
};

// --- C. 左移 (Move Left) 数据序列 ---
// 物理机制：侧向平移步态。利用前后髋关节同向侧摆。Phase 1中LT和RB向左侧外展/内收，触地后利用横向摩擦力实现整体左移。
static const pose_frame_t DATA_LEFT[] = {
    // === Phase 1: 左前(LT) & 右后(RB) 侧向跨步 ===
    // 步骤 1: 左前与右后腿抬起（55°），解除机体单侧与对角斜线的地面约束
    { {55, 90, 90, 90, 55, 90, 90, 90} },
    // 步骤 2: 髋关节横向移形。LT髋外展（125°），RB髋内收（55°），使重心向左侧侧向延伸
    { {55, 125, 90, 90, 55, 55, 90, 90} },
    // 步骤 3: 膝关节下落（90°）踩实地面，此时四足呈横向张开非对称形态
    { {90, 125, 90, 90, 90, 55, 90, 90} },
    // 步骤 4: 髋关节收回中性态（90°），将机体核心横向拉向左侧支撑点
    { {90, 90, 90, 90, 90, 90, 90, 90} },

    // === Phase 2: 右前(RT) & 左后(LB) 侧向跟进 ===
    // 步骤 5: 右前与左后腿抬起（125°），机体当前重量转由已经左移的第一组对角腿支撑
    { {90, 90, 125, 90, 90, 90, 125, 90} },
    // 步骤 6: 髋关节朝相同横向方向移形（RT内收=125°，LB外展=55°），完成收腿动作
    { {90, 90, 125, 125, 90, 90, 125, 55} },
    // 步骤 7: 右前与左后腿膝关节伸直（90°）触地，恢复两侧足端物理间距
    { {90, 90, 90, 125, 90, 90, 90, 55} },
    // 步骤 8: 全局复位至标准中性立姿（90°），完成一次向左平移循环
    { {90, 90, 90, 90, 90, 90, 90, 90} }
};

// --- D. 右移 (Move Right) 数据序列 ---
static const pose_frame_t DATA_RIGHT[] = {
    // === Phase 1: 右前(RT) & 左后(LB) 侧向跨步 ===
    // 步骤 1: 右前与左后腿率先弯曲抬起（125°），解除右侧动作空间限制
    { {90, 90, 125, 90, 90, 90, 125, 90} },
    // 步骤 2: 髋关节执行右向移形（RT外展=55°，LB内收=125°），足端向机体右侧跨出
    { {90, 90, 125, 55, 90, 90, 125, 125} },
    // 步骤 3: 膝关节下落（90°）恢复触地，建立右侧张开的临时物理支撑点
    { {90, 90, 90, 55, 90, 90, 90, 125} },
    // 步骤 4: 髋关节拉回至90°，通过静态摩擦力将机器人身体整体向右侧拉动
    { {90, 90, 90, 90, 90, 90, 90, 90} },

    // === Phase 2: 左前(LT) & 右后(RB) 侧向跟进 ===
    // 步骤 5: 左前与右后腿膝关节收缩抬起（55°），准备将滞后的肢体收回
    { {55, 90, 90, 90, 55, 90, 90, 90} },
    // 步骤 6: 髋关节朝右侧同步摆动收拢（LT内收=55°，RB外展=125°）
    { {55, 55, 90, 90, 55, 125, 90, 90} },
    // 步骤 7: 膝关节恢复90°垂直状态触地，确保四足落地
    { {90, 55, 90, 90, 90, 125, 90, 90} },
    // 步骤 8: 运动学各轴回归稳态值（90°），完成一次向右平移解算
    { {90, 90, 90, 90, 90, 90, 90, 90} }
};
// --- E. 左旋 (Rotate Left) 数据序列 ---
// 物理机制：利用左侧向后、右侧向前的差速扭矩。空中髋关节统一摆至 125°。
static const pose_frame_t DATA_ROT_L[] = {
    // Phase 1: 左前(LT) & 右后(RB) 抬起 (Knee=55)
    { {55, 90, 90, 90, 55, 90, 90, 90} },
    // Phase 1: 空中摆动 (LT向后摆=125, RB向前摆=125)
    { {55, 125, 90, 90, 55, 125, 90, 90} },
    // Phase 1: 触地 (Knees back to 90)
    { {90, 125, 90, 90, 90, 125, 90, 90} },
    // Phase 1: 瞬时归位 (产生逆时针扭矩)
    { {90, 90, 90, 90, 90, 90, 90, 90} },

    // Phase 2: 右前(RT) & 左后(LB) 抬起 (修正 Knee=125)
    { {90, 90, 125, 90, 90, 90, 125, 90} },
    // Phase 2: 空中摆动 (RT向前摆=125, LB向后摆=125)
    { {90, 90, 125, 125, 90, 90, 125, 125} },
    // Phase 2: 触地
    { {90, 90, 90, 125, 90, 90, 90, 125} },
    // Phase 2: 瞬时归位 (继续产生逆时针扭矩)
    { {90, 90, 90, 90, 90, 90, 90, 90} }
};

// --- F. 右旋 (Rotate Right) 数据序列 ---
// 物理机制：利用左侧向前、右侧向后的差速扭矩。空中髋关节统一摆至 55°。
static const pose_frame_t DATA_ROT_R[] = {
    // Phase 1: 左前(LT) & 右后(RB) 抬起 (Knee=55)
    { {55, 90, 90, 90, 55, 90, 90, 90} },
    // Phase 1: 空中摆动 (LT向前摆=55, RB向后摆=55)
    { {55, 55, 90, 90, 55, 55, 90, 90} },
    // Phase 1: 触地 (Knees back to 90)
    { {90, 55, 90, 90, 90, 55, 90, 90} },
    // Phase 1: 瞬时归位 (产生顺时针扭矩)
    { {90, 90, 90, 90, 90, 90, 90, 90} },

    // Phase 2: 右前(RT) & 左后(LB) 抬起
    { {90, 90, 125, 90, 90, 90, 125, 90} },
    // Phase 2: 空中摆动(RT向后摆=55, LB向前摆=55)
    { {90, 90, 125, 55, 90, 90, 125, 55} },
    // Phase 2: 触地
    { {90, 90, 90, 55, 90, 90, 90, 55} },
    // Phase 2: 瞬时归位(继续产生顺时针扭矩)
    { {90, 90, 90, 90, 90, 90, 90, 90} }
};

// --- G. 站立复位 (Stand) ---
static const pose_frame_t DATA_STAND[] = {
    { {90, 90, 90, 90, 90, 90, 90, 90} } 
};

/* ============================================================
 * 2. 序列定义与管理
 * ============================================================ */
#define DEFINE_SEQ(name, data_array) \
    static const gait_sequence_t name = { data_array, sizeof(data_array)/sizeof(pose_frame_t) }

DEFINE_SEQ(SEQ_FORWARD,  DATA_FORWARD);
DEFINE_SEQ(SEQ_BACKWARD, DATA_BACKWARD);
DEFINE_SEQ(SEQ_LEFT,     DATA_LEFT);
DEFINE_SEQ(SEQ_RIGHT,    DATA_RIGHT);
DEFINE_SEQ(SEQ_ROT_L,    DATA_ROT_L);
DEFINE_SEQ(SEQ_ROT_R,    DATA_ROT_R);
DEFINE_SEQ(SEQ_STOP,     DATA_STAND);

// --- 静态上下文 ---
static fsm_event_t motor_evt_buf[16];
static motor_ctx_t motor_ctx;
//// ***************************************************************************
////    上面为旧版本无 PD算法
//// ***************************************************************************

//// 硬件抽象层：一键设置所有舵机
//static void apply_pose(const pose_frame_t* frame) {
//    BSP_Servo_Set_Left_Top_Knee(    frame->angles[ID_LT_KNEE]);
//    BSP_Servo_Set_Left_Top_Hip(     frame->angles[ID_LT_HIP]);
//    BSP_Servo_Set_Right_Top_Knee(   frame->angles[ID_RT_KNEE]);
//    BSP_Servo_Set_Right_Top_Hip(    frame->angles[ID_RT_HIP]);
//    BSP_Servo_Set_Right_Bottom_Knee(frame->angles[ID_RB_KNEE]);
//    BSP_Servo_Set_Right_Bottom_Hip( frame->angles[ID_RB_HIP]);
//    BSP_Servo_Set_Left_Bottom_Knee( frame->angles[ID_LB_KNEE]);
//    BSP_Servo_Set_Left_Bottom_Hip(  frame->angles[ID_LB_HIP]);
////	LOG_RAW("%d,%d,%d,%d,%d,%d,%d,%d\n", 
////						frame->angles[ID_LT_KNEE], frame->angles[ID_LT_HIP],
////						frame->angles[ID_RT_KNEE], frame->angles[ID_RT_HIP],
////						frame->angles[ID_RB_KNEE], frame->angles[ID_RB_HIP],
////						frame->angles[ID_LB_KNEE], frame->angles[ID_LB_HIP]
////				);



//}
//// 通用数据加载器 (Enter Callback)
//static void load_sequence(fsm_t* fsm, const gait_sequence_t* seq) {
//    motor_ctx.current_seq = seq;
//    motor_ctx.seq_step = 0;
//    motor_ctx.last_tick = FSM_GET_TICK();
//    // 立即执行第一帧，提升响应速度
//    if (seq && seq->frames) {
//        apply_pose(&seq->frames[0]);
//    }
//}

//// 各种状态的 Enter 回调包装
//static void enter_stop(fsm_t* f, void* a)     { load_sequence(f, &SEQ_STOP); }
//static void enter_forward(fsm_t* f, void* a)  { load_sequence(f, &SEQ_FORWARD); }
//static void enter_backward(fsm_t* f, void* a) { load_sequence(f, &SEQ_BACKWARD); }
//static void enter_left(fsm_t* f, void* a)     { load_sequence(f, &SEQ_LEFT); }
//static void enter_right(fsm_t* f, void* a)    { load_sequence(f, &SEQ_RIGHT); }
//static void enter_rot_l(fsm_t* f, void* a)    { load_sequence(f, &SEQ_ROT_L); }
//static void enter_rot_r(fsm_t* f, void* a)    { load_sequence(f, &SEQ_ROT_R); }

//// 通用轮询器 (Poll Callback)
//static void on_poll_gait(fsm_t* fsm, void* arg) {
//    motor_ctx_t* ctx = (motor_ctx_t*)arg;
//    const gait_sequence_t* seq = ctx->current_seq;

//    if (!seq || !seq->frames) return;

//    // 检查时间间隔
//    TickType_t now = FSM_GET_TICK();
//    if ((now - ctx->last_tick) < FSM_MS_TO_TICKS(ACTION_INTERVAL_MS)) return;

//    // 指向下一帧
//    ctx->seq_step++;
//    
//    // 循环播放逻辑
//    if (ctx->seq_step >= seq->frame_count) {
//        if (fsm->current_state == STATE_MOTOR_STOP) {
//            // 如果是停止状态，停在最后一帧 (通常是全90度)
//            ctx->seq_step = seq->frame_count - 1; 
//        } else {
//            // 运动状态，循环播放
//            ctx->seq_step = 0; 
//        }
//    }

//    // 执行动作
//    apply_pose(&seq->frames[ctx->seq_step]);
//    
//    ctx->last_tick = now;
//}

//// ***************************************************************************
////   上面为旧版本无 PD算法
//// ***************************************************************************

//// ***************************************************************************
////   下面为新版本PID PD算法用于姿态外环
//// ***************************************************************************

// --- 静态上下文 ---
static fsm_event_t motor_evt_buf[16];
static motor_ctx_t motor_ctx;

/* ============================================================
 * 闭环控制算法参数 (PID & 限位)
 * ============================================================ */
// 目标基准姿态 (水平面)
#define TARGET_PITCH -3.7f
#define TARGET_ROLL  178.2f

// PD 控制器增益参数
#define KP_PITCH 0.4f
#define KD_PITCH 0.15f
#define KP_ROLL  0.4f
#define KD_ROLL  0.15f

// 硬件安全限位
#define KNEE_MAX 135.0f
#define KNEE_MIN 45.0f

// 角度限幅宏
#define CLAMP(x, min, max) (((x) < (min)) ? (min) : (((x) > (max)) ? (max) : (x)))

// PID 历史状态
static float last_err_pitch = 0.0f;
static float last_err_roll = 0.0f;

/* ============================================================
 * P0-① 设备级故障安全：安全态 + 跨任务失联/失效标志
 * 设计原则：不改动原有步态 PD 算法，仅在控制环入口加"安全门禁"。
 * 触发条件：跌倒（倾角超阈）/ IMU 失效 / 网络失联 → 进入安全态收腿趴下、停走。
 * ============================================================ */
volatile uint8_t g_fault_safe_active = 0;
volatile uint8_t g_net_link_lost = 0;

/* IMU 连续读取失败计数：GetData 连续失败超阈值才判失效，滤掉单帧 FIFO 抖动 */
#define IMU_FAIL_THRESHOLD   5
static uint8_t g_imu_fail_cnt = 0;

/* 跌倒检测倾角阈值（度）：单腿着地姿态下，机身倾转超此值即判定翻倒风险 */
#define TIP_OVER_PITCH_DEG  50.0f
#define TIP_OVER_ROLL_DEG   50.0f

/* 进入安全态：所有膝关节收到极限（趴下贴地），髋关节回中，停止任何步态前进。
 * 不依赖 IMU（失效时也能执行），纯开环硬件动作，确保最坏情况下机身不翻倒。 */
void enter_safe_state(void) {
    if (g_fault_safe_active) return;   // 防重复进入
    g_fault_safe_active = 1;
    // 收腿趴下：膝关节全部收到最大角度（身体下沉贴地），髋关节回中（90°）
    BSP_Servo_Set_Left_Top_Knee(KNEE_MAX);
    BSP_Servo_Set_Right_Top_Knee(KNEE_MAX);
    BSP_Servo_Set_Left_Bottom_Knee(KNEE_MAX);
    BSP_Servo_Set_Right_Bottom_Knee(KNEE_MAX);
    BSP_Servo_Set_Left_Top_Hip(90.0f);
    BSP_Servo_Set_Right_Top_Hip(90.0f);
    BSP_Servo_Set_Left_Bottom_Hip(90.0f);
    BSP_Servo_Set_Right_Bottom_Hip(90.0f);
}

/* 退出安全态：故障解除后由控制环调用，恢复 PD 闭环（下一帧 load_sequence 会覆盖角度） */
static void exit_safe_state(void) {
    g_fault_safe_active = 0;
    g_imu_fail_cnt = 0;
}

/* 安全门禁：每周期调用，返回 1 表示应进入/保持安全态。
 * 判定三项独立故障源：① 跌倒 ② IMU 失效 ③ 网络失联 */
static uint8_t fault_safe_gate(void) {
    uint8_t fault = 0;

    /* ① 跌倒检测：基于最新 g_imu_data 倾角（IsWorking 时才可信） */
    if (BSP_MPU6050_IsWorking()) {
        motor_imu_refresh();
        if (fabsf(g_imu_snap.pitch) > TIP_OVER_PITCH_DEG ||
            fabsf(g_imu_snap.roll)  > TIP_OVER_ROLL_DEG) {
            fault |= 0x01;
        }
    }

    /* ② IMU 失效：底层不工作，或连续读取失败超阈值（防单帧抖动误判） */
    if (!BSP_MPU6050_IsWorking()) {
        fault |= 0x02;
    }

    /* ③ 网络失联：心跳超时标志（由 fsm_network 维护） */
    if (g_net_link_lost) {
        fault |= 0x04;
    }

    return fault;
}

/* ============================================================
 * 底层数学工具：角度包裹 (Angle Wrap)
 * 解决倒置安装导致的 180度 -> -180度 奇异点跃变
 * ============================================================ */
static float wrap_180(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/* ============================================================
 * 核心执行器：带姿态前馈补偿的位姿下发
 * ============================================================ */
static void apply_pose_with_pid(const pose_frame_t* frame) {
    // 1. 提取当前原始欧拉角
    motor_imu_refresh();
    float current_pitch = g_imu_snap.pitch;
    float current_roll = g_imu_snap.roll;

    // 2. 计算连续最短路径误差
    float err_pitch = wrap_180(TARGET_PITCH - current_pitch);
    float err_roll = wrap_180(TARGET_ROLL - current_roll);

    // 3. PD 补偿量计算 (略去积分项防震荡)
    float u_pitch = (KP_PITCH * err_pitch) + KD_PITCH * (err_pitch - last_err_pitch);
    float u_roll  = (KP_ROLL * err_roll)   + KD_ROLL  * (err_roll - last_err_roll);

    last_err_pitch = err_pitch;
    last_err_roll = err_roll;

    // 4. 支撑力矩需求分配
    // u_pitch > 0: 机头下沉，需要前腿增加支撑力(+)，后腿减小支撑力(-)
    // u_roll  > 0: 机身左倾，需要左腿增加支撑力(+)，右腿减小支撑力(-)
    float d_LT =  u_pitch + u_roll;  // 左前支撑力需求
    float d_RT =  u_pitch - u_roll;  // 右前支撑力需求
    float d_LB = -u_pitch + u_roll;  // 左后支撑力需求
    float d_RB = -u_pitch - u_roll;  // 右后支撑力需求

    // 5. 逆运动学极性映射 (关节角度解算)
    // 根据步态表：LT_K 与 RB_K 抬腿角度为 55°，增加支撑力需增大角度 (+)
    // 根据步态表：RT_K 与 LB_K 抬腿角度为 125°，增加支撑力需减小角度 (-)
    float lt_k_out = frame->angles[ID_LT_KNEE] + d_LT;
    float rt_k_out = frame->angles[ID_RT_KNEE] - d_RT;
    float lb_k_out = frame->angles[ID_LB_KNEE] - d_LB;
    float rb_k_out = frame->angles[ID_RB_KNEE] + d_RB;

    // 6. 硬件安全边界钳位
    lt_k_out = CLAMP(lt_k_out, KNEE_MIN, KNEE_MAX);
    rt_k_out = CLAMP(rt_k_out, KNEE_MIN, KNEE_MAX);
    lb_k_out = CLAMP(lb_k_out, KNEE_MIN, KNEE_MAX);
    rb_k_out = CLAMP(rb_k_out, KNEE_MIN, KNEE_MAX);

    // 7. 写入硬件寄存器 (髋关节负责步距，膝关节负责高度，仅补偿膝关节)
    BSP_Servo_Set_Left_Top_Knee(lt_k_out);
    BSP_Servo_Set_Right_Top_Knee(rt_k_out);
    BSP_Servo_Set_Left_Bottom_Knee(lb_k_out);
    BSP_Servo_Set_Right_Bottom_Knee(rb_k_out);
    
    BSP_Servo_Set_Left_Top_Hip(frame->angles[ID_LT_HIP]);
    BSP_Servo_Set_Right_Top_Hip(frame->angles[ID_RT_HIP]);
    BSP_Servo_Set_Left_Bottom_Hip(frame->angles[ID_LB_HIP]);
    BSP_Servo_Set_Right_Bottom_Hip(frame->angles[ID_RB_HIP]);
}

// ============================================================
// 序列定义与管理
// ============================================================ 
// 通用数据加载器 (Enter Callback)
static void load_sequence(fsm_t* fsm, const gait_sequence_t* seq) {
    motor_ctx.current_seq = seq;
    motor_ctx.seq_step = 0;
    motor_ctx.last_tick = FSM_GET_TICK();
    
    // 初始化 PID 历史状态，防止动作切换瞬间的微分突变
    motor_imu_refresh();
    last_err_pitch = wrap_180(TARGET_PITCH - g_imu_snap.pitch);
    last_err_roll  = wrap_180(TARGET_ROLL - g_imu_snap.roll);
    
    // 立即基于闭环机制执行第一帧
    if (seq && seq->frames) {
        apply_pose_with_pid(&seq->frames[0]);
    }
}

// 各种状态的 Enter 回调包装
static void enter_stop(fsm_t* f, void* a)     { load_sequence(f, &SEQ_STOP); }
static void enter_forward(fsm_t* f, void* a)  { load_sequence(f, &SEQ_FORWARD); }
static void enter_backward(fsm_t* f, void* a) { load_sequence(f, &SEQ_BACKWARD); }
static void enter_left(fsm_t* f, void* a)     { load_sequence(f, &SEQ_LEFT); }
static void enter_right(fsm_t* f, void* a)    { load_sequence(f, &SEQ_RIGHT); }
static void enter_rot_l(fsm_t* f, void* a)    { load_sequence(f, &SEQ_ROT_L); }
static void enter_rot_r(fsm_t* f, void* a)    { load_sequence(f, &SEQ_ROT_R); }

// ============================================================
// 轮询器：频率解耦 (Poll Callback)
// ============================================================ 
static void on_poll_gait(fsm_t* fsm, void* arg) {
    motor_ctx_t* ctx = (motor_ctx_t*)arg;
    const gait_sequence_t* seq = ctx->current_seq;

    if (!seq || !seq->frames) return;

    TickType_t now = FSM_GET_TICK();
    
    // 1. 低频切帧：仅当到达步态间隔时，步进到下一帧
    if ((now - ctx->last_tick) >= FSM_MS_TO_TICKS(ACTION_INTERVAL_MS)) {
        ctx->seq_step++;
        
        if (ctx->seq_step >= seq->frame_count) {
            if (fsm->current_state == STATE_MOTOR_STOP) {
                ctx->seq_step = seq->frame_count - 1; 
            } else {
                ctx->seq_step = 0; 
            }
        }
        ctx->last_tick = now; 
    }

    // 2. 高频外环：以 20ms 的刷新率(与任务周期一致)，实时叠加姿态补偿
    apply_pose_with_pid(&seq->frames[ctx->seq_step]);
}
//// ***************************************************************************
////   上面为新版本PID PD算法用于姿态外环
//// ***************************************************************************
/* ============================================================
 * 状态描述表 (State Descriptions)
 * ============================================================ */
static const fsm_state_desc_t motor_states[] = {
    // 所有的 Poll 都指向同一个通用函数！
    { STATE_MOTOR_STOP,     enter_stop,     NULL, on_poll_gait },
    { STATE_MOTOR_FORWARD,  enter_forward,  NULL, on_poll_gait },
    { STATE_MOTOR_BACKWARD, enter_backward, NULL, on_poll_gait },
    { STATE_MOTOR_LEFT,     enter_left,     NULL, on_poll_gait },
    { STATE_MOTOR_RIGHT,    enter_right,    NULL, on_poll_gait },
    { STATE_MOTOR_ROT_L,    enter_rot_l,    NULL, on_poll_gait },
    { STATE_MOTOR_ROT_R,    enter_rot_r,    NULL, on_poll_gait },
};

/* ============================================================
 * 状态转换表 (Transition Table)
 * ============================================================ */
#define TRANS_ANY(event, next_state) \
    { STATE_MOTOR_STOP,     event, next_state, NULL, NULL }, \
    { STATE_MOTOR_FORWARD,  event, next_state, NULL, NULL }, \
    { STATE_MOTOR_BACKWARD, event, next_state, NULL, NULL }, \
    { STATE_MOTOR_LEFT,     event, next_state, NULL, NULL }, \
    { STATE_MOTOR_RIGHT,    event, next_state, NULL, NULL }, \
    { STATE_MOTOR_ROT_L,    event, next_state, NULL, NULL }, \
    { STATE_MOTOR_ROT_R,    event, next_state, NULL, NULL }

static const fsm_transition_t motor_trans[] = {
    TRANS_ANY(EVT_MOTOR_STOP,     STATE_MOTOR_STOP),
    TRANS_ANY(EVT_MOTOR_FORWARD,  STATE_MOTOR_FORWARD),
    TRANS_ANY(EVT_MOTOR_BACKWARD, STATE_MOTOR_BACKWARD),
    TRANS_ANY(EVT_MOTOR_LEFT,     STATE_MOTOR_LEFT),
    TRANS_ANY(EVT_MOTOR_RIGHT,    STATE_MOTOR_RIGHT),
    TRANS_ANY(EVT_MOTOR_ROT_L,    STATE_MOTOR_ROT_L),
    TRANS_ANY(EVT_MOTOR_ROT_R,    STATE_MOTOR_ROT_R),
};

// --- 初始化函数 ---
void Motor_FSM_Setup(fsm_t* fsm) {
    if (!fsm) return;
    
    // 初始化上下文
    motor_ctx.seq_step = 0;
    motor_ctx.last_tick = FSM_GET_TICK();
		motor_ctx.current_seq = &SEQ_STOP;

    // 绑定 Core
    fsm_init(fsm, motor_evt_buf, 16,
             motor_trans, sizeof(motor_trans)/sizeof(fsm_transition_t),
             STATE_MOTOR_STOP, 
             &motor_ctx);
    
    // 注册状态回调

    fsm_set_state_callbacks(fsm, motor_states, sizeof(motor_states)/sizeof(fsm_state_desc_t));
		
		event_bus_subscribe(fsm, EVT_MOTOR_STOP);
    event_bus_subscribe(fsm, EVT_MOTOR_FORWARD);
    event_bus_subscribe(fsm, EVT_MOTOR_BACKWARD);
    event_bus_subscribe(fsm, EVT_MOTOR_LEFT);
    event_bus_subscribe(fsm, EVT_MOTOR_RIGHT);
    event_bus_subscribe(fsm, EVT_MOTOR_ROT_L);
    event_bus_subscribe(fsm, EVT_MOTOR_ROT_R);


}

/* ================= 电机任务 ================= */

void Motor_FSM_task(void *pvParameters){
    // 1. 设置电机 FSM (绑定转换表、订阅总线事件)
    // 这里的 Setup 内部调用了 event_bus_subscribe
    Motor_FSM_Setup(&g_Motor_fsm);
		// 在此处异步初始化传感器。此处的阻塞只会挂起电机任务，OLED与网络任务正常运行。
		BSP_MPU6050_Init();
	
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 20ms 周期，保证舵机控制平滑
    TickType_t xLastWakeTime = xTaskGetTickCount();
		while(1) {
				// --- 读取 MPU6050 姿态数据 ---
				// 检查底层 EXTI 中断是否将数据准备就绪标志置位
				if (BSP_MPU6050_IsWorking()) {
					if (BSP_MPU6050_IsDataReady()) {
						BSP_MPU6050_ClearDataReady(); // 立即清除标志位
						// 从 FIFO 读取解算后的数据
					MPU6050_Data_t imu_local;
					if (BSP_MPU6050_GetData(&imu_local) == 0) {
							g_imu_fail_cnt = 0;  // 读取成功，清除失效计数
							state_write(ST_IMU_DATA, &imu_local, sizeof(imu_local));
							// 如果需要调试，可以取消下面这行的注释打印数据
							//printf("%.1f,%.1f,%.1f\r\n", imu_local.yaw, imu_local.pitch, imu_local.roll);
							
							
								// SYS_LOG("MOTO", "IMU: %.1f,%.1f,%.1f\n", g_imu_data.yaw, g_imu_data.pitch, g_imu_data.roll);
								// 【预留接口】可以将 g_imu_data 的数据通过 fsm_push_event 压入状态机，
								// 或者直接给 PID 控制器使用。
							} else {
							// 单帧读取失败：累计计数，超阈值才判 IMU 失效（防 FIFO 偶发抖动误触发安全态）
							if (g_imu_fail_cnt < 0xFF) g_imu_fail_cnt++;
						}
					}
				} else {
					// 底层不工作：失效计数直接拉满，门禁会判 IMU 失效
					g_imu_fail_cnt = IMU_FAIL_THRESHOLD;
				}

				// --- P0-① 故障安全门禁：跌倒 / IMU 失效 / 网络失联 → 进入安全态 ---
				uint8_t fault = fault_safe_gate();
				// IMU 连续失败也计入失效（fault_safe_gate 仅判 !IsWorking，这里补连续失败）
				if (g_imu_fail_cnt >= IMU_FAIL_THRESHOLD) fault |= 0x02;

				if (fault) {
					if (!g_fault_safe_active) {
						enter_safe_state();   // 收腿趴下（纯开环，不依赖 IMU）
					}
					// 故障期停走：跳过步态 FSM 推进，仅维持安全态
				} else {
					if (g_fault_safe_active) {
						exit_safe_state();    // 故障解除
						enter_stop(&g_Motor_fsm, NULL);  // 复位到站立足态，恢复 PD 闭环
					}
				}

				if (!g_fault_safe_active) {
					fsm_run(&g_Motor_fsm);     // 正常：运行步态状态机
				}
		vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
