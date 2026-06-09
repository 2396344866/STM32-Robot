#include "fsm_motor.h"
#include "fsm_core.h"
#include "FreeRTOS.h"
#include "event_bus.h"
#include "sys_events.h"
#include "task.h"
#include "hal_pwm.h" 
#include "bsp_servo.h"
#include "bsp_mpu6050.h"
static fsm_t g_Motor_fsm;
MPU6050_Data_t g_imu_data;
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
		
// 硬件抽象层：一键设置所有舵机
static void apply_pose(const pose_frame_t* frame) {
    BSP_Servo_Set_Left_Top_Knee(    frame->angles[ID_LT_KNEE]);
    BSP_Servo_Set_Left_Top_Hip(     frame->angles[ID_LT_HIP]);
    BSP_Servo_Set_Right_Top_Knee(   frame->angles[ID_RT_KNEE]);
    BSP_Servo_Set_Right_Top_Hip(    frame->angles[ID_RT_HIP]);
    BSP_Servo_Set_Right_Bottom_Knee(frame->angles[ID_RB_KNEE]);
    BSP_Servo_Set_Right_Bottom_Hip( frame->angles[ID_RB_HIP]);
    BSP_Servo_Set_Left_Bottom_Knee( frame->angles[ID_LB_KNEE]);
    BSP_Servo_Set_Left_Bottom_Hip(  frame->angles[ID_LB_HIP]);
//	LOG_RAW("%d,%d,%d,%d,%d,%d,%d,%d\n", 
//						frame->angles[ID_LT_KNEE], frame->angles[ID_LT_HIP],
//						frame->angles[ID_RT_KNEE], frame->angles[ID_RT_HIP],
//						frame->angles[ID_RB_KNEE], frame->angles[ID_RB_HIP],
//						frame->angles[ID_LB_KNEE], frame->angles[ID_LB_HIP]
//				);



}
// 通用数据加载器 (Enter Callback)
static void load_sequence(fsm_t* fsm, const gait_sequence_t* seq) {
    motor_ctx.current_seq = seq;
    motor_ctx.seq_step = 0;
    motor_ctx.last_tick = FSM_GET_TICK();
    // 立即执行第一帧，提升响应速度
    if (seq && seq->frames) {
        apply_pose(&seq->frames[0]);
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

// 通用轮询器 (Poll Callback)
static void on_poll_gait(fsm_t* fsm, void* arg) {
    motor_ctx_t* ctx = (motor_ctx_t*)arg;
    const gait_sequence_t* seq = ctx->current_seq;

    if (!seq || !seq->frames) return;

    // 检查时间间隔
    TickType_t now = FSM_GET_TICK();
    if ((now - ctx->last_tick) < FSM_MS_TO_TICKS(ACTION_INTERVAL_MS)) return;

    // 指向下一帧
    ctx->seq_step++;
    
    // 循环播放逻辑
    if (ctx->seq_step >= seq->frame_count) {
        if (fsm->current_state == STATE_MOTOR_STOP) {
            // 如果是停止状态，停在最后一帧 (通常是全90度)
            ctx->seq_step = seq->frame_count - 1; 
        } else {
            // 运动状态，循环播放
            ctx->seq_step = 0; 
        }
    }

    // 执行动作
    apply_pose(&seq->frames[ctx->seq_step]);
    
    ctx->last_tick = now;
}


//// ***************************************************************************
////   下面为新版本PID PD算法用于姿态外环
//// ***************************************************************************

//// --- 静态上下文 ---
//static fsm_event_t motor_evt_buf[16];
//static motor_ctx_t motor_ctx;

///* ============================================================
// * 闭环控制算法参数 (PID & 限位)
// * ============================================================ */
//// 目标基准姿态 (水平面)
//#define TARGET_PITCH -3.7f
//#define TARGET_ROLL  178.2f

//// PD 控制器增益参数
//#define KP_PITCH 0.4f
//#define KD_PITCH 0.15f
//#define KP_ROLL  0.4f
//#define KD_ROLL  0.15f

//// 硬件安全限位
//#define KNEE_MAX 135.0f
//#define KNEE_MIN 45.0f

//// 角度限幅宏
//#define CLAMP(x, min, max) (((x) < (min)) ? (min) : (((x) > (max)) ? (max) : (x)))

//// PID 历史状态
//static float last_err_pitch = 0.0f;
//static float last_err_roll = 0.0f;

///* ============================================================
// * 底层数学工具：角度包裹 (Angle Wrap)
// * 解决倒置安装导致的 180度 -> -180度 奇异点跃变
// * ============================================================ */
//static float wrap_180(float angle) {
//    while (angle > 180.0f) angle -= 360.0f;
//    while (angle < -180.0f) angle += 360.0f;
//    return angle;
//}

///* ============================================================
// * 核心执行器：带姿态前馈补偿的位姿下发
// * ============================================================ */
//static void apply_pose_with_pid(const pose_frame_t* frame) {
//    // 1. 提取当前原始欧拉角
//    float current_pitch = g_imu_data.pitch;
//    float current_roll = g_imu_data.roll;

//    // 2. 计算连续最短路径误差
//    float err_pitch = wrap_180(TARGET_PITCH - current_pitch);
//    float err_roll = wrap_180(TARGET_ROLL - current_roll);

//    // 3. PD 补偿量计算 (略去积分项防震荡)
//    float u_pitch = (KP_PITCH * err_pitch) + KD_PITCH * (err_pitch - last_err_pitch);
//    float u_roll  = (KP_ROLL * err_roll)   + KD_ROLL  * (err_roll - last_err_roll);

//    last_err_pitch = err_pitch;
//    last_err_roll = err_roll;

//    // 4. 支撑力矩需求分配
//    // u_pitch > 0: 机头下沉，需要前腿增加支撑力(+)，后腿减小支撑力(-)
//    // u_roll  > 0: 机身左倾，需要左腿增加支撑力(+)，右腿减小支撑力(-)
//    float d_LT =  u_pitch + u_roll;  // 左前支撑力需求
//    float d_RT =  u_pitch - u_roll;  // 右前支撑力需求
//    float d_LB = -u_pitch + u_roll;  // 左后支撑力需求
//    float d_RB = -u_pitch - u_roll;  // 右后支撑力需求

//    // 5. 逆运动学极性映射 (关节角度解算)
//    // 根据步态表：LT_K 与 RB_K 抬腿角度为 55°，增加支撑力需增大角度 (+)
//    // 根据步态表：RT_K 与 LB_K 抬腿角度为 125°，增加支撑力需减小角度 (-)
//    float lt_k_out = frame->angles[ID_LT_KNEE] + d_LT;
//    float rt_k_out = frame->angles[ID_RT_KNEE] - d_RT;
//    float lb_k_out = frame->angles[ID_LB_KNEE] - d_LB;
//    float rb_k_out = frame->angles[ID_RB_KNEE] + d_RB;

//    // 6. 硬件安全边界钳位
//    lt_k_out = CLAMP(lt_k_out, KNEE_MIN, KNEE_MAX);
//    rt_k_out = CLAMP(rt_k_out, KNEE_MIN, KNEE_MAX);
//    lb_k_out = CLAMP(lb_k_out, KNEE_MIN, KNEE_MAX);
//    rb_k_out = CLAMP(rb_k_out, KNEE_MIN, KNEE_MAX);

//    // 7. 写入硬件寄存器 (髋关节负责步距，膝关节负责高度，仅补偿膝关节)
//    BSP_Servo_Set_Left_Top_Knee(lt_k_out);
//    BSP_Servo_Set_Right_Top_Knee(rt_k_out);
//    BSP_Servo_Set_Left_Bottom_Knee(lb_k_out);
//    BSP_Servo_Set_Right_Bottom_Knee(rb_k_out);
//    
//    BSP_Servo_Set_Left_Top_Hip(frame->angles[ID_LT_HIP]);
//    BSP_Servo_Set_Right_Top_Hip(frame->angles[ID_RT_HIP]);
//    BSP_Servo_Set_Left_Bottom_Hip(frame->angles[ID_LB_HIP]);
//    BSP_Servo_Set_Right_Bottom_Hip(frame->angles[ID_RB_HIP]);
//}

//// ============================================================
//// 序列定义与管理
//// ============================================================ 
//// 通用数据加载器 (Enter Callback)
//static void load_sequence(fsm_t* fsm, const gait_sequence_t* seq) {
//    motor_ctx.current_seq = seq;
//    motor_ctx.seq_step = 0;
//    motor_ctx.last_tick = FSM_GET_TICK();
//    
//    // 初始化 PID 历史状态，防止动作切换瞬间的微分突变
//    last_err_pitch = wrap_180(TARGET_PITCH - g_imu_data.pitch);
//    last_err_roll  = wrap_180(TARGET_ROLL - g_imu_data.roll);
//    
//    // 立即基于闭环机制执行第一帧
//    if (seq && seq->frames) {
//        apply_pose_with_pid(&seq->frames[0]);
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

//// ============================================================
//// 轮询器：频率解耦 (Poll Callback)
//// ============================================================ 
//static void on_poll_gait(fsm_t* fsm, void* arg) {
//    motor_ctx_t* ctx = (motor_ctx_t*)arg;
//    const gait_sequence_t* seq = ctx->current_seq;

//    if (!seq || !seq->frames) return;

//    TickType_t now = FSM_GET_TICK();
//    
//    // 1. 低频切帧：仅当到达步态间隔时，步进到下一帧
//    if ((now - ctx->last_tick) >= FSM_MS_TO_TICKS(ACTION_INTERVAL_MS)) {
//        ctx->seq_step++;
//        
//        if (ctx->seq_step >= seq->frame_count) {
//            if (fsm->current_state == STATE_MOTOR_STOP) {
//                ctx->seq_step = seq->frame_count - 1; 
//            } else {
//                ctx->seq_step = 0; 
//            }
//        }
//        ctx->last_tick = now; 
//    }

//    // 2. 高频外环：以 20ms 的刷新率(与任务周期一致)，实时叠加姿态补偿
//    apply_pose_with_pid(&seq->frames[ctx->seq_step]);
//}
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

/* ================= 电机任务 ================= */
void Motor_FSM_task(void *pvParameters){
    Motor_FSM_Setup(&g_Motor_fsm);
    
    // 异步初始化传感器，阻塞电机任务但不影响其他系统任务
    BSP_MPU6050_Init();
    
    while(1) {
        // 【物理级优化】丢弃传统的 vTaskDelayUntil 轮询。
        // 电机任务进入阻塞态，释放 100% CPU，死等 PB12_EXTI 中断唤醒。
        // 超时设为 50ms (防止传感器断联导致状态机死锁)。
        // 因 DMP 设置为 100Hz，正常情况下每 10ms 会被完美唤醒一次。
        uint32_t is_notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
        
        if (is_notified > 0 && BSP_MPU6050_IsWorking()) {
            // 被物理中断唤醒：立即提取底层解算好的最新欧拉角
            if (BSP_MPU6050_GetData(&g_imu_data) == 0) {
                // 在获取到最新绝对姿态的瞬间，立刻驱动闭环系统运算输出 PWM
                fsm_run(&g_Motor_fsm);
            }
        } else {
            // 断联保护：若中断丢失触发 50ms 超时，依靠盲走维持 FSM 基本运转
            fsm_run(&g_Motor_fsm);
        }
    }
}

//////////////////////////////////////// 新版本////////////////////////////////

//#include "fsm_motor.h"
//#include "fsm_core.h"
//#include "FreeRTOS.h"
//#include "event_bus.h"
//#include "sys_events.h"
//#include "task.h"
//#include "hal_pwm.h" 
//#include "bsp_servo.h"
//#include "bsp_mpu6050.h"
//#include <math.h>

//static fsm_t g_Motor_fsm;
//MPU6050_Data_t g_imu_data;

///* ============================================================
// * 动态运动学参数配置
// * ============================================================ */
//#define MATH_PI             3.14159265f
//#define GAIT_PERIOD_MS      800.0f        // 完整步态周期时间 (毫秒)
//#define POLL_INTERVAL_MS    20.0f         // FSM轮询周期
//#define PHASE_INC           (POLL_INTERVAL_MS / GAIT_PERIOD_MS) // 相位增量

//#define SWING_AMP_HIP       35.0f         // 髋关节摆动幅度 (度)
//#define SWING_AMP_KNEE      35.0f         // 膝关节抬升幅度 (度)
//#define ANGLE_NEUTRAL       90.0f         // 中性态基准角度

//enum {
//    ID_LT_KNEE = 0, ID_LT_HIP,
//    ID_RT_KNEE, ID_RT_HIP,
//    ID_RB_KNEE, ID_RB_HIP,
//    ID_LB_KNEE, ID_LB_HIP
//};

//// --- 静态上下文结构体扩展 ---
//typedef struct {
//    float current_phase;    // 全局步态相位 [0.0, 1.0)
//    TickType_t last_tick;   // 时间戳
//} dynamic_motor_ctx_t;

//static fsm_event_t motor_evt_buf[16];
//static dynamic_motor_ctx_t dyn_motor_ctx;

///* ============================================================
// * 轨迹发生器计算逻辑 (第一性原理方程应用)
// * ============================================================ */
//#define ANGLE_MIN 45.0f
//#define ANGLE_MAX 125.0f

//static void get_offsets(float p, float *hip_off, float *knee_off) {
//    if (p < 0.5f) { // 摆动相：在空中画半圆抬腿
//        *hip_off  = SWING_AMP_HIP * sinf(MATH_PI * p * 2.0f);
//        *knee_off = -SWING_AMP_KNEE * sinf(MATH_PI * p * 2.0f); // 产生负的偏移量用于抬腿
//    } else { // 支撑相：在地面平滑回退推动机体
//        *hip_off  = SWING_AMP_HIP * cosf(MATH_PI * (p - 0.5f) * 2.0f);
//        *knee_off = 0;
//    }
//}

//static void generate_trot_gait(float phase, uint16_t state, float out_angles[8]) {
//    // 1. 初始化为中性态 (90度)
//    for(int i = 0; i < 8; i++) out_angles[i] = ANGLE_NEUTRAL;
//    if (state == STATE_MOTOR_STOP) return;

//    // 2. 对角相位计算
//    float p1 = phase;                             // 对角组1: 左前(LT) & 右后(RB)
//    float p2 = fmodf(phase + 0.5f, 1.0f);         // 对角组2: 右前(RT) & 左后(LB)
//    float h1, k1, h2, k2;

//    get_offsets(p1, &h1, &k1);
//    get_offsets(p2, &h2, &k2);

//    // ==============================================================
//    // 3. 膝关节绝对控制：无论什么动作，抬腿极性恒定
//    // 根据原版注释：LT与RB抬腿=55(减小)，RT与LB抬腿=125(增大)
//    // 由于 k1/k2 在摆动相是负数，所以 LT/RB 用加号(90+(-35)=55)，RT/LB用减号(90-(-35)=125)
//    // ==============================================================
//    out_angles[ID_LT_KNEE] = ANGLE_NEUTRAL + k1;
//    out_angles[ID_RB_KNEE] = ANGLE_NEUTRAL + k1;
//    out_angles[ID_RT_KNEE] = ANGLE_NEUTRAL - k2;
//    out_angles[ID_LB_KNEE] = ANGLE_NEUTRAL - k2;

//    // ==============================================================
//    // 4. 髋关节运动学映射（基于原版步态矩阵与动力学极性对称校正）
//    // ==============================================================
//    if (state == STATE_MOTOR_FORWARD || state == STATE_MOTOR_BACKWARD) {
//        float dir = (state == STATE_MOTOR_FORWARD) ? 1.0f : -1.0f;
//        
//        // 彻底解决右偏转圈：严格按照原版物理极性对称推导
//        // FORWARD (dir=1): LT前摆变小(-h1), RB前摆变大(+h1), RT前摆变大(+h2), LB前摆变小(-h2)
//        out_angles[ID_LT_HIP] = ANGLE_NEUTRAL - (h1 * dir);
//        out_angles[ID_RB_HIP] = ANGLE_NEUTRAL + (h1 * dir);
//        out_angles[ID_RT_HIP] = ANGLE_NEUTRAL + (h2 * dir);
//        out_angles[ID_LB_HIP] = ANGLE_NEUTRAL - (h2 * dir);
//    } 
//    else if (state == STATE_MOTOR_LEFT || state == STATE_MOTOR_RIGHT) {
//        float dir = (state == STATE_MOTOR_LEFT) ? 1.0f : -1.0f;
//        
//        // 平移（横向平移）：四条腿的髋关节需要同向平移产生横向分力
//        // 原版 LEFT (dir=1): LT=125(+h1), RB=55(-h1), RT=125(+h2), LB=55(-h2)
//        out_angles[ID_LT_HIP] = ANGLE_NEUTRAL + (h1 * dir);
//        out_angles[ID_RB_HIP] = ANGLE_NEUTRAL - (h1 * dir);
//        out_angles[ID_RT_HIP] = ANGLE_NEUTRAL + (h2 * dir);
//        out_angles[ID_LB_HIP] = ANGLE_NEUTRAL - (h2 * dir);
//    }
//    else if (state == STATE_MOTOR_ROT_L || state == STATE_MOTOR_ROT_R) {
//        float dir = (state == STATE_MOTOR_ROT_L) ? 1.0f : -1.0f;
//        
//        // 旋转（原位扭转）：左右两侧执行相反的推进方向以产生转矩
//        // 原版 ROT_L (dir=1): 所有关节一律前摆到最大或同向偏转，四腿全加
//        out_angles[ID_LT_HIP] = ANGLE_NEUTRAL + (h1 * dir);
//        out_angles[ID_RB_HIP] = ANGLE_NEUTRAL + (h1 * dir);
//        out_angles[ID_RT_HIP] = ANGLE_NEUTRAL + (h2 * dir);
//        out_angles[ID_LB_HIP] = ANGLE_NEUTRAL + (h2 * dir);
//    }

//    // ==============================================================
//    // 5. 强制硬件限位 (保护舵机，防止因为超出行程扫齿导致卡死)
//    // ==============================================================
//    for(int i = 0; i < 8; i++) {
//        if(out_angles[i] < ANGLE_MIN) out_angles[i] = ANGLE_MIN;
//        if(out_angles[i] > ANGLE_MAX) out_angles[i] = ANGLE_MAX;
//    }
//}

///* ============================================================
// * 统一轮询器 (Poll Callback)
// * ============================================================ */
//static void on_poll_dynamic_gait(fsm_t* fsm, void* arg) {
//    dynamic_motor_ctx_t* ctx = (dynamic_motor_ctx_t*)arg;
//    TickType_t now = FSM_GET_TICK();
//    if ((now - ctx->last_tick) < FSM_MS_TO_TICKS(POLL_INTERVAL_MS)) return;
//    ctx->last_tick = now;

//    if (fsm->current_state != STATE_MOTOR_STOP) {
//        ctx->current_phase += PHASE_INC;
//        if (ctx->current_phase >= 1.0f) ctx->current_phase -= 1.0f;
//    }

//    float angles[8];
//    generate_trot_gait(ctx->current_phase, fsm->current_state, angles);
//    
//    BSP_Servo_Set_Left_Top_Knee(angles[ID_LT_KNEE]);
//    BSP_Servo_Set_Left_Top_Hip(angles[ID_LT_HIP]);
//    BSP_Servo_Set_Right_Top_Knee(angles[ID_RT_KNEE]);
//    BSP_Servo_Set_Right_Top_Hip(angles[ID_RT_HIP]);
//    BSP_Servo_Set_Right_Bottom_Knee(angles[ID_RB_KNEE]);
//    BSP_Servo_Set_Right_Bottom_Hip(angles[ID_RB_HIP]);
//    BSP_Servo_Set_Left_Bottom_Knee(angles[ID_LB_KNEE]);
//    BSP_Servo_Set_Left_Bottom_Hip(angles[ID_LB_HIP]);
//}

//// --- 状态切入复位函数 ---
//static void reset_phase_on_enter(fsm_t* f, void* a) {
//    dyn_motor_ctx.last_tick = FSM_GET_TICK();
//}

///* ============================================================
// * 状态描述表与转换表集成
// * ============================================================ */
//static const fsm_state_desc_t motor_states[] = {
//    { STATE_MOTOR_STOP,     reset_phase_on_enter, NULL, on_poll_dynamic_gait },
//    { STATE_MOTOR_FORWARD,  reset_phase_on_enter, NULL, on_poll_dynamic_gait },
//    { STATE_MOTOR_BACKWARD, reset_phase_on_enter, NULL, on_poll_dynamic_gait },
//    { STATE_MOTOR_LEFT,     reset_phase_on_enter, NULL, on_poll_dynamic_gait },
//    { STATE_MOTOR_RIGHT,    reset_phase_on_enter, NULL, on_poll_dynamic_gait },
//    { STATE_MOTOR_ROT_L,    reset_phase_on_enter, NULL, on_poll_dynamic_gait },
//    { STATE_MOTOR_ROT_R,    reset_phase_on_enter, NULL, on_poll_dynamic_gait },
//};

//static const fsm_transition_t motor_trans[] = {
//    { STATE_MOTOR_STOP,     EVT_MOTOR_FORWARD,  STATE_MOTOR_FORWARD,  NULL, NULL },
//    { STATE_MOTOR_STOP,     EVT_MOTOR_BACKWARD, STATE_MOTOR_BACKWARD, NULL, NULL },
//    { STATE_MOTOR_STOP,     EVT_MOTOR_LEFT,     STATE_MOTOR_LEFT,     NULL, NULL },
//    { STATE_MOTOR_STOP,     EVT_MOTOR_RIGHT,    STATE_MOTOR_RIGHT,    NULL, NULL },
//    { STATE_MOTOR_STOP,     EVT_MOTOR_ROT_L,    STATE_MOTOR_ROT_L,    NULL, NULL },
//    { STATE_MOTOR_STOP,     EVT_MOTOR_ROT_R,    STATE_MOTOR_ROT_R,    NULL, NULL },
//    
//    { STATE_MOTOR_FORWARD,  EVT_MOTOR_STOP,     STATE_MOTOR_STOP,     NULL, NULL },
//    { STATE_MOTOR_BACKWARD, EVT_MOTOR_STOP,     STATE_MOTOR_STOP,     NULL, NULL },
//    { STATE_MOTOR_LEFT,     EVT_MOTOR_STOP,     STATE_MOTOR_STOP,     NULL, NULL },
//    { STATE_MOTOR_RIGHT,    EVT_MOTOR_STOP,     STATE_MOTOR_STOP,     NULL, NULL },
//    { STATE_MOTOR_ROT_L,    EVT_MOTOR_STOP,     STATE_MOTOR_STOP,     NULL, NULL },
//    { STATE_MOTOR_ROT_R,    EVT_MOTOR_STOP,     STATE_MOTOR_STOP,     NULL, NULL },
//};

//void Motor_FSM_Setup(fsm_t* fsm) {
//    dyn_motor_ctx.current_phase = 0.0f;
//    dyn_motor_ctx.last_tick = FSM_GET_TICK();
//    fsm_init(fsm, motor_evt_buf, 16, motor_trans, 12, STATE_MOTOR_STOP, &dyn_motor_ctx);
//    fsm_set_state_callbacks(fsm, motor_states, 7);
//    event_bus_subscribe(fsm, EVT_MOTOR_STOP);
//    event_bus_subscribe(fsm, EVT_MOTOR_FORWARD);
//    event_bus_subscribe(fsm, EVT_MOTOR_BACKWARD);
//    event_bus_subscribe(fsm, EVT_MOTOR_LEFT);
//    event_bus_subscribe(fsm, EVT_MOTOR_RIGHT);
//    event_bus_subscribe(fsm, EVT_MOTOR_ROT_L);
//    event_bus_subscribe(fsm, EVT_MOTOR_ROT_R);
//}

//void Motor_FSM_task(void *pvParameters){
//    Motor_FSM_Setup(&g_Motor_fsm);
//    BSP_MPU6050_Init();
//    const TickType_t xFrequency = pdMS_TO_TICKS(20); 
//    TickType_t xLastWakeTime = xTaskGetTickCount();
//    
//    while(1) {
//        if (BSP_MPU6050_IsWorking() && BSP_MPU6050_IsDataReady()) {
//            BSP_MPU6050_ClearDataReady(); 
//            BSP_MPU6050_GetData(&g_imu_data);
//        }
//        fsm_run(&g_Motor_fsm);
//        vTaskDelayUntil(&xLastWakeTime, xFrequency);
//    }
//}
