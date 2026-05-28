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
 * ============================================================ */

// --- A. 前进 (Forward) 数据序列 ---
static const pose_frame_t DATA_FORWARD[] = {
    // Phase 1: 左前(LT) & 右后(RB) 抬起
    // LT_K=55, RB_K=55, others=90
    { {55, 90, 90, 90, 55, 90, 90, 90} },
    // Phase 1: 摆动
    // LT_H=55, RB_H=125, Knees hold 55
    { {55, 55, 90, 90, 55, 125, 90, 90} },
    // Phase 1: 放下 (Knees back to 90, Hips hold)
    { {90, 55, 90, 90, 90, 125, 90, 90} },
    // Phase 1: 归位 (Hips back to 90) -> Neutral
    { {90, 90, 90, 90, 90, 90, 90, 90} },
    
    // Phase 2: 右前(RT) & 左后(LB) 抬起
    // LB_K=125, RT_K=125 (Based on your code), others=90
    { {90, 90, 125, 90, 90, 90, 125, 90} },
    // Phase 2: 摆动
    // LB_H=55, RT_H=125, Knees hold
    { {90, 90, 125, 125, 90, 90, 125, 55} },
    // Phase 2: 放下
    { {90, 90, 90, 125, 90, 90, 90, 55} },
    // Phase 2: 归位 -> Neutral
    { {90, 90, 90, 90, 90, 90, 90, 90} }
};

// --- B. 后退 (Backward) 数据序列 ---
static const pose_frame_t DATA_BACKWARD[] = {
    // Phase 1: 左前 & 右后 抬腿 (55)
    { {55, 90, 90, 90, 55, 90, 90, 90} },
    // Phase 1: 后摆 (LT_H=125, RB_H=55)
    { {55, 125, 90, 90, 55, 55, 90, 90} },
    // Phase 1: 放下
    { {90, 125, 90, 90, 90, 55, 90, 90} },
    // Phase 1: 归位
    { {90, 90, 90, 90, 90, 90, 90, 90} },

    // Phase 2: 右前 & 左后 抬腿 (LB_K=125, RT_K=125)
    { {90, 90, 125, 90, 90, 90, 125, 90} },
    // Phase 2: 后摆 (LB_H=125, RT_H=55)
    { {90, 90, 125, 55, 90, 90, 125, 125} },
    // Phase 2: 放下
    { {90, 90, 90, 55, 90, 90, 90, 125} },
    // Phase 2: 归位
    { {90, 90, 90, 90, 90, 90, 90, 90} }
};

// --- C. 左移 (Move Left) 数据序列 ---
static const pose_frame_t DATA_LEFT[] = {
    // Phase 1: LT_K(55), RB_K(55)
    { {55, 90, 90, 90, 55, 90, 90, 90} },
    // Phase 1: LT_H(125), RB_H(55)
    { {55, 125, 90, 90, 55, 55, 90, 90} },
    // Phase 1: Knees 90
    { {90, 125, 90, 90, 90, 55, 90, 90} },
    // Phase 1: Hips 90
    { {90, 90, 90, 90, 90, 90, 90, 90} },

    // Phase 2: LB_K(125), RT_K(125)
    { {90, 90, 125, 90, 90, 90, 125, 90} },
    // Phase 2: LB_H(55), RT_H(125)
    { {90, 90, 125, 125, 90, 90, 125, 55} },
    // Phase 2: Knees 90
    { {90, 90, 90, 125, 90, 90, 90, 55} },
    // Phase 2: Hips 90
    { {90, 90, 90, 90, 90, 90, 90, 90} }
};

// --- D. 右移 (Move Right) 数据序列 ---
static const pose_frame_t DATA_RIGHT[] = {
    // Phase 1: RT_K(125), LB_K(125)
    { {90, 90, 125, 90, 90, 90, 125, 90} },
    // Phase 1: RT_H(55), LB_H(125)
    { {90, 90, 125, 55, 90, 90, 125, 125} },
    // Phase 1: Knees 90
    { {90, 90, 90, 55, 90, 90, 90, 125} },
    // Phase 1: Hips 90
    { {90, 90, 90, 90, 90, 90, 90, 90} },

    // Phase 2: LT_K(55), RB_K(55)
    { {55, 90, 90, 90, 55, 90, 90, 90} },
    // Phase 2: LT_H(55), RB_H(125)
    { {55, 55, 90, 90, 55, 125, 90, 90} },
    // Phase 2: Knees 90
    { {90, 55, 90, 90, 90, 125, 90, 90} },
    // Phase 2: Hips 90
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
							if (BSP_MPU6050_GetData(&g_imu_data) == 0) {
									// 如果需要调试，可以取消下面这行的注释打印数据
									//printf("%.1f,%.1f,%.1f\r\n", g_imu_data.yaw, g_imu_data.pitch, g_imu_data.roll);
									
								
								
									// SYS_LOG("MOTO", "IMU: %.1f,%.1f,%.1f\n", g_imu_data.yaw, g_imu_data.pitch, g_imu_data.roll);
									// 【预留接口】你可以将 g_imu_data 的数据通过 fsm_push_event 压入状态机，
									// 或者直接给 PID 控制器使用。
								}
						}
					}
			fsm_run(&g_Motor_fsm);
			vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
