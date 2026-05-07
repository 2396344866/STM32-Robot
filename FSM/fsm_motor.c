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
// 机制：对角1(左前/右后)先抬起。左前空中向前摆，右后空中向后摆。落地后瞬间归位产生左旋扭力。
static const pose_frame_t DATA_ROT_L[] = {
    // Phase 1: 左前 & 右后 抬腿 (Knee -> 55)
    { {55, 90, 90, 90, 55, 90, 90, 90} },
    // Phase 1: 空中摆动 (左前向前摆->55，右后向后摆->55)
    { {55, 55, 90, 90, 55, 55, 90, 90} },
    // Phase 1: 放下触地
    { {90, 55, 90, 90, 90, 55, 90, 90} },
    // Phase 1: 瞬时归位 (产生物理扭力，机身左旋)
    { {90, 90, 90, 90, 90, 90, 90, 90} },

    // Phase 2: 右前 & 左后 抬腿 (Knee -> 125)
    { {90, 90, 125, 90, 90, 90, 125, 90} },
    // Phase 2: 空中摆动 (右前向前摆->125，左后向后摆->125)
    { {90, 90, 125, 125, 90, 90, 125, 125} },
    // Phase 2: 放下触地
    { {90, 90, 90, 125, 90, 90, 90, 125} },
    // Phase 2: 瞬时归位 (产生物理扭力，机身左旋)
    { {90, 90, 90, 90, 90, 90, 90, 90} }
};

// --- F. 右旋 (Rotate Right) 数据序列 ---
// 机制：反转摆动方向。左前空中向后摆，落地向前推；右前空中向前摆，落地向后推。
static const pose_frame_t DATA_ROT_R[] = {
    // Phase 1: 左前 & 右后 抬腿 (Knee -> 55)
    { {55, 90, 90, 90, 55, 90, 90, 90} },
    // Phase 1: 空中摆动 (左前向后摆->125，右后向前摆->125)
    { {55, 125, 90, 90, 55, 125, 90, 90} },
    // Phase 1: 放下触地
    { {90, 125, 90, 90, 90, 125, 90, 90} },
    // Phase 1: 瞬时归位
    { {90, 90, 90, 90, 90, 90, 90, 90} },

    // Phase 2: 右前 & 左后 抬腿 (Knee -> 125)
    { {90, 90, 125, 90, 90, 90, 125, 90} },
    // Phase 2: 空中摆动 (右前向后摆->55，左后向前摆->55)
    { {90, 90, 125, 55, 90, 90, 125, 55} },
    // Phase 2: 放下触地
    { {90, 90, 90, 55, 90, 90, 90, 55} },
    // Phase 2: 瞬时归位
    { {90, 90, 90, 90, 90, 90, 90, 90} }
};

// --- G. 站立复位 (Stand) ---
// 这是一个逐步复位的序列，防止舵机打架
static const pose_frame_t DATA_STAND[] = {
    { {90, 90, 90, 90, 90, 90, 90, 90} } // 简单起见，一帧复位。若需动画可增加中间帧
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
//		LOG_RAW("%d,%d,%d,%d,%d,%d,%d,%d\n", 
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
/* ============================================================
 * 状态轮询函数 (Polling Functions) - 非阻塞实现

// 1. 站立复位 (Stand)
static void on_poll_stand(fsm_t* fsm, void* arg) {
    motor_ctx_t* ctx = (motor_ctx_t*)arg;
    TickType_t now = FSM_GET_TICK();
    if ((now - ctx->last_tick) < FSM_MS_TO_TICKS(STAND_INTERVAL_MS)) return;

    switch (ctx->seq_step) {
        case 0: BSP_Servo_SetAngle_left_top_knee(90); break;
        case 1: BSP_Servo_SetAngle_left_buttom_knee(90); break;
        case 2: BSP_Servo_SetAngle_right_top_knee(90); break;
        case 3: BSP_Servo_SetAngle_right_buttom_knee(90); break;
        case 4: BSP_Servo_SetAngle_left_top_hip(90); break;
        case 5: BSP_Servo_SetAngle_left_buttom_hip(90); break;
        case 6: BSP_Servo_SetAngle_right_top_hip(90); break;
        case 7: BSP_Servo_SetAngle_right_buttom_hip(90); break;
        default: return; // 完成后保持
    }
    ctx->seq_step++;
    ctx->last_tick = now;
}

// 2. 前进 (Move Forward / Move On)
static void on_poll_move_forward(fsm_t* fsm, void* arg) {
    motor_ctx_t* ctx = (motor_ctx_t*)arg;
    TickType_t now = FSM_GET_TICK();
    if ((now - ctx->last_tick) < FSM_MS_TO_TICKS(ACTION_INTERVAL_MS)) return;
    switch (ctx->seq_step) {
        // --- 阶段一：左前 & 右后 抬起前摆 ---
        case 0: 
            BSP_Servo_SetAngle_left_top_knee(55 );
            BSP_Servo_SetAngle_right_buttom_knee(55 );
            break;
        case 1: 
            BSP_Servo_SetAngle_left_top_hip(55 );
            BSP_Servo_SetAngle_right_buttom_hip(125 );
            break;
        case 2: 
            BSP_Servo_SetAngle_left_top_knee(90 );
            BSP_Servo_SetAngle_right_buttom_knee(90 );
            break;
        case 3: 
            BSP_Servo_SetAngle_left_top_hip(90 );
            BSP_Servo_SetAngle_right_buttom_hip(90 );
            break;
        // --- 阶段二：右前 & 左后 抬起前摆 ---
        case 4: 
            BSP_Servo_SetAngle_left_buttom_knee(125 );
            BSP_Servo_SetAngle_right_top_knee(125 );
            break;
        case 5: 
            BSP_Servo_SetAngle_left_buttom_hip(55 );
            BSP_Servo_SetAngle_right_top_hip(125 );
            break;
        case 6: 
            BSP_Servo_SetAngle_right_top_knee(90 );
            BSP_Servo_SetAngle_left_buttom_knee(90 );
            break;
        case 7: 
            BSP_Servo_SetAngle_left_buttom_hip(90 );
            BSP_Servo_SetAngle_right_top_hip(90 );
            break;
        default: 
            ctx->seq_step = -1; // 循环
            break;
    }
    ctx->seq_step++;
    ctx->last_tick = now;
}

// 3. 后退 (Move Backward) - 逻辑与前进相反
static void on_poll_move_backward(fsm_t* fsm, void* arg) {
    motor_ctx_t* ctx = (motor_ctx_t*)arg;
    TickType_t now = FSM_GET_TICK();
    if ((now - ctx->last_tick) < FSM_MS_TO_TICKS(ACTION_INTERVAL_MS)) return;

    switch (ctx->seq_step) {
        // --- 阶段一：左前 & 右后 向后 ---
        case 0:
            BSP_Servo_SetAngle_left_top_knee(55);
            BSP_Servo_SetAngle_right_buttom_knee(55);
            break;
        case 1:
            BSP_Servo_SetAngle_left_top_hip(125); // 后摆
            BSP_Servo_SetAngle_right_buttom_hip(55);
            break;
        case 2:
            BSP_Servo_SetAngle_left_top_knee(90);
            BSP_Servo_SetAngle_right_buttom_knee(90);
            break;
        case 3:
            BSP_Servo_SetAngle_left_top_hip(90);
            BSP_Servo_SetAngle_right_buttom_hip(90);
            break;
        // --- 阶段二：右前 & 左后 向后 ---
        case 4:
            BSP_Servo_SetAngle_left_buttom_knee(125);
            BSP_Servo_SetAngle_right_top_knee(125);
            break;
        case 5:
            BSP_Servo_SetAngle_left_buttom_hip(125); // 后摆
            BSP_Servo_SetAngle_right_top_hip(55);
            break;
        case 6:
            BSP_Servo_SetAngle_right_top_knee(90);
            BSP_Servo_SetAngle_left_buttom_knee(90);
            break;
        case 7:
            BSP_Servo_SetAngle_left_buttom_hip(90);
            BSP_Servo_SetAngle_right_top_hip(90);
            break;
        default:
            ctx->seq_step = -1; // 循环
            break;
    }
    ctx->seq_step++;
    ctx->last_tick = now;
}

// 4. 左平移 (Move Left)
static void on_poll_move_left(fsm_t* fsm, void* arg) {
    motor_ctx_t* ctx = (motor_ctx_t*)arg;
    TickType_t now = FSM_GET_TICK();
    if ((now - ctx->last_tick) < FSM_MS_TO_TICKS(ACTION_INTERVAL_MS)) return;

    switch (ctx->seq_step) {
        // 阶段一
        case 0: BSP_Servo_SetAngle_left_top_knee(55);  BSP_Servo_SetAngle_right_buttom_knee(55); break;
        case 1: BSP_Servo_SetAngle_left_top_hip(125);  BSP_Servo_SetAngle_right_buttom_hip(55);  break;
        case 2: BSP_Servo_SetAngle_left_top_knee(90);  BSP_Servo_SetAngle_right_buttom_knee(90); break;
        case 3: BSP_Servo_SetAngle_left_top_hip(90);   BSP_Servo_SetAngle_right_buttom_hip(90);  break;
        // 阶段二
        case 4: BSP_Servo_SetAngle_left_buttom_knee(125); BSP_Servo_SetAngle_right_top_knee(125); break;
        case 5: BSP_Servo_SetAngle_left_buttom_hip(55);   BSP_Servo_SetAngle_right_top_hip(125);  break;
        case 6: BSP_Servo_SetAngle_right_top_knee(90);    BSP_Servo_SetAngle_left_buttom_knee(90); break;
        case 7: BSP_Servo_SetAngle_left_buttom_hip(90);   BSP_Servo_SetAngle_right_top_hip(90);    break;
        default: ctx->seq_step = -1; break;
    }
    ctx->seq_step++;
    ctx->last_tick = now;
}

// 5. 右平移 (Move Right) - 对称修正
static void on_poll_move_right(fsm_t* fsm, void* arg) {
    motor_ctx_t* ctx = (motor_ctx_t*)arg;
    TickType_t now = FSM_GET_TICK();
    if ((now - ctx->last_tick) < FSM_MS_TO_TICKS(ACTION_INTERVAL_MS)) return;

    switch (ctx->seq_step) {
        // 阶段一：与左移相反，先动右前/左后？或者保持顺序但角度反向
        case 0: BSP_Servo_SetAngle_right_top_knee(125); BSP_Servo_SetAngle_left_buttom_knee(125); break; 
        case 1: BSP_Servo_SetAngle_right_top_hip(55);   BSP_Servo_SetAngle_left_buttom_hip(125);  break;
        case 2: BSP_Servo_SetAngle_right_top_knee(90);  BSP_Servo_SetAngle_left_buttom_knee(90);  break;
        case 3: BSP_Servo_SetAngle_right_top_hip(90);   BSP_Servo_SetAngle_left_buttom_hip(90);   break;
        // 阶段二：再动 左前(Top_L) 和 右后(Bottom_R)
        case 4: BSP_Servo_SetAngle_left_top_knee(55);   BSP_Servo_SetAngle_right_buttom_knee(55); break;
        case 5: BSP_Servo_SetAngle_left_top_hip(55);    BSP_Servo_SetAngle_right_buttom_hip(125); break;
        case 6: BSP_Servo_SetAngle_left_top_knee(90);   BSP_Servo_SetAngle_right_buttom_knee(90); break;
        case 7: BSP_Servo_SetAngle_left_top_hip(90);    BSP_Servo_SetAngle_right_buttom_hip(90);  break;
        default: ctx->seq_step = -1; break;
    }
    ctx->seq_step++;
    ctx->last_tick = now;
}

// 6. 左旋转 (Rotate Left)
static void on_poll_rotate_left(fsm_t* fsm, void* arg) {
    motor_ctx_t* ctx = (motor_ctx_t*)arg;
    TickType_t now = FSM_GET_TICK();
    if ((now - ctx->last_tick) < FSM_MS_TO_TICKS(ACTION_INTERVAL_MS)) return;

    switch (ctx->seq_step) {
        case 0: // 抬起对角腿，并扭腰
            BSP_Servo_SetAngle_left_top_hip(120);
            BSP_Servo_SetAngle_right_buttom_hip(120);
            BSP_Servo_SetAngle_left_top_knee(60);	
            BSP_Servo_SetAngle_right_buttom_knee(60);	
            break;
        case 1: // 切换重心，另一组对角抬起
            BSP_Servo_SetAngle_left_top_hip(150);
            BSP_Servo_SetAngle_right_buttom_hip(150);
            BSP_Servo_SetAngle_right_top_hip(120);	
            BSP_Servo_SetAngle_left_buttom_hip(120);	
            BSP_Servo_SetAngle_right_top_knee(90);	
            BSP_Servo_SetAngle_left_buttom_knee(90);	
            BSP_Servo_SetAngle_left_top_knee(30);	
            BSP_Servo_SetAngle_right_buttom_knee(30);
            break;
        case 2: // 复位过程
            BSP_Servo_SetAngle_right_top_hip(150);	
            BSP_Servo_SetAngle_left_buttom_hip(150);		
            BSP_Servo_SetAngle_left_top_hip(90);
            BSP_Servo_SetAngle_right_buttom_hip(90);
            BSP_Servo_SetAngle_right_top_knee(120);	
            BSP_Servo_SetAngle_left_buttom_knee(120);
            break;
        case 3: // 完全复位
            BSP_Servo_SetAngle_left_top_knee(90);  
            BSP_Servo_SetAngle_left_buttom_knee(90);  
            BSP_Servo_SetAngle_right_top_knee(90);  
            BSP_Servo_SetAngle_right_buttom_knee(90);
            BSP_Servo_SetAngle_left_top_hip(90);	
            BSP_Servo_SetAngle_left_buttom_hip(90);	
            BSP_Servo_SetAngle_right_top_hip(90);	
            BSP_Servo_SetAngle_right_buttom_hip(90);	
            break;
        default: ctx->seq_step = -1; break;
    }
    ctx->seq_step++;
    ctx->last_tick = now;
}

// 7. 右旋转 (Rotate Right)
static void on_poll_rotate_right(fsm_t* fsm, void* arg) {
    motor_ctx_t* ctx = (motor_ctx_t*)arg;
    TickType_t now = FSM_GET_TICK();
    if ((now - ctx->last_tick) < FSM_MS_TO_TICKS(ACTION_INTERVAL_MS)) return;

    // 逻辑与左旋对称：角度方向相反 (120 -> 60, 150 -> 30)
    switch (ctx->seq_step) {
        case 0:
            BSP_Servo_SetAngle_left_top_hip(60);
            BSP_Servo_SetAngle_right_buttom_hip(60);
            BSP_Servo_SetAngle_left_top_knee(60);	
            BSP_Servo_SetAngle_right_buttom_knee(60);
            break;
        case 1:
            BSP_Servo_SetAngle_left_top_hip(30);
            BSP_Servo_SetAngle_right_buttom_hip(30);
            BSP_Servo_SetAngle_right_top_hip(60);	
            BSP_Servo_SetAngle_left_buttom_hip(60);	
            BSP_Servo_SetAngle_right_top_knee(90);	
            BSP_Servo_SetAngle_left_buttom_knee(90);	
            BSP_Servo_SetAngle_left_top_knee(30);	
            BSP_Servo_SetAngle_right_buttom_knee(30);
            break;
        case 2:
            BSP_Servo_SetAngle_right_top_hip(30);	
            BSP_Servo_SetAngle_left_buttom_hip(30);		
            BSP_Servo_SetAngle_left_top_hip(90);
            BSP_Servo_SetAngle_right_buttom_hip(90);
            BSP_Servo_SetAngle_right_top_knee(120);	
            BSP_Servo_SetAngle_left_buttom_knee(120);
            break;
        case 3:
            BSP_Servo_SetAngle_left_top_knee(90);  
            BSP_Servo_SetAngle_left_buttom_knee(90);  
            BSP_Servo_SetAngle_right_top_knee(90);  
            BSP_Servo_SetAngle_right_buttom_knee(90);
            BSP_Servo_SetAngle_left_top_hip(90);	
            BSP_Servo_SetAngle_left_buttom_hip(90);	
            BSP_Servo_SetAngle_right_top_hip(90);	
            BSP_Servo_SetAngle_right_buttom_hip(90);	
            break;
        default: ctx->seq_step = -1; break;
    }
    ctx->seq_step++;
    ctx->last_tick = now;
}
 * ============================================================ */

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
