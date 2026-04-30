/**
 * @file sys_events.h
 * @brief 系统事件ID定义
 * 
 * 该文件定义了整个系统中使用的所有事件ID。
 * 这些事件被用于：
 * - 状态机之间的通信（通过事件总线）
 * - 中断服务程序向任务层传递事件
 * - 模块间的解耦通信
 * 
 * @note 事件ID从0开始顺序递增，EVT_MAX_NUM用于数组大小定义
 */

#ifndef __SYS_EVENTS_H
#define __SYS_EVENTS_H

#include <stdint.h>

/**
 * @enum sys_event_t
 * @brief 系统事件枚举
 * 
 * 按照功能模块对事件进行分类组织，便于维护和理解。
 * 每个事件ID在系统中是唯一的。
 */
typedef enum {
    /** @name 空事件（占位用） */
    /**@{*/
    EVT_NONE = 0,   /**< 无效事件，用于初始化或错误处理 */
    /**@}*/

    /* --- 1. 物理输入事件 (来自按键中断) --- */
    /**
     * @name 物理输入事件
     * @brief 由硬件外设触发，通常在中断上下文中产生
     */
    /**@{*/
    EVT_KEY1_SHORT_PRESS,   /**< 按键1短按*/
    EVT_KEY1_LONG_PRESS,    /**< 按键1长按*/
    EVT_KEY2_SHORT_PRESS,   /**< 按键2短按 */
    EVT_KEY2_LONG_PRESS,    /**< 按键2长按 */
    /**@}*/

    /* --- 2. 主控系统内部逻辑事件 (Main FSM ) --- */
    /**
     * @name 系统内部逻辑事件
     * @brief 由主控状态机内部产生，用于驱动自身状态流转
     */
    /**@{*/
    EVT_INIT_DONE,          /**< 系统初始化完成，准备进入就绪状态 */
    EVT_SELECT_1,           /**< 逻辑选中模式1（如菜单项1被选中） */
    EVT_SELECT_2,           /**< 逻辑选中模式2 */
    EVT_SELECT_3,           /**< 逻辑选中模式3 */
    EVT_TIMEOUT,            /**< 菜单/操作超时，返回默认状态 */
    EVT_ERROR,              /**< 系统错误事件，触发错误处理流程 */
		/* --- 新增：传感器与避障事件 --- */
		EVT_WARN_OBSTACLE,      /**< 超声波探测到危险距离，触发避障锁 */
    EVT_OBSTACLE_CLEARED,   /**< 障碍物已移除，解除避障锁 */
    EVT_SENSOR_DATA_READY,  /**< 传感器数据就绪 (可选订阅) */
    /**@}*/

    /* --- 3. 电机指令事件 (通过总线广播) --- */
    /**
     * @name 电机控制事件
     * @brief 由主控模块发布，电机模块订阅执行
     * @note 这些事件通过事件总线广播，实现模块解耦
     */
    /**@{*/
    EVT_MOTOR_STOP,         /**< 电机停止转动 */
    EVT_MOTOR_FORWARD,      /**< 电机前进（直线运动） */
    EVT_MOTOR_BACKWARD,     /**< 电机后退（直线运动） */
    EVT_MOTOR_LEFT,         /**< 电机左转（差速转向） */
    EVT_MOTOR_RIGHT,        /**< 电机右转（差速转向） */
    EVT_MOTOR_ROT_L,        /**< 电机原地左旋转 */
    EVT_MOTOR_ROT_R,        /**< 电机原地右旋转 */
		
		
    /* --- 4. 网络状态事件 (发布给 Main FSM 刷新 UI) --- */
    /**
     * @name 物理输入事件
     * @brief 由硬件外设触发，通常在中断上下文中产生
     */
    /**@{*/
    EVT_NET_STATUS_INIT,
    EVT_NET_STATUS_WIFI_CONN,
    EVT_NET_STATUS_MQTT_CONN,
    EVT_NET_STATUS_ONLINE,
    EVT_NET_STATUS_ERROR,
    /**@}*/
    /**
     * @brief 通用电机执行指令
     * @details 用于需要参数化的复杂电机动作
     * @param param 参数含义（由调用者和接收者约定）：
     *        - 位[7:0]：动作类型（速度、角度等）
     *        - 位[15:8]：动作参数
     *        - 位[31:16]：保留
     */
    EVT_MOTOR_EXECUTE,
    /**@}*/
		
		EVT_NET_EULER_OPEN,   // 开启上报欧拉角信息
		EVT_NET_EULER_CLOSE,  // 关闭上报欧拉角信息
    /**
     * @name 事件计数
     * @brief 用于定义数组大小和边界检查
     */
    /**@{*/
    EVT_MAX_NUM             /**< 事件总数（必须保持在枚举最后） */
    /**@}*/

} sys_event_t;

#endif /* __SYS_EVENTS_H */
