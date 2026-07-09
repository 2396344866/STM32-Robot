# 四足机器人 FSM 应用层框架文档

## 1. 概述

本项目采用**事件驱动有限状态机（FSM）** 作为应用层核心架构，通过**事件总线**实现模块间解耦。整个应用层由四个独立运行的 RTOS 任务（状态机）组成：

- **主系统控制**：管理操作界面（OLED 显示）和用户交互逻辑。包含避障安全锁。
- **电机控制**：驱动 8 路舵机执行步态序列，并采集 IMU 数据。
- **网络通信**：通过 ESP8266 + MQTT 连接阿里云 IoT，收发控制指令及传感器数据。
- **按键扫描**：基于 FreeRTOS 定时器的按键消抖与状态机，产生短按 / 长按事件。
- **环境传感**：根据网络状态动态休眠/唤醒，非阻塞轮询超声波、DHT11 与烟雾传感器，触发安全拦截。


四个状态机通过**事件总线**相互通信，结构图如下：

```mermaid
graph TD
    subgraph Tasks
        MT[Main_System_FSM] -->|读取队列| KEY_Q[KeyLogicQueue]
        MT -- 订阅/发布 --> BUS[事件总线 event_bus]
        MOT[Motor_FSM] -- 订阅/发布 --> BUS
        NET[Network_FSM] -- 订阅/发布 --> BUS
        SENS[Sensor_FSM] -- 订阅/发布 --> BUS
    end
    KEY_Timer[KeyScan定时器] -->|推送按键事件| KEY_Q
    SENS -- 数据就绪/避障警告 --> BUS
    NET -- 解析阿里云指令 --> BUS
    BUS -->|广播| MT
    BUS -->|广播| MOT
    BUS -->|广播| NET
```

## 2. 框架核心模块

### 2.1 状态机核心 (`fsm_core`)---无需修改

提供基于**事件队列**（环形缓冲区）和**状态迁移表**的轻量级状态机实现。

**关键数据结构：**

- `fsm_event_t`：事件 ID + 参数。
- `fsm_transition_t`：迁移规则（当前状态、事件、目标状态、守卫条件、迁移动作）。
- `fsm_state_desc_t`：状态回调（进入/退出/轮询）。
- `fsm_t`：状态机实例句柄，包含事件队列、迁移表、状态描述表、用户数据等。

**主要 API：**

| 函数 | 功能 |
|------|------|
| `fsm_init` | 初始化状态机（绑定队列、迁移表、起始状态） |
| `fsm_set_state_callbacks` | 注册状态回调表（on_enter/on_exit/on_poll） |
| `fsm_push_event` | 向队列推送事件（线程安全，可在中断中调用） |
| `fsm_run` | 驱动状态机：执行当前状态的 `on_poll`，然后处理所有队列中的事件 |

**执行流程：**
1. 每个 RTOS 任务周期调用 `fsm_run`。
2. 先执行当前状态的 `on_poll`（用于非阻塞轮询，如网络 AT 指令发送、舵机序列播放）。
3. 循环取出事件队列中的所有事件，匹配迁移表，依据守卫条件决定是否迁移，执行状态切换回调（`on_exit` → `trans_action` → `on_enter`）或自流转动作。

### 2.2 事件总线 (`event_bus`)---无需修改

实现**订阅 - 发布**模式，采用静态数组存储订阅关系，无动态内存分配。

**主要 API：**

| 函数 | 功能 |
|------|------|
| `event_bus_init` | 清空订阅表 |
| `event_bus_subscribe(fsm, event_id)` | 状态机订阅指定事件 ID |
| `event_bus_publish(event_id, param)` | 广播事件，所有订阅者都会收到 |

容量限制： 由 sys_config.h 中的 BUS_MAX_SUBS 定义。订阅超限会静默失败。

### 2.3 系统事件定义 (`sys_events.h`)--可增删查改

所有事件 ID 集中在枚举 `sys_event_t` 中，按功能分组：

- **物理输入**：`EVT_KEY1_SHORT_PRESS`, `EVT_KEY1_LONG_PRESS`, `EVT_KEY2_...`
- **系统内部**：`EVT_INIT_DONE`, `EVT_SELECT_1/2/3`, `EVT_TIMEOUT`, `EVT_ERROR`
- **电机指令**：`EVT_MOTOR_STOP/FORWARD/BACKWARD/LEFT/RIGHT/ROT_L/ROT_R`, `EVT_MOTOR_EXECUTE`
- **网络状态**：`EVT_NET_STATUS_INIT/WIFI_CONN/MQTT_CONN/ONLINE/ERROR`
- **欧拉角上报**：`EVT_NET_EULER_OPEN`, `EVT_NET_EULER_CLOSE`
- **传感器与避障**：`EVT_WARN_OBSTACLE`, `EVT_OBSTACLE_CLEARED`, `EVT_SENSOR_DATA_READY`



### 2.4 全局配置 (`sys_config.h`)--可增删查改

统一管理 RTOS 依赖、临界区、调试开关。

```c
#define USE_FREERTOS              // 启用 FreeRTOS 支持
#define BUS_MAX_SUBS 30           // 事件总线最大订阅数
#define ENABLE_DEBUG_PRINT 1      // 全局调试打印开关（1 开，0 关）

// FreeRTOS 模式下提供的工具宏
#define FSM_GET_TICK()       xTaskGetTickCount()
#define FSM_MS_TO_TICKS(ms)  pdMS_TO_TICKS(ms)
#define FSM_DELAY_MS(ms)     vTaskDelay(pdMS_TO_TICKS(ms))
#define FSM_ENTER_CRITICAL() taskENTER_CRITICAL()
#define FSM_EXIT_CRITICAL()  taskEXIT_CRITICAL()
```

当 `ENABLE_DEBUG_PRINT == 0` 时，`printf` 被宏替换为空操作，`SYS_LOG` 等日志宏也失效。

---

## 3. 应用层状态机详细设计

### 3.1 按键扫描状态机 (`fsm_key`)

**文件：** `fsm_key.c/h`  
**任务方式：** 不单独创建任务，使用 FreeRTOS **软件定时器** 每 10ms 触发一次扫描。

**引脚定义：**

| 按键 | GPIO | 功能 | 事件 |
|------|------|------|------|
| K1 | PA12 | 模式选择 | `EVT_KEY1_SHORT_PRESS` / `EVT_KEY1_LONG_PRESS` |
| K2 | PB13 | 确认 / 停止 | `EVT_KEY2_SHORT_PRESS` / `EVT_KEY2_LONG_PRESS` |

**消抖逻辑：**
- 按下持续扫描，累加计数器 `press_cnt`。
- 当 `press_cnt >= KEY_DEBOUNCE_TICKS`（2 ticks = 20ms）且释放时，判定为短按，发送短按事件到 `xKeyLogicQueue`。
- 若 `press_cnt == KEY_LONG_TICKS`（80 ticks = 800ms），触发长按事件（一次性），并继续计数。
**输出队列：** `xKeyLogicQueue`，由主系统状态机任务接收。
**队列创建：** 队列在 `KEY_Init` 前由主任务创建，长度为 8，元素为 `uint16_t`。
```mermaid
stateDiagram-v2
    [*] --> Idle : 初始
    Idle --> Pressed : 引脚拉低
    Pressed --> ShortPress : 释放且计时在消抖与长按之间
    Pressed --> LongPress : 计时 = 长按阈值
    ShortPress --> Idle : 事件入队
    LongPress --> Pressed : 继续计数
    Pressed --> Idle : 引脚拉高(抖动)
```

### 3.2 主系统状态机 (`fsm_main_system`)

**文件：** `fsm_main_system.c/h`  
**任务：** `Main_System_FSM_task`，周期 50ms。

**状态定义：**

| 状态 | 含义 |
|------|------|
| `STATE_INIT` | 系统初始化 |
| `STATE_LINKING` | 等待网络连接（MQTT 上线） |
| `STATE_IDLE` | 待机界面（OLED 显示待机） |
| `STATE_MOTOR_CTRL` | 电机控制界面（显示动作菜单） |
| `STATE_BLOCKED` | 避障锁定状态（物理隔离所有指令） |
| `STATE_ERROR` | 网络连接失败 UI 报错界面（显示 2 秒后自动退回待机） |

**状态流转图：**

```mermaid
stateDiagram-v2
    [*] --> STATE_INIT
    STATE_INIT --> STATE_LINKING : EVT_INIT_DONE
    STATE_LINKING --> STATE_MOTOR_CTRL : EVT_NET_STATUS_ONLINE
    STATE_LINKING --> STATE_ERROR : EVT_NET_STATUS_ERROR
    STATE_ERROR --> STATE_IDLE : EVT_TIMEOUT (2秒报错展示)
    STATE_IDLE --> STATE_MOTOR_CTRL : 任意按键短按 / APP 指令
    STATE_MOTOR_CTRL --> STATE_IDLE : K2 长按 / 超时(EVT_TIMEOUT)
    STATE_MOTOR_CTRL --> STATE_MOTOR_CTRL : K1短按(切换动作), K2短按(启停), 电机事件同步
    STATE_IDLE --> STATE_BLOCKED : EVT_WARN_OBSTACLE (传感器检测到障碍)
    STATE_MOTOR_CTRL --> STATE_BLOCKED : EVT_WARN_OBSTACLE (传感器检测到障碍)
    STATE_BLOCKED --> STATE_IDLE : EVT_OBSTACLE_CLEARED (障碍已移除)
```

**关键上下文：**

- `m1_menu_index`：当前选中的电机动作索引（`MOTOR_ITEMS`）。
- `run_source`：0 = 停止，1 = 本地按键，2 = APP 云端。用于 OLED 显示区分来源。
- `xMenuTimer`：10 秒无操作自动进入 IDLE 的单次定时器。有动作时重启或停止。

**主要功能：**

- 读取 `xKeyLogicQueue` 获得按键事件，直接推入自己的 FSM。
- 通过事件总线订阅以下事件，实现与被控端的解耦：
  - `EVT_NET_STATUS_ONLINE`
  - `EVT_MOTOR_STOP/FORWARD/BACKWARD/LEFT/RIGHT/ROT_L/ROT_R`
  - `EVT_WARN_OBSTACLE`、`EVT_OBSTACLE_CLEARED`   <!-- 新增 -->
- 根据当前状态和事件，向总线发布电机控制指令（`event_bus_publish`）。
- 更新 OLED 显示（`BSP_OLED_ShowString` 等）。
- **避障安全锁**：当收到 `EVT_WARN_OBSTACLE` 时强制进入 `STATE_BLOCKED`，立即停止电机、清除运行源并关闭待机定时器，该状态下**不再响应**任何按键或 APP 指令，直到收到 `EVT_OBSTACLE_CLEARED` 后返回 `STATE_IDLE`，实现物理隔离。

### 3.3 电机控制状态机 (`fsm_motor`)

**文件：** `fsm_motor.c/h`  
**任务：** `Motor_FSM_task`，周期 20ms。

**状态定义：** 与电机运动一一对应：

- `STATE_MOTOR_STOP`
- `STATE_MOTOR_FORWARD`
- `STATE_MOTOR_BACKWARD`
- `STATE_MOTOR_LEFT`
- `STATE_MOTOR_RIGHT`
- `STATE_MOTOR_ROT_L`
- `STATE_MOTOR_ROT_R`

**驱动原理：** 采用**频率解耦的闭环前馈补偿架构**。系统存在两条独立的时间轴：

1.  **低频步态发生器 (200ms 周期)：** 采用数据驱动模式。每种动作预定义一个 `gait_sequence_t`（包含 8 路舵机基准角度）。轮询器按照 200ms 的固定间隔切换动作帧，形成基础运动轨迹。
2.  **高频姿态补偿环 (20ms 周期)：**
    基于 PD 控制算法的姿态外环。与 RTOS 任务调度周期（20ms）强绑定。在步态帧切换的间隙，实时提取 MPU6050 欧拉角，计算补偿量并动态叠加到当前基准步态上。

**闭环算法核心机制：**
- **奇异点包裹 (Angle Wrap)：** 针对 MPU6050 倒置安装导致的水平面 ±180° 跃变问题，底层引入最短路径取模算法，将绝对误差强制映射至 `[-180°, 180°]` 连续区间，消除微分爆炸。
- **逆运动学极性映射：** 根据四足机器人的物理结构，对补偿量进行差分分配。
    - 前腿（LT/RT）与后腿（LB/RB）在 Pitch 轴补偿呈反向关系。
    - 左腿（LT/LB）与右腿（RT/RB）在 Roll 轴补偿呈反向关系。
- **硬件级钳位 (Clamp)：** 所有经过叠加运算的最终关节指令，在下发至 PWM 驱动前，均经过 `[45°, 135°]` 的安全物理限位截断，防止执行器扫齿。

**四足机器人步态数据序列（Gait Data Sequences）：**
本文件详细记录了四足机器人六自由度动作的动作序列帧。系统硬件舵机排序映射如下：
`{ 左前膝(LT_K), 左前髋(LT_H), 右前膝(RT_K), 右前髋(RT_H), 右后膝(RB_K), 右后髋(RB_H), 左后膝(LB_K), 左后髋(LB_H) }`
极性说明：
- **左前(LT) & 右后(RB)**：膝关节角度减小（90° -> 55°）为收腿抬高；髋关节角度减小为前摆。
- **右前(RT) & 左后(LB)**：膝关节角度增大（90° -> 125°）为收腿抬高；髋关节角度增大为前摆。
```c
// --- A. 前进 (Forward) 数据序列 ---
// 物理机制：标准的对角小跑（Trot步态）。通过交替抬起两组对角腿，并在空中完成髋关节前摆，触地后髋关节向后驱动机体前行。
static const pose_frame_t DATA_FORWARD[] = {
    // === Phase 1: 左前(LT) & 右后(RB) 对角相摆动 ===
    // 步骤 1: 左前与右后腿弯曲抬起，卸载支撑力矩，机体由右前与左后腿支撑
    { {55, 90, 90, 90, 55, 90, 90, 90} },
    // 步骤 2: 抬起腿的髋关节在空中跨步（LT前摆，RB前摆）；同时支撑腿的髋关节保持或微调
    { {55, 55, 90, 90, 55, 125, 90, 90} },
    // 步骤 3: 膝关节恢复至90°伸直触地，重新建立四足支撑态，准备交接转入支撑相
    { {90, 55, 90, 90, 90, 125, 90, 90} },
    // 步骤 4: 髋关节统一复位至中性态（90°），在此过程中产生向后的相对推力驱动机体前进
    { {90, 90, 90, 90, 90, 90, 90, 90} },
    
    // === Phase 2: 右前(RT) & 左后(LB) 对角相摆动 ===
    // 步骤 5: 右前与左后腿弯曲抬起（注意物理对称极性，Knee变为125°），转入空中摆动相
    { {90, 90, 125, 90, 90, 90, 125, 90} },
    // 步骤 6: 抬起腿的髋关节在空中完成跨步（RT前摆=125°，LB前摆=55°）
    { {90, 90, 125, 125, 90, 90, 125, 55} },
    // 步骤 7: 右前与左后膝关节伸直（恢复至90°）触地，完成当前相的动作下发
    { {90, 90, 90, 125, 90, 90, 90, 55} },
    // 步骤 8: 整体髋关节回摆复位至中性态（90°），推动机体平滑前移，回归初始就绪状态
    { {90, 90, 90, 90, 90, 90, 90, 90} }
};

// --- B. 后退 (Backward) 数据序列
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
    // 步骤 8: 整体髋关节复位至中性态（90°），完成一个完整的后退步态周期
    { {90, 90, 90, 90, 90, 90, 90, 90} }
};

// --- C. 左移 (Move Left) 数据序列
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

// --- D. 右移 (Move Right) 数据序列
// 物理机制：侧向平移的镜像控制。利用对角腿的髋关节朝右侧外展与内收，触地后拉动身躯整体右移。
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

// --- E. 左旋 (Rotate Left) 数据序列
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

// --- F. 右旋 (Rotate Right) 数据序列
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
```

**状态转换表：** 使用宏 `TRANS_ANY` 生成从任意当前状态到目标状态的迁移规则。例如收到 `EVT_MOTOR_FORWARD` 时，无论当前在何状态，都切换到 `STATE_MOTOR_FORWARD`。进入状态时调用 `enter_forward` 加载序列并立即执行第一帧。

**IMU 集成：** 在任务循环中检查 MPU6050 的就绪标志，读取欧拉角存入全局变量 `g_imu_data`，供网络任务上报。

### 3.4 网络通信状态机 (`fsm_network`)

**文件：** `fsm_network.c/h`  
**任务：** `Network_FSM_Task`，无固定周期（依赖队列阻塞和 FSM 轮询）。

**状态定义：**

| 状态 | 含义 |
|------|------|
| `STATE_NET_INIT` | 发送 AT 指令初始化 WiFi 模块 |
| `STATE_NET_WIFI_CONN` | 连接 WiFi |
| `STATE_NET_MQTT_CONN` | MQTT 配置与连接 |
| `STATE_NET_ONLINE` | 在线，可收发数据 |
| `STATE_NET_ERROR` | 错误状态 |

**状态流转图：**

```mermaid
stateDiagram-v2
    [*] --> STATE_NET_INIT
    STATE_NET_INIT --> STATE_NET_WIFI_CONN : EVT_INIT_DONE
    STATE_NET_WIFI_CONN --> STATE_NET_MQTT_CONN : 收到 "WIFI GOT IP" (EVT_SELECT_1)
    STATE_NET_MQTT_CONN --> STATE_NET_ONLINE : MQTT 连接成功 (EVT_SELECT_2)
    STATE_NET_WIFI_CONN --> STATE_NET_ERROR : EVT_ERROR (15秒超时)
    STATE_NET_MQTT_CONN --> STATE_NET_ERROR : EVT_ERROR
    STATE_NET_ONLINE --> STATE_NET_ERROR : EVT_ERROR
    STATE_NET_ERROR --> STATE_NET_INIT : EVT_TIMEOUT (休息 5 秒后硬件复位)
```

**关键行为：**

- **INIT 阶段：** 通过 `on_poll` 以非阻塞方式逐步发送 AT 指令（AT+RST、ATE0、AT+CWMODE=1），每步间隔 1 秒，防止堵塞。
- **WIFI 连接：** 在 `on_enter_wifi` 中发送 AT+CWJAP。
- **MQTT 连接：** 分步进行清理、用户配置、客户端 ID、连接、订阅主题，每个步骤之间至少间隔 2~3 秒。
- **在线状态：** 持续 `on_poll_online`，定时发布传感器数据（温度/湿度/烟雾/光照）以及欧拉角（若开启）。同时解析来自阿里云的下行 JSON，根据 `"value":1` 和动作名称向事件总线发布电机指令或欧拉角开关事件。
- **欧拉角上报开关：** 在网络状态机的 `on_poll_online` 中动态检查。该功能由 APP 通过解析 `Euler_angle_open` 字段控制，`value:1` 发布 `EVT_NET_EULER_OPEN` 事件触发 `action_euler_switch` 置位使能位。
- **阿里云 JSON 解析：** 使用简单的子串匹配 `strstr`，支持的控制指令：

- `"move_on"`, `"move_back"`, `"move_left"`, `"move_right"`, `"move_left_rotate"`, `"move_right_rotate"`, `"move_stop"`
- `"Euler_angle_open"` 及 `"value":1` / `"value":0` 控制欧拉角上报开关。

**调试模式暂停：** 若 `ENABLE_DEBUG_PRINT` 开启且通过调试串口收到任意数据，设置 `g_fsm_paused = 1`，网络状态机的 `on_poll` 将停止执行，以便手动控制 ESP8266。

---


### 3.5 传感器状态机 (`fsm_sensor`)

**文件：** `fsm_sensor.c/h`  
**任务：** `Sensor_FSM_Task`，动态周期（ACTIVE 态 100ms，SLEEP 态 500ms）。

**状态定义：**

| 状态 | 含义 |
|------|------|
| `STATE_SENS_SLEEP` | 休眠模式，硬件暂停，等待网络上线 |
| `STATE_SENS_ACTIVE` | 活跃采集模式，硬件运行，轮询传感器 |

**状态流转图：**

```mermaid
stateDiagram-v2
    [*] --> STATE_SENS_SLEEP
    STATE_SENS_SLEEP --> STATE_SENS_ACTIVE : EVT_NET_STATUS_ONLINE
    STATE_SENS_ACTIVE --> STATE_SENS_SLEEP : EVT_NET_STATUS_ERROR
    STATE_SENS_ACTIVE --> STATE_SENS_SLEEP : EVT_NET_STATUS_INIT
```

**主要功能：**
- **状态切换**：根据网络状态自动休眠/唤醒硬件传感器，节省功耗。
- **非阻塞采集**：聚合超声波、烟雾（ADC）与 DHT11 的数据，仅在 ACTIVE 状态下执行。
- **数据共享**：维护全局结构体 `g_sensor_data` 供网络状态机定时读取与上报。
- **避障裁决**：当距离在 `(2cm, 15cm)` 区间时，发布 `EVT_WARN_OBSTACLE`；当距离恢复至 `≥17cm` 时，发布 `EVT_OBSTACLE_CLEARED`。带有 2cm 物理迟滞，避免临界抖动。
- **DHT11 降频读取**：每 20 次轮询（约 2 秒）才读取一次温湿度，减少硬件抢占开销。


---


## 4. 事件流与订阅关系总览

| 事件 ID | 发布者 | 订阅者（状态机） | 用途 |
|---------|--------|------------------|------|
| `EVT_KEY*_PRESS` | 按键扫描（队列） | Main_System | 用户界面交互（菜单导航与启停） |
| `EVT_INIT_DONE` | Main_System / Network | 各自内部 / Main | 初始化完成通知 |
| `EVT_NET_STATUS_ONLINE` | Network | Main_System | 通知主控网络已可用 |
| `EVT_MOTOR_*` | Main_System（按键 / 菜单）或 Network（APP） | Motor | 执行舵机动作 |
| `EVT_SELECT_1/2` | Network 内部（解析串口返回） | Network 自身 | 驱动 WiFi → MQTT → ONLINE 的流转 |
| `EVT_NET_EULER_OPEN/CLOSE` | Network（解析 APP 指令） | Network 自身 | 控制欧拉角上报 |
| `EVT_TIMEOUT` | 主系统内部 (IdleTimeoutTimer) | Main_System 自身 | 10秒无操作切至待机 |
| `EVT_WARN_OBSTACLE/EVT_OBSTACLE_CLEARED` | Sensor | Main_System | 避障安全锁的触发和解除 |
| `EVT_NET_STATUS_INIT` | Network | Sensor | 网络复位时通知传感器进入休眠 |
| `EVT_NET_STATUS_ERROR` | Network | Sensor | 网络异常时通知传感器进入休眠 |
| `EVT_NET_STATUS_ONLINE` | Network | Sensor | 唤醒传感器进入活跃采集 |
**发布方式：**
- 按键事件通过 **FreeRTOS 队列** `xKeyLogicQueue` 传递给 Main_System 任务。
- 其他跨任务事件均通过 **事件总线** `event_bus_publish` 广播。

---

## 5. 任务与定时器配置

| 任务 / 定时器 | 调度方式 | 周期 / 触发条件 |
|--------------|----------|----------------|
| `KeyScanTimer` | 软件定时器 | 10ms 周期性 |
| `Main_System_FSM_task` | RTOS 任务 | 50ms 周期（vTaskDelayUntil） |
| `Motor_FSM_task` | RTOS 任务 | 20ms 周期 |
| `IdleTimeoutTimer` | 单次定时器 | Main_System 中 10 秒超时进入待机 |
| `Network_FSM_Task` | RTOS 任务 | 事件驱动 + 50ms 轮询超时（ `xQueueReceive` 非阻塞） |
| `Sensor_FSM_Task` | RTOS 任务 | 动态周期（ACTIVE 态 100ms，SLEEP 态 500ms） |

**多频调度说明：**
电机任务（`Motor_FSM_task`）虽然配置为 20ms 周期，但其内部状态机轮询器（`on_poll_gait`）实现了时间戳分频。
- 网络任务（`Network_FSM_Task`）通过总线异步下发控制指令，主导状态流转。
- 电机任务内部利用 `FSM_GET_TICK()` 计算差值，独立维持 20ms 的 PID 刷新率与 200ms 的动作步进率，确保底层运动学解算不受网络延迟或 UI 阻塞的影响。

---

## 6. 调试与日志系统

在 `sys_config.h` 中通过 `ENABLE_DEBUG_PRINT` 控制全工程日志输出。

- **开启时：** `SYS_LOG("TAG", "format", ...)` 可输出带标签的调试信息；`printf` 重定向到 USART2（调试串口）。
- **关闭时：** 所有 `printf`、`SYS_LOG` 和 `LOG_RAW` 均替换为 `((void)0)`，不占用串口带宽和 Flash 空间。

建议开发阶段设为 `1`，发布时设为 `0`。

此外，网络状态机还支持**调试暂停机制**：当 `ENABLE_DEBUG_PRINT == 1` 且通过调试串口接收到数据时，`g_fsm_paused` 置位，暂停自动轮询，以便手动调试 WiFi 模块。
注意: 暂停期间，电机 FSM 和主系统 FSM 的状态轮询不受影响。该机制仅在 Debug 版本生效。
---

## 7. 完整架构图



```mermaid
graph TD
    subgraph 物理层
        KEYS[按键 K1/K2]
        ESP[ESP8266]
        IMU[MPU6050]
        OLED[OLED]
        SERVO[8路舵机]
        SENSORS[DHT11/HCSR04/MQ2]
    end

    subgraph HAL/DD/BSP
        BSP[bsp_*.c]
    end

    subgraph 应用层 FSM
        KEY_FSM[fsm_key 扫描]
        MAIN_FSM[fsm_main_system]
        MOTOR_FSM[fsm_motor]
        NET_FSM[fsm_network]
        SENSOR_FSM[fsm_sensor]
    end

    subgraph 基础件
        EB[event_bus]
        FC[fsm_core]
        QUEUE[xKeyLogicQueue]
    end

    KEYS -->|引脚轮询| KEY_FSM
    KEY_FSM -->|事件入队| QUEUE
    QUEUE -->|xQueueReceive| MAIN_FSM
    MAIN_FSM -->|event_bus_publish| EB
    NET_FSM -->|event_bus_publish| EB
    SENSOR_FSM -->|event_bus_publish| EB
    EB -->|fsm_push_event| MAIN_FSM
    EB -->|fsm_push_event| MOTOR_FSM
    EB -->|fsm_push_event| NET_FSM

    %% 【可拓展】闭环反馈数据流
    IMU -.->|20ms高频读取欧拉角| MOTOR_FSM
    MOTOR_FSM -.->|姿态补偿叠加| SERVO
    MAIN_FSM -->|BSP 调用| BSP
    MOTOR_FSM -->|BSP 调用| BSP
    NET_FSM -->|BSP 调用| BSP
    SENSOR_FSM -->|BSP 调用| BSP
    BSP -->|驱动| ESP
    BSP -->|驱动| OLED
    BSP -->|驱动| IMU
    BSP -->|驱动| SERVO
    BSP -->|驱动| SENSORS
```


## 8. 扩展与二次开发指南

- **添加新事件：** 在 `sys_events.h` 枚举中添加，但需在 `EVT_MAX_NUM` 之前。随后在需要响应的状态机迁移表中增加相应条目，并通过 `event_bus_subscribe` 订阅。
- **添加新状态：** 在对应状态机的状态枚举中添加，设计迁移规则，实现 `on_enter/on_exit/on_poll` 回调，并注册到 `fsm_state_desc_t` 数组中。
- **添加新动作序列：** 在 `fsm_motor.c` 中参照 `DATA_FORWARD` 格式定义 `pose_frame_t` 数组，创建 `gait_sequence_t`，然后增加对应的进入回调、状态和转换规则。
- **修改阿里云解析逻辑：** 根据实际的物模型 JSON，调整 `parse_aliyun_payload` 中的字符串匹配。

---

> **版本记录：** V1.11  
> **适用硬件：** STM32F103 + ESP8266 + MPU6050 + SSD1306  
> **依赖：** FreeRTOS, STM32 标准外设库, INV_MPU/DMP 库