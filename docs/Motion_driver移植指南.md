# Motion_driver 移植指南（InvenSense Motion Driver 6.12）

> **为什么有这份文档 / 为什么目录不在仓库里**
> `Motion_driver/` 是 InvenSense（现 TDK）的 **Motion Driver 6.12** 专有库，受 InvenSense 专有许可约束，**不得公开再分发**。因此本仓库已从 Git 跟踪中移除该目录（本地仍保留，便于编译/参考）。本文说明如何在本地或新机器上把它重新接入本项目。

---

## 1. 文件清单与角色

| 文件 | 角色 | 是否专有 |
|------|------|----------|
| `inv_mpu.c` / `inv_mpu.h` | MPU6050 寄存器访问核心：自检、校准、`mpu_init`、FIFO、低功耗 | 是 |
| `inv_mpu_dmp_motion_driver.c` / `.h` | DMP 数字运动处理器固件加载与特征使能（四元数输出等） | 是 |
| `dmpKey.h` / `dmpmap.h` | DMP 内存映射与密钥表，供 `dmp_load_motion_driver_firmware()` 下载 DMP 镜像 | 是 |
| `libmpllib.lib` | InvenSense **预编译**的 MPL（Motion Processing Library），被上述 `.c` 链接调用 | 是（二进制） |

> 本项目**自有**的桥接与算法代码（可公开）位于：
> - `DD_Device_Driver/dev_mpu6050.c`（实现 `Sensors_I2C_WriteRegister/ReadRegister`、`Dev_MPU6050_Init`、`Dev_MPU6050_Read_DMP`、`quat_to_euler`）
> - `BSP_Board_Support_Package/bsp_mpu6050.c`（I2C/EXTI 适配器注入）
> - `HAL_Hardware_Driver_Layer/hal_hard_i2c.c`（I2C2 时序，PB10/11，100kHz）

---

## 2. 本项目中的集成方式（调用链）

```
fsm_motor.c (20ms 姿态环)
   └─ BSP_MPU6050_GetData()
        └─ Dev_MPU6050_Read_DMP()            [dev_mpu6050.c]
             ├─ dmp_read_fifo(gyro,accel,quat,...)   ← Motion_driver/inv_mpu_dmp_motion_driver.c
             └─ quat_to_euler(quat, ypr)             ← dev_mpu6050.c（项目自有）
                  └─ 输出 pitch/roll/yaw(float) 给 PD 作偏差
```

**I2C 后端注入**（关键移植点）：
- `Motion_driver/inv_mpu.c` 第 48–49 行把库内 `i2c_write`/`i2c_read` 宏映射到
  `Sensors_I2C_WriteRegister` / `Sensors_I2C_ReadRegister`。
- 这两个函数由 `dev_mpu6050.c` 实现，内部调用 `HAL_HardI2C_WriteMem` / `ReadMem`
  （I2C2，从机地址 `0x68`）。**移植时只要保证这两个函数指向你的 I2C 驱动即可。**

**DMP 固件加载**：`Dev_MPU6050_Init()` 在 `mpu_init(NULL)` 之后，通过
`Sensors_I2C_WriteRegister(0x68, 0x37/0x38, …)` 配置 INT 引脚，并调用
`dmp_load_motion_driver_firmware()`（依赖 `dmpKey.h`/`dmpmap.h`）把 DMP 镜像烧入 MPU6050。
**没有这三个文件，DMP 四元数输出不可用。**

---

## 3. 本地恢复步骤（重新接入编译）

> 仓库根目录下的 `Motion_driver/` 在本地磁盘仍然保留（只是 `git status` 显示为 untracked）。
> 若是全新克隆或文件丢失，按下面做。

1. **获取库文件**（二选一）：
   - 从本机备份取：`C:\Users\123\Desktop\Motion_driver_backup_<时间戳>\`（随移除操作生成）；
   - 或从 InvenSense 官方 **Embedded Motion Driver (EMD) 6.12** 软件包中提取同名文件。
2. **放置目录**：把 7 个文件整体放进仓库根目录的 `Motion_driver/` （保持原文件名不变）。
3. **Keil 包含路径**：`Options → C/C++ → Include Paths` 增加 `.\Motion_driver`。
4. **加入编译**：把 `inv_mpu.c`、`inv_mpu_dmp_motion_driver.c` 添加进工程（或确认已加）。
5. **链接库**：在 `Options → Linker` 中加入 `Motion_driver\libmpllib.lib`
   （该 `.lib` 为 **ARM/Keil (ARMCC)** 工具链预编译；见第 4 节工具链注意事项）。
6. **编译验证**：见第 5 节。

---

## 4. 工具链注意事项（libmpllib.lib）

- `libmpllib.lib` 是 InvenSense 为 **ARMCC/Keil** 预编译的 MPL，**直接用 Keil 链接即可**。
- 若改用 **GCC (arm-none-eabi-gcc)**：该 `.lib` 是 ARMCC 格式，GCC **无法直接链接**。可选方案：
  1. 联系 InvenSense 获取 GCC 版 `.a`（需签署 EMD 许可）；
  2. 仅使用 `inv_mpu.c` + `inv_mpu_dmp_motion_driver.c` 的**源码**部分（DMP 四元数路径不依赖 MPL），把 MPL 相关符号按需桩化（stub）或裁剪；
  3. 用 `arm-none-eabi-ar` 配合 `fromelf` 转换（不推荐，依赖具体版本）。
- 本项目当前工程为 **Keil/ARMCC**，故直接链接 `.lib` 即可，无需上述处理。

---

## 5. 验证清单

- [ ] `Dev_MPU6050_Init()` 返回 0，串口打印 `[OK] MPU6050 Ready`。
- [ ] 20ms 姿态环 `dmp_read_fifo` 返回 0，`sensors` 含 `INV_WXYZ_QUAT` 标志。
- [ ] `quat_to_euler` 输出 pitch/roll/yaw 在静止时稳定（±2° 以内），转动时有连续变化。
- [ ] 姿态数据接入 PD 后机器人站立/步态平稳（与移除前行为一致）。

---

## 6. 许可与合规提醒

- Motion Driver 6.12 为 **InvenSense 专有软件**，许可禁止未经授权的公开再分发。
- 已将其移出本仓库的 Git 跟踪与远程；切勿再次 `git add` 或 `git commit` 该目录。
- 若需开源本项目，请保留本文档作为“如何本地获取专有依赖”的说明，而不要附带库文件。
- 备份目录 `Motion_driver_backup_*` 仅存于本地，请勿上传到任何公开位置。

---

## 7. 一键核对（移除后）

```bash
# 确认已从 Git 跟踪移除（输出应为空）
git ls-files | grep -i Motion_driver

# 确认磁盘上仍在（本地编译可用）
ls Motion_driver/

# 确认 .gitignore 已忽略，避免误 add
grep -n "Motion_driver" .gitignore
```
