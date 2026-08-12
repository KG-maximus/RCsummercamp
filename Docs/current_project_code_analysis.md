# 当前下位机工程代码分析报告

工程路径：

`D:/Users/EmbeddedSystem/RCsummercamp/RobotLower computer`

本文用于快速理解当前代码在“干什么”、“怎么干”，以及后续调真车时应该“怎么做”。

## 1. 全局框架

当前工程已经从 CubeMX 生成的基础 HAL 工程，扩展成了一个初步完整的下位机底盘控制框架：

```text
Core/
  CubeMX/HAL/FreeRTOS 生成层
  负责芯片时钟、GPIO、DMA、USART、FDCAN、任务创建、中断入口

APP/Common/
  公共底层能力
  负责 PID、FDCAN 通用发送/过滤、M3508 电机驱动、统一回调、协议结构体

APP/Chassis/
  底盘业务层
  负责麦轮解算、速度规划、底盘状态机、底盘任务、电机端口适配
```

主控制链路：

```text
上位机/遥控/总状态机
        |
        v
构造 ChassisCommand_t
        |
        v
ChassisTask_PostCommand() 放入队列
        |
        v
chassisTask 每 3ms 取命令、更新状态机
        |
        v
ChassisControl_Step()
        |
        +--> 速度模式：SlewLimiter 限加速度
        |
        +--> 路径模式：七段 S 曲线规划
        |
        v
ChassisMecanum_Inverse() 算四轮目标 rpm
        |
        v
ChassisPort_SendMotorRpm()
        |
        v
M3508GroupUpdate() 级联 PID 算 C620 电流
        |
        v
FDCAN2 发送 0x200 电流帧
```

电机反馈链路：

```text
C620/M3508 反馈帧 0x201~0x204
        |
        v
FDCAN2 中断
        |
        v
HAL_FDCAN_RxFifo0Callback()
        |
        v
bsp_callback 分发给注册模块
        |
        v
ChassisPort_FDCANRxHandler()
        |
        v
M3508GroupParseFeedback()
        |
        v
ChassisTask 读取四轮 rpm
        |
        v
ChassisMecanum_Forward() 反算实际 Vx/Vy/Wz
        |
        v
ChassisControl_UpdateOdometry() 更新里程计估计
```

## 2. Core 层

### 2.1 main.c

`Core/Src/main.c` 是 CubeMX 主入口。

它负责：

- 初始化 MPU；
- 调用 `HAL_Init()`；
- 配置系统时钟；
- 初始化 GPIO、DMA、FDCAN1、FDCAN2、FDCAN3、USART1；
- 初始化并启动 FreeRTOS。

这里没有写底盘业务逻辑，这是正确的。`main.c` 只负责系统启动，具体控制逻辑放在 FreeRTOS 任务和 APP 模块里。

### 2.2 freertos.c

`Core/Src/freertos.c` 创建了两个任务：

```text
defaultTask
  普通优先级
  每 500ms 翻转 STATUS_LED
  用于确认程序烧录后是否正常运行

chassisTask
  AboveNormal 优先级
  周期 3ms
  负责底盘控制主循环
```

`defaultTask` 是当前最简单的运行测试手段。烧录后 LED 闪烁，说明程序至少已经正常启动并进入 FreeRTOS。

`chassisTask` 在创建前会先调用 `ChassisTask_Init()`，初始化成功后才创建任务。

### 2.3 fdcan.c

当前三路 FDCAN 引脚：

```text
FDCAN1: PA11 RX, PA12 TX
FDCAN2: PB12 RX, PB13 TX
FDCAN3: PF6 RX, PF7 TX
```

结合原理图，底盘电机总线使用 FDCAN2，也就是：

```c
#define CHASSIS_M3508_CAN_HANDLE (&hfdcan2)
```

FDCAN2 的 `RxFifo0ElmtsNbr` 已经设置为 4，适合同时接收四个 C620 的反馈帧。

### 2.4 stm32h7xx_it.c

中断文件中，FDCAN1/2/3 中断只调用：

```c
HAL_FDCAN_IRQHandler(&hfdcanx);
```

真正的接收处理放在 HAL 回调 `HAL_FDCAN_RxFifo0Callback()` 中，再由 `bsp_callback` 分发。

## 3. APP/Common 层

### 3.1 robot_protocol.h

`robot_protocol.h` 定义的是机器人内部命令格式，不是串口或 CAN 的原始字节协议。

核心命令结构：

```c
typedef struct
{
    uint32_t sequence;
    uint32_t issued_at_ms;
    uint32_t valid_for_ms;
    ChassisCommandSource_t source;
    ChassisCommandType_t type;
    ChassisReferenceFrame_t frame;
    union
    {
        RobotBodyVelocity_t body_velocity;
        RobotPose2D_t pose;
    } payload;
} ChassisCommand_t;
```

它可以表达：

- 心跳；
- 使能；
- 禁用；
- 平滑停止；
- 急停；
- 清除故障；
- 车体速度控制；
- 相对位移运动；
- 绝对位姿运动。

后续上位机通信模块应该先解析原始数据帧，再构造 `ChassisCommand_t`，最后投递给底盘任务。

### 3.2 ControlAlgorithm.c/h

该模块提供：

- 普通 PID；
- 级联 PID。

当前 M3508 驱动使用级联 PID：

```text
外环：速度误差 rpm -> 目标电流
内环：实际电流 -> 输出电流命令
```

目前 PID 参数全部是 0，并且 `CHASSIS_M3508_PID_CONFIG_READY` 也是 0，所以电机不会输出非零电流。这是安全保护，不是错误。

### 3.3 fdcan_common.c/h

提供两个通用接口：

```text
FDCANStandardInit()
  配置标准帧范围过滤器
  开启 FIFO0 新消息中断
  启动 FDCAN

FDCANSendStandard()
  发送标准 ID、Classic CAN、0~8 字节数据帧
```

当前适合 C620/M3508 的 Classic CAN 通信。

### 3.4 bsp_callback.c/h

该模块统一管理 FDCAN 接收回调。

它实现：

```c
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t rx_fifo0_its)
```

收到标准帧后，会分发给所有注册过的处理函数。

这样以后多个模块都要用 FDCAN 时，不需要每个人都修改 HAL 回调函数。

注意：该函数运行在中断中，只适合做轻量工作，不能阻塞，不能做复杂计算。

### 3.5 M3508.c/h

该模块封装 M3508/C620 电机驱动。

主要功能：

```text
M3508GroupInit()
  初始化一组 4 个电机

M3508GroupSetTarget()
  设置某个电机目标 rpm

M3508GroupParseFeedback()
  解析 0x201~0x208 反馈帧

M3508GroupUpdate()
  根据目标 rpm 和反馈 rpm 计算电流，并发送 C620 控制帧
```

当前底盘电机默认：

```text
控制帧：0x200
反馈帧：
  ID1 -> 0x201
  ID2 -> 0x202
  ID3 -> 0x203
  ID4 -> 0x204
```

## 4. APP/Chassis 层

### 4.1 chassis_config.h

这是底盘实车参数的唯一入口。

当前最重要的安全开关：

```c
#define CHASSIS_APP_CONFIG_READY       (0U)
#define CHASSIS_M3508_PID_CONFIG_READY (0U)
```

含义：

```text
CHASSIS_APP_CONFIG_READY = 0
  底盘控制器不会真正初始化

CHASSIS_M3508_PID_CONFIG_READY = 0
  电机驱动只会发送零电流
```

后续需要补充：

- 车轮有效半径 `CHASSIS_WHEEL_RADIUS_M`；
- 半轴距 `CHASSIS_HALF_WHEELBASE_M`；
- 半轮距 `CHASSIS_HALF_TRACK_M`；
- 减速比 `CHASSIS_GEAR_RATIO`；
- 电机最大 rpm；
- 四个电机方向；
- 最大平移/旋转速度；
- 最大加速度；
- 七段 S 曲线规划参数；
- M3508 PID 参数。

### 4.2 chassis_mecanum.c/h

这是四轮 X 形麦克纳姆轮运动学解算模块。

坐标系：

```text
+X  前进
+Y  左移
+Wz 从上往下看逆时针旋转
轮序固定 FL, FR, RL, RR
```

逆运动学公式：

```text
FL = (Vx - Vy - L*Wz) / r
FR = (Vx + Vy + L*Wz) / r
RL = (Vx + Vy - L*Wz) / r
RR = (Vx - Vy + L*Wz) / r
```

其中：

```text
L = half_wheelbase_m + half_track_m
r = wheel_radius_m
```

然后将车轮角速度转换为电机转速：

```text
motor_rpm = wheel_radps * gear_ratio * 60 / (2π) * motor_direction
```

`motor_direction[]` 只用于修正电机安装方向，不能用于交换轮子位置。

该模块还会进行四轮等比例限幅。如果任意电机超过最大 rpm，就把四个目标 rpm 同比例缩小，保证底盘运动方向不变。

### 4.3 SlewLimiter

`ChassisMecanum_SlewLimiter_t` 是一阶速度斜坡限制器。

它限制每个周期速度变化量：

```text
本周期最大变化量 = 最大加速度 × 控制周期
```

例如控制周期 3ms，最大加速度 1m/s²，则每周期 Vx 最多变化：

```text
1 × 0.003 = 0.003 m/s
```

它适合速度模式下防止突然给满速度，不等同于完整路径规划。

### 4.4 StrategyAlgorithm.c/h

这是七段 S 曲线速度规划模块。

状态包括：

```text
phase1 加加速
phase2 匀加速
phase3 减加速
phase4 匀速
phase5 加减速
phase6 匀减速
phase7 减减速
idle   到位
```

它用于“给定当前位置和目标位置后，生成平滑速度曲线”。

当前底盘控制中使用两条规划器：

```text
planner_translation
  用于平移距离规划

planner_yaw
  用于旋转角度规划
```

### 4.5 chassis_control.c/h

这是底盘总控制状态机。

状态包括：

```text
UNINITIALIZED
DISABLED
IDLE
VELOCITY
TRAJECTORY
STOPPING
FAULT
```

它负责：

- 配置合法性检查；
- 命令合法性检查；
- 命令来源优先级判断；
- 命令超时保护；
- 反馈超时保护；
- 急停锁存；
- 速度限幅；
- 路径规划；
- 麦轮逆解；
- 麦轮正解；
- 里程计积分；
- 故障安全输出零 rpm。

真车调试时，如果底盘不动，优先看：

```text
configuration_ready
port_ready
feedback_valid
fault_flags
last_command_result
```

### 4.6 chassis_task.c/h

这是 FreeRTOS 底盘任务。

每 3ms 执行一次：

```text
1. 获取当前时间
2. 检查电机端口是否 ready
3. 读取四轮反馈 rpm
4. 更新底盘反馈
5. 处理命令队列
6. 推进一步底盘状态机
7. 发送四轮目标 rpm
8. 根据状态决定是否允许电机输出
9. 发布状态快照
10. 等待下一个 3ms 周期
```

它使用 `osDelayUntil()`，适合固定周期控制任务。

### 4.7 chassis_port.c/h

这是底盘算法和 M3508/C620 驱动之间的适配层。

它负责把：

```text
FL/FR/RL/RR 四轮 rpm
```

映射到：

```text
C620 电机 ID 1/2/3/4
```

如果后续发现某个轮子对应的 CAN ID 不对，应该优先修改 `chassis_config.h` 中的 ID 映射，而不是修改麦轮公式。

## 5. 当前程序能做什么

当前程序已经具备：

- STM32H723 外设初始化；
- FreeRTOS 任务创建；
- LED 闪烁测试；
- FDCAN2 底盘电机总线初始化；
- C620/M3508 反馈解析；
- 底盘状态机；
- 速度命令、停止、急停、清故障；
- 相对/绝对位姿命令接口；
- 麦轮逆解；
- 麦轮正解；
- M3508 目标 rpm 接口。

但目前还不能直接让真车运动，因为：

```text
CHASSIS_APP_CONFIG_READY = 0
CHASSIS_M3508_PID_CONFIG_READY = 0
机械参数仍是 0
PID 参数仍是 0
还没有上位机/遥控命令生产者
```

## 6. 推荐实车调试顺序

### 第一步：测试烧录和板子运行

- 烧录程序；
- 看 STATUS_LED 是否 500ms 闪烁；
- 确认程序正常启动。

### 第二步：测试 FDCAN2 反馈

- 接一个 C620 + M3508；
- 确认电机 ID；
- 观察反馈帧是否更新；
- 检查 `speed_rpm/current/temperature` 是否正常。

### 第三步：确认四个电机 ID

- 分别确认 FL、FR、RL、RR 对应的 C620 ID；
- 修改 `CHASSIS_M3508_ID_FL/FR/RL/RR`。

### 第四步：填写机械参数

- 车轮有效半径；
- 半轴距；
- 半轮距；
- 减速比；
- 最大 rpm；
- 速度和加速度限制。

### 第五步：单电机 PID 标定

- 先单独测一个电机；
- 小速度、小电流开始；
- 调速度环和电流环；
- 确认不振荡、不发热、响应正常。

### 第六步：四轮架空方向标定

- 架空底盘；
- 发很小的前进速度；
- 检查四轮方向；
- 方向反的轮子修改 `motor_direction`。

### 第七步：低速落地测试

依次测试：

- 前进；
- 后退；
- 左移；
- 右移；
- 顺时针旋转；
- 逆时针旋转。

### 第八步：接入上位机通信

上位机通信模块应该：

```text
接收原始数据
校验数据
解析命令
构造 ChassisCommand_t
调用 ChassisTask_PostCommand()
```

不要绕过底盘状态机直接发电机 rpm。

## 7. 后续开发建议

建议优先补充：

- 一个简单的底盘调试命令模块；
- 底盘状态打印；
- 单电机测试模式；
- FDCAN 发送函数返回值；
- M3508 反馈读取保护；
- `StrategyAlogrithm.h` 文件名拼写修正；
- `chassis_config.h` 参数说明文档；
- 上位机通信协议解析模块；
- 总状态机与底盘命令接口。

## 8. 最推荐的阅读顺序

```text
1. chassis_config.h
   先看参数和安全开关

2. robot_protocol.h
   看上层能发什么命令

3. chassis_task.c
   看 FreeRTOS 如何周期调度底盘

4. chassis_control.c
   看底盘状态机如何处理命令

5. chassis_mecanum.c
   看 Vx/Vy/Wz 如何变成四轮 rpm

6. chassis_port.c
   看四轮 rpm 如何接到 M3508/C620

7. M3508.c
   看电机反馈和电流帧如何处理

8. bsp_callback.c
   看 FDCAN 中断如何统一分发
```

## 9. 总结

当前代码已经不是简单的“让车动一下”的测试程序，而是一个比较清晰的底盘控制框架。

它的核心思想是：

```text
上层只发语义命令
底盘状态机负责安全和模式切换
规划器负责平滑速度
麦轮解算负责运动学
端口层负责连接真实电机
M3508 驱动负责 FDCAN 反馈和电流输出
```

接下来真正要做的，是把机械参数填准，把 FDCAN 反馈跑通，把 PID 调稳，再逐步打开安全开关。
