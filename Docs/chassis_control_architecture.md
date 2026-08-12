# 整车底盘运动控制架构

## 1. 目标

这套架构把现有的七段 S 曲线速度规划和 X 形麦克纳姆轮解算串成一条能够继续接入真车的控制链，同时把上位机协议、电机协议、FreeRTOS 调度和纯算法分开。

```text
上位机 / 本地遥控 / 机器人总状态机
                |
                | 已解析的 ChassisCommand_t
                v
       FreeRTOS 命令队列（只传数据）
                |
                v
       chassis_task：5 ms 固定周期
                |
                v
       chassis_control：状态机与安全监督
          |                       ^
          |                       |
          v                       |
  S 曲线 / SlewLimiter       电机反馈正解与里程计
          |                       ^
          v                       |
      麦轮逆运动学                |
          |                       |
          v                       |
       四个目标 rpm               |
          |                       |
          v                       |
  chassis_port：电机协议适配层 ----+
          |
          v
     FDCAN 与四个电机驱动器
```

## 2. 文件职责

| 文件 | 职责 |
| --- | --- |
| `APP/Common/robot_protocol.h` | 定义模块间已经解析好的底盘命令，不绑定 UART、USB 或 CAN 字节格式。 |
| `APP/Chassis/chassis_config.h` | 真车尺寸、减速比、方向、速度、加速度、超时等参数的唯一配置入口。 |
| `APP/Chassis/chassis_control.c/.h` | 纯控制核心：状态机、命令仲裁、规划、坐标变换、麦轮正逆解、里程计、超时和故障。 |
| `APP/Chassis/chassis_task.c/.h` | CMSIS-RTOS2 任务和命令队列，固定周期调用控制核心。 |
| `APP/Chassis/chassis_port.c/.h` | 运动控制与真实电机协议之间的适配口。当前是安全弱实现。 |
| `APP/Chassis/StrategyAlgorithm.c` | 七段 S 曲线位置规划。 |
| `APP/Chassis/chassis_mecanum.c` | X 形麦轮正逆运动学和速度斜坡。 |

`Core` 中的 CubeMX 文件只负责初始化外设和创建任务，不放机器人业务逻辑。

## 3. 两种运动模式

### 3.1 速度模式

速度模式适合人工遥控、视觉实时修正和上位机连续控制。上层直接给：

```text
Vx：向前速度，m/s
Vy：向左速度，m/s
Wz：逆时针角速度，rad/s
```

控制链为：

```text
BODY_VELOCITY 命令
       -> SlewLimiter 限制突变
       -> 麦轮逆解
       -> 四电机 rpm
```

上位机必须周期刷新速度命令或心跳。命令超时后，控制器先进入 `STOPPING`，利用斜坡限制器减速到零，然后进入故障状态，等待明确清故障和重新使能。

### 3.2 位移/位姿模式

位移模式适合自动流程，例如“前进 1.5 m，同时逆时针旋转 90 度”。

平移使用一条路径 S 曲线，而不是 X、Y 各自独立规划：

```text
起点 P0 = (x0, y0)
终点 P1 = (x1, y1)
路径距离 D = |P1 - P0|
路径方向 n = (P1 - P0) / D

S 曲线输出路径速度 v
Vx_odom = n.x * v
Vy_odom = n.y * v
```

这样斜向运动时，X/Y 始终保持同一比例，理论轨迹是直线。Yaw 另用一条 S 曲线规划，所以平移和旋转能够同时完成。

绝对 yaw 命令会先转换为当前位置到目标角度的最短角差。例如当前为
`+179°`、目标为 `-179°` 时，控制器选择约 `+2°`，不会反向旋转 `358°`。
相对 yaw 命令则保留上层明确给出的正负方向和圈数。

规划速度首先位于固定的 ODOM 坐标系，随后根据当前 yaw 转到机器人 BODY 坐标系，再交给麦轮逆解：

```text
Vx_body =  cos(yaw) * Vx_odom + sin(yaw) * Vy_odom
Vy_body = -sin(yaw) * Vx_odom + cos(yaw) * Vy_odom
```

### 3.3 相对运动示例

机器人当前朝向任意，要求沿自身前方移动 1 m 并逆时针旋转 90 度：

```c
ChassisCommand_t command = {0};

command.sequence = 1U;
command.source = CHASSIS_SOURCE_AUTONOMY;
command.type = CHASSIS_COMMAND_MOVE_RELATIVE;
command.frame = CHASSIS_FRAME_BODY;
command.valid_for_ms = 100U;
command.payload.pose.x_m = 1.0f;
command.payload.pose.y_m = 0.0f;
command.payload.pose.yaw_rad = 1.5707963f;

(void)ChassisTask_PostCommand(&command, 0U);
```

这是任务层接口示例，不是上位机线缆报文格式。

## 4. 上位机对接

上位机接收任务应只负责：

1. 接收 UART/USB/CAN 数据。
2. 检查帧头、长度、CRC、序号和协议版本。
3. 将整数缩放值转换成 SI 单位。
4. 构造 `ChassisCommand_t`。
5. 调用 `ChassisTask_PostCommand()`。

不要让上位机接收中断直接操作电机，也不要跨模块修改底盘静态变量。

### 4.1 使能与速度控制

```c
static uint32_t host_sequence;

void Host_EnableChassis(void)
{
    ChassisCommand_t command = {0};

    command.sequence = ++host_sequence;
    command.source = CHASSIS_SOURCE_HOST;
    command.type = CHASSIS_COMMAND_ENABLE;
    command.frame = CHASSIS_FRAME_BODY;
    command.valid_for_ms = 100U;
    (void)ChassisTask_PostCommand(&command, 0U);
}

void Host_SetChassisVelocity(float vx, float vy, float wz)
{
    ChassisCommand_t command = {0};

    command.sequence = ++host_sequence;
    command.source = CHASSIS_SOURCE_HOST;
    command.type = CHASSIS_COMMAND_BODY_VELOCITY;
    command.frame = CHASSIS_FRAME_BODY;
    command.valid_for_ms = 100U;
    command.payload.body_velocity.vx_mps = vx;
    command.payload.body_velocity.vy_mps = vy;
    command.payload.body_velocity.wz_radps = wz;
    (void)ChassisTask_PostCommand(&command, 0U);
}
```

必须先收到有效电机反馈、清除故障并成功使能，速度命令才会被接受。

### 4.2 心跳

长时间自动轨迹不能只发送一次位移命令后永远不通信。命令源应周期发送 `CHASSIS_COMMAND_HEARTBEAT`，推荐周期小于 `CHASSIS_COMMAND_TIMEOUT_MS / 3`。

心跳应使用与当前控制命令相同的 `source`，并继续递增该源的序号。低优先级来源不能用心跳夺取高优先级来源的控制权。

### 4.3 状态回传

上位机遥测任务可以调用：

```c
ChassisTask_Status_t status;

if (ChassisTask_GetStatus(&status))
{
    /* 序列化以下信息并回传：
     * status.control.state
     * status.control.fault_flags
     * status.control.pose
     * status.control.commanded_body_velocity
     * status.control.actual_body_velocity
     * status.control.target_motor_rpm[]
     * status.control.feedback_motor_rpm[]
     * status.control.motor_scale
     */
}
```

建议上位机显示状态、故障位、四轮目标/反馈和 `motor_scale`，调试效率会高很多。

## 5. 命令仲裁

控制源优先级由枚举值决定：

```text
SAFETY > LOCAL > AUTONOMY > HOST
```

规则：

- 当前来源持续发送命令或心跳时，低优先级来源不能覆盖它。
- 更高优先级来源可以接管。
- `ESTOP`、`DISABLE` 和 `STOP` 属于立即安全命令，不受普通所有权阻挡。
- 锁存了 `ESTOP` 后，只有 `SAFETY` 来源能够发送 `CLEAR_FAULT`。
- 同一来源使用递增序号，旧包和重复包会被拒绝。
- 当前来源超时后，其他来源可以取得控制权。

比赛流程状态机通常使用 `AUTONOMY`；本地遥控器使用 `LOCAL`；调试上位机使用 `HOST`；急停监督使用 `SAFETY`。

## 6. 控制状态机

```text
UNINITIALIZED
      |
      | 配置和算法初始化成功
      v
  DISABLED <---------------------------+
      | ENABLE                         | DISABLE
      v                                |
    IDLE ---- BODY_VELOCITY ----> VELOCITY
      |                                |
      +---- MOVE_* -----------> TRAJECTORY
      |                                |
      +<--------- 正常完成 ------------+
      |
      +---- STOP/命令超时 -----> STOPPING
                                     |
                                     | 速度降为零
                                     v
                               IDLE 或 FAULT

任意活动状态 -- ESTOP/反馈超时/发送失败/算法错误 --> FAULT
FAULT -- CLEAR_FAULT --> DISABLED（仍需重新 ENABLE）
```

急停不会自动恢复。清故障也不会自动使能，避免通信恢复瞬间突然继续运动。

## 7. 下位电机通信对接

当前 `chassis_port.c` 是弱符号安全占位实现：

- `ChassisPort_IsReady()` 始终返回 0。
- 不发送 FDCAN 报文。
- 控制器无法使能真车。

电机驱动负责人需要根据真实电机手册实现：

```c
uint8_t ChassisPort_Init(void);
uint8_t ChassisPort_IsReady(void);
uint8_t ChassisPort_SendMotorRpm(const float motor_rpm[4]);
uint8_t ChassisPort_ReadMotorRpm(float motor_rpm[4], uint32_t *time_ms);
void ChassisPort_SetMotorEnabled(uint8_t enabled);
```

必须统一：

| 项目 | 约定 |
| --- | --- |
| 轮序 | 数组固定为 FL、FR、RL、RR。 |
| 输入单位 | `SendMotorRpm` 接收电机轴 rpm。 |
| 反馈单位 | `ReadMotorRpm` 返回电机轴 rpm。 |
| 正负方向 | 由 `motor_direction[]` 统一修正，端口层不要再随意反号。 |
| 时间戳 | 返回该组四轮反馈完整更新的时间。 |
| 完整性 | 四个反馈必须属于接近的采样时刻；缺一轮时不要报告新快照。 |
| 发送失败 | 返回 0，控制任务将锁存 `CHASSIS_FAULT_MOTOR_TX` 并尝试发零。 |

FDCAN 接收中断只做快速取帧和更新驱动层缓存；报文解析后由 `ReadMotorRpm()` 给任务读取，不要在中断中做 S 曲线、麦轮解算或阻塞等待。

## 8. 反馈、正解与里程计

每个周期如果获得四轮新反馈，控制器调用 `ChassisMecanum_Forward()` 得到 BODY 速度，再根据 yaw 转到 ODOM 坐标系积分：

```text
dx/dt = cos(yaw)*Vx_body - sin(yaw)*Vy_body
dy/dt = sin(yaw)*Vx_body + cos(yaw)*Vy_body
dyaw/dt = Wz
```

当前是纯轮速里程计，麦轮横移打滑会造成累计误差。真车后续应让定位模块融合 IMU/视觉，并提供一个“校正当前 pose”的接口。绝不能把轮速积分当成比赛场地上的绝对真值。

## 9. FreeRTOS 协调

- `chassisTask` 优先级为 `osPriorityAboveNormal`，周期 5 ms。
- 默认 LED 任务继续以普通优先级运行。
- 上位机、总状态机通过消息队列生产命令，底盘任务是唯一消费者。
- 消息队列中 `ESTOP` 优先级最高，`DISABLE/STOP` 高于普通运动命令。
- 状态快照由互斥锁保护，遥测任务只读副本。
- 电机反馈由端口层缓存，底盘任务周期读取。

控制任务中不应进行串口打印、文件操作或长时间等待。调试数据交给低优先级遥测任务发送。

## 10. 真车启用前必须完成

在 `chassis_config.h` 中填写并核对：

- 轮子有效半径。
- 半轴距与半轮距。
- 电机轴/车轮减速比。
- 电机轴最大 rpm。
- FL、FR、RL、RR 四个方向系数。
- 最大 Vx、Vy、Wz。
- 速度模式加速度限制。
- 平移路径与旋转的 S 曲线参数。
- 命令和反馈超时。

然后完成：

1. 实现 `chassis_port` 的真实电机协议。
2. 确认四轮 CAN ID 和数组下标映射。
3. 架空测试六个基本方向。
4. 确认反馈正负号与目标一致。
5. 实测通信超时会停机。
6. 实测急停锁存，清故障后不会自动运动。
7. 低速落地标定轮径与旋转等效力臂。
8. 最后把 `CHASSIS_APP_CONFIG_READY` 从 `0U` 改为 `1U`。

在上述工作完成前保持 `CONFIG_READY=0`，底盘任务虽然存在，但不会初始化运动控制器，也不会驱动真车。
