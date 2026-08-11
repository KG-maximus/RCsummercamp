# 四轮 X 形麦克纳姆轮底盘算法技术文档

## 1. 文档范围

本文档说明当前工程中的四轮 X 形麦克纳姆轮底盘运动学模块，面向第一次接触全向底盘、STM32 和电机通信的开发者。

对应源码：

- `RobotLower computer/APP/Chassis/chassis_mecanum.c`
- `RobotLower computer/APP/Chassis/chassis_mecanum.h`

模块负责以下三件事：

1. 把底盘目标速度 `(Vx, Vy, Wz)` 转换为四个电机目标转速，即逆运动学。
2. 把四个电机反馈转速转换为底盘估算速度，即正运动学。
3. 限制速度指令每个周期的变化量，减少突然加减速造成的冲击。

模块不负责生成 FDCAN 报文，不包含电机 CAN ID、控制模式和反馈协议，也不直接调用 HAL FDCAN 发送函数。当前工程的 `fdcan.c` 仅完成 FDCAN1、FDCAN2、FDCAN3 的 CubeMX 初始化，电机协议层仍需后续实现。

---

## 2. 麦克纳姆轮为什么能够全向移动

普通轮子的轮缘只能沿轮子滚动方向有效驱动车体，横向移动通常需要转向机构。麦克纳姆轮的轮缘由一圈倾斜滚子组成，常见滚子轴线与轮平面约成 45 度。轮子转动时，地面对滚子的作用力可以分解成纵向和横向两个分量。

单个麦轮的横向分量会使车体产生侧向趋势，但四个轮子的安装方向经过组合后，可以让某些分量互相抵消，让另一些分量相加：

- 四轮纵向分量同向叠加，底盘前进或后退。
- 对角轮采用不同转向，纵向分量抵消、横向分量叠加，底盘左右横移。
- 左右两侧形成相反的切向作用，产生绕底盘中心的旋转力矩。
- 三种作用可以同时存在，因此底盘可以一边平移一边旋转。

当前代码适用于滚子从上方观察呈 X 形组合的安装方式。如果安装成 O 形，横移项和旋转项的符号关系会不同，不能直接使用当前公式。

### 2.1 理想模型的前提

代码中的公式基于以下假设：

1. 四个麦轮半径相同，滚子角度为常见的 45 度。
2. 四轮中心构成矩形，车体参考点位于矩形中心。
3. 四个轮子始终接触地面，滚子能够正常转动。
4. 轮子与地面之间没有纵向或横向打滑。
5. 车架没有明显变形，轮轴互相平行。

实车不能完全满足这些条件。因此运动学结果应被理解为理想估计，而不是绝对真实位移。需要精确定位时，应融合编码器、IMU、视觉或外部定位信息。

---

## 3. 坐标系、轮序和正方向

所有调用者必须使用同一套坐标约定，否则最常见的现象就是“前进正常但横移或旋转方向相反”。

本模块规定：

- `+X`：机器人车头方向，即向前。
- `+Y`：机器人左侧方向，即向左横移。
- `+Wz`：从机器人上方朝地面观察时逆时针旋转。

从上方观察的轮子编号如下：

```text
                    车头 +X
                       ^
                       |
          FL 左前轮    |    右前轮 FR
                       |
          RL 左后轮    |    右后轮 RR
                       +----------> -Y（车体右侧）

车体左侧为 +Y
```

数组顺序始终固定为：

```text
motor_rpm[0] = FL 左前轮
motor_rpm[1] = FR 右前轮
motor_rpm[2] = RL 左后轮
motor_rpm[3] = RR 右后轮
```

代码通过 `ChassisMecanum_Wheel_t` 固定这些下标。不能为了修正某个电机方向而交换数组位置，否则逆解、正解和电机反馈的对应关系都会被破坏。

---

## 4. 机械参数配置

底盘参数保存在 `ChassisMecanum_Config_t` 中。

| 参数 | 单位 | 含义 | 测量与配置方法 |
| --- | --- | --- | --- |
| `wheel_radius_m` | m | 麦轮承载后的有效半径 `r` | 先用轮子直径的一半作为初值，再通过直线行驶距离标定。必须使用米。 |
| `half_wheelbase_m` | m | 底盘中心到前轮轴线或后轮轴线的距离 | 测量前后轮轴中心距离，再除以 2。 |
| `half_track_m` | m | 底盘中心到左轮中心或右轮中心的距离 | 测量左右轮中心距离，再除以 2。 |
| `gear_ratio` | 无量纲 | 电机转数除以车轮转数 | 直驱为 `1.0f`；电机转 10 圈、车轮转 1 圈则填写 `10.0f`。 |
| `max_motor_rpm` | rpm | 电机轴允许的最大目标转速 | 依据电机和驱动器手册，并留出安全余量。它不是车轮 rpm。 |
| `motor_direction[4]` | `+1/-1` | 电机命令正方向修正 | 通过架空低速测试逐轮确定，顺序必须是 FL、FR、RL、RR。 |

### 4.1 有效轮径

麦轮标称直径不一定等于实际运动学直径。轮胎压缩、滚子形状、负载和地面材料都会改变有效轮径。初次调试可使用卡尺测量值，随后再做距离标定。

若当前配置为 `r_old`，正解估算距离为 `D_est`，实测距离为 `D_actual`，可使用以下关系更新初值：

```text
r_new = r_old × D_actual / D_est
```

标定应低速进行并多次取平均，避免打滑影响结果。

### 4.2 半轴距与半轮距

代码中的旋转等效力臂为：

```text
L = half_wheelbase_m + half_track_m
```

这两个参数分别描述底盘中心到轮子的纵向和横向距离。当前理想模型的旋转公式只使用二者之和，但仍建议分别保存真实尺寸，以便代码含义清晰，并为后续扩展保留几何信息。

### 4.3 减速比

`gear_ratio` 的定义是：

```text
gear_ratio = 电机轴转数 / 车轮转数
```

例如减速比为 10:1 时，电机转 10 圈，车轮转 1 圈，因此填写 `10.0f`。把减速比填成倒数会让目标转速相差 100 倍，是必须优先检查的配置错误。

### 4.4 电机方向修正

`motor_direction[]` 只处理以下差异：

- 左右电机镜像安装，导致相同物理轮向对应相反的电机轴转向。
- 减速箱输出方向不同。
- 电机驱动器对“正 rpm”的定义不同。
- 编码器反馈正方向与底盘数学正方向不同。

推荐标定流程：

1. 将底盘架空，确保四个轮子不会接触地面。
2. 先把四个 `motor_direction` 都设为 `+1`。
3. 给很小的纯前进目标，例如 `Vx=0.1 m/s`、`Vy=0`、`Wz=0`。
4. 观察每个轮子的物理转向。
5. 方向错误的轮子将对应项改为 `-1`。
6. 再进行低速落地测试。

不要通过交换 FL、FR、RL、RR 来处理方向问题。

---

## 5. 逆运动学

逆运动学回答的问题是：底盘希望以 `(Vx, Vy, Wz)` 运动时，四个车轮分别需要多大的角速度。

输入结构体为：

```c
typedef struct
{
    float vx_mps;   /* 前后速度，m/s */
    float vy_mps;   /* 横移速度，m/s */
    float wz_radps; /* 自转角速度，rad/s */
} ChassisMecanum_BodyVelocity_t;
```

定义：

- `r = wheel_radius_m`
- `L = half_wheelbase_m + half_track_m`
- `FL、FR、RL、RR` 表示车轮角速度，单位为 `rad/s`

当前 X 形安装对应公式为：

```text
FL = (Vx - Vy - L×Wz) / r
FR = (Vx + Vy + L×Wz) / r
RL = (Vx + Vy - L×Wz) / r
RR = (Vx - Vy + L×Wz) / r
```

矩阵形式为：

```text
| FL |       |  1  -1  -L | | Vx |
| FR | = 1/r |  1   1   L | | Vy |
| RL |       |  1   1  -L | | Wz |
| RR |       |  1  -1   L |
```

### 5.1 每个公式项的含义

`Vx` 在四个公式中都是正号，表示纯前进时四个轮子的数学方向相同。

`Vy` 的符号按 `-、+、+、-` 排列，使四轮纵向作用互相抵消，而横向作用叠加。

`L×Wz` 的符号按 `-、+、-、+` 排列，使左右轮产生相反的切向速度，从而形成绕中心旋转的效果。

最后除以半径 `r`，是因为轮缘线速度与车轮角速度满足：

```text
轮缘线速度 = 车轮角速度 × 轮半径
车轮角速度 = 轮缘线速度 / 轮半径
```

### 5.2 基本运动的轮速符号

下表中的符号是乘以 `motor_direction[]` 之前的数学轮速符号。

| 目标运动 | Vx | Vy | Wz | FL | FR | RL | RR |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 前进 | `+` | `0` | `0` | `+` | `+` | `+` | `+` |
| 后退 | `-` | `0` | `0` | `-` | `-` | `-` | `-` |
| 向左横移 | `0` | `+` | `0` | `-` | `+` | `+` | `-` |
| 向右横移 | `0` | `-` | `0` | `+` | `-` | `-` | `+` |
| 逆时针旋转 | `0` | `0` | `+` | `-` | `+` | `-` | `+` |
| 顺时针旋转 | `0` | `0` | `-` | `+` | `-` | `+` | `-` |

### 5.3 平移和旋转同时发生

麦轮解算是线性叠加。比如同时前进、左移并逆时针旋转时：

```text
FL = 前进贡献 - 左移贡献 - 旋转贡献
FR = 前进贡献 + 左移贡献 + 旋转贡献
RL = 前进贡献 + 左移贡献 - 旋转贡献
RR = 前进贡献 - 左移贡献 + 旋转贡献
```

某个轮子的结果可能变小、变为零，甚至改变方向。这并不表示算法错误，而是不同运动分量在该轮上发生了抵消。

### 5.4 数值示例

假设：

```text
r = 0.10 m
half_wheelbase_m = 0.20 m
half_track_m = 0.15 m
L = 0.35 m
Vx = 1.00 m/s
Vy = 0.20 m/s
Wz = 0.50 rad/s
```

旋转线速度项为：

```text
L×Wz = 0.35×0.50 = 0.175 m/s
```

四轮角速度为：

```text
FL = (1.00 - 0.20 - 0.175) / 0.10 =  6.25 rad/s
FR = (1.00 + 0.20 + 0.175) / 0.10 = 13.75 rad/s
RL = (1.00 + 0.20 - 0.175) / 0.10 = 10.25 rad/s
RR = (1.00 - 0.20 + 0.175) / 0.10 =  9.75 rad/s
```

这些值还是车轮角速度，必须继续转换成电机轴 rpm。

---

## 6. 从车轮 rad/s 转换到电机 rpm

代码中的换算关系为：

```text
motor_rpm = wheel_radps
          × gear_ratio
          × 60/(2π)
          × motor_direction
```

其中：

```text
60/(2π) ≈ 9.5493
```

例如车轮角速度为 `10 rad/s`，减速比为 `10`，方向修正为 `+1`：

```text
motor_rpm = 10 × 10 × 9.5493 × 1 ≈ 954.93 rpm
```

若该电机实际安装方向相反，把对应 `motor_direction` 设置为 `-1`，最终命令变成 `-954.93 rpm`，幅值不变。

---

## 7. 四轮等比例限幅

逆解可能要求某个电机超过 `max_motor_rpm`。代码先找出四个目标中的最大绝对值：

```text
max_abs_rpm = max(|FL|, |FR|, |RL|, |RR|)
```

如果没有超过上限：

```text
scale = 1.0
```

如果超过上限：

```text
scale = max_motor_rpm / max_abs_rpm
四个 motor_rpm 同时乘以 scale
```

例如：

```text
原始目标：[2000, 1000, 1000, 2000] rpm
电机上限：1500 rpm
scale：1500/2000 = 0.75
最终输出：[1500, 750, 750, 1500] rpm
```

四轮比例没有改变，所以底盘的运动方向和旋转比例保持不变，只是整体速度降低。

如果只把两个 `2000 rpm` 截断为 `1500 rpm`，却保留另外两个 `1000 rpm`，结果会变成 `[1500, 1000, 1000, 1500]`。四轮比例从 `2:1:1:2` 变成 `1.5:1:1:1.5`，对应的 `(Vx, Vy, Wz)` 比例随之改变，机器人就可能偏航或偏离预期轨迹。

`ChassisMecanum_MotorCommand_t.scale` 可以用于诊断：

- `scale == 1.0f`：本次目标在电机能力范围内。
- `scale < 1.0f`：本次目标被整体压缩。
- `scale` 经常明显小于 1：上层给出的速度过大，或机械参数、减速比配置有误。

---

## 8. 正运动学

`ChassisMecanum_Forward()` 使用四个电机的实际反馈 rpm，估算底盘当前的 `(Vx, Vy, Wz)`。

### 8.1 先还原车轮角速度

输入必须是电机轴反馈 rpm，顺序仍为 FL、FR、RL、RR。代码先撤销方向修正和减速比：

```text
wheel_radps = motor_rpm
              × motor_direction
              × 2π/60
              / gear_ratio
```

由于 `motor_direction` 只能取 `+1` 或 `-1`，乘两次方向系数等价于恢复数学轮速正方向。

### 8.2 反算车体速度

```text
Vx = r/4 × ( FL + FR + RL + RR )

Vy = r/4 × (-FL + FR + RL - RR )

Wz = r/(4L) × (-FL + FR - RL + RR )
```

`Vx` 使用四轮共同分量，`Vy` 使用交叉符号提取横移分量，`Wz` 使用左右差动分量并除以旋转力臂。

### 8.3 正解为什么不等于真实速度

正解只能说明“按照理想无滑动模型，编码器转速对应的底盘速度”。以下情况会产生误差：

- 急加速导致轮子空转，编码器有转速但车体没有等比例移动。
- 麦轮横移时滚子与地面存在较大滑动。
- 四轮负载不均，有的轮子接地压力不足。
- 有效轮径与配置值不一致。
- 地面不平、滚子卡滞或轴承阻力不同。
- 电机反馈有延迟、滤波或量化误差。

正解适合做速度监控、粗略里程计和故障诊断。比赛机器人需要高精度位姿时，应与 IMU、视觉或其他定位系统融合。

---

## 9. SlewLimiter 速度斜坡限制器

如果速度指令从 `0` 突然跳到 `2 m/s`，逆解会立刻给出较大的电机目标转速。电机电流、机械冲击和车轮打滑都会明显增加。

`ChassisMecanum_SlewLimiterStep()` 对每个轴分别限制一个周期内允许发生的速度变化：

```text
本周期最大速度变化量 = 最大加速度 × delta_time_s
```

以 X 方向为例：

```text
当前 Vx = 0 m/s
目标 Vx = 2 m/s
最大加速度 = 1 m/s²
控制周期 dt = 0.01 s
```

本周期最多增加：

```text
1 × 0.01 = 0.01 m/s
```

因此第一次输出 `0.01 m/s`，第二次输出 `0.02 m/s`，持续调用后形成线性速度斜坡。

### 9.1 初始化

```c
ChassisMecanum_Status_t ChassisMecanum_SlewLimiterInit(
    ChassisMecanum_SlewLimiter_t *limiter,
    float max_vx_accel_mps2,
    float max_vy_accel_mps2,
    float max_wz_accel_radps2);
```

三个加速度参数必须是有限正数。初始化后，限制器认为底盘当前静止。

### 9.2 周期推进

```c
ChassisMecanum_Status_t ChassisMecanum_SlewLimiterStep(
    ChassisMecanum_SlewLimiter_t *limiter,
    const ChassisMecanum_BodyVelocity_t *target,
    float delta_time_s,
    ChassisMecanum_BodyVelocity_t *limited);
```

`delta_time_s` 必须使用秒。例如 1 ms 任务周期应传入 `0.001f`，5 ms 周期应传入 `0.005f`。

X、Y、Wz 三个方向独立限制，逻辑简单且方便调试。它并不严格限制三轴组合后的矢量合加速度。

### 9.3 Reset

```c
void ChassisMecanum_SlewLimiterReset(
    ChassisMecanum_SlewLimiter_t *limiter,
    const ChassisMecanum_BodyVelocity_t *velocity);
```

- `velocity == NULL`：把当前速度记忆清零，适合急停或重新使能。
- `velocity` 指向有效速度：以该速度作为新的斜坡起点，适合控制模式平滑切换。
- Reset 不会改变三个加速度上限。

### 9.4 与七段 S 曲线的区别

| 项目 | SlewLimiter | 七段 S 曲线速度规划 |
| --- | --- | --- |
| 主要输入 | 目标速度 | 当前位置和目标位置 |
| 主要作用 | 限制速度变化率 | 规划完整的加速、匀速、减速过程 |
| 加速度连续性 | 加速度可发生阶跃 | 通过 jerk 限制使加速度更平滑 |
| 是否考虑剩余距离 | 不考虑 | 考虑减速距离和目标位置 |
| 适用场景 | 遥控速度保护、简单指令滤波 | 自动路径、定点运动、精确停靠 |

如果七段 S 曲线已经输出平滑速度，SlewLimiter 可以不再使用，或者把加速度上限设置得略宽松。限制过严会让实际速度跟不上规划速度，改变原来的停车位置和轨迹。

---

## 10. API 与错误状态

### 10.1 初始化解算器

```c
ChassisMecanum_Status_t ChassisMecanum_Init(
    ChassisMecanum_t *mecanum,
    const ChassisMecanum_Config_t *config);
```

初始化会检查所有浮点配置是否有限，检查轮径、减速比和最大 rpm 是否为正，并确认每个方向项只能为 `+1` 或 `-1`。配置通过后会复制到解算器实例中。

### 10.2 逆解

```c
ChassisMecanum_Status_t ChassisMecanum_Inverse(
    const ChassisMecanum_t *mecanum,
    const ChassisMecanum_BodyVelocity_t *body_velocity,
    ChassisMecanum_MotorCommand_t *motor_command);
```

调用成功后，`motor_command->motor_rpm[]` 保存四个目标转速，`motor_command->scale` 保存限幅比例。

### 10.3 正解

```c
ChassisMecanum_Status_t ChassisMecanum_Forward(
    const ChassisMecanum_t *mecanum,
    const float motor_rpm[CHASSIS_MECANUM_WHEEL_COUNT],
    ChassisMecanum_BodyVelocity_t *body_velocity);
```

输入为四个电机轴反馈 rpm，输出为估算车体速度。

### 10.4 状态码

| 状态 | 含义 | 调用者建议 |
| --- | --- | --- |
| `CHASSIS_MECANUM_STATUS_OK` | 计算成功 | 可以使用输出。 |
| `CHASSIS_MECANUM_STATUS_NULL_POINTER` | 传入空指针 | 不发送旧电机命令，检查调用参数。 |
| `CHASSIS_MECANUM_STATUS_INVALID_CONFIG` | 机械参数或加速度配置非法 | 保持电机停机，检查单位和正负号。 |
| `CHASSIS_MECANUM_STATUS_NOT_INITIALIZED` | 尚未成功初始化 | 先调用相应 Init 函数。 |
| `CHASSIS_MECANUM_STATUS_INVALID_INPUT` | 输入含 NaN、无穷大或非法 dt | 丢弃本次命令并记录错误。 |

逆解出错时不应继续发送旧缓存中的非零命令。安全策略应由底盘任务和电机驱动层共同实现。

---

## 11. 推荐调用流程

### 11.1 系统初始化阶段

```text
HAL 与时钟初始化
        ↓
GPIO、FDCAN 外设初始化
        ↓
ChassisMecanum_Init()
        ↓
可选：ChassisMecanum_SlewLimiterInit()
        ↓
初始化电机协议、FDCAN 过滤器和接收通知（后续驱动层实现）
        ↓
创建底盘周期任务
```

下面示例只使用当前源码已经存在的麦轮接口。机械数据是演示值，实车必须替换。

```c
#include "chassis_mecanum.h"

static ChassisMecanum_t g_mecanum;
static ChassisMecanum_SlewLimiter_t g_limiter;

static ChassisMecanum_Status_t Chassis_AlgorithmInit(void)
{
    const ChassisMecanum_Config_t config =
    {
        .wheel_radius_m = 0.076f,       /* 示例：有效半径 76 mm */
        .half_wheelbase_m = 0.200f,     /* 示例：半轴距 200 mm */
        .half_track_m = 0.180f,         /* 示例：半轮距 180 mm */
        .gear_ratio = 10.0f,            /* 示例：10:1 减速箱 */
        .max_motor_rpm = 3000.0f,       /* 示例值，需查电机手册 */
        .motor_direction = {1, 1, 1, 1} /* 架空测试后再确定 */
    };
    ChassisMecanum_Status_t status;

    status = ChassisMecanum_Init(&g_mecanum, &config);
    if (status != CHASSIS_MECANUM_STATUS_OK)
    {
        return status;
    }

    status = ChassisMecanum_SlewLimiterInit(
        &g_limiter,
        1.0f, /* X 最大加速度，m/s^2 */
        1.0f, /* Y 最大加速度，m/s^2 */
        2.0f  /* Wz 最大角加速度，rad/s^2 */
    );

    return status;
}
```

### 11.2 固定周期控制

推荐把以下流程放在 1 ms、2 ms 或 5 ms 的固定周期任务中：

```c
static ChassisMecanum_Status_t Chassis_CalculateTargets(
    const ChassisMecanum_BodyVelocity_t *target,
    float delta_time_s,
    ChassisMecanum_MotorCommand_t *command)
{
    ChassisMecanum_BodyVelocity_t limited;
    ChassisMecanum_Status_t status;

    status = ChassisMecanum_SlewLimiterStep(
        &g_limiter,
        target,
        delta_time_s,
        &limited
    );
    if (status != CHASSIS_MECANUM_STATUS_OK)
    {
        return status;
    }

    return ChassisMecanum_Inverse(&g_mecanum, &limited, command);
}
```

任务中的逻辑边界应当是：

```c
ChassisMecanum_BodyVelocity_t target =
{
    .vx_mps = 0.5f,
    .vy_mps = 0.0f,
    .wz_radps = 0.0f
};
ChassisMecanum_MotorCommand_t command;
ChassisMecanum_Status_t status;

status = Chassis_CalculateTargets(&target, 0.005f, &command);
if (status == CHASSIS_MECANUM_STATUS_OK)
{
    /*
     * command.motor_rpm[FL/FR/RL/RR] 已经可以交给电机驱动层。
     * 当前工程尚无电机协议发送 API，因此这里不能直接调用不存在的函数。
     * 后续应由电机驱动模块完成 rpm 编码、CAN ID 选择和 FDCAN 发送。
     */
}
else
{
    /*
     * 进入安全处理：要求电机驱动层发送零目标或执行失能。
     * 不要继续发送上一个周期遗留的非零 command。
     */
}
```

### 11.3 FDCAN 发送对接边界

电机驱动模块至少需要明确以下契约：

| 契约 | 必须统一的内容 |
| --- | --- |
| 轮序映射 | FL、FR、RL、RR 分别对应哪个电机节点和 CAN ID。 |
| 控制模式 | 驱动器接收速度、转矩还是位置命令。麦轮输出当前是目标 rpm。 |
| 单位与缩放 | 报文中 `1` 代表多少 rpm，是否需要整数化和大小端转换。 |
| 正方向 | 驱动器反馈正方向必须与 `motor_direction[]` 的标定一致。 |
| 发送周期 | 电机协议允许的命令频率和超时保护时间。 |
| 饱和策略 | 驱动层不能再次独立改变四轮比例；若必须限幅，应反馈给底盘层统一处理。 |
| 故障行为 | 总线关闭、节点掉线、反馈超时或解算失败时如何停机。 |

不应把 CAN ID、字节打包和 HAL 发送代码写进 `chassis_mecanum.c`。这样运动学模块才能保持可测试、可复用，也能适配不同品牌电机。

### 11.4 接收反馈并调用正解

当电机驱动层完成四个节点的反馈解析后，应整理成固定轮序的电机轴 rpm 数组：

```c
float feedback_motor_rpm[CHASSIS_MECANUM_WHEEL_COUNT];
ChassisMecanum_BodyVelocity_t estimated_velocity;
ChassisMecanum_Status_t status;

/*
 * 反馈解析层负责更新：
 * feedback_motor_rpm[FL]
 * feedback_motor_rpm[FR]
 * feedback_motor_rpm[RL]
 * feedback_motor_rpm[RR]
 */

status = ChassisMecanum_Forward(
    &g_mecanum,
    feedback_motor_rpm,
    &estimated_velocity
);

if (status == CHASSIS_MECANUM_STATUS_OK)
{
    /*
     * estimated_velocity.vx_mps
     * estimated_velocity.vy_mps
     * estimated_velocity.wz_radps
     * 可提供给里程计、状态监控或后续传感器融合模块。
     */
}
```

只有四个反馈都有效且时间接近时才适合做正解。如果某个电机反馈超时，应先标记数据无效，不要把旧值与三个新值混合计算。

---

## 12. 实车调试步骤

### 12.1 调试前安全检查

1. 准备急停开关或能够立即切断电机动力电源的方法。
2. 第一次运行时把底盘可靠架空。
3. 将最大目标转速和加速度设置得很低。
4. 确认电机驱动器有通信超时停机保护。
5. 确认轮子不会卷入电线、衣物和工具。
6. 调试人员不要站在机器人预期运动方向上。

### 12.2 架空方向测试

依次测试以下六组目标：

```text
前进：Vx > 0, Vy = 0, Wz = 0
后退：Vx < 0, Vy = 0, Wz = 0
左移：Vx = 0, Vy > 0, Wz = 0
右移：Vx = 0, Vy < 0, Wz = 0
逆时针：Vx = 0, Vy = 0, Wz > 0
顺时针：Vx = 0, Vy = 0, Wz < 0
```

先记录逆解输出的数学符号，再观察乘过 `motor_direction[]` 后电机的实际方向。纯前进不正确时优先调整 `motor_direction[]`；前进正常但横移模式不符合符号表时，应重新检查轮子位置和 X/O 安装方式。

### 12.3 低速前进测试

1. 设定低速纯前进命令。
2. 检查机器人是否直行。
3. 记录四个电机目标值和反馈值。
4. 若持续向一侧偏，检查轮径、摩擦、机械阻力和速度闭环，不要立刻修改运动学符号。
5. 测量实际距离，用于标定有效轮径。

### 12.4 横移测试

横移是麦轮最容易打滑的工况：

1. 先使用低速度和低加速度。
2. 检查 FL/RR 与 FR/RL 是否形成两组相反方向。
3. 检查滚子是否自由转动。
4. 比较左右横移是否对称。
5. 地面材质变化会显著影响横移效果，应在比赛场地相近材料上调试。

### 12.5 旋转测试

1. 给低速纯旋转命令。
2. 检查机器人是否绕几何中心附近旋转。
3. 若旋转同时伴随明显平移，检查轮序、电机反馈对应关系和轮子接地情况。
4. 记录正解角速度，并与 IMU 或外部测角结果比较。

### 12.6 组合运动测试

基础运动全部正确后，再逐步测试：

```text
前进 + 左移
前进 + 旋转
横移 + 旋转
前进 + 横移 + 旋转
```

观察 `command.scale`。如果组合运动经常触发限幅，应降低上层速度目标，而不是提高超过电机能力的 `max_motor_rpm`。

---

## 13. 机械参数标定建议

### 13.1 轮径标定

1. 低速直行较长距离，例如 3 m 到 5 m。
2. 用电机反馈和正解累计估算距离 `D_est`。
3. 测量真实距离 `D_actual`。
4. 使用 `r_new = r_old × D_actual / D_est` 更新轮径。
5. 正反方向各测多次并取平均。

### 13.2 旋转力臂标定

1. 低速原地旋转若干圈。
2. 使用正解积分得到估算角度 `theta_est`。
3. 使用 IMU 或外部标记测得真实角度 `theta_actual`。
4. 可用下式修正等效力臂初值：

```text
L_new = L_old × theta_est / theta_actual
```

该值是包含麦轮侧滑影响的“等效力臂”，可能与尺子测得的几何值不同。应记录标定条件，不要用一次高加速度测试直接覆盖机械测量值。

### 13.3 最大转速和加速度

1. `max_motor_rpm` 先依据手册设置保守值。
2. 架空验证反馈转速与目标转速的单位一致。
3. 落地后逐步提高最大速度。
4. 观察电流、温度、打滑和母线电压波动。
5. 再分别提高 X、Y、Wz 加速度上限。

横移加速度通常需要比前后加速度更保守，因为麦轮横移更容易滑动。

---

## 14. 常见问题排查

| 现象 | 优先检查 | 原因说明 |
| --- | --- | --- |
| 初始化返回 `INVALID_CONFIG` | 轮径、减速比、最大 rpm、方向数组 | 参数必须有限，正值参数不能为零，方向只能是 `+1/-1`。 |
| 四轮都不动 | 解算返回值、电机使能、FDCAN 是否启动 | 运动学只计算 rpm，不会自动发送或使能电机。 |
| 前进命令变成后退 | 四个 `motor_direction[]` | 四轮整体方向定义与驱动器相反。 |
| 前进时原地旋转 | FL/FR/RL/RR 映射或部分方向错误 | 某些电机节点与数组下标不对应。 |
| 前进正常，横移方向相反 | X/O 安装方式、Y 坐标定义 | 不能仅凭前进判断麦轮公式是否匹配。 |
| 横移时同时旋转 | 轮序、轮径差、滚子卡滞、反馈闭环 | 四轮实际速度比例没有达到逆解要求。 |
| 旋转时中心漂移 | 负载不均、轮子接地、尺寸误差 | 理想模型要求四轮受力和几何关系对称。 |
| 目标 rpm 异常大 | 米/毫米单位、减速比方向、轮径 | 把毫米直接填入米制参数或减速比写反很常见。 |
| `scale` 经常很小 | 上层速度要求过高 | 逆解频繁超过电机最大转速。 |
| 正解速度方向错误 | 反馈轮序和反馈正方向 | 正解同样依赖 `motor_direction[]` 和固定轮序。 |
| 正解距离与实测不符 | 有效轮径和打滑 | 编码器只能测轮子转动，不能直接测车体真实移动。 |
| 加速很猛 | SlewLimiter 未使用、dt 错误 | `delta_time_s` 必须使用秒，且限制器需周期调用。 |
| 速度始终跟不上目标 | 加速度上限过小或重复限速 | 七段 S 曲线和 SlewLimiter 可能同时限制速度。 |
| 偶发继续发送旧命令 | 未处理解算错误状态 | 解算失败时应进入零目标或失能策略。 |

---

## 15. 团队对接清单

麦轮算法与电机驱动联调前，至少确认以下内容：

- [ ] 四轮机械位置已经固定为 FL、FR、RL、RR。
- [ ] 麦轮滚子确认为 X 形安装。
- [ ] 所有长度统一使用米。
- [ ] `gear_ratio` 使用“电机转数/车轮转数”。
- [ ] `max_motor_rpm` 是电机轴 rpm，而不是车轮 rpm。
- [ ] `motor_direction[]` 已通过架空测试确认。
- [ ] 电机驱动层明确四个节点与轮序的映射。
- [ ] FDCAN 协议明确 rpm 的缩放、符号、大小端和控制模式。
- [ ] 四个反馈值使用相同单位并带有超时有效性判断。
- [ ] 底盘任务检查所有运动学函数返回值。
- [ ] 解算失败和通信超时时有明确的停机策略。
- [ ] 控制周期和 `delta_time_s` 一致。
- [ ] 基础六方向测试通过后才开始组合运动。
- [ ] 参数修改有版本记录，能够回退到已验证配置。

---

## 16. 总结

当前麦轮模块的核心边界十分明确：上层提供车体速度，运动学层输出四个电机 rpm，电机驱动层负责 FDCAN 协议；反馈方向则由电机驱动层提供四个实际 rpm，运动学层估算车体速度。

保证系统正确的关键不是只记住四条公式，而是始终统一以下四件事：坐标系、轮子顺序、物理单位和电机正方向。完成这些基础约定后，再通过等比例限幅、速度斜坡、反馈正解和传感器融合逐步提高底盘的安全性与控制精度。
