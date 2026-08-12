#include "chassis_mecanum.h"

#include <float.h>
#include <stddef.h>

/*
 * =========================== 算法总览 ===========================
 *
 * 麦克纳姆轮底盘有三个运动自由度：
 *   Vx：前后平移；Vy：左右平移；Wz：绕底盘中心自转。
 *
 * 每个轮子的目标速度，是这三个运动分量在该轮上的线性叠加：
 *
 *   轮速 = 前后贡献 + 横移贡献 + 自转贡献
 *
 * “逆运动学”解决的是：已知 Vx/Vy/Wz，四个轮子分别该转多快。
 * “正运动学”解决的是：已知四个轮子的反馈转速，底盘实际怎样运动。
 *
 * 本文件把物理量分成三层，阅读时不要混淆：
 *   1. 车体速度：m/s、rad/s；
 *   2. 车轮角速度：rad/s；
 *   3. 电机轴速度：rpm，包含减速比和安装方向修正。
 *
 * 代码不依赖 HAL 和 FreeRTOS，因而可以单元测试，也能被不同电机
 * 驱动复用。FDCAN 标识符、控制模式和报文格式应留在电机驱动层。
 * ================================================================
 */

/* 车轮角速度 rad/s 与电机转速 rpm 之间的单位换算系数。 */
#define CHASSIS_MECANUM_PI               (3.14159265358979323846f)
#define CHASSIS_MECANUM_RADPS_TO_RPM     (60.0f / (2.0f * CHASSIS_MECANUM_PI))
#define CHASSIS_MECANUM_RPM_TO_RADPS     ((2.0f * CHASSIS_MECANUM_PI) / 60.0f)

static float ChassisMecanum_Abs(float value)
{
    /* 只需要简单绝对值，避免为此引入额外数学库依赖。 */
    return (value < 0.0f) ? -value : value;
}

static float ChassisMecanum_ClampDelta(float current, float target, float max_delta)
{
    const float delta = target - current;

    /* 每次只允许向目标靠近 max_delta，避免速度指令发生阶跃。 */
    if (delta > max_delta)
    {
        return current + max_delta;
    }
    if (delta < -max_delta)
    {
        return current - max_delta;
    }
    return target;
}

static uint8_t ChassisMecanum_IsFinite(float value)
{
    /* value == value 可排除 NaN；FLT_MAX 检查可排除正负无穷大。 */
    return (uint8_t)((value == value) &&
                     (value <= FLT_MAX) &&
                     (value >= -FLT_MAX));
}

static uint8_t ChassisMecanum_IsBodyVelocityValid(
    const ChassisMecanum_BodyVelocity_t *velocity)
{
    /* 三个速度分量只要有一个异常，本次命令整体拒绝。 */
    return (uint8_t)(ChassisMecanum_IsFinite(velocity->vx_mps) &&
                     ChassisMecanum_IsFinite(velocity->vy_mps) &&
                     ChassisMecanum_IsFinite(velocity->wz_radps));
}

static uint8_t ChassisMecanum_IsConfigValid(
    const ChassisMecanum_Config_t *config)
{
    uint32_t index;

    /* 先排除 NaN/无穷大，防止异常数值继续进入浮点运算。 */
    if ((!ChassisMecanum_IsFinite(config->wheel_radius_m)) ||
        (!ChassisMecanum_IsFinite(config->half_wheelbase_m)) ||
        (!ChassisMecanum_IsFinite(config->half_track_m)) ||
        (!ChassisMecanum_IsFinite(config->gear_ratio)) ||
        (!ChassisMecanum_IsFinite(config->max_motor_rpm)))
    {
        return 0U;
    }

    /* 轮径、减速比、转速上限必须为正，旋转力臂也不能为零。 */
    if ((config->wheel_radius_m <= 0.0f) ||
        (config->half_wheelbase_m < 0.0f) ||
        (config->half_track_m < 0.0f) ||
        ((config->half_wheelbase_m + config->half_track_m) <= 0.0f) ||
        (config->gear_ratio <= 0.0f) ||
        (config->max_motor_rpm <= 0.0f))
    {
        return 0U;
    }

    /* 电机方向只允许 +1 或 -1，不能用 0“临时禁用”某个轮子。 */
    for (index = 0U; index < CHASSIS_MECANUM_WHEEL_COUNT; ++index)
    {
        if ((config->motor_direction[index] != 1) &&
            (config->motor_direction[index] != -1))
        {
            return 0U;
        }
    }

    return 1U;
}

ChassisMecanum_Status_t ChassisMecanum_Init(
    ChassisMecanum_t *mecanum,
    const ChassisMecanum_Config_t *config)
{
    /* 第一步：检查地址，避免在参数错误时访问非法内存。 */
    if ((mecanum == NULL) || (config == NULL))
    {
        return CHASSIS_MECANUM_STATUS_NULL_POINTER;
    }
    /* 第二步：检查物理参数，避免除零或输出没有意义的轮速。 */
    if (!ChassisMecanum_IsConfigValid(config))
    {
        mecanum->initialized = 0U;
        return CHASSIS_MECANUM_STATUS_INVALID_CONFIG;
    }

    /* 复制一份配置，调用者后续修改原变量不会偷偷改变解算参数。 */
    mecanum->config = *config;
    mecanum->initialized = 1U;
    return CHASSIS_MECANUM_STATUS_OK;
}

ChassisMecanum_Status_t ChassisMecanum_Inverse(
    const ChassisMecanum_t *mecanum,
    const ChassisMecanum_BodyVelocity_t *body_velocity,
    ChassisMecanum_MotorCommand_t *motor_command)
{
    /* wheel_radps 是公式直接算出的车轮角速度，尚未考虑电机减速比。 */
    float wheel_radps[CHASSIS_MECANUM_WHEEL_COUNT];
    /* max_abs_rpm 用来找出四轮中负担最重、绝对转速最大的那个电机。 */
    float max_abs_rpm = 0.0f;
    /* scale 最终会告诉调用者本次指令是否因为电机上限被整体压缩。 */
    float scale = 1.0f;
    /* rotation_arm_m 即公式中的 L，是底盘自转时的等效几何力臂。 */
    float rotation_arm_m;
    float motor_rpm;
    uint32_t index;

    /*
     * 逆解的错误检查顺序：指针 -> 是否初始化 -> 输入数字是否合法。
     * 出错时不生成新的 motor_command，调用者不应发送旧缓存命令。
     */
    if ((mecanum == NULL) || (body_velocity == NULL) || (motor_command == NULL))
    {
        return CHASSIS_MECANUM_STATUS_NULL_POINTER;
    }
    if (mecanum->initialized == 0U)
    {
        return CHASSIS_MECANUM_STATUS_NOT_INITIALIZED;
    }
    if (!ChassisMecanum_IsBodyVelocityValid(body_velocity))
    {
        return CHASSIS_MECANUM_STATUS_INVALID_INPUT;
    }

    /*
     * 麦轮自转项的等效力臂 L = 半轴距 + 半轮距。
     * 底盘越长或越宽，同样角速度所需的轮缘线速度就越大。
     */
    rotation_arm_m = mecanum->config.half_wheelbase_m +
                     mecanum->config.half_track_m;

    /*
     * X 形麦轮逆运动学，结果是每个“车轮”的角速度 rad/s：
     *   FL = (Vx - Vy - L*Wz) / r
     *   FR = (Vx + Vy + L*Wz) / r
     *   RL = (Vx + Vy - L*Wz) / r
     *   RR = (Vx - Vy + L*Wz) / r
     *
     * 公式中的三个部分可以拆开理解：
     *
     * 1. Vx 对四轮的贡献全部同号，所以四轮一起推动车体前后移动；
     * 2. Vy 在左右前后轮上的符号交错，滚子的斜向反力合成为横移；
     * 3. L*Wz 在左右轮上方向相反，从而形成绕中心旋转的力矩；
     * 4. 最后除以轮半径 r，把轮缘线速度 m/s 换成角速度 rad/s。
     *
     * 例如只要求前进 1 m/s，则 Vy=0、Wz=0，四个车轮角速度
     * 都等于 1/r。若轮半径为 0.1 m，四轮就是 10 rad/s。
     * 注意：下面符号是“车轮数学正方向”，尚未乘 motor_direction。
     *
     * 用符号快速检查接线与方向：
     *   仅前进：四轮同号；
     *   仅左移：FL-/FR+/RL+/RR-；
     *   仅逆时针：FL-/FR+/RL-/RR+。
     */
    /* 左前轮 FL：横移项为负，自转项为负。 */
    wheel_radps[CHASSIS_MECANUM_WHEEL_FRONT_LEFT] =
        (body_velocity->vx_mps - body_velocity->vy_mps -
         rotation_arm_m * body_velocity->wz_radps) /
        mecanum->config.wheel_radius_m;
    /* 右前轮 FR：横移项为正，自转项为正。 */
    wheel_radps[CHASSIS_MECANUM_WHEEL_FRONT_RIGHT] =
        (body_velocity->vx_mps + body_velocity->vy_mps +
         rotation_arm_m * body_velocity->wz_radps) /
        mecanum->config.wheel_radius_m;
    /* 左后轮 RL：横移项为正，自转项为负。 */
    wheel_radps[CHASSIS_MECANUM_WHEEL_REAR_LEFT] =
        (body_velocity->vx_mps + body_velocity->vy_mps -
         rotation_arm_m * body_velocity->wz_radps) /
        mecanum->config.wheel_radius_m;
    /* 右后轮 RR：横移项为负，自转项为正。 */
    wheel_radps[CHASSIS_MECANUM_WHEEL_REAR_RIGHT] =
        (body_velocity->vx_mps - body_velocity->vy_mps +
         rotation_arm_m * body_velocity->wz_radps) /
        mecanum->config.wheel_radius_m;

    /*
     * 把车轮 rad/s 换成电机轴 rpm：
     * 电机转速 = 车轮角速度 * 减速比 * 单位换算 * 安装方向修正。
     *
     * 例如减速比为 10，车轮需要 60 rpm，则电机轴需要 600 rpm。
     * motor_direction 只决定最终发送给驱动器的正负号，不影响幅值。
     */
    for (index = 0U; index < CHASSIS_MECANUM_WHEEL_COUNT; ++index)
    {
        motor_rpm = wheel_radps[index] * mecanum->config.gear_ratio *
                    CHASSIS_MECANUM_RADPS_TO_RPM *
                    (float)mecanum->config.motor_direction[index];
        motor_command->motor_rpm[index] = motor_rpm;

        /* 正转和反转都受同一个转速上限约束，因此比较绝对值。 */
        if (ChassisMecanum_Abs(motor_rpm) > max_abs_rpm)
        {
            max_abs_rpm = ChassisMecanum_Abs(motor_rpm);
        }
    }

    /*
     * 任一电机超速时，四个目标转速一起等比例缩小。
     * 不能只截断超速的那个轮子，否则 Vx、Vy、Wz 的比例会被破坏，
     * 机器人实际运动方向就会偏离指令。
     *
     * 例：原目标为 [2000, 1000, 1000, 2000] rpm，而上限是 1500 rpm，
     * scale=1500/2000=0.75，输出变为 [1500, 750, 750, 1500] rpm。
     * 四轮比例保持不变，所以底盘方向不变，只是整体运动变慢。
     */
    if (max_abs_rpm > mecanum->config.max_motor_rpm)
    {
        scale = mecanum->config.max_motor_rpm / max_abs_rpm;
        for (index = 0U; index < CHASSIS_MECANUM_WHEEL_COUNT; ++index)
        {
            motor_command->motor_rpm[index] *= scale;
        }
    }

    /* scale 可用于上层判断“本次速度要求是否超过底盘能力”。 */
    motor_command->scale = scale;
    return CHASSIS_MECANUM_STATUS_OK;
}

ChassisMecanum_Status_t ChassisMecanum_Forward(
    const ChassisMecanum_t *mecanum,
    const float motor_rpm[CHASSIS_MECANUM_WHEEL_COUNT],
    ChassisMecanum_BodyVelocity_t *body_velocity)
{
    /* 这里的输入必须是驱动器反馈的电机轴 rpm，而不是车轮 rpm。 */
    float wheel_radps[CHASSIS_MECANUM_WHEEL_COUNT];
    float rotation_arm_m;
    uint32_t index;

    if ((mecanum == NULL) || (motor_rpm == NULL) || (body_velocity == NULL))
    {
        return CHASSIS_MECANUM_STATUS_NULL_POINTER;
    }
    if (mecanum->initialized == 0U)
    {
        return CHASSIS_MECANUM_STATUS_NOT_INITIALIZED;
    }

    /* 先撤销安装方向和减速比，再把电机 rpm 还原为车轮 rad/s。 */
    for (index = 0U; index < CHASSIS_MECANUM_WHEEL_COUNT; ++index)
    {
        if (!ChassisMecanum_IsFinite(motor_rpm[index]))
        {
            return CHASSIS_MECANUM_STATUS_INVALID_INPUT;
        }

        wheel_radps[index] = motor_rpm[index] *
                             (float)mecanum->config.motor_direction[index] *
                             CHASSIS_MECANUM_RPM_TO_RADPS /
                             mecanum->config.gear_ratio;
    }

    rotation_arm_m = mecanum->config.half_wheelbase_m +
                     mecanum->config.half_track_m;

    /*
     * 正运动学是上面逆解矩阵的反变换。
     * 四轮反馈共同估算 Vx、Vy、Wz，因此可把编码器/CAN 反馈直接传入。
     *
     * Vx：四个车轮角速度相加，再乘 r/4；
     * Vy：按 [-FL +FR +RL -RR] 组合，再乘 r/4；
     * Wz：按 [-FL +FR -RL +RR] 组合，再除以 4L。
     *
     * 正解得到的是“轮子按照理想无滑动模型推算出的速度”，不是绝对
     * 真实速度。急加速、侧向横移或地面较滑时，编码器有转速并不代表
     * 车体移动了同样距离，因此定位系统仍需 IMU/视觉等传感器校正。
     */
    /* 四轮同向分量相加，得到底盘前后速度 Vx。 */
    body_velocity->vx_mps = mecanum->config.wheel_radius_m * 0.25f *
        (wheel_radps[CHASSIS_MECANUM_WHEEL_FRONT_LEFT] +
         wheel_radps[CHASSIS_MECANUM_WHEEL_FRONT_RIGHT] +
         wheel_radps[CHASSIS_MECANUM_WHEEL_REAR_LEFT] +
         wheel_radps[CHASSIS_MECANUM_WHEEL_REAR_RIGHT]);
    /* 交叉符号组合提取横移分量 Vy。 */
    body_velocity->vy_mps = mecanum->config.wheel_radius_m * 0.25f *
        (-wheel_radps[CHASSIS_MECANUM_WHEEL_FRONT_LEFT] +
          wheel_radps[CHASSIS_MECANUM_WHEEL_FRONT_RIGHT] +
          wheel_radps[CHASSIS_MECANUM_WHEEL_REAR_LEFT] -
          wheel_radps[CHASSIS_MECANUM_WHEEL_REAR_RIGHT]);
    /* 左右轮的差动分量除以旋转力臂，得到自转角速度 Wz。 */
    body_velocity->wz_radps = mecanum->config.wheel_radius_m *
        (-wheel_radps[CHASSIS_MECANUM_WHEEL_FRONT_LEFT] +
          wheel_radps[CHASSIS_MECANUM_WHEEL_FRONT_RIGHT] -
          wheel_radps[CHASSIS_MECANUM_WHEEL_REAR_LEFT] +
          wheel_radps[CHASSIS_MECANUM_WHEEL_REAR_RIGHT]) /
        (4.0f * rotation_arm_m);

    return CHASSIS_MECANUM_STATUS_OK;
}

ChassisMecanum_Status_t ChassisMecanum_SlewLimiterInit(
    ChassisMecanum_SlewLimiter_t *limiter,
    float max_vx_accel_mps2,
    float max_vy_accel_mps2,
    float max_wz_accel_radps2)
{
    /*
     * 三个加速度上限必须为正：
     *   max_vx_accel_mps2：每秒最多改变多少前后速度；
     *   max_vy_accel_mps2：每秒最多改变多少横移速度；
     *   max_wz_accel_radps2：每秒最多改变多少角速度。
     */
    if (limiter == NULL)
    {
        return CHASSIS_MECANUM_STATUS_NULL_POINTER;
    }
    if ((!ChassisMecanum_IsFinite(max_vx_accel_mps2)) ||
        (!ChassisMecanum_IsFinite(max_vy_accel_mps2)) ||
        (!ChassisMecanum_IsFinite(max_wz_accel_radps2)) ||
        (max_vx_accel_mps2 <= 0.0f) ||
        (max_vy_accel_mps2 <= 0.0f) ||
        (max_wz_accel_radps2 <= 0.0f))
    {
        return CHASSIS_MECANUM_STATUS_INVALID_CONFIG;
    }

    /* 初始状态按底盘静止处理。若机器人已在运动，可随后调用 Reset 对齐。 */
    limiter->current.vx_mps = 0.0f;
    limiter->current.vy_mps = 0.0f;
    limiter->current.wz_radps = 0.0f;
    limiter->max_vx_accel_mps2 = max_vx_accel_mps2;
    limiter->max_vy_accel_mps2 = max_vy_accel_mps2;
    limiter->max_wz_accel_radps2 = max_wz_accel_radps2;
    limiter->initialized = 1U;
    return CHASSIS_MECANUM_STATUS_OK;
}

ChassisMecanum_Status_t ChassisMecanum_SlewLimiterStep(
    ChassisMecanum_SlewLimiter_t *limiter,
    const ChassisMecanum_BodyVelocity_t *target,
    float delta_time_s,
    ChassisMecanum_BodyVelocity_t *limited)
{
    if ((limiter == NULL) || (target == NULL) || (limited == NULL))
    {
        return CHASSIS_MECANUM_STATUS_NULL_POINTER;
    }
    if (limiter->initialized == 0U)
    {
        return CHASSIS_MECANUM_STATUS_NOT_INITIALIZED;
    }
    if ((!ChassisMecanum_IsFinite(delta_time_s)) ||
        (delta_time_s <= 0.0f) ||
        (!ChassisMecanum_IsBodyVelocityValid(target)))
    {
        return CHASSIS_MECANUM_STATUS_INVALID_INPUT;
    }

    /*
     * 本周期最大允许变化量 = 最大加速度 * 周期时间。
     * X、Y、Wz 分别限制，控制周期有轻微抖动时也能按实际 dt 工作。
     *
     * 例：当前 Vx=0，目标 Vx=2 m/s，最大加速度=1 m/s^2，dt=0.01 s，
     * 本周期 Vx 最多增加 1*0.01=0.01 m/s，所以输出 0.01 m/s。
     * 连续调用后速度会形成直线斜坡，逐步接近 2 m/s。
     *
     * 三个轴独立限幅，优点是逻辑直接、容易调试；代价是组合运动时
     * 合加速度可能大于任意单轴上限。若以后需要严格限制合加速度，
     * 可在应用层增加矢量归一化，但不应偷偷改变本函数的接口语义。
     */
    limiter->current.vx_mps = ChassisMecanum_ClampDelta(
        limiter->current.vx_mps,
        target->vx_mps,
        limiter->max_vx_accel_mps2 * delta_time_s);
    limiter->current.vy_mps = ChassisMecanum_ClampDelta(
        limiter->current.vy_mps,
        target->vy_mps,
        limiter->max_vy_accel_mps2 * delta_time_s);
    limiter->current.wz_radps = ChassisMecanum_ClampDelta(
        limiter->current.wz_radps,
        target->wz_radps,
        limiter->max_wz_accel_radps2 * delta_time_s);

    /* 保存状态并返回同一份结果，下一周期将从该速度继续逼近目标。 */
    *limited = limiter->current;
    return CHASSIS_MECANUM_STATUS_OK;
}

void ChassisMecanum_SlewLimiterReset(
    ChassisMecanum_SlewLimiter_t *limiter,
    const ChassisMecanum_BodyVelocity_t *velocity)
{
    /*
     * Reset 不会修改三个加速度上限，只重设“当前速度记忆”。
     * 急停、模式切换、底盘重新使能时可传 NULL 清零；若要无突变地接管
     * 一个已经运动的底盘，应传入当前估算速度作为新的起点。
     */
    if (limiter == NULL)
    {
        return;
    }

    /* 空指针或非法速度都按安全的“静止”状态处理。 */
    if ((velocity == NULL) || (!ChassisMecanum_IsBodyVelocityValid(velocity)))
    {
        limiter->current.vx_mps = 0.0f;
        limiter->current.vy_mps = 0.0f;
        limiter->current.wz_radps = 0.0f;
        return;
    }

    limiter->current = *velocity;
}
