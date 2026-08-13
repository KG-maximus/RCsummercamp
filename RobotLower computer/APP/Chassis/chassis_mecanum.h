#ifndef CHASSIS_MECANUM_H
#define CHASSIS_MECANUM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * ============================ 使用前先读 ============================
 *
 * 这个模块只负责“运动学换算”，不直接控制电机，也不发送 CAN 报文：
 *
 *   车体目标速度(Vx, Vy, Wz)
 *              |
 *              v
 *   可选：SlewLimiter 限制速度突变
 *              |
 *              v
 *   ChassisMecanum_Inverse() 麦轮逆解
 *              |
 *              v
 *   四个 motor_rpm[FL, FR, RL, RR]
 *              |
 *              v
 *   电机驱动层封装并通过 FDCAN 发送
 *
 * 电机反馈方向则相反：
 *
 *   FDCAN 收到四个电机实际 rpm
 *              |
 *              v
 *   ChassisMecanum_Forward() 麦轮正解
 *              |
 *              v
 *   估算底盘实际 Vx、Vy、Wz
 *
 * 数学模型的机械前提：
 * 1. 四个麦轮尺寸相同，滚子角度为常见的 45 度；
 * 2. 四轮组成矩形，车体参考点位于矩形中心；
 * 3. 滚子按 X 形安装，轮子与地面接触良好；
 * 4. 本模型不补偿轮胎打滑、地面不平、轮径误差和机械变形。
 *
 * 因此，公式给出的是理想运动关系。需要高精度定位时，还应融合
 * 编码器、IMU 或视觉反馈，并通过实车标定修正有效轮径和轮距。
 * ==================================================================
 */

/*
 * 四轮 X 形麦克纳姆轮底盘解算模块。
 *
 * 统一使用车体坐标系：
 *   +X：车头方向（向前）
 *   +Y：车体左侧
 *   +Wz：从车顶向下看逆时针旋转
 *
 * 轮子编号（从车顶向下看）：
 *
 *             车头 +X
 *                ^
 *       FL 左前轮 | 右前轮 FR
 *                 |
 *       RL 左后轮 | 右后轮 RR
 *                +------> 车体右侧为 -Y
 *
 * 本模块按 X 形滚子安装方式建立公式。motor_direction[] 用来处理
 * 电机安装朝向、减速箱输出方向或 CAN 驱动器正方向不一致的问题。
 */
#define CHASSIS_MECANUM_WHEEL_COUNT  (4U)

/* 数组下标固定为 FL、FR、RL、RR；与电机驱动对接时必须保持此顺序。 */
typedef enum
{
    CHASSIS_MECANUM_WHEEL_FRONT_LEFT = 0,
    CHASSIS_MECANUM_WHEEL_FRONT_RIGHT,
    CHASSIS_MECANUM_WHEEL_REAR_LEFT,
    CHASSIS_MECANUM_WHEEL_REAR_RIGHT
} ChassisMecanum_Wheel_t;

/* 所有公开函数的返回状态，调用者可据此判断参数或初始化是否正确。 */
typedef enum
{
    CHASSIS_MECANUM_STATUS_OK = 0,          /* 计算成功。 */
    CHASSIS_MECANUM_STATUS_NULL_POINTER,    /* 传入了空指针。 */
    CHASSIS_MECANUM_STATUS_INVALID_CONFIG,  /* 轮径、尺寸或限值配置非法。 */
    CHASSIS_MECANUM_STATUS_NOT_INITIALIZED, /* 尚未成功调用初始化函数。 */
    CHASSIS_MECANUM_STATUS_INVALID_INPUT    /* 输入含 NaN、无穷大或非法时间。 */
} ChassisMecanum_Status_t;

/* 底盘机械参数。长度统一用米，速度统一用 rpm 或 SI 单位。 */
typedef struct
{
    float wheel_radius_m;       /* 麦轮承载后的有效半径，不是空载标称直径的一半。 */
    float half_wheelbase_m;     /* 底盘中心到前/后轮轴线的距离，即轴距的一半。 */
    float half_track_m;         /* 底盘中心到左/右轮中心的距离，即轮距的一半。 */
    float gear_ratio;           /* 电机转数/车轮转数；直驱填写 1.0f。 */
    float max_motor_rpm;        /* 电机轴允许的最大转速，解算结果会按比例限幅。 */
    int8_t motor_direction[CHASSIS_MECANUM_WHEEL_COUNT]; /* 每项只能为 +1 或 -1。 */
} ChassisMecanum_Config_t;

/*
 * motor_direction[] 的确定方法：
 * 1. 先把四项都填为 +1；
 * 2. 架空底盘，只发送很小的“纯前进”命令；
 * 3. 按机器人实际前进所需方向检查每个轮子；
 * 4. 方向相反的轮子改为 -1。
 *
 * 该数组修正的是“电机正转定义”，不会改变 X 形麦轮公式本身。
 * 千万不要通过交换 FL/FR/RL/RR 数组位置来修正某个电机的方向。
 */

/* 希望底盘实现的车体速度，三个分量可以同时存在。 */
typedef struct
{
    float vx_mps;   /* 前后速度，m/s；正值前进，负值后退。 */
    float vy_mps;   /* 横移速度，m/s；正值左移，负值右移。 */
    float wz_radps; /* 自转角速度，rad/s；正值逆时针，负值顺时针。 */
} ChassisMecanum_BodyVelocity_t;

/* 逆解结果，可直接作为四个电机速度闭环的目标值。 */
typedef struct
{
    float motor_rpm[CHASSIS_MECANUM_WHEEL_COUNT]; /* 顺序固定为 FL、FR、RL、RR。 */
    float scale; /* 未超速为 1；超速时四轮共同乘以该比例，保证运动方向不变。 */
} ChassisMecanum_MotorCommand_t;

/* 解算器实例。应用层只需保存它，不应直接修改 initialized。 */
typedef struct
{
    ChassisMecanum_Config_t config;
    uint8_t initialized;
} ChassisMecanum_t;

/*
 * 一阶速度斜坡限制器：限制每个控制周期内速度指令能变化多少。
 * 它用于抑制突然给满速度造成的打滑和电流冲击，不等同于完整的
 * 七段 S 曲线位置规划器；后者由 APP/Common 的 StrategyAlgorithm 模块负责。
 */
typedef struct
{
    ChassisMecanum_BodyVelocity_t current;
    float max_vx_accel_mps2;
    float max_vy_accel_mps2;
    float max_wz_accel_radps2;
    uint8_t initialized;
} ChassisMecanum_SlewLimiter_t;

/*
 * 推荐的周期调用顺序（示意，不是需要原样复制的业务代码）：
 *
 *   target = 上位机或机器人状态机给出的速度;
 *   ChassisMecanum_SlewLimiterStep(&limiter, &target, dt, &limited);
 *   ChassisMecanum_Inverse(&mecanum, &limited, &command);
 *   Motor_SendRpm(command.motor_rpm);  // 由 FDCAN 驱动层实现
 *
 * 建议把该流程放在固定周期任务中，例如 1 ms、2 ms 或 5 ms。
 * dt 必须使用任务的实际周期，并统一换算成秒。
 */

/* 检查并保存机械参数；任何逆解或正解之前必须成功调用一次。 */
ChassisMecanum_Status_t ChassisMecanum_Init(
    ChassisMecanum_t *mecanum,
    const ChassisMecanum_Config_t *config);

/*
 * 逆运动学：把“底盘想怎样运动”转换成四个电机目标转速。
 * 典型调用链：上位机/状态机速度指令 -> 本函数 -> FDCAN 电机驱动发送。
 */
ChassisMecanum_Status_t ChassisMecanum_Inverse(
    const ChassisMecanum_t *mecanum,
    const ChassisMecanum_BodyVelocity_t *body_velocity,
    ChassisMecanum_MotorCommand_t *motor_command);

/*
 * 正运动学：用四个电机的实际反馈转速估算底盘当前速度。
 * 可用于里程计、故障诊断和速度闭环；输入顺序仍为 FL、FR、RL、RR。
 */
ChassisMecanum_Status_t ChassisMecanum_Forward(
    const ChassisMecanum_t *mecanum,
    const float motor_rpm[CHASSIS_MECANUM_WHEEL_COUNT],
    ChassisMecanum_BodyVelocity_t *body_velocity);

/* 初始化斜坡限制器，并设置 X、Y、旋转三个方向的最大加速度。 */
ChassisMecanum_Status_t ChassisMecanum_SlewLimiterInit(
    ChassisMecanum_SlewLimiter_t *limiter,
    float max_vx_accel_mps2,
    float max_vy_accel_mps2,
    float max_wz_accel_radps2);

/*
 * 按实际控制周期推进一次限速。delta_time_s 必须使用秒，例如 1 ms
 * 周期传入 0.001f；limited 是本周期允许送进逆解函数的速度。
 */
ChassisMecanum_Status_t ChassisMecanum_SlewLimiterStep(
    ChassisMecanum_SlewLimiter_t *limiter,
    const ChassisMecanum_BodyVelocity_t *target,
    float delta_time_s,
    ChassisMecanum_BodyVelocity_t *limited);

/* 将限制器当前值重置为指定速度；velocity 为 NULL 时重置为静止。 */
void ChassisMecanum_SlewLimiterReset(
    ChassisMecanum_SlewLimiter_t *limiter,
    const ChassisMecanum_BodyVelocity_t *velocity);

#ifdef __cplusplus
}
#endif

#endif
