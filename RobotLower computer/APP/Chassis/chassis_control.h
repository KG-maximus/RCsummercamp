#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "StrategyAlogrithm.h" /* APP/Common：与具体底盘无关的一维 S 曲线算法。 */
#include "chassis_mecanum.h"
#include "robot_protocol.h"

/*
 * 故障位图。fault_flags 可以同时包含多个原因，使用按位与/或判断：
 *   if ((flags & CHASSIS_FAULT_FEEDBACK_TIMEOUT) != 0U) { ... }
 * 一旦进入 FAULT，ChassisControl_Step 会输出四路 0 rpm；清除故障前须先
 * 排除硬件原因。ESTOP 故障只能由 CHASSIS_SOURCE_SAFETY 来源清除。
 */
#define CHASSIS_FAULT_NONE              (0UL)
#define CHASSIS_FAULT_CONFIG            (1UL << 0)
#define CHASSIS_FAULT_COMMAND_TIMEOUT   (1UL << 1)
#define CHASSIS_FAULT_FEEDBACK_TIMEOUT  (1UL << 2)
#define CHASSIS_FAULT_ALGORITHM         (1UL << 3)
#define CHASSIS_FAULT_PORT_NOT_READY    (1UL << 4)
#define CHASSIS_FAULT_MOTOR_TX          (1UL << 5)
#define CHASSIS_FAULT_ESTOP             (1UL << 6)

typedef enum
{
    CHASSIS_CONTROL_STATE_UNINITIALIZED = 0, /* 尚未通过 ChassisControl_Init。 */
    CHASSIS_CONTROL_STATE_DISABLED,          /* 已禁用，不接受运动命令，输出零目标。 */
    CHASSIS_CONTROL_STATE_IDLE,              /* 已使能但无运动请求，保持零速度。 */
    CHASSIS_CONTROL_STATE_VELOCITY,          /* BODY_VELOCITY 模式，按斜坡跟踪速度。 */
    CHASSIS_CONTROL_STATE_TRAJECTORY,        /* 相对/绝对位姿模式，按 S 曲线运行。 */
    CHASSIS_CONTROL_STATE_STOPPING,          /* 命令超时或 STOP 后，受限减速停车。 */
    CHASSIS_CONTROL_STATE_FAULT              /* 故障锁存，持续输出零目标，等待清故障。 */
} ChassisControl_State_t;

/* 所有公开控制接口的执行结果，供上层记录或采取下一步安全动作。 */
typedef enum
{
    CHASSIS_CONTROL_OK = 0,
    CHASSIS_CONTROL_NULL_POINTER,
    CHASSIS_CONTROL_NOT_INITIALIZED,
    CHASSIS_CONTROL_INVALID_CONFIG,
    CHASSIS_CONTROL_INVALID_COMMAND,
    CHASSIS_CONTROL_COMMAND_REJECTED,
    CHASSIS_CONTROL_ALGORITHM_ERROR
} ChassisControl_Result_t;

typedef struct
{
    /* 单位取决于使用场景：平移为 m/s^2、m/s、m/s^3；偏航为 rad/s^2、rad/s、rad/s^3。 */
    float max_acceleration; /* 七段 S 曲线允许的最大加速度。 */
    float max_velocity;     /* 七段 S 曲线允许的最大速度。 */
    float max_jerk;         /* 加速度变化率上限，用于减小机械冲击。 */
} ChassisControl_PlannerLimit_t;

/*
 * 控制器初始化配置。由 chassis_task.c 的 ChassisTask_BuildConfig() 从
 * chassis_config.h 宏生成。各上限必须为正，否则初始化会失败并置配置故障。
 */
typedef struct
{
    ChassisMecanum_Config_t mecanum; /* 轮径、几何尺寸、减速比、方向与 rpm 上限。 */
    float max_vx_mps;                /* 车体 +X/-X 速度上限，m/s。 */
    float max_vy_mps;                /* 车体 +Y/-Y 速度上限，m/s。 */
    float max_wz_radps;              /* 车体偏航角速度上限，rad/s。 */
    float max_vx_accel_mps2;         /* 速度模式前后加速度上限，m/s^2。 */
    float max_vy_accel_mps2;         /* 速度模式横移加速度上限，m/s^2。 */
    float max_wz_accel_radps2;       /* 速度模式偏航加速度上限，rad/s^2。 */
    ChassisControl_PlannerLimit_t planner_translation; /* 直线路径的 S 曲线限制。 */
    ChassisControl_PlannerLimit_t planner_yaw;         /* 偏航目标的 S 曲线限制。 */
    uint32_t command_timeout_ms;     /* 命令/HEARTBEAT 失效时间，ms。 */
    uint32_t feedback_timeout_ms;    /* 电机反馈失效时间，ms。 */
    uint8_t require_motor_feedback;  /* 非零时，必须收到新鲜反馈才可运行。 */
} ChassisControl_Config_t;

/*
 * 可公开读取的控制器状态快照。任务层会使用 Mutex 复制它，外部调用者应只读
 * 快照，不要直接修改 ChassisControl_t 内部成员。
 * pose/target_pose 使用 ODOM 坐标系：x/y 单位 m，yaw 单位 rad。
 * 三组车体速度均使用 BODY 坐标系：+X 前、+Y 左、+Wz 逆时针。
 */
typedef struct
{
    ChassisControl_State_t state;   /* 当前控制状态机状态。 */
    uint32_t fault_flags;           /* 当前锁存的故障位图。 */
    ChassisCommandSource_t active_source; /* 最近一次执行命令的来源，仅用于状态回传。 */
    uint32_t last_sequence;         /* 最近一次执行命令的上游序号，仅用于状态回传。 */
    uint32_t last_command_ms;       /* 最近一次接收控制命令的 HAL tick，ms。 */
    uint32_t last_feedback_ms;      /* 最后一次有效电机反馈的 HAL tick，ms。 */
    RobotPose2D_t pose;             /* 由轮速正解积分得到的里程计位姿。 */
    RobotPose2D_t target_pose;      /* 当前轨迹模式的 ODOM 目标位姿。 */
    RobotBodyVelocity_t requested_body_velocity; /* 上层请求或轨迹产生的目标速度。 */
    RobotBodyVelocity_t commanded_body_velocity; /* 经斜坡/限幅后实际送入逆解的速度。 */
    RobotBodyVelocity_t actual_body_velocity;    /* 电机反馈正解估算的实际速度。 */
    float target_motor_rpm[CHASSIS_MECANUM_WHEEL_COUNT];   /* 发往四轮的目标 rpm。 */
    float feedback_motor_rpm[CHASSIS_MECANUM_WHEEL_COUNT]; /* 四轮反馈 rpm。 */
    float motor_scale;              /* 电机超速时的统一缩放系数；1 表示未缩放。 */
    uint8_t feedback_valid;         /* 至少成功收到并解析过一组电机反馈。 */
    uint8_t initialized;            /* 控制器是否已成功完成初始化。 */
} ChassisControl_Status_t;

/* ChassisControl_Step 的输出。send_motor_targets 非零时任务层才应发送数组。 */
typedef struct
{
    float motor_rpm[CHASSIS_MECANUM_WHEEL_COUNT];
    uint8_t send_motor_targets;
} ChassisControl_Output_t;

typedef struct
{
    /* 以下字段是控制器的工作内存，由本模块内部维护，应用层无需直接读写。 */
    ChassisControl_Config_t config;
    ChassisMecanum_t mecanum;
    ChassisMecanum_SlewLimiter_t slew_limiter;
    SpeedPlan_TypeDef planner_translation;
    SpeedPlan_TypeDef planner_yaw;
    RobotPose2D_t trajectory_start_pose;
    float trajectory_direction_x;
    float trajectory_direction_y;
    float trajectory_distance_m;
    RobotBodyVelocity_t velocity_target;
    ChassisControl_Status_t status;
} ChassisControl_t;

/*
 * 初始化控制器并校验全部配置。
 * 必须在首次 SubmitCommand/Step/UpdateMotorFeedback 前调用；now_ms 通常传
 * HAL_GetTick()。成功后控制器进入 DISABLED，仍需接收 ENABLE 命令才可运动。
 */
ChassisControl_Result_t ChassisControl_Init(
    ChassisControl_t *control,
    const ChassisControl_Config_t *config,
    uint32_t now_ms);

/*
 * 提交一条已完成单位换算的语义命令。该函数在 chassis_task 的控制上下文
 * 调用；其他任务应使用 ChassisTask_PostCommand() 投递，避免并发改状态。
 * now_ms 通常传 HAL_GetTick()；指令来源仲裁、序号去重和业务级有效期由上位机
 * 或机器人总状态机完成，本函数只做格式合法性、状态机和硬件安全检查。
 */
ChassisControl_Result_t ChassisControl_SubmitCommand(
    ChassisControl_t *control,
    const ChassisCommand_t *command,
    uint32_t now_ms);

/*
 * 将 Port 层读取到的 FL/FR/RL/RR 电机轴反馈 rpm 写入控制器，并做正运动学
 * 得到车体实际速度。它不直接从中断调用，FDCAN 回调只更新驱动反馈；任务
 * 在固定周期读取快照后调用本接口。
 */
ChassisControl_Result_t ChassisControl_UpdateMotorFeedback(
    ChassisControl_t *control,
    const float motor_rpm[CHASSIS_MECANUM_WHEEL_COUNT],
    uint32_t feedback_time_ms);

/*
 * 推进一次控制状态机。delta_time_s 必须为正、单位为秒；输出 motor_rpm 的
 * 顺序固定为 FL/FR/RL/RR。无论 DISABLED 或 FAULT，本函数都会生成零 rpm
 * 的安全输出，调用者仍应将其发往驱动层。
 */
ChassisControl_Result_t ChassisControl_Step(
    ChassisControl_t *control,
    uint32_t now_ms,
    float delta_time_s,
    ChassisControl_Output_t *output);

/*
 * 由 Port/任务层报告 CAN 发送失败、端口未就绪等控制器外部故障。传入的位图
 * 会被锁存并强制进入 FAULT；不能以 CHASSIS_FAULT_NONE 清除已有故障。
 */
void ChassisControl_SetExternalFault(
    ChassisControl_t *control,
    uint32_t fault_flags);

/* 复制状态快照到调用者提供的缓冲区；空指针时静默返回。 */
void ChassisControl_GetStatus(
    const ChassisControl_t *control,
    ChassisControl_Status_t *status);

#ifdef __cplusplus
}
#endif

#endif
