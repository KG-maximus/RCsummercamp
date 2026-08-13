#ifndef CHASSIS_H
#define CHASSIS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "M3508.h"
#include "StrategyAlogrithm.h"
#include "chassis_mecanum.h"
#include "robot_protocol.h"

/*
 * ============================ 底盘模块总览 ============================
 *
 * 业务代码只需要包含本文件。底盘内部的数据流按下面顺序运行：
 *
 *   上位机/机器人总状态机
 *          |  ChassisSet...() 或 ChassisPostCommand()
 *          v
 *   FreeRTOS 命令队列
 *          v
 *   chassis.c 固定周期任务：安全状态机 -> 速度规划 -> 麦轮逆解
 *          v
 *   APP/Common/M3508.c：速度环 + 电流环 -> C620 CAN 控制帧
 *          v
 *   FDCAN 回调：只解析角度/转速/电流/温度，不在中断中运行控制算法
 *          v
 *   ChassisState + ChassisGetState()：向上位机和总状态机反馈完整快照
 *
 * 坐标和轮序是整个模块的统一约定：
 *   +X = 向前，+Y = 向左，+Wz = 从车顶看逆时针；
 *   四轮数组始终为 FL（左前）、FR（右前）、RL（左后）、RR（右后）。
 *
 * 最常用的调用顺序：
 *   1. freertos.c 在启动调度器前调用 ChassisInit()；
 *   2. 创建线程并以 ChassisTask() 作为入口；
 *   3. 上层先调用 ChassisEnable()，再周期发送速度或位置目标；
 *   4. 上位机回传任务周期调用 ChassisGetState()；
 *   5. 失联或异常时调用 ChassisStop()/ChassisEmergencyStop()。
 * ======================================================================
 */

/* 故障是位图，可同时记录多个原因；FAULT 状态下始终发送零电流。 */
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
    CHASSIS_CONTROL_STATE_UNINITIALIZED = 0, /* RTOS、参数或驱动尚未完成初始化。 */
    CHASSIS_CONTROL_STATE_DISABLED,          /* 已初始化但禁止输出，目标为零。 */
    CHASSIS_CONTROL_STATE_IDLE,              /* 已使能且静止，等待运动命令。 */
    CHASSIS_CONTROL_STATE_VELOCITY,          /* 按 Vx/Vy/Wz 速度命令运动。 */
    CHASSIS_CONTROL_STATE_TRAJECTORY,        /* 按相对/绝对位姿目标运行 S 曲线。 */
    CHASSIS_CONTROL_STATE_STOPPING,          /* 正按加速度限制减速停车。 */
    CHASSIS_CONTROL_STATE_FAULT              /* 故障锁存，仅允许安全类恢复命令。 */
} ChassisControl_State_t;

/* 命令处理结果。入队成功不等于执行成功，最终结果可从 ChassisState 读取。 */
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

/*
 * 底盘所有可观察运行信息的总结构体。
 *
 * 全局对象 ChassisState 主要方便 Keil Watch 直接观察。控制任务是它的唯一
 * 写入者；其他任务和上位机通信代码不能直接修改或依赖一次无锁读取，应调用
 * ChassisGetState() 取得 Mutex 保护的一致快照。
 */
typedef struct
{
    ChassisControl_State_t state;       /* 当前底盘状态机。 */
    uint32_t fault_flags;               /* CHASSIS_FAULT_* 的组合。 */
    ChassisControl_Result_t last_command_result; /* 最近处理命令的最终结果。 */
    ChassisCommandSource_t active_source; /* 最近被执行命令的来源。 */
    uint32_t last_sequence;             /* 最近命令的上游序号，仅用于诊断。 */
    uint32_t last_command_ms;           /* 最近有效命令的本机 HAL tick。 */
    uint32_t last_feedback_ms;          /* 最近一组有效四轮反馈的 HAL tick。 */
    uint32_t update_count;              /* 成功发布状态快照的累计次数。 */

    RobotPose2D_t pose;                 /* 轮速里程计估算位姿，m/m/rad。 */
    RobotPose2D_t target_pose;          /* 当前轨迹目标位姿，m/m/rad。 */
    RobotBodyVelocity_t target_velocity;  /* 上层或规划器请求速度。 */
    RobotBodyVelocity_t command_velocity; /* 限速/斜坡后送入麦轮逆解的速度。 */
    RobotBodyVelocity_t feedback_velocity;/* 麦轮正解得到的实际车体速度。 */

    float target_motor_rpm[CHASSIS_MECANUM_WHEEL_COUNT];   /* FL/FR/RL/RR 目标电机轴 rpm。 */
    float feedback_motor_rpm[CHASSIS_MECANUM_WHEEL_COUNT]; /* FL/FR/RL/RR 反馈电机轴 rpm。 */
    M3508Feedback_TypeDef motor_feedback[CHASSIS_MECANUM_WHEEL_COUNT]; /* 原始角度、转速、电流、温度和累计帧数。 */
    uint32_t motor_feedback_ms[CHASSIS_MECANUM_WHEEL_COUNT]; /* FL/FR/RL/RR 最近反馈的 HAL tick，供逐轮超时诊断。 */
    float motor_scale;                 /* 轮速超限统一缩放系数，1 表示未缩放。 */

    uint8_t initialized;               /* ChassisInit 已创建 RTOS 对象和驱动。 */
    uint8_t configuration_ready;       /* chassis_config 参数通过校验。 */
    uint8_t port_ready;                /* 四台 C620 均有未超时的反馈。 */
    uint8_t rtos_objects_ready;        /* 命令队列与状态 Mutex 已创建。 */
    uint8_t feedback_valid;            /* 控制器至少处理过一组完整反馈。 */
    uint8_t motor_output_enabled;      /* 状态机允许 PID 输出；仍受 READY 宏保护。 */
} ChassisState_TypeDef;

extern ChassisState_TypeDef ChassisState;

/*
 * 创建底盘命令队列、状态 Mutex、M3508 组和 FDCAN 回调。必须在
 * osKernelInitialize() 之后、osKernelStart() 之前调用。本函数不会使能电机；
 * 即使 chassis_config 尚未填写，它也会保留只接收 CAN 反馈的安全调试能力。
 */
uint8_t ChassisInit(void);

/* FreeRTOS 固定周期任务入口，由 freertos.c 创建线程，业务代码不直接调用。 */
void ChassisTask(void *argument);

/*
 * 最通用的对上接口：把一条完整语义命令投递到队列。timeout_ms 是等待队列
 * 空位的最长时间；中断回调不可调用。返回 1U 仅表示成功入队，执行结果随后
 * 从 ChassisGetState()->last_command_result 获取。
 */
uint8_t ChassisPostCommand(const ChassisCommand_t *command, uint32_t timeout_ms);

/*
 * 以下便捷接口供上位机解析任务或 Robot 总状态机使用。source 用来标识命令
 * 来源，不在底盘内做复杂的业务控制权仲裁；timeout_ms 只控制入队等待时间。
 */
uint8_t ChassisEnable(ChassisCommandSource_t source, uint32_t timeout_ms);
uint8_t ChassisDisable(ChassisCommandSource_t source, uint32_t timeout_ms);
uint8_t ChassisStop(ChassisCommandSource_t source, uint32_t timeout_ms);
uint8_t ChassisEmergencyStop(uint32_t timeout_ms);
uint8_t ChassisClearFault(ChassisCommandSource_t source, uint32_t timeout_ms);

/* 设置车体系速度：vx/vy 单位 m/s，wz 单位 rad/s。上层应周期刷新以避免超时。 */
uint8_t ChassisSetBodyVelocity(
    float vx_mps,
    float vy_mps,
    float wz_radps,
    ChassisCommandSource_t source,
    uint32_t timeout_ms);

/* 设置相对位姿增量；frame 指定增量按 BODY 还是 ODOM 坐标系解释。 */
uint8_t ChassisSetRelativePose(
    float x_m,
    float y_m,
    float yaw_rad,
    ChassisReferenceFrame_t frame,
    ChassisCommandSource_t source,
    uint32_t timeout_ms);

/* 设置 ODOM 坐标系绝对位姿目标，单位为 m/m/rad。 */
uint8_t ChassisSetAbsolutePose(
    float x_m,
    float y_m,
    float yaw_rad,
    ChassisCommandSource_t source,
    uint32_t timeout_ms);

/* 复制一致状态快照；成功返回 1U，Mutex 短暂忙时返回 0U，本轮可稍后重试。 */
uint8_t ChassisGetState(ChassisState_TypeDef *state);

/*
 * ======================== 控制器内部类型与接口 ========================
 * 下面内容供 chassis.c 与 chassis_control.c 协作，不属于上位机业务接口。
 * 保留纯状态机实现为独立 .c，便于阅读算法；外部模块不要直接调用这些函数。
 */
typedef struct
{
    float max_acceleration;
    float max_velocity;
    float max_jerk;
} ChassisControl_PlannerLimit_t;

typedef struct
{
    ChassisMecanum_Config_t mecanum;
    float max_vx_mps;
    float max_vy_mps;
    float max_wz_radps;
    float max_vx_accel_mps2;
    float max_vy_accel_mps2;
    float max_wz_accel_radps2;
    ChassisControl_PlannerLimit_t planner_translation;
    ChassisControl_PlannerLimit_t planner_yaw;
    uint32_t command_timeout_ms;
    uint32_t feedback_timeout_ms;
    uint8_t require_motor_feedback;
} ChassisControl_Config_t;

typedef struct
{
    ChassisControl_State_t state;
    uint32_t fault_flags;
    ChassisCommandSource_t active_source;
    uint32_t last_sequence;
    uint32_t last_command_ms;
    uint32_t last_feedback_ms;
    RobotPose2D_t pose;
    RobotPose2D_t target_pose;
    RobotBodyVelocity_t requested_body_velocity;
    RobotBodyVelocity_t commanded_body_velocity;
    RobotBodyVelocity_t actual_body_velocity;
    float target_motor_rpm[CHASSIS_MECANUM_WHEEL_COUNT];
    float feedback_motor_rpm[CHASSIS_MECANUM_WHEEL_COUNT];
    float motor_scale;
    uint8_t feedback_valid;
    uint8_t initialized;
} ChassisControl_Status_t;

typedef struct
{
    float motor_rpm[CHASSIS_MECANUM_WHEEL_COUNT];
    uint8_t send_motor_targets;
} ChassisControl_Output_t;

typedef struct
{
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

ChassisControl_Result_t ChassisControl_Init(
    ChassisControl_t *control,
    const ChassisControl_Config_t *config,
    uint32_t now_ms);
ChassisControl_Result_t ChassisControl_SubmitCommand(
    ChassisControl_t *control,
    const ChassisCommand_t *command,
    uint32_t now_ms);
ChassisControl_Result_t ChassisControl_UpdateMotorFeedback(
    ChassisControl_t *control,
    const float motor_rpm[CHASSIS_MECANUM_WHEEL_COUNT],
    uint32_t feedback_time_ms);
ChassisControl_Result_t ChassisControl_Step(
    ChassisControl_t *control,
    uint32_t now_ms,
    float delta_time_s,
    ChassisControl_Output_t *output);
void ChassisControl_SetExternalFault(
    ChassisControl_t *control,
    uint32_t fault_flags);
void ChassisControl_GetStatus(
    const ChassisControl_t *control,
    ChassisControl_Status_t *status);

#ifdef __cplusplus
}
#endif

#endif
