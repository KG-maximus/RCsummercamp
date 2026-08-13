#ifndef ROBOT_PROTOCOL_H
#define ROBOT_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * 这是机器人内部的“语义协议”，用于上位机通信任务、机器人总状态机和
 * 下位机执行模块之间传递已解析的命令与状态。它不是串口、USB 或 CAN 的
 * 线缆字节格式。
 *
 * 上位机/总状态机职责：完成帧校验、大小端转换、单位换算、指令来源仲裁、
 * 去重/排序和业务级有效期管理，再构造 ChassisCommand_t 投递给底盘任务。
 * 下位机职责：按照最近接收的有效语义命令执行，并始终保留参数有限性、
 * 物理限速、反馈超时、CAN 故障和急停等硬件安全保护。
 */

typedef enum
{
    CHASSIS_SOURCE_NONE = 0,
    CHASSIS_SOURCE_HOST,       /* 普通上位机调试命令。 */
    CHASSIS_SOURCE_AUTONOMY,   /* 机器人总状态机/自动流程。 */
    CHASSIS_SOURCE_LOCAL,      /* 本地遥控器或操作手。 */
    CHASSIS_SOURCE_SAFETY      /* 急停与安全监督，优先级最高。 */
} ChassisCommandSource_t;

typedef enum
{
    CHASSIS_FRAME_BODY = 0, /* 机器人自身坐标系：+X 前、+Y 左。 */
    CHASSIS_FRAME_ODOM      /* 里程计固定坐标系。 */
} ChassisReferenceFrame_t;

typedef enum
{
    CHASSIS_COMMAND_HEARTBEAT = 0,
    CHASSIS_COMMAND_ENABLE,
    CHASSIS_COMMAND_DISABLE,
    CHASSIS_COMMAND_STOP,          /* 使用斜坡限制器减速停车。 */
    CHASSIS_COMMAND_ESTOP,         /* 立即输出零并锁存急停故障。 */
    CHASSIS_COMMAND_CLEAR_FAULT,
    CHASSIS_COMMAND_BODY_VELOCITY,
    CHASSIS_COMMAND_MOVE_RELATIVE,
    CHASSIS_COMMAND_MOVE_ABSOLUTE
} ChassisCommandType_t;

typedef struct
{
    float vx_mps;
    float vy_mps;
    float wz_radps;
} RobotBodyVelocity_t;

typedef struct
{
    float x_m;
    float y_m;
    float yaw_rad;
} RobotPose2D_t;

typedef struct
{
    /*
     * 以下三个字段由上位机/总状态机用于调试、回传或自身仲裁；当前底盘执行
     * 层不再据此拒绝命令，避免把“谁拥有业务控制权”的策略重复放在下位机。
     * issued_at_ms 为 0 时，ChassisTask_PostCommand() 会填写接收时 HAL tick。
     */
    uint32_t sequence;       /* 上游命令序号；建议同一来源单调递增。 */
    uint32_t issued_at_ms;   /* 上游生成/下位机接收时间，单位 ms。 */
    uint32_t valid_for_ms;   /* 上游业务有效期，单位 ms；供上游仲裁与诊断。 */
    ChassisCommandSource_t source;
    ChassisCommandType_t type;
    ChassisReferenceFrame_t frame;
    union
    {
        RobotBodyVelocity_t body_velocity;
        RobotPose2D_t pose;
    } payload;
} ChassisCommand_t;

#ifdef __cplusplus
}
#endif

#endif
