#ifndef ROBOT_PROTOCOL_H
#define ROBOT_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * 这是机器人内部的“语义协议”，用于任务和模块之间传递已经解析好的命令。
 * 它不是串口、USB 或 CAN 的线缆字节格式。上位机通信模块应先完成帧校验、
 * 大小端转换和单位换算，再构造 ChassisCommand_t 投递给底盘任务。
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
    uint32_t sequence;       /* 同一命令源单调递增；0 表示暂不检查序号。 */
    uint32_t issued_at_ms;   /* 生成命令的本机时间；投递函数可自动填写。 */
    uint32_t valid_for_ms;   /* 队列中允许滞留的时间；0 使用控制器默认值。 */
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
