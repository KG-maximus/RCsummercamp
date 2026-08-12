#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

/*
 * 真车参数唯一入口。
 *
 * 只有下列参数完成测量、复核和架空测试后，才把
 * CHASSIS_APP_CONFIG_READY 改为 1U。保持 0U 时，底盘任务会运行，
 * 但控制器不会初始化，也不会向电机发送运动目标。
 */
#define CHASSIS_APP_CONFIG_READY                 (0U)

/*
 * M3508 speed/current PID is intentionally disabled until each wheel has been
 * bench-tested with a zero-current boot and conservative gains. Keeping this
 * flag at 0 makes the driver transmit only the C620 zero-current stop frame.
 */
#define CHASSIS_M3508_PID_CONFIG_READY           (0U)

/* FDCAN2 is the chassis motor bus on this controller board (PB12 RX/PB13 TX). */
#define CHASSIS_M3508_CAN_HANDLE                 (&hfdcan2)
#define CHASSIS_M3508_CONTROL_ID                 (0x200U)

/* The fixed physical-to-protocol mapping: FL, FR, RL, RR -> C620 IDs 1..4. */
#define CHASSIS_M3508_ID_FL                      (1U)
#define CHASSIS_M3508_ID_FR                      (2U)
#define CHASSIS_M3508_ID_RL                      (3U)
#define CHASSIS_M3508_ID_RR                      (4U)

/* Start with all zero gains; fill after one-motor bench tuning. */
#define CHASSIS_M3508_SPEED_KP                   (0.0f)
#define CHASSIS_M3508_SPEED_KI                   (0.0f)
#define CHASSIS_M3508_SPEED_KD                   (0.0f)
#define CHASSIS_M3508_SPEED_MAX_OUT              (0.0f)
#define CHASSIS_M3508_SPEED_MAX_IOUT             (0.0f)
#define CHASSIS_M3508_CURRENT_KP                 (0.0f)
#define CHASSIS_M3508_CURRENT_KI                 (0.0f)
#define CHASSIS_M3508_CURRENT_KD                 (0.0f)
#define CHASSIS_M3508_CURRENT_MAX_OUT            (0.0f)
#define CHASSIS_M3508_CURRENT_MAX_IOUT           (0.0f)

/* FreeRTOS 调度参数。 */
/* 3 ms: M3508 feedback is received asynchronously in the FDCAN ISR. */
#define CHASSIS_CONTROL_PERIOD_MS                (3U)
#define CHASSIS_COMMAND_QUEUE_LENGTH             (8U)
#define CHASSIS_TASK_STACK_BYTES                 (3072U)

/* 机械参数：当前全部是待填写值，长度必须使用米。 */
#define CHASSIS_WHEEL_RADIUS_M                   (0.0f)
#define CHASSIS_HALF_WHEELBASE_M                 (0.0f)
#define CHASSIS_HALF_TRACK_M                     (0.0f)
#define CHASSIS_GEAR_RATIO                       (0.0f)
#define CHASSIS_MAX_MOTOR_RPM                    (0.0f)

/* 顺序固定为 FL、FR、RL、RR；每项只能填写 +1 或 -1。 */
#define CHASSIS_MOTOR_DIRECTION_FL               (1)
#define CHASSIS_MOTOR_DIRECTION_FR               (1)
#define CHASSIS_MOTOR_DIRECTION_RL               (1)
#define CHASSIS_MOTOR_DIRECTION_RR               (1)

/* 上层允许请求的车体速度上限。 */
#define CHASSIS_MAX_VX_MPS                       (0.0f)
#define CHASSIS_MAX_VY_MPS                       (0.0f)
#define CHASSIS_MAX_WZ_RADPS                     (0.0f)

/* 速度模式与停车过程使用的一阶加速度限制。 */
#define CHASSIS_MAX_VX_ACCEL_MPS2                (0.0f)
#define CHASSIS_MAX_VY_ACCEL_MPS2                (0.0f)
#define CHASSIS_MAX_WZ_ACCEL_RADPS2              (0.0f)

/* 路径平移与旋转各使用一条七段 S 曲线，避免 X/Y 独立规划造成轨迹弯曲。 */
#define CHASSIS_PLAN_TRANSLATION_MAX_ACCEL_MPS2  (0.0f)
#define CHASSIS_PLAN_TRANSLATION_MAX_SPEED_MPS   (0.0f)
#define CHASSIS_PLAN_TRANSLATION_MAX_JERK_MPS3   (0.0f)

#define CHASSIS_PLAN_YAW_MAX_ACCEL_RADPS2        (0.0f)
#define CHASSIS_PLAN_YAW_MAX_SPEED_RADPS         (0.0f)
#define CHASSIS_PLAN_YAW_MAX_JERK_RADPS3         (0.0f)

/* 安全超时。上位/总状态机应周期发送 HEARTBEAT。 */
#define CHASSIS_COMMAND_TIMEOUT_MS               (300U)
#define CHASSIS_FEEDBACK_TIMEOUT_MS              (100U)
#define CHASSIS_REQUIRE_MOTOR_FEEDBACK           (1U)

#endif
