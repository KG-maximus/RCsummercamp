#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

/*
 * ============================= 配置说明 =============================
 *
 * 本文件是旧 RobotLower computer 工程中“底盘真车参数”的集中入口。
 * chassis_task.c 会将本文件的宏整理为 ChassisControl_Config_t，再交给
 * 运动学、速度规划和 M3508/C620 适配层使用。因此，通常只应在本文件
 * 修改底盘参数，不要在各个 .c 文件里散落地改同一个物理量。
 *
 * 推荐配置顺序：
 * 1. 保持两个 READY 宏为 0U，先确认 FDCAN 能收到四个电调反馈；
 * 2. 实测车轮有效半径、半轴距、半轮距、减速比和每轮安装方向；
 * 3. 架空底盘，以低速度验证 FL/FR/RL/RR 顺序与运动方向；
 * 4. 单电机完成 M3508 速度/电流环 PID 标定；
 * 5. 填写速度、加速度、S 曲线和超时参数，复核后才依次打开 READY。
 *
 * 说明：本文件中的 FDCAN2 PB12/PB13 是“本旧工程”的既有配置描述。
 * 它不能直接作为其他主控板（例如 Camptest V2.5）的引脚依据；迁移时
 * 应以目标工程的 .ioc 和真实板卡原理图为准。
 * ====================================================================
 */

/*
 * 应用级总使能开关。
 *
 * 只有下列机械参数、运动上限与规划器参数完成测量、复核和架空测试后，才
 * 把 CHASSIS_APP_CONFIG_READY 改为 1U。保持 0U 时底盘任务仍会创建，
 * 但 ChassisControl 不会初始化，也不会向电机发送运动目标。这是避免把
 * 全部为 0 的占位参数带到真车上的第一道保护。
 */
#define CHASSIS_APP_CONFIG_READY                 (0U)

/*
 * M3508/C620 PID 使能开关。
 *
 * 每个轮子完成“上电零输出、低速架空、方向确认、保守增益调试”之前必须
 * 保持 0U。为 0U 时，chassis_port 仍可初始化和解析 C620 反馈，但会让
 * M3508 驱动只发送零电流停机帧；为 1U 后才允许 PID 根据目标 rpm 输出
 * 控制电流。它不替代 CHASSIS_APP_CONFIG_READY，两者都应经过实车验证。
 */
#define CHASSIS_M3508_PID_CONFIG_READY           (0U)

/*
 * 底盘电机所在 CAN 总线及 C620 控制帧标准 ID。
 *
 * CHASSIS_M3508_CAN_HANDLE 必须对应完成初始化、滤波器配置和启动的 FDCAN
 * HAL 句柄。此旧工程约定 FDCAN2：PB12 为 RX、PB13 为 TX。
 * 0x200 是 DJI C620 常见的 1~4 号电调“电流控制组帧”ID，实际发送格式由
 * APP/Common/M3508.c 封装。修改电调组或总线前，必须同步核对驱动实现。
 */
#define CHASSIS_M3508_CAN_HANDLE                 (&hfdcan2)
#define CHASSIS_M3508_CONTROL_ID                 (0x200U)

/*
 * 物理车轮到 C620 电调 ID 的固定映射，数组顺序始终是 FL、FR、RL、RR：
 * FL（左前）-> 1，FR（右前）-> 2，RL（左后）-> 3，RR（右后）-> 4。
 * 若实际 CAN ID 与此不同，应在确认电调拨码/配置后修改对应宏；不能通过
 * 交换运动学数组下标来“凑方向”，否则正逆解和反馈会对应错轮。
 */
#define CHASSIS_M3508_ID_FL                      (1U)
#define CHASSIS_M3508_ID_FR                      (2U)
#define CHASSIS_M3508_ID_RL                      (3U)
#define CHASSIS_M3508_ID_RR                      (4U)

/*
 * M3508 级联 PID 参数。外环通常以速度 rpm 为反馈，内环以电流/转矩为
 * 反馈；具体算法、积分限幅和输出单位以 APP/Common/ControlAlgorithm.c、
 * M3508.c 的实际实现为准。所有 0.0f 都是安全占位值，未单独完成调参前
 * 不应填写猜测值，更不能在四轮同时接地的情况下直接提高输出限幅。
 */
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

/*
 * FreeRTOS 调度参数。
 * CHASSIS_CONTROL_PERIOD_MS：chassis_task 的固定控制周期，单位 ms。当前
 * 为 3 ms；M3508 反馈由 FDCAN 中断异步更新，任务只在本周期读取快照并
 * 计算下一帧目标。
 * CHASSIS_COMMAND_QUEUE_LENGTH：跨任务命令队列能暂存的最大命令数。
 * CHASSIS_TASK_STACK_BYTES：CMSIS-RTOS v2 创建线程时使用的栈空间，单位
 * 为字节；它需要与 freertos.c 的线程属性保持一致。
 */
#define CHASSIS_CONTROL_PERIOD_MS                (3U)
#define CHASSIS_COMMAND_QUEUE_LENGTH             (8U)
#define CHASSIS_TASK_STACK_BYTES                 (3072U)

/*
 * 机械参数：当前全部是待填写值，必须实测后填写。
 *
 * CHASSIS_WHEEL_RADIUS_M：带载后车轮与地面的有效半径，单位 m；不要只取
 * 标称直径的一半。
 * CHASSIS_HALF_WHEELBASE_M：车体中心到前/后轮轴线的距离，单位 m。
 * CHASSIS_HALF_TRACK_M：车体中心到左/右轮中心线的距离，单位 m。
 * 运动学旋转力臂 L = 半轴距 + 半轮距。
 * CHASSIS_GEAR_RATIO：电机轴转数 / 车轮转数；直驱填 1.0f。
 * CHASSIS_MAX_MOTOR_RPM：电机轴安全转速上限，单位 rpm；超过时四轮会
 * 等比例缩放，保持期望运动方向而整体降速。
 */
#define CHASSIS_WHEEL_RADIUS_M                   (0.0f)
#define CHASSIS_HALF_WHEELBASE_M                 (0.0f)
#define CHASSIS_HALF_TRACK_M                     (0.0f)
#define CHASSIS_GEAR_RATIO                       (0.0f)
#define CHASSIS_MAX_MOTOR_RPM                    (0.0f)

/*
 * 电机安装方向修正，顺序固定为 FL、FR、RL、RR，每项只能填 +1 或 -1。
 * 首先全部填 +1，在底盘架空状态下发送很小的纯前进命令；实际反向的轮子
 * 改为 -1。此处只修正“该电机的正 rpm 定义”，不可用 0 禁用某个轮子。
 */
#define CHASSIS_MOTOR_DIRECTION_FL               (1)
#define CHASSIS_MOTOR_DIRECTION_FR               (1)
#define CHASSIS_MOTOR_DIRECTION_RL               (1)
#define CHASSIS_MOTOR_DIRECTION_RR               (1)

/*
 * 上层允许请求的车体速度上限。车体系定义为 +X 前进、+Y 左移、+Wz 从
 * 上方看逆时针；单位分别为 m/s、m/s、rad/s。控制器采用统一比例限幅，
 * 防止独立截断某一轴后改变合速度方向或转弯半径。
 */
#define CHASSIS_MAX_VX_MPS                       (0.0f)
#define CHASSIS_MAX_VY_MPS                       (0.0f)
#define CHASSIS_MAX_WZ_RADPS                     (0.0f)

/*
 * 速度模式和 STOP 减速过程使用的一阶加速度上限，单位为 m/s^2、m/s^2、
 * rad/s^2。它们由 ChassisMecanum_SlewLimiter 在每个控制周期按
 * “最大变化量 = 加速度上限 * 实际 dt”推进，不是位置模式的 S 曲线参数。
 */
#define CHASSIS_MAX_VX_ACCEL_MPS2                (0.0f)
#define CHASSIS_MAX_VY_ACCEL_MPS2                (0.0f)
#define CHASSIS_MAX_WZ_ACCEL_RADPS2              (0.0f)

/*
 * 位置模式的七段 S 曲线参数。平移路径只使用一条规划器，沿起点到终点的
 * 直线计算速度，避免 X/Y 两条独立曲线造成轨迹弯曲；偏航另用一条规划器。
 * 平移单位为 m/s^2、m/s、m/s^3，偏航单位为 rad/s^2、rad/s、rad/s^3。
 */
#define CHASSIS_PLAN_TRANSLATION_MAX_ACCEL_MPS2  (0.0f)
#define CHASSIS_PLAN_TRANSLATION_MAX_SPEED_MPS   (0.0f)
#define CHASSIS_PLAN_TRANSLATION_MAX_JERK_MPS3   (0.0f)

#define CHASSIS_PLAN_YAW_MAX_ACCEL_RADPS2        (0.0f)
#define CHASSIS_PLAN_YAW_MAX_SPEED_RADPS         (0.0f)
#define CHASSIS_PLAN_YAW_MAX_JERK_RADPS3         (0.0f)

/*
 * 安全超时，单位均为 ms。
 * CHASSIS_COMMAND_TIMEOUT_MS：当前命令源未在此时间内继续发送命令或
 * HEARTBEAT 时，控制器转入 STOPPING 并按斜坡减速。
 * CHASSIS_FEEDBACK_TIMEOUT_MS：任一电机反馈过期的判定窗口。
 * CHASSIS_REQUIRE_MOTOR_FEEDBACK：为 1U 时，使能与运行均要求新鲜反馈；
 * 超时会进入故障并输出零目标。首次联调建议保持为 1U。
 */
#define CHASSIS_COMMAND_TIMEOUT_MS               (300U)
#define CHASSIS_FEEDBACK_TIMEOUT_MS              (100U)
#define CHASSIS_REQUIRE_MOTOR_FEEDBACK           (1U)

#endif
