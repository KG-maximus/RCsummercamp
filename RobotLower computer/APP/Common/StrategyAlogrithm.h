#ifndef __STRATEGYALGORITHM_H_
#define __STRATEGYALGORITHM_H_

#include "main.h"

/*
 * ======================== 七段 S 曲线规划器 ========================
 *
 * 本模块位于 APP/Common，是不依赖底盘、电机、CAN 和 FreeRTOS 的通用
 * 一维运动规划算法：输入当前位置和目标位置，输出本周期的速度大小、方向
 * 和规划状态。chassis_control 只是其中一个使用者，它会创建两个实例：一个
 * 规划 ODOM 坐标中的直线路径长度，另一个规划偏航角。单位由调用场景决定：
 *   平移：位移 m、速度 m/s、加速度 m/s^2、jerk m/s^3；
 *   偏航：位移 rad、速度 rad/s、加速度 rad/s^2、jerk rad/s^3。
 *
 * 对上接口：使用者调用 SpeedPlanInit() 完成配置，周期调用
 * SpeedPlanUpdate() 输入“当前位置/目标位置”，随后读取 v、direction_flag
 * 与 state 作为本周期输出。对下接口：本模块不直接驱动任何执行器，调用者
 * 应把 direction_flag * v 转换成自身的速度、电机或姿态目标。
 * 不要在运行中随意直接改 a/v/s/state，否则打断重规划所依赖的速度继承关系
 * 会失效。
 * ====================================================================
 */

/* 七段 S 曲线状态机。init 必须为 0，以兼容结构体零初始化后的新规划状态。 */
typedef enum
{
    init = 0,  /* 外部置入新目标后的重规划入口，不进行位移积分。 */
    phase1,    /* 正 jerk 加速：加速度从 0 平滑升高。 */
    phase2,    /* 匀加速：加速度保持 +a_limit。 */
    phase3,    /* 负 jerk 过渡：加速度回落至 0，趋近 v_limit。 */
    phase3_end,/* 加速完成判定点，根据剩余距离选择匀速或减速。 */
    phase4,    /* 匀速巡航。短距离轨迹可能跳过此状态。 */
    phase5,    /* 负 jerk 减速：加速度从 0 降至 -a_limit。 */
    phase6,    /* 匀减速：加速度保持 -a_limit。 */
    phase7,    /* 正 jerk 收尾：加速度回升到 0，速度收敛到 0。 */
    idle       /* 已到位，保持静止；v 与 a 均为 0。 */
} SpeedPlanState_TypeDef;

/* 单个一维规划器的完整运行状态。字段由 SpeedPlanUpdate() 维护。 */
typedef struct
{
    SpeedPlanState_TypeDef state; /* 当前七段 S 曲线状态。 */

    float j;     /* 用户配置的 jerk 总上限。 */
    float a_max; /* 用户配置的加速度总上限。 */
    float v_max; /* 用户配置的速度总上限。 */

    float j_limit; /* 本次规划实际 jerk 上限；短距离时会自适应降低。 */
    float a_limit; /* 本次规划实际加速度上限；受 j_limit/v_limit 共同约束。 */
    float v_limit; /* 本次规划实际峰值速度；不会超过 v_max。 */

    float a; /* 当前加速度大小；方向由 direction_flag 单独表示。 */
    float v; /* 当前速度大小，非负；实际有符号速度为 direction_flag * v。 */
    float s; /* 从本次起点累计的绝对位移，非负。 */

    float error_s;          /* 目标相对起点的带符号位移 = target - actual。 */
    float direction_flag;   /* 运动方向，仅为 +1.0f 或 -1.0f。 */
    float position_initial; /* 开始本次规划时的实际位置，用于观察和诊断。 */

    uint32_t time_stamp; /* 上次 SpeedPlanUpdate 的 HAL_GetTick()，单位 ms。 */
} SpeedPlan_TypeDef;

/*
 * 初始化规划器。a_max、v_max、j 均必须按调用场景使用同一单位体系，且应为
 * 正值。初始化不产生运动；调用者需要把 state 设为 init 或通过上层的新轨迹
 * 命令触发重规划，再周期调用 SpeedPlanUpdate()。
 */
void SpeedPlanInit(SpeedPlan_TypeDef *sp, float a_max, float v_max, float j);

/*
 * 以当前实际位置和目标位置推进一次/重建一次规划。内部按 1 ms 子步积分，
 * 并使用 HAL_GetTick() 计算实际 dt；调用周期建议为 1~10 ms，且不应长期
 * 阻塞。调用间隔超过 100 ms 时，代码会钳位 dt，避免一次执行过多子步。
 * position_actual 应来自可靠里程计/传感器；若 state 为 init，会按新目标
 * 自动处理同向或反向打断时的速度继承与安全减速。
 */
void SpeedPlanUpdate(SpeedPlan_TypeDef *sp, float position_actual, float position_target);

#endif
