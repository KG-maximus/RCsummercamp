#ifndef CHASSIS_PORT_H
#define CHASSIS_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "chassis_mecanum.h"

/*
 * ========================= 底盘硬件适配边界 =========================
 *
 * ChassisPort 是底盘算法与具体 M3508/C620/FDCAN 协议之间的唯一边界。
 * 上层只以 FL、FR、RL、RR 顺序读写“电机轴 rpm”，不需要知道 CAN 标准 ID、
 * 反馈帧字节格式、PID 或 HAL 句柄。当前具体协议封装在 chassis_port.c 与
 * APP/Common/M3508.c；修改电调型号或 CAN 协议时应优先改适配层，而不是
 * 修改 chassis_mecanum 的数学逻辑。
 *
 * 对下接口：ChassisPort_SendMotorRpm() 把四轮目标 rpm 转为 M3508/C620 的
 * CAN 控制帧；ChassisPort_SetMotorEnabled() 负责立即允许/切断 PID 电流输出。
 * 对上接口：ChassisPort_ReadMotorRpm() 提供已解析的轮速与时间戳，供控制器
 * 做正运动学和状态回传；外部业务模块不应绕过 ChassisTask 直接调用发帧接口。
 *
 * FDCAN 接收回调只会注册轻量解析函数：匹配句柄后将一帧反馈交给 M3508
 * 驱动更新缓存。复杂状态机、运动学和 CAN 发帧均在 chassis_task 的普通
 * 任务上下文完成，避免在中断中执行耗时控制算法。
 * ====================================================================
 */

/*
 * 初始化 PID 模板、M3508 电机组、FDCAN 接收回调和电机反馈滤波范围。
 * 本函数可重复调用；已初始化后仅返回当前就绪状态。成功不等于四轮在线，
 * 仍须用 ChassisPort_IsReady() 判断是否收到了四个电调的新鲜反馈。
 */
uint8_t ChassisPort_Init(void);

/*
 * 检查端口和四个电调是否就绪：已初始化、每轮至少有一次反馈，且反馈时间
 * 没超过 CHASSIS_FEEDBACK_TIMEOUT_MS。返回 1U 表示可安全尝试使能，0U 表示
 * 不能运动，应检查 CAN 供电、CANH/CANL、ID、终端和过滤器配置。
 */
uint8_t ChassisPort_IsReady(void);

/*
 * 为 FL/FR/RL/RR 四轮写入速度目标，单位为电机轴 rpm。该接口只写目标并
 * 调用 M3508GroupUpdate；当电机未使能或 PID READY 宏为 0U 时，驱动会保持
 * 零电流输出。调用者必须传入四元素数组，返回 1U 表示接口执行完成。
 */
uint8_t ChassisPort_SendMotorRpm(
    const float motor_rpm[CHASSIS_MECANUM_WHEEL_COUNT]);

/*
 * 复制缓存中的 FL/FR/RL/RR 电机轴反馈 rpm，并返回这组数据中最新的时间戳
 * （HAL tick，单位 ms）。函数不会等待 CAN 帧；没有四轮新鲜反馈时立即返回
 * 0U，固定周期的 chassis_task 应据此维持安全故障策略。
 */
uint8_t ChassisPort_ReadMotorRpm(
    float motor_rpm[CHASSIS_MECANUM_WHEEL_COUNT],
    uint32_t *feedback_time_ms);

/*
 * 设置是否允许 M3508 PID 输出。传 0U 时会立刻调用驱动下发零电流停机帧；
 * 传非零只表示允许后续目标生效，实际仍受 PID READY 宏和反馈状态约束。
 */
void ChassisPort_SetMotorEnabled(uint8_t enabled);

#ifdef __cplusplus
}
#endif

#endif
