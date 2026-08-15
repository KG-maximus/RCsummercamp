#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#include "fdcan.h"

/*
 * BRT38M-COM4096D32-RT1 双万向轮编码器模块。
 *
 * 本模块只完成两件事：
 * 1. 接收上层 FDCAN 转交的编码器位置帧，并保存两只编码器的原始计数；
 * 2. 在普通任务中将两个相对计数换算为底盘平面坐标 x、y。
 *
 * 坐标约定：ID1 编码器的轮子滚动距离定义为 x，ID2 的轮子滚动距离定义为 y。
 * 首次收到每只编码器的合法位置反馈时，该位置自动成为本次上电后的软件原点，
 * 因此 x、y 均从 0 m 开始。本模块不写入编码器硬件零点。
 */

/* ------------------------- 可调配置：集中在此处 ------------------------- */

/* 设为 0U 时，FreeRTOS 默认任务不初始化、也不更新编码器模块。 */
#define ENCODER_ENABLE                    (1U)

/* 普通任务调用 Encoder_Update() 的周期，单位 ms。当前编码器查询使用 20 ms。 */
#define ENCODER_UPDATE_PERIOD_MS          (20U)

/*
 * 两只编码器的 FDCAN 和节点 ID 配置。
 * x 编码器：ID1，当前接 FDCAN1；y 编码器：ID2，当前接 FDCAN2。
 * 更换 CAN 接口或修改编码器 ID 时，只修改此处，不修改协议解析代码。
 */
#define ENCODER_X_FDCAN_HANDLE            (&hfdcan1)
#define ENCODER_X_NODE_ID                 (1U)
#define ENCODER_Y_FDCAN_HANDLE            (&hfdcan2)
#define ENCODER_Y_NODE_ID                 (2U)

/* 两只万向轮当前均按直径 10 cm 计算；距离单位统一为 m。 */
#define ENCODER_WHEEL_DIAMETER_M          (0.10f)
#define ENCODER_WHEEL_RADIUS_M            (ENCODER_WHEEL_DIAMETER_M * 0.5f)

/*
 * 当实际滚动的正方向与编码器位置计数增加方向一致时设为 +1；相反时设为 -1。
 * 这只影响最终 x/y 的正负号，不修改编码器本身的方向配置。
 */
#define ENCODER_X_DIRECTION_SIGN          (1)
#define ENCODER_Y_DIRECTION_SIGN          (1)

/* BRT38M 固定协议与型号参数，不应作为现场调参项。 */
#define ENCODER_COUNTS_PER_REVOLUTION     (4096U)
#define ENCODER_CMD_READ_POSITION          (0x01U)
#define ENCODER_POSITION_REQUEST_LENGTH   (4U)
#define ENCODER_POSITION_REPLY_LENGTH     (7U)

/*
 * 模块唯一输出：以底盘启动时的软件原点为参考的平面坐标。
 * x_m：ID1/FDCAN1 万向轮的累计滚动距离，单位 m。
 * y_m：ID2/FDCAN2 万向轮的累计滚动距离，单位 m。
 */
typedef struct
{
    float x_m;
    float y_m;
} Encoder_PositionTypeDef;

/*
 * 初始化编码器模块。
 *
 * 本函数在调度器启动前调用一次：
 * - 按上方宏绑定 ID1/FDCAN1 与 ID2/FDCAN2；
 * - 注册 FDCAN 接收回调；
 * - 为两条总线配置各自节点 ID 的接收过滤器并启动 FDCAN。
 *
 * 它不发送位置查询。首次有效反馈到来前，Encoder_GetPosition() 返回 x=y=0。
 */
void Encoder_Init(void);

/*
 * 上层 FDCAN 的原始帧输入接口。
 *
 * 参数来自 FDCAN 接收回调：fdcan_handle 表示帧来自哪条总线，std_id 为标准 CAN ID，
 * data 为完整 8 字节接收缓存。本模块只接收格式为“位置读取回复”的 ID1/ID2 帧；
 * 其余帧直接忽略。函数可在 FDCAN 中断回调中调用，内部只保存整数计数，
 * 不进行浮点换算、串口输出或阻塞操作。
 */
void Encoder_InputFDCANFrame(FDCAN_HandleTypeDef *fdcan_handle,
                             uint32_t std_id,
                             const uint8_t data[8]);

/*
 * 固定周期更新接口，应在普通任务中按 ENCODER_UPDATE_PERIOD_MS 调用。
 * 每次调用会向 ID1 和 ID2 各发送一次位置读取请求，并把最近一次接收的原始计数
 * 换算为 x_m、y_m。不要在 FDCAN 中断回调中调用本函数。
 */
void Encoder_Update(void);

/*
 * 读取当前坐标输出。返回指针只读，始终指向模块内部保存的 x_m/y_m。
 * 调用者不应修改该结构体内容；读取前应至少调用过一次 Encoder_Update()。
 */
const Encoder_PositionTypeDef *Encoder_GetPosition(void);

#endif /* ENCODER_H */
