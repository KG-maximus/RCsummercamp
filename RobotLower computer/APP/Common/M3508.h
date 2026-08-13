#ifndef __M3508_H_
#define __M3508_H_

#include "fdcan_common.h"
#include "ControlAlgorithm.h"

/*
DJI C620电调CAN通信协议(适用于C610/C620 + M2006/M3508系列电机)

控制帧(主控->电调,标准帧,DLC 8):
    0x200: 控制1~4号电调电流,DATA[2*(id-1)]为高8位,DATA[2*(id-1)+1]为低8位
    0x1FF: 控制5~8号电调电流,字段排布同上
    电流给定值范围 -16384~16384,对应电调输出转矩电流 -20~20A

反馈帧(电调->主控,标准帧,DLC 8,ID = 0x200 + 电调ID,默认1kHz发送):
    DATA[0-1]: 转子机械角度,高8位在前,范围0~8191对应0~360°
    DATA[2-3]: 转速,单位rpm
    DATA[4-5]: 实际转矩电流
    DATA[6]  : 电机温度,单位摄氏度
    DATA[7]  : 保留
*/

#define M3508_CTRL_ID_1TO4 0x200u     /* 1~4号电调电流控制帧ID */
#define M3508_CTRL_ID_5TO8 0x1FFu     /* 5~8号电调电流控制帧ID */
#define M3508_FEEDBACK_ID_BASE 0x200u /* 反馈帧ID = 该宏 + 电调ID */

#define M3508_ID_MIN 1u
#define M3508_ID_MAX 8u

#define M3508_CURRENT_RAW_MAX 16384 /* 电流给定值幅值上限,对应20A */

/* 双环(速度环+电流环)速度控制默认参数,调参从这里改 */
#define M3508_SPEED_KP         9.0f    /* 速度环kp,误差单位rpm,输出为电流环的目标电流(raw) */
#define M3508_SPEED_KI         0.02f
#define M3508_SPEED_KD         9.0f
#define M3508_SPEED_MAX_OUT    16384.0f /* 速度环输出限幅 */
#define M3508_SPEED_MAX_IOUT   650.0f   /* 速度环积分限幅,抗积分饱和 */

#define M3508_CURRENT_KP       0.9f    /* 电流环kp,误差为raw电流,输出为最终电流给定(raw) */
#define M3508_CURRENT_KI       0.0002f
#define M3508_CURRENT_KD       0.1f
#define M3508_CURRENT_MAX_OUT  16384.0f /* 电流环输出限幅,协议满量程16384 */
#define M3508_CURRENT_MAX_IOUT 1500.0f  /* 电流环积分限幅 */

#define M3508_GROUP_SIZE 4u /* DJI协议下1~4号、5~8号电调各共享一帧控制帧,每帧最多4台 */

typedef struct
{
    uint16_t angle;       // 转子机械角度,0~8191对应0~360°
    int16_t speed_rpm;    // 转速,单位rpm
    int16_t current;      // 实际转矩电流(原始值)
    uint8_t temperature;  // 电机温度,单位摄氏度

    uint32_t update_cnt; // 累计收到反馈帧的次数,可用于判断电调是否离线
} M3508Feedback_TypeDef;

/* 双环(速度环+电流环)速度控制:目标转速直接闭环,不做速度规划;PID通用算法见Common/ControlAlgorithm */
typedef struct
{
    CascadePID_TypeDef pid; /* outer=速度环(误差=目标转速-反馈转速rpm),inner=电流环(误差=电流环目标-反馈电流raw) */
    float speed_target;     /* 目标转速,单位rpm */
    int16_t current_output; /* 本次计算得到的电流给定值 */
} M3508Control_TypeDef;

typedef struct
{
    uint8_t id; // 电调ID,1~8
    M3508Control_TypeDef control;
    M3508Feedback_TypeDef feedback;
} M3508_TypeDef;

typedef struct
{
    uint16_t ctrl_id;                          /* 组的控制id: M3508_CTRL_ID_1TO4 或 M3508_CTRL_ID_5TO8 */
    FDCAN_HandleTypeDef *FDCAN_Handle;          /* 该组挂载的FDCAN总线实例 */
    M3508_TypeDef motor[M3508_GROUP_SIZE]; /* 组内4台电调,按ctrl_id自动分配id(1~4或5~8) */
} M3508Group_TypeDef;

/* 一组共享同一控制帧的电调(1~4号或5~8号,由ctrl_id决定);所有对外接口均以组为操作对象,
   组内4台电调的结构体由本结构体直接持有,无需调用者另外定义或登记 */

/* 初始化电调组:ctrl_id取M3508_CTRL_ID_1TO4或M3508_CTRL_ID_5TO8,决定组内4台电调的id(1~4或5~8);
   组内每台电调的控制结构体、级联PID(按M3508_*默认参数)、反馈均在此一并初始化 */
void M3508GroupInit(M3508Group_TypeDef *group, FDCAN_HandleTypeDef *FDCAN_Handle, uint16_t ctrl_id);

/* 下发组内某台电调的目标转速,单位rpm;id需落在group->ctrl_id对应的1~4或5~8范围内,否则忽略;
   不触发规划,PID直接跟踪新目标 */
void M3508GroupSetTarget(M3508Group_TypeDef *group, uint8_t id, float speed_target);

/* 解析一帧反馈数据到组内对应电调:先由std_id判断该帧是否属于本组(id落在1~4或5~8范围内),
   命中则解析反馈并返回1,不命中则不作任何处理并返回0;调用者在HAL_FDCAN_RxFifo0Callback中
   取得std_id和数据后调用,可依次对多个组尝试直至命中 */
uint8_t M3508GroupParseFeedback(M3508Group_TypeDef *group, uint32_t std_id, const uint8_t *rx_data);

/* 周期调用:推进组内4台电调的速度环+电流环级联PID,并把4台电调的电流给定一次性打包发送 */
void M3508GroupUpdate(M3508Group_TypeDef *group);

#endif
