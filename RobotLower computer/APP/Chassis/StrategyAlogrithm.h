#ifndef __STRATEGYALGORITHM_H_
#define __STRATEGYALGORITHM_H_

#include "main.h"

/* 七段S曲线速度规划状态机,init必须为0以匹配结构体零初始化 */
typedef enum
{
    init = 0,  /* 外部触发新目标,进行重规划 */
    phase1,    /* 加加速段 */
    phase2,    /* 匀加速段 */
    phase3,    /* 减加速段,趋近v_limit */
    phase3_end,
    phase4,    /* 匀速段 */
    phase5,    /* 加减速段 */
    phase6,    /* 匀减速段 */
    phase7,    /* 减减速段,趋近0 */
    idle       /* 已到位,保持静止 */
} SpeedPlanState_TypeDef;

typedef struct
{
    SpeedPlanState_TypeDef state;

    float j;     /* 加加速度(jerk)配置上限 */
    float a_max; /* 加速度配置上限 */
    float v_max; /* 速度配置上限 */

    float j_limit; /* 本次规划实际使用的加加速度限幅,按位移自适应缩放 */
    float a_limit; /* 本次规划实际使用的加速度限幅 */
    float v_limit; /* 本次规划实际使用的速度限幅 */

    float a; /* 当前加速度 */
    float v; /* 当前速度 */
    float s; /* 当前已走过的位移(绝对值) */

    float error_s;          /* 本次规划的目标位移(带符号) */
    float direction_flag;   /* 运动方向,+1或-1 */
    float position_initial; /* 规划起点位置 */

    uint32_t time_stamp; /* 上一次SpeedPlanUpdate的HAL_GetTick() */
} SpeedPlan_TypeDef;

/* 初始化速度规划句柄,a_max/v_max/j为该规划实例允许的最大加速度/速度/加加速度 */
void SpeedPlanInit(SpeedPlan_TypeDef *sp, float a_max, float v_max, float j);

/* 按position_target重新/继续规划,周期调用,内部按1ms子步长积分,建议调用周期不超过数十ms */
void SpeedPlanUpdate(SpeedPlan_TypeDef *sp, float position_actual, float position_target);

#endif
