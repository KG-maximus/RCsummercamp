#include "StrategyAlogrithm.h"
#include <math.h>

static float CalcV1(float a_max, float j)
{
    return (a_max * a_max) / (2.0f * j);
}

/* 正确的S曲线减速距离 */
static float CalcDecelDist(float v, float a_max, float j)
{
    float v_jerk = CalcV1(a_max, j);

    if (v <= 2.0f * v_jerk)
    {
        /* 三角S曲线：phase5 + phase7，无匀减速段 */
        return powf(v, 1.5f) / sqrtf(j);
    }

    /* 完整7段S曲线减速：phase5 + phase6 + phase7 */
    return (v * v) / (2.0f * a_max) + (v * a_max) / (2.0f * j);
}

/* 单步子步积分，dt 必须很小（如 1ms） */
static void SpeedPlanOneStep(SpeedPlan_TypeDef *sp, float dt)
{
    float v1 = CalcV1(sp->a_limit, sp->j_limit);

    switch (sp->state)
    {
    case phase1:
    {
        sp->a += sp->j_limit * dt;
        if (sp->a >= sp->a_limit)
        {
            sp->a = sp->a_limit;
            sp->state = phase2;
        }
        sp->v += sp->a * dt;
        if (sp->v >= sp->v_limit)
        {
            sp->v = sp->v_limit;
            sp->a = 0.0f;
            sp->state = phase3_end;
        }
        sp->s += sp->v * dt;
        break;
    }

    case phase2:
    {
        sp->v += sp->a * dt;
        if (sp->v >= sp->v_limit - v1)
        {
            sp->state = phase3;
        }
        sp->s += sp->v * dt;
        break;
    }

    case phase3:
    {
        sp->a -= sp->j_limit * dt;
        sp->v += sp->a * dt;
        if (sp->v > sp->v_limit)
        {
            sp->v = sp->v_limit;
            sp->a = 0.0f;
            sp->state = phase3_end;
        }
        sp->s += sp->v * dt;

        if (sp->a <= 0.0f)
        {
            sp->a = 0.0f;
            sp->state = phase3_end;
        }
        break;
    }

    case phase3_end:
    {
        float decel_dist = CalcDecelDist(sp->v, sp->a_limit, sp->j_limit);
        if (sp->s >= fabsf(sp->error_s) - decel_dist)
            sp->state = phase5;
        else
            sp->state = phase4;
        break;
    }

    case phase4:
    {
        sp->s += sp->v * dt;
        float decel_dist = CalcDecelDist(sp->v, sp->a_limit, sp->j_limit);
        if (sp->s >= fabsf(sp->error_s) - decel_dist)
        {
            sp->state = phase5;
        }
        break;
    }

    case phase5:
    {
        sp->a -= sp->j_limit * dt;
        if (sp->a <= -sp->a_limit)
        {
            sp->a = -sp->a_limit;
            sp->state = phase6;
        }
        sp->v += sp->a * dt;
        sp->s += sp->v * dt;
        break;
    }

    case phase6:
    {
        sp->v += sp->a * dt;
        sp->s += sp->v * dt;
        if (sp->v <= v1)
        {
            sp->state = phase7;
        }
        break;
    }

    case phase7:
    {
        sp->a += sp->j_limit * dt;
        sp->v += sp->a * dt;
        sp->s += sp->v * dt;

        if (sp->a >= 0.0f || sp->v <= 0.0f)
        {
            sp->a = 0.0f;
            sp->v = 0.0f;
            sp->s = fabsf(sp->error_s);
            sp->state = idle;
        }
        break;
    }

    default:
        break;
    }
}

void SpeedPlanInit(SpeedPlan_TypeDef *sp, float a_max, float v_max, float j)
{
    sp->state = init;

    sp->j = j;
    sp->a_max = a_max;
    sp->v_max = v_max;

    sp->j_limit = j;
    sp->a_limit = a_max;
    sp->v_limit = v_max;

    sp->a = 0.0f;
    sp->v = 0.0f;
    sp->s = 0.0f;

    sp->error_s = 0.0f;
    sp->direction_flag = 1.0f;
    sp->position_initial = 0.0f;

    sp->time_stamp = HAL_GetTick();
}

void SpeedPlanUpdate(SpeedPlan_TypeDef *sp, float position_actual, float position_target)
{
    uint32_t now = HAL_GetTick();
    float dt_total = (now - sp->time_stamp) * 0.001f;
    sp->time_stamp = now;

    if (dt_total <= 0.0f)
        return;

    /* 调用间隔异常过长(任务被长时间阻塞等)时钳位,避免下方子步积分循环次数暴增卡死任务 */
    if (dt_total > 0.1f)
        dt_total = 0.1f;

    /* ---------- idle：已到位，保持静止，不清零 s（保留 error_s 供观察） ---------- */
    if (sp->state == idle)
    {
        sp->a = 0.0f;
        sp->v = 0.0f;
        return;
    }

    /* ---------- init：外部触发的新目标启动或打断重规划 ---------- */
    if (sp->state == init)
    {
        float err = position_target - position_actual;
        if (fabsf(err) <= 0.03f)
        {
            sp->state = idle;
            return;
        }

        float old_v = sp->v;
        float old_dir = sp->direction_flag;

        sp->error_s = err;
        sp->direction_flag = (err >= 0.0f) ? 1.0f : -1.0f;

        /* 方向反转：衰减继承速度 */
        if (old_dir * sp->direction_flag < 0.0f)
        {
            old_v *= 0.3f;
        }

        sp->position_initial = position_actual;

        sp->s = 0.0f;
        sp->a = 0.0f;

        float S = fabsf(sp->error_s);
        float v1 = CalcV1(sp->a_max, sp->j);
        float b = sp->a_max * sp->a_max / sp->j;
        float discriminant = b * b + 4.0f * S * sp->a_max;
        float v_peak = (-b + sqrtf(discriminant)) / 2.0f;

        if (v_peak < 2.0f * v1)
        {
            v_peak = powf(0.5f * sp->j * S * S, 1.0f / 3.0f);
        }

        float scale = 1.0f;
        if (S < 0.08f)
            scale = 0.15f;
        else if (S < 0.20f)
            scale = 0.35f;
        else if (S < 0.60f)
            scale = 0.65f;
        else if (S < 1.20f)
            scale = 0.90f;
        v_peak *= scale;

        sp->v_limit = fminf(v_peak, sp->v_max);
        if (sp->v_limit < 0.0f)
            sp->v_limit = 0.0f;

        if (S < 0.08f)
            sp->j_limit = 8.0f;
        else if (S < 0.15f)
            sp->j_limit = 12.0f;
        else if (S < 0.30f)
            sp->j_limit = 16.0f;
        else
            sp->j_limit = sp->j;

        float v1_limit = CalcV1(sp->a_max, sp->j_limit);
        if (sp->v_limit < 2.0f * v1_limit)
            sp->a_limit = sqrtf(sp->j_limit * sp->v_limit);
        else
            sp->a_limit = sp->a_max;

        /* ========== 打断智能处理 ========== */
        if (old_dir * sp->direction_flag > 0.0f) /* 同向打断 */
        {
            float decel_needed = CalcDecelDist(old_v, sp->a_limit, sp->j_limit);

            if (S >= decel_needed * 1.1f)
            {
                /* 距离充裕：根据当前速度与新峰值的差距决定是继承、减速还是重新加速 */
                if (old_v > sp->v_limit * 1.05f)
                {
                    /* 当前速度超过新峰值，需要减速适配 */
                    sp->v = fminf(old_v, sp->v_limit * 1.05f);
                    sp->state = phase3_end;
                }
                else if (old_v < sp->v_limit * 0.85f)
                {
                    /* 速度还低，继续加速到新的 v_limit */
                    sp->v = old_v;
                    sp->a = 0.0f;
                    sp->state = phase1;
                }
                else
                {
                    /* 速度接近峰值，丝滑继承匀速 */
                    sp->v = old_v;
                    sp->state = phase3_end;
                }
            }
            else
            {
                /* 距离不够：强制降到安全速度 */
                float v_safe = old_v;
                for (int i = 0; i < 15 && v_safe > 0.05f; i++)
                {
                    v_safe *= 0.82f;
                    decel_needed = CalcDecelDist(v_safe, sp->a_limit, sp->j_limit);
                    if (decel_needed <= S)
                        break;
                }
                sp->v = v_safe;
                sp->state = phase3_end;
            }
        }
        else /* 反向打断 */
        {
            /* 制动距离安全检查 */
            float v_safe = old_v;
            if (v_safe > 0.05f)
            {
                float decel_needed = CalcDecelDist(v_safe, sp->a_limit, sp->j_limit);
                if (decel_needed > S)
                {
                    for (int i = 0; i < 15 && v_safe > 0.05f; i++)
                    {
                        v_safe *= 0.82f;
                        decel_needed = CalcDecelDist(v_safe, sp->a_limit, sp->j_limit);
                        if (decel_needed <= S)
                            break;
                    }
                }
            }

            if (v_safe > 0.10f)
            {
                sp->v = v_safe;
                sp->state = phase3_end;
            }
            else
            {
                sp->v = 0.0f;
                sp->state = phase1;
            }
        }

        sp->time_stamp = now; /* 重置时间戳，防止本次 dt 爆炸 */
        return;               /* 本次只做初始化，下次正常周期再开始积分 */
    }

    /* ---------- 子步精密积分：1ms 步长 ---------- */
    const float dt_step = 0.001f;
    int n_steps = (int)(dt_total / dt_step);
    float remainder = dt_total - n_steps * dt_step;

    for (int i = 0; i < n_steps && sp->state != idle; i++)
    {
        SpeedPlanOneStep(sp, dt_step);
    }
    if (sp->state != idle && remainder > 1e-6f)
    {
        SpeedPlanOneStep(sp, remainder);
    }

    /* ---------- 唯一硬保护 ---------- */
    if (sp->state != idle)
    {
        if (sp->s >= fabsf(sp->error_s) - 0.001f || sp->v < 0.0f)
        {
            sp->a = 0.0f;
            sp->v = 0.0f;
            sp->s = fabsf(sp->error_s);
            sp->state = idle;
        }
    }
}
