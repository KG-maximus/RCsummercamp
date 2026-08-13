#include "StrategyAlogrithm.h"
#include <math.h>

/*
 * 本实现位于 APP/Common，采用“加速度的变化率 jerk 受限”的七段 S 曲线。
 * SpeedPlan_TypeDef 中的 v/a/s 均保存绝对值大小，direction_flag 单独保存
 * 正负方向，因此同一算法可以复用于前进、后退、机械臂直线轴和偏航。
 *
 * 调用者的典型用法：
 *   1. SpeedPlanInit() 初始化限值和状态；
 *   2. 需要发起或打断轨迹时，将 sp->state 置为 init；
 *   3. 在固定周期调用 SpeedPlanUpdate(actual, target)；
 *   4. 使用 direction_flag * v 作为本周期的有符号速度命令。
 *
 * 本文件假定 sp 非空，且 a_max/v_max/j 为有限正数。底盘控制器会在调用前
 * 验证这些条件；其他模块直接复用时，也必须完成同样的参数保护。
 */

/*
 * 当 jerk 从 0 上升到 a_max 后再回落到 0 时，单次“加速+减 jerk”过程的
 * 特征速度 v1。它用于判断当前曲线能否出现匀加速段/匀减速段。
 * 公式单位跟随调用场景：平移得到 m/s，偏航得到 rad/s。
 */
static float CalcV1(float a_max, float j)
{
    return (a_max * a_max) / (2.0f * j);
}

/*
 * 计算以当前速度 v 完全减到 0 所需的最短理论位移。
 *
 * 若速度较小，无法达到完整的 -a_max 平台，只会经过 phase5 和 phase7，
 * 形成三角形 S 曲线；反之则会包含 phase6 匀减速段，形成完整减速曲线。
 * phase3_end/phase4 据此提前切换到减速，避免到目标点后才开始制动。
 */
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

/*
 * 对状态机积分一个很小时间片。SpeedPlanUpdate 会把实际 dt 拆成 1 ms 子步，
 * 使状态切换和数值积分更稳定；此私有函数本身不校验参数，调用者已保证
 * dt 为正且很小。
 */
static void SpeedPlanOneStep(SpeedPlan_TypeDef *sp, float dt)
{
    float v1 = CalcV1(sp->a_limit, sp->j_limit); /* 进入 phase7 的速度阈值。 */

    switch (sp->state)
    {
    case phase1:
    {
        /* 正 jerk：加速度 0 -> +a_limit，速度平滑抬升。 */
        sp->a += sp->j_limit * dt;
        if (sp->a >= sp->a_limit)
        {
            sp->a = sp->a_limit;
            sp->state = phase2;
        }
        /* 速度和位移采用显式积分；direction_flag 不在这里参与运算。 */
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
        /* 匀加速：保持 +a_limit，直到需要开始把加速度回落。 */
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
        /* 负 jerk：加速度 +a_limit -> 0，最终速度不超过 v_limit。 */
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
        /*
         * 已完成加速。若当前累计位移已接近“剩余制动距离”，立即进入减速；
         * 否则保持当前速度进入 phase4 匀速段。
         */
        float decel_dist = CalcDecelDist(sp->v, sp->a_limit, sp->j_limit);
        if (sp->s >= fabsf(sp->error_s) - decel_dist)
            sp->state = phase5;
        else
            sp->state = phase4;
        break;
    }

    case phase4:
    {
        /* 匀速巡航，每个子步都重新估计制动距离，避免错过减速时机。 */
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
        /* 负 jerk：加速度 0 -> -a_limit，减速过程的第一段。 */
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
        /* 匀减速：保持 -a_limit，速度降至能平滑收尾的 v1。 */
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
        /* 正 jerk 收尾：加速度 -a_limit -> 0，速度收敛为 0。 */
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
        /* init/idle 不应进入积分；异常状态也在此处保持不动作，等待外层处理。 */
        break;
    }
}

void SpeedPlanInit(SpeedPlan_TypeDef *sp, float a_max, float v_max, float j)
{
    /*
     * 这里不做空指针或限值检查，调用者必须保证参数有效。初始化只写软件
     * 状态，不会让电机运动；真正开始规划需要后续把 state 置为 init 并
     * 周期调用 Update。
     */
    sp->state = init;

    sp->j = j;
    sp->a_max = a_max;
    sp->v_max = v_max;

    /* 初始时本次实际限值等于总上限；短距离新规划时会被 Update 自适应改写。 */
    sp->j_limit = j;
    sp->a_limit = a_max;
    sp->v_limit = v_max;

    sp->a = 0.0f;
    sp->v = 0.0f;
    sp->s = 0.0f;

    sp->error_s = 0.0f;
    sp->direction_flag = 1.0f;
    sp->position_initial = 0.0f;

    /* 用 HAL tick 建立第一次 dt 的时间基准，单位 ms。 */
    sp->time_stamp = HAL_GetTick();
}

void SpeedPlanUpdate(SpeedPlan_TypeDef *sp, float position_actual, float position_target)
{
    /*
     * 注意：该函数没有空指针和 NaN 防护，适合已受 ChassisControl 配置校验
     * 保护的固定周期任务。单独使用时必须在调用点保证 sp/位置输入有效。
     */
    uint32_t now = HAL_GetTick();
    /* 无符号 tick 相减可跨越 HAL_GetTick() 的自然溢出。 */
    float dt_total = (now - sp->time_stamp) * 0.001f;
    sp->time_stamp = now;

    /* 同一个 tick 内重复调用时没有可积分的时间，直接返回。 */
    if (dt_total <= 0.0f)
        return;

    /*
     * 调用间隔异常过长（例如任务被长时间阻塞）时钳位到 100 ms，避免下方
     * 子步循环次数暴增并反过来继续阻塞控制任务。该保护会牺牲该段时间内
     * 的轨迹精度，但比按陈旧 dt 一次性追赶更安全。
     */
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
        /* err 是带方向的剩余位移；正负号由 direction_flag 保存。 */
        float err = position_target - position_actual;
        /*
         * 0.03 是到位死区，单位随规划器实例而定：平移时为 0.03 m，偏航
         * 时为 0.03 rad。进入死区后不再为极小误差反复启动轨迹。
         */
        if (fabsf(err) <= 0.03f)
        {
            sp->state = idle;
            return;
        }

        /* 保存被打断前的绝对速度和方向，后续根据新目标复用或安全衰减。 */
        float old_v = sp->v;
        float old_dir = sp->direction_flag;

        sp->error_s = err;
        sp->direction_flag = (err >= 0.0f) ? 1.0f : -1.0f;

        /* 方向反转时不能完整继承反向的旧速度，先衰减以降低制动压力。 */
        if (old_dir * sp->direction_flag < 0.0f)
        {
            old_v *= 0.3f;
        }

        sp->position_initial = position_actual;

        sp->s = 0.0f;
        sp->a = 0.0f;

        /* S 是本次要走的绝对距离；理论峰值速度依赖 S、a_max 与 jerk。 */
        float S = fabsf(sp->error_s);
        float v1 = CalcV1(sp->a_max, sp->j);
        float b = sp->a_max * sp->a_max / sp->j;
        float discriminant = b * b + 4.0f * S * sp->a_max;
        float v_peak = (-b + sqrtf(discriminant)) / 2.0f;

        /* 短距离时达不到完整加速度平台，改用三角 S 曲线的峰值速度公式。 */
        if (v_peak < 2.0f * v1)
        {
            v_peak = powf(0.5f * sp->j * S * S, 1.0f / 3.0f);
        }

        /*
         * 小位移主动降低峰值速度，减少“刚起步就刹车”的抖动。这些 0.08/
         * 0.20/0.60/1.20 阈值使用与 position 相同的单位；当前算法把它们
         * 写死在实现中，复用于不同量纲场景时应在实车上审查其合理性。
         */
        float scale = 1.0f;
        /*
         * 短距离还会使用更低的 jerk 限值，令加速度变化更缓。8/12/16 的
         * 量纲同 j；它们是当前历史算法的固定调校值，并不是硬件通用常数。
         */
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

        /* 根据缩放后的 v_limit 与 j_limit 判断实际可达到的 a_limit。 */
        float v1_limit = CalcV1(sp->a_max, sp->j_limit);
        if (sp->v_limit < 2.0f * v1_limit)
            sp->a_limit = sqrtf(sp->j_limit * sp->v_limit);
        else
            sp->a_limit = sp->a_max;

        /* ========== 打断处理：优先避免新目标距离不足以容纳当前动能 ========== */
        if (old_dir * sp->direction_flag > 0.0f) /* 同向打断 */
        {
            float decel_needed = CalcDecelDist(old_v, sp->a_limit, sp->j_limit);

            /* 额外乘 1.1 预留数值积分和机械误差余量。 */
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
                /*
                 * 距离不够：最多迭代 15 次，逐步衰减继承速度，寻找能在剩余
                 * 距离内停下的近似安全值。它是启发式保护，并非严格最优解。
                 */
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
            /* 反向前先做制动距离安全检查，不能直接把有速度的规划器翻向。 */
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

        /* 重置时间戳，防止重规划分支后又将本次大 dt 用于积分。 */
        return;               /* 本次只做初始化，下次正常周期再开始积分 */
    }

    /*
     * ---------- 子步积分：以 1 ms 为主步长 ----------
     * n_steps 覆盖完整的 1 ms 片段，remainder 用最后一个小片段补齐实际 dt。
     * 状态变为 idle 后立即停止积分，避免在已到位状态继续积累误差。
     */
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

    /*
     * ---------- 最终到位硬保护 ----------
     * 数值积分可能在最后一个子步略微越过目标；接近目标或出现负速度时统一
     * 钳制到终点并进入 idle，防止状态机在终点附近反复振荡。
     */
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
