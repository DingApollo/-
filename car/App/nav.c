/*
 * nav.c —— 三层导航栈实现
 *   估计层: 里程计推算 + 航向互补滤波 + 片上标量 EKF(GPS 纠偏)
 *   控制层: 位置环 go-to-goal + 梯形限幅 -> 编码器计数/周期(喂现有速度环)
 *   应用层: 示教-复现(教一次,一键重走 / 倒序撤收)
 *
 * 依赖: 仅标准库 + nav_config.h。不含 HAL,便于单元测试和移植到从桶 F103。
 */
#include "App/nav.h"
#include "App/nav_config.h"
#include <math.h>

/* ============================ 内部状态 ============================ */
static Nav_Pose g_pose;

/* 位置环速度斜坡的上一拍值(用于加速度限幅=梯形) */
static float g_v_prev;

/* 示教轨迹缓存 */
static Nav_Pose g_traj[NAV_TRAJ_MAX];
static int      g_traj_n;      /* 已录点数 */
static int      g_rep_idx;     /* 复现游标 */
static int      g_rep_dir;     /* +1 正序 / -1 倒序 */

/* 打滑检测状态 */
static uint8_t  g_slip;        /* 当前判定:1=打滑 */
static float    g_slip_res;    /* 最近一拍角速度残差(rad/s) */
static uint32_t g_slip_cnt;    /* 累计打滑次数 */
static uint8_t  g_slip_set_n;  /* 连续超限拍数(消抖) */
static uint8_t  g_slip_clr_n;  /* 连续正常拍数(消抖) */

/* ============================ 小工具 ============================ */
/* 归一化到 (-pi, pi];角度处理漏了这步会导致原地打转,务必用 */
static float wrap_pi(float a)
{
    while (a >  3.14159265f) a -= 6.28318531f;
    while (a <= -3.14159265f) a += 6.28318531f;
    return a;
}

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* m/s -> 编码器计数/周期(位置环输出接速度环设定值的单位换算) */
static int16_t mps_to_counts(float v_mps)
{
    float c = v_mps * NAV_DT / NAV_METER_PER_COUNT;
    if (c >  32000.0f) c =  32000.0f;
    if (c < -32000.0f) c = -32000.0f;
    return (int16_t)c;
}

/* ============================ 生命周期 ============================ */
void Nav_Init(void)
{
    g_pose.x = g_pose.y = g_pose.theta = 0.0f;
    g_pose.Px = g_pose.Py = 0.0f;
    g_v_prev = 0.0f;
    g_traj_n = 0;
    g_rep_idx = 0;
    g_rep_dir = 1;

    g_slip = 0U;
    g_slip_res = 0.0f;
    g_slip_cnt = 0U;
    g_slip_set_n = 0U;
    g_slip_clr_n = 0U;
}

void Nav_ResetPose(float x, float y, float theta)
{
    g_pose.x = x;
    g_pose.y = y;
    g_pose.theta = wrap_pi(theta);
    g_pose.Px = g_pose.Py = 0.0f;
}

Nav_Pose Nav_GetPose(void) { return g_pose; }

/* ============================ 估计层 ============================ */
void Nav_UpdateOdometry(int32_t enc_l, int32_t enc_r, float gyro_z)
{
    /* 计数增量 -> 每轮位移(m),含方向符号 */
    float dL = (float)(NAV_ENC_SIGN_L * enc_l) * NAV_METER_PER_COUNT;
    float dR = (float)(NAV_ENC_SIGN_R * enc_r) * NAV_METER_PER_COUNT;
    float ds = 0.5f * (dL + dR);              /* 车体中心前进量 */

    float dtheta_enc  = (dR - dL) / NAV_TRACK_L;    /* 编码器推的角增量 */
    float dtheta_gyro = gyro_z * NAV_DT;            /* 陀螺推的角增量 */

    /* ================= 打滑检测 =================
     * 编码器推算的角速度 vs 陀螺实测角速度。
     * 驱动轮离地空转 -> 编码器"看见"转动,车体实际没转 -> 残差暴露。 */
    float omega_enc = dtheta_enc / NAV_DT;
    float residual  = fabsf(omega_enc - gyro_z);
    uint8_t moving;

    g_slip_res = residual;

    /* 车基本没动时不判定,避免静止时陀螺零偏/噪声误报 */
    moving = ((fabsf(omega_enc) > NAV_SLIP_MIN_MOTION) ||
              (fabsf(gyro_z)    > NAV_SLIP_MIN_MOTION) ||
              (fabsf(ds)        > NAV_SLIP_MIN_MOTION * NAV_DT)) ? 1U : 0U;

    if (moving && (residual > NAV_SLIP_GATE)) {
        g_slip_clr_n = 0U;
        if (g_slip_set_n < 255U) g_slip_set_n++;
        if (!g_slip && (g_slip_set_n >= NAV_SLIP_SET_N)) {
            g_slip = 1U;
            g_slip_cnt++;
        }
    } else {
        g_slip_set_n = 0U;
        if (g_slip_clr_n < 255U) g_slip_clr_n++;
        if (g_slip && (g_slip_clr_n >= NAV_SLIP_CLR_N)) {
            g_slip = 0U;
        }
    }

#if NAV_SLIP_CONSERVATIVE
    /* 打滑时取两轮中位移较小者:空转轮读数虚高,着地轮才可信。
     * 仅在已判定打滑时启用——正常转弯两轮本就不等长,不能一概取小。 */
    if (g_slip) {
        float aL = fabsf(dL);
        float aR = fabsf(dR);
        float mag = (aL < aR) ? aL : aR;
        ds = (ds >= 0.0f) ? mag : -mag;
    }
#endif

    /* --- 航向互补滤波: 高频信陀螺(不怕打滑) + 低频用编码器纠偏 ---
     * 打滑期间编码器航向完全不可信,权重切到 100% 陀螺 */
    {
        float w_gyro = g_slip ? 1.0f : NAV_HEADING_GYRO_W;
        g_pose.theta += w_gyro * dtheta_gyro + (1.0f - w_gyro) * dtheta_enc;
        g_pose.theta = wrap_pi(g_pose.theta);
    }

    /* --- 用融合后的航向把前进量投影到 x/y --- */
    g_pose.x += ds * cosf(g_pose.theta);
    g_pose.y += ds * sinf(g_pose.theta);

    /* --- 没有绝对参照,不确定度只增(这就是漂移);
     *     打滑期间抬高过程噪声,等于告诉上层"这段别信我" --- */
    {
        float q = g_slip ? NAV_EKF_Q_SLIP : NAV_EKF_Q;
        g_pose.Px += q;
        g_pose.Py += q;
    }
}

/* ---------- 打滑检测查询接口 ---------- */
uint8_t  Nav_IsSlipping(void)      { return g_slip; }
float    Nav_GetSlipResidual(void) { return g_slip_res; }
uint32_t Nav_GetSlipCount(void)    { return g_slip_cnt; }

void Nav_UpdateGPS(float x_gps, float y_gps)
{
    /* x 轴一维卡尔曼: K = P/(P+R) = "这次该信 GPS 几分" */
    float Kx = g_pose.Px / (g_pose.Px + NAV_EKF_R_GPS);
    g_pose.x  += Kx * (x_gps - g_pose.x);
    g_pose.Px *= (1.0f - Kx);

    float Ky = g_pose.Py / (g_pose.Py + NAV_EKF_R_GPS);
    g_pose.y  += Ky * (y_gps - g_pose.y);
    g_pose.Py *= (1.0f - Ky);

    /* 注意: 不纠 theta —— GPS 静止时航向不可靠,航向永远交给陀螺 */
}

/* ============================ 控制层: 位置环 ============================ */
void Nav_GotoPoint(float xt, float yt,
                   int16_t *tgt_left, int16_t *tgt_right, uint8_t *arrived)
{
    float dx = xt - g_pose.x;
    float dy = yt - g_pose.y;
    float rho   = sqrtf(dx * dx + dy * dy);                 /* 到目标距离 */
    float alpha = wrap_pi(atan2f(dy, dx) - g_pose.theta);   /* 航向该偏多少 */

    /* 外环 P: 距离越近 rho 越小 -> 自动减速(天然梯形的减速段) */
    float v = NAV_KP_RHO * rho;
    v = clampf(v, 0.0f, NAV_V_MAX);

    /* 加速度限幅: 限制 v 每拍变化量(梯形的加速斜坡) */
    float dv_max = NAV_A_MAX * NAV_DT;
    v = clampf(v, g_v_prev - dv_max, g_v_prev + dv_max);

    /* 航向控制 */
    float w = clampf(NAV_KP_ALPHA * alpha, -NAV_W_MAX, NAV_W_MAX);

    /* 头没摆正先原地转,别画弧跑偏 */
    if (fabsf(alpha) > NAV_ALIGN_GATE) v = 0.0f;

    /* 到点 */
    uint8_t done = (rho < NAV_ARRIVE_R) ? 1U : 0U;
    if (done) { v = 0.0f; w = 0.0f; }

    g_v_prev = v;

    /* (v,w) -> 左右轮线速度(m/s) -> 编码器计数/周期 */
    float vL = v - w * NAV_TRACK_L * 0.5f;
    float vR = v + w * NAV_TRACK_L * 0.5f;
    *tgt_left  = mps_to_counts(vL);
    *tgt_right = mps_to_counts(vR);
    if (arrived) *arrived = done;
}

/* ============================ 示教-复现 ============================ */
void Nav_TeachReset(void) { g_traj_n = 0; }

void Nav_TeachSample(void)
{
    if (g_traj_n < NAV_TRAJ_MAX) {
        g_traj[g_traj_n++] = g_pose;   /* 直接存 EKF 估计出的位姿 */
    }
}

int Nav_TeachCount(void) { return g_traj_n; }

void Nav_RepeatReset(void)
{
    g_rep_dir = 1;
    g_rep_idx = 0;
    g_v_prev = 0.0f;
}

void Nav_RepeatReverse(void)      /* 撤收: 从末点倒着走回起点 */
{
    g_rep_dir = -1;
    g_rep_idx = (g_traj_n > 0) ? (g_traj_n - 1) : 0;
    g_v_prev = 0.0f;
}

void Nav_RepeatStep(int16_t *tgt_left, int16_t *tgt_right, uint8_t *done)
{
    if (done) *done = 0U;

    if (g_traj_n == 0 ||
        g_rep_idx < 0 || g_rep_idx >= g_traj_n) {
        *tgt_left = 0; *tgt_right = 0;
        if (done) *done = 1U;
        return;
    }

    Nav_Pose *p = &g_traj[g_rep_idx];
    uint8_t arrived = 0U;
    Nav_GotoPoint(p->x, p->y, tgt_left, tgt_right, &arrived);

    /* 到当前点附近就切下一个(用较宽半径,复现不必逐点精确停) */
    float dx = p->x - g_pose.x, dy = p->y - g_pose.y;
    if ((dx * dx + dy * dy) < (NAV_REPEAT_ARRIVE_R * NAV_REPEAT_ARRIVE_R)) {
        g_rep_idx += g_rep_dir;
        if (g_rep_idx < 0 || g_rep_idx >= g_traj_n) {
            *tgt_left = 0; *tgt_right = 0;
            if (done) *done = 1U;   /* 全程走完 */
        }
    }
}
