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
/* 上一拍的角速度指令(用于判换向,触发脚轮预对齐) */
static float g_w_prev;
/* 脚轮预对齐剩余拍数;>0 表示正处于预对齐窗口 */
static uint16_t g_align_ticks;

/* 示教轨迹缓存 */
static Nav_Pose g_traj[NAV_TRAJ_MAX];
static int      g_traj_n;      /* 已录点数 */
static int      g_rep_idx;     /* 复现游标 */
static int      g_rep_dir;     /* +1 正序 / -1 倒序 */

/* GPS 采纳/拒收统计 */
static uint32_t g_gps_accept;
static uint32_t g_gps_reject;
static uint16_t g_gps_reject_run;  /* 连续拒收计数(防滤波发散) */
static uint32_t g_gps_reset;       /* 强制重定位次数 */

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
    g_w_prev = 0.0f;
    g_align_ticks = 0U;
    g_traj_n = 0;
    g_rep_idx = 0;
    g_rep_dir = 1;

    g_slip = 0U;
    g_slip_res = 0.0f;
    g_slip_cnt = 0U;
    g_slip_set_n = 0U;
    g_slip_clr_n = 0U;

    g_gps_accept = 0U;
    g_gps_reject = 0U;
    g_gps_reject_run = 0U;
    g_gps_reset = 0U;
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
     *     打滑期间抬高过程噪声,等于告诉上层"这段别信我"。
     *     里程计误差主要随走过的距离累积,故按 |ds| 追加一项——
     *     少了这项 Px 会被严重低估,导致 GPS 纠偏被马氏门限全部拒掉。 */
    {
        float q = (g_slip ? NAV_EKF_Q_SLIP : NAV_EKF_Q)
                + NAV_EKF_Q_DIST * fabsf(ds);
        g_pose.Px += q;
        g_pose.Py += q;
    }
}

/* ---------- 打滑检测查询接口 ---------- */
uint8_t  Nav_IsSlipping(void)      { return g_slip; }
float    Nav_GetSlipResidual(void) { return g_slip_res; }
uint32_t Nav_GetSlipCount(void)    { return g_slip_cnt; }

/* ---------- 脚轮预对齐状态查询 ---------- */
uint8_t  Nav_IsCasterAligning(void) { return (g_align_ticks > 0U) ? 1U : 0U; }

float Nav_GpsRFromQuality(uint8_t fix_type, uint8_t sats, float hdop)
{
    float r;

    /* 硬门限:不满足直接判该帧不可用 */
    if (fix_type == 0U) return 0.0f;                 /* 无效解 */
    if (sats < NAV_GPS_MIN_SATS) return 0.0f;        /* 星数不足 */
    if (hdop > NAV_GPS_MAX_HDOP) return 0.0f;        /* 几何精度差 */

    switch (fix_type) {
        case 4U:  r = NAV_GPS_R_RTK_FIX;   break;    /* RTK 固定解 */
        case 5U:  r = NAV_GPS_R_RTK_FLOAT; break;    /* RTK 浮点解 */
        case 2U:  r = NAV_GPS_R_DGPS;      break;    /* 差分 */
        default:  r = NAV_GPS_R_SPS;       break;    /* 单点 */
    }

    /* HDOP 越大越不可信,按比例放大方差 */
    r *= (1.0f + hdop);
    return r;
}

uint8_t Nav_UpdateGPS(float x_gps, float y_gps, float r_gps)
{
    float dx, dy, sx, sy, d2, Kx, Ky;

    /* r<=0 约定为"该帧不可用"(由 Nav_GpsRFromQuality 判出) */
    if (r_gps <= 0.0f) {
        g_gps_reject++;
        return 0U;
    }

    dx = x_gps - g_pose.x;
    dy = y_gps - g_pose.y;
    sx = g_pose.Px + r_gps;
    sy = g_pose.Py + r_gps;

    /* --- 马氏距离门限: 防多路径突跳 ---
     * 偏差按"估计不确定度+观测不确定度"归一化再判。
     * 于是里程计漂久了(Px大)能接受大修正,刚校正过(Px小)则拒绝突跳。*/
    d2 = (dx * dx) / sx + (dy * dy) / sy;
    if (d2 > NAV_GPS_GATE_CHI2) {
        g_gps_reject++;
        if (g_gps_reject_run < 65535U) g_gps_reject_run++;

        /* --- 连续拒收保护(防滤波发散) ---
         * 连续拒这么多帧,只有两种可能:滤波器过度自信(Px被低估),
         * 或里程计已经跑飞。两种情况都必须强制以 GPS 重定位,
         * 否则门限会把正确的观测一直挡在外面,永久锁死。 */
        if (g_gps_reject_run >= NAV_GPS_REJECT_RESET_N) {
            g_pose.x  = x_gps;
            g_pose.y  = y_gps;
            g_pose.Px = r_gps;      /* 重定位后不确定度就是这帧 GPS 的 */
            g_pose.Py = r_gps;
            g_gps_reject_run = 0U;
            g_gps_reset++;
            return 2U;              /* 2 = 已强制重定位 */
        }
        return 0U;
    }
    g_gps_reject_run = 0U;

    /* 一维卡尔曼(每轴独立): K = P/(P+R) = "这次该信 GPS 几分" */
    Kx = g_pose.Px / sx;
    g_pose.x  += Kx * dx;
    g_pose.Px *= (1.0f - Kx);

    Ky = g_pose.Py / sy;
    g_pose.y  += Ky * dy;
    g_pose.Py *= (1.0f - Ky);

    /* 注意: 不纠 theta —— GPS 静止时航向不可靠,航向永远交给陀螺 */
    g_gps_accept++;
    return 1U;
}

uint32_t Nav_GetGpsAcceptCount(void) { return g_gps_accept; }
uint32_t Nav_GetGpsRejectCount(void) { return g_gps_reject; }
uint32_t Nav_GetGpsResetCount(void)  { return g_gps_reset; }

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

#if NAV_CASTER_ALIGN_EN
    /* ---------- 脚轮预对齐 ----------
     * 起步或换向时,万向轮要先绕立轴摆到新方向,这期间胎面横向蹭地把车推偏。
     * 头 NAV_CASTER_ALIGN_MS 用极低速走完这段,把横移压到最小。 */
    {
        uint8_t was_still = (fabsf(g_v_prev) < NAV_CASTER_FLIP_DZ) &&
                            (fabsf(g_w_prev) < NAV_CASTER_FLIP_DZ);
        uint8_t want_move = (fabsf(v) > NAV_CASTER_FLIP_DZ) ||
                            (fabsf(w) > NAV_CASTER_FLIP_DZ);
        /* 换向:前后(或转向)符号翻转,且两拍都确实在动(带死区,防噪声反复触发) */
        uint8_t v_flip = ((v * g_v_prev) < 0.0f) &&
                         (fabsf(v) > NAV_CASTER_FLIP_DZ) &&
                         (fabsf(g_v_prev) > NAV_CASTER_FLIP_DZ);
        uint8_t w_flip = ((w * g_w_prev) < 0.0f) &&
                         (fabsf(w) > NAV_CASTER_FLIP_DZ) &&
                         (fabsf(g_w_prev) > NAV_CASTER_FLIP_DZ);

        if ((was_still && want_move) || v_flip || w_flip) {
            g_align_ticks = (uint16_t)NAV_CASTER_ALIGN_TICKS;
        }

        if (g_align_ticks > 0U) {
            g_align_ticks--;
            v = clampf(v, -NAV_CASTER_ALIGN_V, NAV_CASTER_ALIGN_V);
            w = clampf(w, -NAV_CASTER_ALIGN_W, NAV_CASTER_ALIGN_W);
        }
    }
#endif

    g_v_prev = v;
    g_w_prev = w;

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
    g_w_prev = 0.0f;
    g_align_ticks = 0U;   /* 下一拍会因"静止->起步"重新触发预对齐 */
}

void Nav_RepeatReverse(void)      /* 撤收: 从末点倒着走回起点 */
{
    g_rep_dir = -1;
    g_rep_idx = (g_traj_n > 0) ? (g_traj_n - 1) : 0;
    g_v_prev = 0.0f;
    g_w_prev = 0.0f;
    g_align_ticks = 0U;   /* 倒序起步同样需要脚轮重新摆正 */
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
