/*
 * nav_config.h —— 导航/编队栈的可调参数集中处
 *
 * 三层架构(见 nav.c):
 *   估计层(里程计+航向融合+EKF) -> 位置环(go-to-goal) -> 现有速度环 PID
 *
 * ★★ 会后必须实测并填入下面三个参数,否则里程计按比例出错 ★★
 */
#ifndef NAV_CONFIG_H
#define NAV_CONFIG_H

/* ===================== 必测三参数 ===================== */
/* 轮径(m)。65mm 已知 */
#define NAV_WHEEL_D            0.065f
/* ★实测:驱动某一轮正好转一整圈,读编码器计数变化量(绝对值)。
 * 这一个数已经包含了 线数×减速比×编码器倍频,不用再分开算。 */
#define NAV_COUNTS_PER_REV     4000.0f
/* ★实测:左右驱动轮中心距 L(m) */
#define NAV_TRACK_L            0.20f

/* 一个编码器计数 = 走多少米(自动推导,别手改) */
#define NAV_METER_PER_COUNT    (3.14159265f * NAV_WHEEL_D / NAV_COUNTS_PER_REV)

/* 编码器方向符号:若某轮前进时计数反而变小,把对应改成 -1 */
#define NAV_ENC_SIGN_L         (+1)
#define NAV_ENC_SIGN_R         (+1)

/* ===================== 控制周期 ===================== */
/* 与 main.c 的 PID_PERIOD_MS 保持一致(10ms) */
#define NAV_DT                 0.01f

/* ===================== 位置环(外环)增益/限幅 ===================== */
#define NAV_KP_RHO             1.2f    /* 距离 -> 前进速度 */
#define NAV_V_MAX              0.40f   /* 最大前进速度 m/s */
#define NAV_A_MAX              0.60f   /* 加速度限幅 m/s^2(= 梯形斜坡) */
#define NAV_KP_ALPHA           2.5f    /* 航向偏差 -> 角速度 */
#define NAV_W_MAX              1.5f    /* 最大角速度 rad/s */
#define NAV_ALIGN_GATE         0.6f    /* |航向偏差|>此值先原地转(v=0),rad */
#define NAV_ARRIVE_R           0.05f   /* 到点判定半径 m */

/* ===================== 航向互补滤波 ===================== */
/* 高频信陀螺、低频用编码器纠偏;0.98 = 主要信陀螺 */
#define NAV_HEADING_GYRO_W     0.98f

/* ===================== 片上 EKF(标量每轴) ===================== */
#define NAV_EKF_Q              1e-4f   /* 过程噪声:每步长多少不确定 */
#define NAV_EKF_R_GPS          4.0f    /* GPS 观测噪声:越大越不信 GPS */

/* ===================== 示教-复现 ===================== */
#define NAV_TRAJ_MAX           1000    /* 轨迹点上限(×12B ≈ 12KB RAM) */
#define NAV_TEACH_PERIOD_MS    100     /* 示教采样周期 */
#define NAV_REPEAT_ARRIVE_R    0.08f   /* 复现时换下一点的半径 m */

#endif /* NAV_CONFIG_H */
