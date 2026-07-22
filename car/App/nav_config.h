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
/* 同一个周期的整数(ms)形式,用于按毫秒算拍数 */
#define NAV_PERIOD_MS          10

/* ===================== 位置环(外环)增益/限幅 ===================== */
#define NAV_KP_RHO             1.2f    /* 距离 -> 前进速度 */
#define NAV_V_MAX              0.40f   /* 最大前进速度 m/s */
#define NAV_A_MAX              0.60f   /* 加速度限幅 m/s^2(= 梯形斜坡) */
#define NAV_KP_ALPHA           2.5f    /* 航向偏差 -> 角速度 */
#define NAV_W_MAX              1.5f    /* 最大角速度 rad/s */
#define NAV_ALIGN_GATE         0.6f    /* |航向偏差|>此值先原地转(v=0),rad */
#define NAV_ARRIVE_R           0.05f   /* 到点判定半径 m */

/* ============ 脚轮预对齐(起步/换向时先低速让万向轮摆正) ============
 * 原理:脚轮有偏距,换向时必须先绕立轴转到新方向,这期间胎面在地上横向蹭,
 *      产生的横向力 F=μN 可达 15N(比驱动牵引力还大),把车体推偏几毫米。
 *      而差速里程计模型假设车体只沿轮向运动,这段横移编码器完全测不到。
 * 对策:起步和换向的头 300ms 用极低速走,横向力同样存在但位移小;
 *      脚轮摆正后后续行程就干净了。
 * 注:主桶有 SLAM 可纠正残余误差,本功能主要改善起步段的里程计质量。 */
#define NAV_CASTER_ALIGN_EN    1       /* 0=关闭本功能 */
#define NAV_CASTER_ALIGN_MS    300     /* 预对齐时长(ms) */
#define NAV_CASTER_ALIGN_V     0.05f   /* 预对齐期间前进速度上限 m/s */
#define NAV_CASTER_ALIGN_W     0.30f   /* 预对齐期间角速度上限 rad/s */
/* 方向翻转判定的死区:速度绝对值低于此值不算"有方向",避免噪声反复触发 */
#define NAV_CASTER_FLIP_DZ     0.02f
/* 拍数(自动推导,别手改) */
#define NAV_CASTER_ALIGN_TICKS (NAV_CASTER_ALIGN_MS / NAV_PERIOD_MS)

/* ===================== 航向互补滤波 ===================== */
/* 高频信陀螺、低频用编码器纠偏;0.98 = 主要信陀螺 */
#define NAV_HEADING_GYRO_W     0.98f

/* ===================== 片上 EKF(标量每轴) ===================== */
#define NAV_EKF_Q              1e-4f   /* 过程噪声:每拍固定增量(静止时的缓慢发散) */
/* ★ 里程计误差主要随「走过的距离」增长,不是随时间。只用固定 Q 会让 Px 被
 * 严重低估 -> 滤波器过度自信 -> 马氏门限把正确的 GPS 也拒掉 -> 永久锁死。
 * 本项按每走 1 米增加多少位置方差(m^2/m)计。
 * ★★ 当前取 0.02(保守):因为 NAV_COUNTS_PER_REV / NAV_TRACK_L / 轮径
 *    三个参数尚未实测,是估计值,里程计存在未知的系统性比例误差。
 *    等三参数标定完成、实测漂移率后,可降到 0.005 左右(对应约2%漂移)以
 *    提高对 GPS 突跳的辨别力。宁可先放宽,也不要一开始就把正确观测拒掉。 */
#define NAV_EKF_Q_DIST         0.02f
#define NAV_EKF_R_GPS          4.0f    /* GPS 观测噪声缺省值(方差,m^2);
                                        * 现在推荐按实时质量传入,见 nav.h */

/* ============ GPS 质量门限与观测噪声映射 ============
 * 背景:园区/城市峡谷的多路径会让 GPS 位置突跳几十米。
 *      信了一个跳飞的点,会把里程计/SLAM 的正确位姿直接拽歪——
 *      比完全不用 GPS 糟糕得多。原则:宁可漏用,不可误用。 */
/* 最少卫星数 / 最大 HDOP,不满足直接判定不可用 */
#define NAV_GPS_MIN_SATS       6
#define NAV_GPS_MAX_HDOP       2.0f
/* 各定位类型的基础观测方差(m^2),按 NMEA GGA 第6字段分类 */
#define NAV_GPS_R_RTK_FIX      0.02f   /* RTK 固定解  ~2cm */
#define NAV_GPS_R_RTK_FLOAT    0.5f    /* RTK 浮点解  ~50cm */
#define NAV_GPS_R_DGPS         1.0f    /* 差分       ~1m */
#define NAV_GPS_R_SPS          6.0f    /* 单点       ~2.5m(方差≈6) */
/* 马氏距离门限(卡方):观测与估计的偏差按各自不确定度归一化后超过此值即拒收。
 * 9.0 ≈ 3σ。用马氏距离而非固定米数,好处是"里程计已漂很久(Px大)时允许大修正,
 * 刚校正过(Px小)时拒绝突跳"——自动适应,不用手调阈值。 */
#define NAV_GPS_GATE_CHI2      9.0f
/* 连续拒收保护:若连续拒收超过此数,说明要么滤波器过度自信、要么里程计已跑飞。
 * 此时强制用当前 GPS 重定位,否则会永久锁死再也纠不回来(滤波发散)。 */
#define NAV_GPS_REJECT_RESET_N 20

/* ============ 打滑检测(编码器 vs 陀螺交叉验证) ============
 * 原理:陀螺测的是"车体真的转了多少",编码器测的是"轮子转了多少"。
 *      驱动轮离地空转时,编码器看见转动而车体没转,残差立刻暴露。
 * 覆盖:单轮空转/打滑(四点支撑横滚抬轮的主要失效模式)。
 * 盲区:两轮同步等量打滑(如整车被卡住原地空转)——无角速度差,测不出;
 *      需要时可再加"加速度计 vs 编码器加速度"交叉验证。 */
/* 角速度残差门限(rad/s)。太小→陀螺噪声/轮距标定误差引起误报,
 * 太大→漏报。先用 0.35,实测后按 BALDBG 观察到的静态残差上调 2~3 倍 */
#define NAV_SLIP_GATE          0.35f
/* 消抖:连续 N 拍超限才判打滑;连续 M 拍正常才解除(解除比判定迟钝,偏保守) */
#define NAV_SLIP_SET_N         3
#define NAV_SLIP_CLR_N         10
/* 活动门限(rad/s):车基本没动时不做判定,避免静止噪声误报 */
#define NAV_SLIP_MIN_MOTION    0.05f
/* 打滑期间的过程噪声:远大于正常值,等于告诉上层 EKF/SLAM"这段别信我" */
#define NAV_EKF_Q_SLIP         2e-2f
/* 1=打滑时用"两轮中位移较小者"作保守位移估计(空转轮读数虚高,着地轮才可信)
 * 0=照常用两轮平均值 */
#define NAV_SLIP_CONSERVATIVE  1

/* ===================== 示教-复现 ===================== */
#define NAV_TRAJ_MAX           1000    /* 轨迹点上限(×12B ≈ 12KB RAM) */
#define NAV_TEACH_PERIOD_MS    100     /* 示教采样周期 */
#define NAV_REPEAT_ARRIVE_R    0.08f   /* 复现时换下一点的半径 m */

#endif /* NAV_CONFIG_H */
