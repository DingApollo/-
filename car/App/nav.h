/*
 * nav.h —— 里程计 / 航向融合 / 片上EKF / 位置环 / 示教-复现
 *
 * 设计目标:与 main.c 解耦。本模块不碰 HAL、不读全局,所有输入用参数传入,
 * 所有输出用参数返回;主循环只需几行胶水就能接上。
 *
 * ============ 在 main.c 里怎么接(10ms 控制 tick 内)============
 *   1) 顶部: #include "App/nav.h"
 *   2) 上电初始化一次:        Nav_Init();
 *   3) 每个 control_tick(已算好 speed_left/right = 编码器计数增量):
 *        float gyro_z = (float)h30.gyro.z * 1e-6f;   // rad/s(按现有换算)
 *        Nav_UpdateOdometry(speed_left, speed_right, gyro_z);
 *   4) GPS 来一帧且质量好时:  Nav_UpdateGPS(x_gps_m, y_gps_m);
 *   5) 自主行驶时(非 PS2、pid_enabled=1),用位置环产生速度目标:
 *        int16_t tl, tr; uint8_t arrived;
 *        Nav_RepeatStep(&tl, &tr, &arrived);   // 或 Nav_GotoPoint(...)
 *        target_left = tl; target_right = tr;  // 直接喂现有速度环
 *   6) 示教态:每 100ms 调 Nav_TeachSample();
 */
#ifndef NAV_H
#define NAV_H

#include <stdint.h>

typedef struct {
    float x, y, theta;   /* 位姿估计:米、米、弧度 */
    float Px, Py;        /* x/y 各自的不确定度(协方差,标量简化) */
} Nav_Pose;

/* ---------- 生命周期 ---------- */
void Nav_Init(void);
void Nav_ResetPose(float x, float y, float theta);  /* 设原点/重定位 */

/* ---------- 估计层 ---------- */
/* 高频预测:enc_l/enc_r = 本周期左右编码器计数增量;gyro_z = rad/s */
void Nav_UpdateOdometry(int32_t enc_l, int32_t enc_r, float gyro_z);
/* 低频纠偏:仅在 GPS 质量好时调用;x/y 为局部平面坐标(米) */
void Nav_UpdateGPS(float x_gps, float y_gps);
/* 读当前估计 */
Nav_Pose Nav_GetPose(void);

/* ---------- 打滑检测(编码器 vs 陀螺交叉验证) ----------
 * 用途:驱动轮离地空转时,编码器虚报位移会污染 SLAM。
 *      上层(香橙派)读到 slip=1 应丢弃或大幅降权这一段里程计。
 * 说明:检测在 Nav_UpdateOdometry() 内部自动完成,无需额外调用。 */
uint8_t  Nav_IsSlipping(void);      /* 1=当前判定打滑,里程计不可信 */
float    Nav_GetSlipResidual(void); /* 最近一拍角速度残差(rad/s),调阈值用 */
uint32_t Nav_GetSlipCount(void);    /* 上电以来累计打滑次数,诊断底盘/路面 */

/* ---------- 控制层:位置环(go-to-goal) ---------- */
/* 给一个目标点,算出左右轮速度目标(单位=编码器计数/周期,直接写 target_left/right)。
 * 返回值经 arrived 输出是否到点。*/
void Nav_GotoPoint(float xt, float yt,
                   int16_t *tgt_left, int16_t *tgt_right, uint8_t *arrived);

/* ---------- 示教-复现 ---------- */
void Nav_TeachReset(void);                 /* 开始示教前清空 */
void Nav_TeachSample(void);                /* 每 NAV_TEACH_PERIOD_MS 调一次 */
int  Nav_TeachCount(void);                 /* 已录点数 */

void Nav_RepeatReset(void);                /* 从头复现 */
void Nav_RepeatReverse(void);             /* 撤收:倒序复现(开回起点) */
/* 复现一步:内部推进目标点,输出当前应给的轮速目标;done=1 表示全程走完 */
void Nav_RepeatStep(int16_t *tgt_left, int16_t *tgt_right, uint8_t *done);

#endif /* NAV_H */
