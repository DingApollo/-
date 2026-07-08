/**
 * @file    balance.h
 * @brief   平衡辅助接口：PD 参数/零点/跌倒判定阈值/修正叠加入口
 */
#ifndef BALANCE_H
#define BALANCE_H

#include <stdint.h>
#include "main.h"

#define BAL_DEBUG_PERIOD_MS 200U
#define BAL_PITCH_KP_DEFAULT 6000.0f
#define BAL_PITCH_KD_DEFAULT 200.0f
#define BAL_ROLL_KP_DEFAULT 3000.0f
#define BAL_ROLL_KD_DEFAULT 120.0f
#define BAL_MAX_CORR_DEFAULT 5000.0f
#define BAL_PWM_MAX 1200
#define BAL_U_MIN_ACTIVE 80.0f
#define BAL_ANGLE_FULL_RAD   0.12f
#define BAL_ANGLE_DEADBAND_RAD 0.008f
#define BAL_GYRO_DEADBAND_RAD_S 0.06f
#define BAL_FALLEN_GYRO_MAX_RAD_S 0.45f
#define BAL_AUTO_ENABLE_DEFAULT 0U
#define BAL_AUTO_ZERO_DELAY_MS 500U
#define BAL_MAX_PITCH_RAD_DEFAULT 0.785398f
#define BAL_MAX_ROLL_RAD_DEFAULT 0.610865f
/* 0：横滚修正左右同向（示警桩/双轮同驱）；1：横滚差速（平衡车，左右一正一反） */
#define BAL_ROLL_DIFFERENTIAL 0

extern uint8_t bal_enabled;
extern uint8_t bal_debug_enabled;
extern uint8_t bal_test_mode;
extern uint8_t bal_output_active;
extern uint8_t bal_fallen;
extern uint8_t bal_zero_valid;
extern uint8_t bal_auto_zero_pending;
extern uint8_t bal_last_sta;
extern float bal_last_pitch_err;
extern float bal_last_roll_err;
extern float bal_pitch_zero;
extern float bal_roll_zero;
extern float bal_pitch_kp;
extern float bal_pitch_kd;
extern float bal_roll_kp;
extern float bal_roll_kd;
extern float bal_max_corr;
extern float bal_max_pitch_rad;
extern float bal_max_roll_rad;
extern float bal_pitch_sign;
extern float bal_roll_sign;
extern float bal_last_pitch;
extern float bal_last_roll;
extern float bal_last_u_pitch;
extern float bal_last_u_roll;
extern int16_t bal_last_motor_l;
extern int16_t bal_last_motor_r;
extern uint32_t bal_stable_since;
extern uint32_t bal_last_debug_tick;

uint8_t balance_h30_online(void);
void balance_zero_now(void);
void balance_auto_update(void);
void balance_apply(int16_t *left, int16_t *right);

#endif
