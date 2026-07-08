/**
 * @file    balance.c
 * @brief   平衡辅助：以 H30 俯仰/横滚角做 PD 修正并叠加到电机输出
 * @note    示警桩模式下横滚修正左右同向（BAL_ROLL_DIFFERENTIAL=0，非平衡车差速）
 *          含自动零点采集、角度/角速度死区、跌倒判定（超限且角速度小则停）
 *          依赖: h30.h(姿态源), main.h
 */
#include "balance.h"
#include "h30.h"
#include <string.h>
#include <math.h>
#include "main.h"

uint8_t bal_enabled = 0U;
uint8_t bal_debug_enabled = 0U;
uint8_t bal_test_mode = 0U;
uint8_t bal_output_active = 0U;
uint8_t bal_fallen = 0U;
uint8_t bal_zero_valid = 0U;
uint8_t bal_auto_zero_pending = 1U;
uint8_t bal_last_sta = 0U;
float bal_last_pitch_err = 0.0f;
float bal_last_roll_err = 0.0f;
float bal_pitch_zero = 0.0f;
float bal_roll_zero = 0.0f;
float bal_pitch_kp = BAL_PITCH_KP_DEFAULT;
float bal_pitch_kd = BAL_PITCH_KD_DEFAULT;
float bal_roll_kp = BAL_ROLL_KP_DEFAULT;
float bal_roll_kd = BAL_ROLL_KD_DEFAULT;
float bal_max_corr = BAL_MAX_CORR_DEFAULT;
float bal_max_pitch_rad = BAL_MAX_PITCH_RAD_DEFAULT;
float bal_max_roll_rad = BAL_MAX_ROLL_RAD_DEFAULT;
float bal_pitch_sign = 1.0f;
float bal_roll_sign = 1.0f;
float bal_last_pitch = 0.0f;
float bal_last_roll = 0.0f;
float bal_last_u_pitch = 0.0f;
float bal_last_u_roll = 0.0f;
int16_t bal_last_motor_l = 0;
int16_t bal_last_motor_r = 0;
uint32_t bal_stable_since = 0U;
uint32_t bal_last_debug_tick = 0U;

uint8_t balance_h30_online(void)
{
  if (h30_online == 0U) {
    return 0U;
  }
  if ((HAL_GetTick() - h30_last_frame_tick) > H30_ONLINE_TIMEOUT_MS) {
    return 0U;
  }
  return 1U;
}

static void balance_snapshot(
  float *pitch_rad,
  float *roll_rad,
  float *gyro_pitch,
  float *gyro_roll
)
{
  H30_FrameTypeDef frame_local;

  __disable_irq();
  memcpy(&frame_local, &h30_frame, sizeof(frame_local));
  __enable_irq();

  *pitch_rad = (float)frame_local.attitude.pitch * 0.000001f;
  *roll_rad = (float)frame_local.attitude.roll * 0.000001f;
  *gyro_pitch = (float)frame_local.gyro.gy * 0.000001f;
  *gyro_roll = (float)frame_local.gyro.gx * 0.000001f;
}

void balance_zero_now(void)
{
  float pitch_rad = 0.0f;
  float roll_rad = 0.0f;
  float gyro_pitch = 0.0f;
  float gyro_roll = 0.0f;

  if (balance_h30_online() == 0U) {
    return;
  }

  balance_snapshot(&pitch_rad, &roll_rad, &gyro_pitch, &gyro_roll);
  (void)gyro_pitch;
  (void)gyro_roll;
  bal_pitch_zero = pitch_rad;
  bal_roll_zero = roll_rad;
  bal_zero_valid = 1U;
  bal_auto_zero_pending = 0U;
  bal_fallen = 0U;
  bal_stable_since = HAL_GetTick();
}

void balance_auto_update(void)
{
  uint32_t now = HAL_GetTick();

  if (bal_enabled == 0U || bal_auto_zero_pending == 0U || bal_zero_valid != 0U) {
    return;
  }
  if (balance_h30_online() == 0U) {
    bal_stable_since = 0U;
    return;
  }

  if (bal_stable_since == 0U) {
    bal_stable_since = now;
    return;
  }

  if ((now - bal_stable_since) >= BAL_AUTO_ZERO_DELAY_MS) {
    balance_zero_now();
  }
}

static float balance_clampf(float v, float lo, float hi)
{
  if (v < lo) {
    return lo;
  }
  if (v > hi) {
    return hi;
  }
  return v;
}

/* 按 PD 输出线性映射到 BAL_PWM_MAX；倾角仅作小幅度兜底，避免略倾就顶满 PWM_MAX */
static int16_t balance_pd_to_pwm(float angle_err, float u_pd)
{
  float abs_u = fabsf(u_pd);
  float abs_a = fabsf(angle_err);
  float scale;
  int32_t mag;
  int16_t sign;

  if (abs_u < BAL_U_MIN_ACTIVE && abs_a < BAL_ANGLE_DEADBAND_RAD) {
    return 0;
  }

  scale = abs_u / bal_max_corr;
  if (scale < 0.05f && abs_a >= BAL_ANGLE_DEADBAND_RAD) {
    scale = (abs_a / BAL_ANGLE_FULL_RAD) * 0.35f;
  }
  if (scale > 1.0f) {
    scale = 1.0f;
  }
  if (scale <= 0.0f) {
    return 0;
  }

  mag = (int32_t)(scale * (float)BAL_PWM_MAX);
  if (mag > (int32_t)BAL_PWM_MAX) {
    mag = (int32_t)BAL_PWM_MAX;
  }
  if (mag <= 0) {
    return 0;
  }

  if (abs_u >= BAL_U_MIN_ACTIVE) {
    sign = (u_pd >= 0.0f) ? 1 : -1;
  } else {
    sign = (angle_err >= 0.0f) ? 1 : -1;
  }

  return (int16_t)(sign * mag);
}

void balance_apply(int16_t *left, int16_t *right)
{
  float pitch_rad;
  float roll_rad;
  float gyro_pitch;
  float gyro_roll;
  float pitch_err;
  float roll_err;
  float u_pitch;
  float u_roll;
  float corr_l;
  float corr_r;
  int32_t out_l;
  int32_t out_r;

  bal_last_sta = 0U;

  if (bal_enabled == 0U) {
    bal_last_sta = 1U;
    return;
  }
  if (balance_h30_online() == 0U) {
    bal_last_sta = 2U;
    return;
  }
  if (bal_zero_valid == 0U) {
    balance_zero_now();
    if (bal_zero_valid == 0U) {
      bal_last_sta = 6U;
      return;
    }
  }

  balance_snapshot(&pitch_rad, &roll_rad, &gyro_pitch, &gyro_roll);
  bal_last_pitch = pitch_rad;
  bal_last_roll = roll_rad;

  pitch_err = (pitch_rad - bal_pitch_zero) * bal_pitch_sign;
  roll_err = (roll_rad - bal_roll_zero) * bal_roll_sign;
  bal_last_pitch_err = pitch_err;
  bal_last_roll_err = roll_err;

  if (bal_test_mode == 0U &&
      (((fabsf(pitch_err) > bal_max_pitch_rad) &&
        (fabsf(gyro_pitch) < BAL_FALLEN_GYRO_MAX_RAD_S)) ||
       ((fabsf(roll_err) > bal_max_roll_rad) &&
        (fabsf(gyro_roll) < BAL_FALLEN_GYRO_MAX_RAD_S))))
  {
    bal_fallen = 1U;
    bal_last_sta = 3U;
    bal_last_u_pitch = 0.0f;
    bal_last_u_roll = 0.0f;
    bal_last_motor_l = *left;
    bal_last_motor_r = *right;
    return;
  }

  bal_fallen = 0U;

  /* 姿态与角速度均在死区内：H30 放着不动也不应起转 */
  if (bal_test_mode == 0U) {
    float gyro_mag = sqrtf(gyro_pitch * gyro_pitch + gyro_roll * gyro_roll);

    if ((fabsf(pitch_err) < BAL_ANGLE_DEADBAND_RAD) &&
        (fabsf(roll_err) < BAL_ANGLE_DEADBAND_RAD) &&
        (gyro_mag < BAL_GYRO_DEADBAND_RAD_S))
    {
      bal_last_sta = 4U;
      bal_last_u_pitch = 0.0f;
      bal_last_u_roll = 0.0f;
      bal_last_motor_l = *left;
      bal_last_motor_r = *right;
      return;
    }
  }

  u_pitch = bal_pitch_kp * pitch_err + bal_pitch_kd * gyro_pitch;
  u_roll = bal_roll_kp * roll_err + bal_roll_kd * gyro_roll;
  u_pitch = balance_clampf(u_pitch, -bal_max_corr, bal_max_corr);
  u_roll = balance_clampf(u_roll, -bal_max_corr, bal_max_corr);
  bal_last_u_pitch = u_pitch;
  bal_last_u_roll = u_roll;

  {
    int32_t corr_sum;
    int16_t pwm_pitch = balance_pd_to_pwm(pitch_err, u_pitch);
    int16_t pwm_roll = balance_pd_to_pwm(roll_err, u_roll);

#if BAL_ROLL_DIFFERENTIAL
    corr_l = (float)(pwm_pitch + pwm_roll);
    corr_r = (float)(pwm_pitch - pwm_roll);
#else
    corr_sum = (int32_t)pwm_pitch + (int32_t)pwm_roll;
    if (corr_sum > (int32_t)BAL_PWM_MAX) {
      corr_sum = (int32_t)BAL_PWM_MAX;
    } else if (corr_sum < -(int32_t)BAL_PWM_MAX) {
      corr_sum = -(int32_t)BAL_PWM_MAX;
    }
    corr_l = (float)corr_sum;
    corr_r = (float)corr_sum;
#endif
  }

  out_l = (int32_t)(*left) + (int32_t)corr_l;
  out_r = (int32_t)(*right) + (int32_t)corr_r;

  if (out_l > (int32_t)BAL_PWM_MAX) {
    out_l = (int32_t)BAL_PWM_MAX;
  } else if (out_l < -(int32_t)BAL_PWM_MAX) {
    out_l = -(int32_t)BAL_PWM_MAX;
  }
  if (out_r > (int32_t)BAL_PWM_MAX) {
    out_r = (int32_t)BAL_PWM_MAX;
  } else if (out_r < -(int32_t)BAL_PWM_MAX) {
    out_r = -(int32_t)BAL_PWM_MAX;
  }

  *left = (int16_t)out_l;
  *right = (int16_t)out_r;
  bal_last_motor_l = *left;
  bal_last_motor_r = *right;
  bal_last_sta = 5U;
}
