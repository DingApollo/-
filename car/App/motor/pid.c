/**
 * @file    pid.c
 * @brief   双轮速度 PID 闭环（10ms 周期）+ 前馈；参数可存入片内 Flash
 * @note    编码器: TIM2/TIM4 正交解码，速度=每周期计数增量
 *          Flash 存参地址按 F407VET6(512KB) 的 Sector7 布局，换 VGT6 需改 pid.h
 *          依赖: main.h(PWM_MAX), HAL Flash 驱动
 */
#include "pid.h"
#include <string.h>
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"

uint8_t pid_enabled = 0;
int16_t target_left = 0;
int16_t target_right = 0;
int16_t speed_left = 0;
int16_t speed_right = 0;
int16_t pid_out_left = 0;
int16_t pid_out_right = 0;
int32_t enc_last_left = 0;
int32_t enc_last_right = 0;
float pid_kp = 18.0f;
float pid_ki = 0.8f;
float pid_kd = 0.0f;
float pid_kff = 55.0f;
float pid_kff_high = 105.0f;
float pid_kff_split = 20.0f;
float pid_i_left = 0.0f;
float pid_i_right = 0.0f;
float pid_prev_e_left = 0.0f;
float pid_prev_e_right = 0.0f;

static float pid_effective_kff(int16_t target);

int16_t pid_step(
  int16_t target,
  int16_t measured,
  float *i_term,
  float *prev_e
)
{
  const float dt_s = (float)PID_PERIOD_MS / 1000.0f;
  float e = (float)target - (float)measured;
  float p_term = 0.0f;
  float d_term = 0.0f;
  float ff_term = 0.0f;
  float out = 0.0f;

  p_term = pid_kp * e;

  if (e <= PID_I_SEP_ERR && e >= -PID_I_SEP_ERR) {
    *i_term += pid_ki * e * dt_s;
  }
  if (*i_term > (float)PWM_MAX) *i_term = (float)PWM_MAX;
  if (*i_term < -(float)PWM_MAX) *i_term = -(float)PWM_MAX;

  if (dt_s > 0.0f) {
    d_term = pid_kd * (e - *prev_e) / dt_s;
  }

  ff_term = pid_effective_kff(target) * (float)target;
  out = ff_term + p_term + (*i_term) + d_term;
  *prev_e = e;

  if (out > (float)PWM_MAX) out = (float)PWM_MAX;
  if (out < -(float)PWM_MAX) out = -(float)PWM_MAX;
  return (int16_t)out;
}

static float pid_effective_kff(int16_t target)
{
  int16_t abs_target = target;

  if (abs_target < 0) {
    abs_target = (int16_t)(-abs_target);
  }

  return ((float)abs_target <= pid_kff_split) ? pid_kff : pid_kff_high;
}

void pid_reset_state(void)
{
  pid_i_left = 0.0f;
  pid_i_right = 0.0f;
  pid_prev_e_left = 0.0f;
  pid_prev_e_right = 0.0f;
  pid_out_left = 0;
  pid_out_right = 0;
}

int16_t pid_map_user_speed(int16_t user_spd)
{
  int32_t mapped;

  if (user_spd == 0) {
    return 0;
  }

  mapped = ((int32_t)user_spd * (int32_t)PID_SPEED_FULL) / (int32_t)PID_USER_FULL;
  if (mapped > PID_SPEED_FULL) {
    mapped = PID_SPEED_FULL;
  } else if (mapped < -PID_SPEED_FULL) {
    mapped = -PID_SPEED_FULL;
  }
  return (int16_t)mapped;
}

void pid_load_params(void)
{
  const uint32_t *flash = (const uint32_t *)PID_PARAM_FLASH_ADDR;

  if (flash[0] == PID_PARAM_MAGIC) {
    memcpy(&pid_kp, &flash[1], sizeof(float));
    memcpy(&pid_ki, &flash[2], sizeof(float));
    memcpy(&pid_kd, &flash[3], sizeof(float));
    if (flash[5] != 0xFFFFFFFFUL) {
      memcpy(&pid_kff, &flash[5], sizeof(float));
    }
    if (flash[6] != 0xFFFFFFFFUL) {
      memcpy(&pid_kff_high, &flash[6], sizeof(float));
    }
    if (flash[7] != 0xFFFFFFFFUL) {
      memcpy(&pid_kff_split, &flash[7], sizeof(float));
    }
    if (flash[4] != 0xFFFFFFFFUL) {
      pid_enabled = (flash[4] & PID_FLAG_ENABLED) ? 1U : 0U;
    } else {
      pid_enabled = 0;
    }
  }
}

HAL_StatusTypeDef pid_save_params(void)
{
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t sector_error = 0;
  uint32_t words[8];
  HAL_StatusTypeDef st;

  words[0] = PID_PARAM_MAGIC;
  memcpy(&words[1], &pid_kp, sizeof(float));
  memcpy(&words[2], &pid_ki, sizeof(float));
  memcpy(&words[3], &pid_kd, sizeof(float));
  words[4] = pid_enabled ? PID_FLAG_ENABLED : 0UL;
  memcpy(&words[5], &pid_kff, sizeof(float));
  memcpy(&words[6], &pid_kff_high, sizeof(float));
  memcpy(&words[7], &pid_kff_split, sizeof(float));

  HAL_FLASH_Unlock();

  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  erase.Sector = PID_PARAM_FLASH_SECTOR;
  erase.NbSectors = 1;

  st = HAL_FLASHEx_Erase(&erase, &sector_error);
  if (st == HAL_OK) {
    uint32_t addr = PID_PARAM_FLASH_ADDR;
    for (uint32_t i = 0; i < 8; i++) {
      st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, words[i]);
      if (st != HAL_OK) {
        break;
      }
      addr += 4U;
    }
  }

  HAL_FLASH_Lock();
  return st;
}
