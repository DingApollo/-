/**
 * @file    ps2.c
 * @brief   PS2 无线手柄：GPIO 软件模拟 SPI 时序读帧，摇杆/按键映射为左右轮指令
 * @note    引脚: PB0=DAT, PB1=CMD, PB12=CS, PB13=CLK；20ms 轮询，300ms 失联清零
 *          L1 键控制示警灯；手柄摇杆动作时暂停平衡修正（ps2_drive_quiet）
 *          依赖: delay_us(时序), motor/pid/balance/app_ctrl/lamp
 */
#include "ps2.h"
#include "delay_us.h"
#include "motor.h"
#include "pid.h"
#include "balance.h"
#include "app_state.h"
#include "app_ctrl.h"
#include "lamp.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

uint8_t ps2_enabled = 0;
uint8_t ps2_online = 0;
uint8_t ps2_debug_enabled = 0;
uint8_t ps2_mode = 0xFF;
uint8_t ps2_frame[9];
uint8_t ps2_lx = PS2_AXIS_CENTER;
uint8_t ps2_ly = PS2_AXIS_CENTER;
uint8_t ps2_rx = PS2_AXIS_CENTER;
uint8_t ps2_ry = PS2_AXIS_CENTER;
uint16_t ps2_buttons = 0xFFFFU;
uint16_t ps2_buttons_prev = 0xFFFFU;
int16_t ps2_cmd_left = 0;
int16_t ps2_cmd_right = 0;
uint32_t ps2_last_poll_tick = 0;
uint32_t ps2_last_ok_tick = 0;
uint32_t ps2_last_debug_tick = 0;
uint32_t ps2_last_config_tick = 0;

static const uint8_t ps2_cmd_read_data[9] = {0x01U, 0x42U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
static const uint8_t ps2_cmd_enter_config[5] = {0x01U, 0x43U, 0x00U, 0x01U, 0x00U};
static const uint8_t ps2_cmd_set_mode[9] = {0x01U, 0x44U, 0x00U, 0x01U, 0x03U, 0x00U, 0x00U, 0x00U, 0x00U};
static const uint8_t ps2_cmd_exit_config[9] = {0x01U, 0x43U, 0x00U, 0x00U, 0x5AU, 0x5AU, 0x5AU, 0x5AU, 0x5AU};
static uint8_t ps2_shift_byte(uint8_t tx)
{
  uint8_t rx = 0;

  for (uint8_t i = 0; i < 8U; i++) {
    if ((tx & (1U << i)) != 0U) {
      PS2_CMD_HIGH();
    } else {
      PS2_CMD_LOW();
    }

    PS2_CLK_LOW();
    delay_us(PS2_SHIFT_DELAY_US);

    if (PS2_DAT_READ() == GPIO_PIN_SET) {
      rx |= (uint8_t)(1U << i);
    }

    PS2_CLK_HIGH();
    delay_us(PS2_SHIFT_DELAY_US);
  }

  PS2_CMD_HIGH();
  delay_us(PS2_BYTE_DELAY_US);
  return rx;
}

static void ps2_transfer(const uint8_t *tx, uint8_t *rx, uint8_t len)
{
  PS2_CMD_HIGH();
  PS2_CLK_HIGH();
  PS2_CS_LOW();
  delay_us(PS2_BYTE_DELAY_US);

  for (uint8_t i = 0; i < len; i++) {
    uint8_t value = ps2_shift_byte(tx[i]);
    if (rx != NULL) {
      rx[i] = value;
    }
  }

  PS2_CS_HIGH();
  delay_us(PS2_BYTE_DELAY_US);
}

uint8_t ps2_read_frame(uint8_t *frame)
{
  memset(frame, 0xFF, 9U);
  ps2_transfer(ps2_cmd_read_data, frame, 9U);
  return (frame[1] == 0x41U || frame[1] == 0x73U || frame[1] == 0x79U) ? 1U : 0U;
}

uint8_t ps2_configure(void)
{
  uint8_t frame[9];

  ps2_last_config_tick = HAL_GetTick();
  ps2_online = 0U;
  ps2_mode = 0xFFU;

  (void)ps2_read_frame(frame);
  HAL_Delay(1);
  if (ps2_read_frame(frame) != 0U &&
      (((frame[1] & 0xF0U) == 0x70U) || frame[1] == 0x41U)) {
    ps2_online = 1U;
    ps2_mode = frame[1];
    ps2_last_ok_tick = HAL_GetTick();
    return 1U;
  }

  for (uint8_t retry = 0; retry < 5U; retry++) {
    ps2_transfer(ps2_cmd_enter_config, NULL, sizeof(ps2_cmd_enter_config));
    HAL_Delay(1);
    ps2_transfer(ps2_cmd_set_mode, NULL, sizeof(ps2_cmd_set_mode));
    HAL_Delay(1);
    ps2_transfer(ps2_cmd_exit_config, NULL, sizeof(ps2_cmd_exit_config));
    HAL_Delay(2);

    if (ps2_read_frame(frame) != 0U &&
        (((frame[1] & 0xF0U) == 0x70U) || frame[1] == 0x41U)) {
      ps2_online = 1U;
      ps2_mode = frame[1];
      ps2_last_ok_tick = HAL_GetTick();
      return 1U;
    }
  }

  return 0U;
}

static uint8_t ps2_axis_at_rest(uint8_t raw)
{
  int32_t d = (int32_t)raw - (int32_t)PS2_AXIS_CENTER;

  if (d < 0) {
    d = -d;
  }
  return (d <= (int32_t)PS2_BAL_QUIET_DEADZONE) ? 1U : 0U;
}

/* 摇杆在死区内且未按方向键，才允许 H30 平衡介入 */
uint8_t ps2_drive_quiet(void)
{
  if (ps2_online == 0U) {
    return 1U;
  }
  if (ps2_axis_at_rest(ps2_lx) == 0U || ps2_axis_at_rest(ps2_ly) == 0U) {
    return 0U;
  }
  if (PS2_BTN_PRESSED(ps2_buttons, PS2_BTN_UP) ||
      PS2_BTN_PRESSED(ps2_buttons, PS2_BTN_DOWN) ||
      PS2_BTN_PRESSED(ps2_buttons, PS2_BTN_LEFT) ||
      PS2_BTN_PRESSED(ps2_buttons, PS2_BTN_RIGHT))
  {
    return 0U;
  }
  return 1U;
}

static int16_t ps2_axis_to_pwm(uint8_t raw, uint8_t invert)
{
  int32_t centered;
  int32_t span;
  int32_t mag;
  int32_t scaled;

  if (invert != 0U) {
    centered = (int32_t)PS2_AXIS_CENTER - (int32_t)raw;
  } else {
    centered = (int32_t)raw - (int32_t)PS2_AXIS_CENTER;
  }

  if (centered > -PS2_AXIS_DEADZONE && centered < PS2_AXIS_DEADZONE) {
    return 0;
  }

#if PS2_AXIS_BINARY_OUTPUT
  return (centered > 0) ? (int16_t)PS2_PWM_MAX : (int16_t)(-PS2_PWM_MAX);
#endif

  if (centered > 0) {
    mag = centered - (int32_t)PS2_AXIS_DEADZONE;
    span = (int32_t)(255 - PS2_AXIS_CENTER) - (int32_t)PS2_AXIS_DEADZONE;
  } else {
    mag = -centered - (int32_t)PS2_AXIS_DEADZONE;
    span = (int32_t)PS2_AXIS_CENTER - (int32_t)PS2_AXIS_DEADZONE;
    mag = -mag;
  }

  if (span < 1) {
    span = 1;
  }

  scaled = (mag * (int32_t)PS2_PWM_MAX) / span;

  if (scaled > PS2_PWM_MAX) {
    scaled = PS2_PWM_MAX;
  }
  if (scaled < -PS2_PWM_MAX) {
    scaled = -PS2_PWM_MAX;
  }
  return (int16_t)scaled;
}

/* 差速合成后统一限幅，避免 |f|+|t| 时单侧先被 clamp 导致后退+转向偏慢 */
static void ps2_scale_forward_turn(int32_t *forward, int32_t *turn)
{
  int32_t left;
  int32_t right;
  int32_t max_w;
  int32_t abs_r;

  if (forward == NULL || turn == NULL) {
    return;
  }

  left = *forward + *turn;
  right = *forward - *turn;
  max_w = (left >= 0) ? left : -left;
  abs_r = (right >= 0) ? right : -right;
  if (abs_r > max_w) {
    max_w = abs_r;
  }

  if (max_w > (int32_t)PS2_PWM_MAX) {
    *forward = (*forward * (int32_t)PS2_PWM_MAX) / max_w;
    *turn = (*turn * (int32_t)PS2_PWM_MAX) / max_w;
  }
}

static void ps2_update_drive(void)
{
  int16_t forward = 0;
  int16_t turn = 0;
  int32_t left = 0;
  int32_t right = 0;

  if (ps2_online == 0U) {
    ps2_cmd_left = 0;
    ps2_cmd_right = 0;
    return;
  }

  if (PS2_BTN_PRESSED(ps2_buttons, PS2_BTN_UP) ||
      PS2_BTN_PRESSED(ps2_buttons, PS2_BTN_DOWN) ||
      PS2_BTN_PRESSED(ps2_buttons, PS2_BTN_LEFT) ||
      PS2_BTN_PRESSED(ps2_buttons, PS2_BTN_RIGHT))
  {
    if (PS2_BTN_PRESSED(ps2_buttons, PS2_BTN_UP) &&
        !PS2_BTN_PRESSED(ps2_buttons, PS2_BTN_DOWN)) {
      forward = (int16_t)PS2_PWM_MAX;
    } else if (PS2_BTN_PRESSED(ps2_buttons, PS2_BTN_DOWN) &&
               !PS2_BTN_PRESSED(ps2_buttons, PS2_BTN_UP)) {
      forward = (int16_t)(-PS2_PWM_MAX);
    }

    if (PS2_BTN_PRESSED(ps2_buttons, PS2_BTN_RIGHT) &&
        !PS2_BTN_PRESSED(ps2_buttons, PS2_BTN_LEFT)) {
      turn = (int16_t)PS2_PWM_MAX;
    } else if (PS2_BTN_PRESSED(ps2_buttons, PS2_BTN_LEFT) &&
               !PS2_BTN_PRESSED(ps2_buttons, PS2_BTN_RIGHT)) {
      turn = (int16_t)(-PS2_PWM_MAX);
    }
  } else {
  /* LX 前进、LY 转向；模拟量时斜向合成，经 ps2_scale_forward_turn 限幅使单侧不超 PS2_PWM_MAX */
#if PS2_AXIS_BINARY_OUTPUT && PS2_AXIS_CARDINAL_ONLY
  {
    int32_t fwd = (int32_t)PS2_AXIS_CENTER - (int32_t)ps2_lx;
    int32_t trn = (int32_t)ps2_ly - (int32_t)PS2_AXIS_CENTER;
    int32_t abs_fwd = (fwd >= 0) ? fwd : -fwd;
    int32_t abs_trn = (trn >= 0) ? trn : -trn;

    if (abs_fwd <= PS2_AXIS_DEADZONE) {
      fwd = 0;
      abs_fwd = 0;
    }
    if (abs_trn <= PS2_AXIS_DEADZONE) {
      trn = 0;
      abs_trn = 0;
    }

    if (abs_fwd == 0 && abs_trn == 0) {
      ps2_cmd_left = 0;
      ps2_cmd_right = 0;
      return;
    }

    if (abs_fwd >= abs_trn) {
      forward = (fwd > 0) ? (int16_t)PS2_PWM_MAX : (int16_t)(-PS2_PWM_MAX);
      turn = 0;
    } else {
      forward = 0;
      turn = (trn > 0) ? (int16_t)PS2_PWM_MAX : (int16_t)(-PS2_PWM_MAX);
    }
  }
#else
  forward = ps2_axis_to_pwm(ps2_lx, 1U);
  turn = ps2_axis_to_pwm(ps2_ly, 0U);
  turn = (int16_t)(((int32_t)turn * PS2_TURN_GAIN_NUM) / PS2_TURN_GAIN_DEN);
#endif
  }

  {
    int32_t fwd = (int32_t)forward;
    int32_t trn = (int32_t)turn;
    ps2_scale_forward_turn(&fwd, &trn);
    forward = (int16_t)fwd;
    turn = (int16_t)trn;
  }

  left = (int32_t)forward + (int32_t)turn;
  right = (int32_t)forward - (int32_t)turn;

  if (left > PS2_PWM_MAX) left = PS2_PWM_MAX;
  if (left < -PS2_PWM_MAX) left = -PS2_PWM_MAX;
  if (right > PS2_PWM_MAX) right = PS2_PWM_MAX;
  if (right < -PS2_PWM_MAX) right = -PS2_PWM_MAX;

  ps2_cmd_left = (int16_t)left;
  ps2_cmd_right = (int16_t)right;
}

static uint8_t ps2_btn_just_pressed(uint16_t mask)
{
  return (PS2_BTN_PRESSED(ps2_buttons, mask) &&
          !PS2_BTN_PRESSED(ps2_buttons_prev, mask)) ? 1U : 0U;
}

/* 无电脑现场测试：△ 开关平衡，□ 开关强测试模式；返回 1 表示本帧已急停勿再驱动 */
static uint8_t ps2_handle_buttons(void)
{
  if (ps2_btn_just_pressed(PS2_BTN_CROSS)) {
    estop_triggered = 1U;
    cmd_left = 0;
    cmd_right = 0;
    target_left = 0;
    target_right = 0;
    ps2_cmd_left = 0;
    ps2_cmd_right = 0;
    pid_reset_state();
    Motor_Stop();
    ps2_buttons_prev = ps2_buttons;
    return 1U;
  }

  if (ps2_btn_just_pressed(PS2_BTN_CIRCLE)) {
    estop_triggered = 0U;
    balance_zero_now();
    cmd_left = 0;
    cmd_right = 0;
    target_left = 0;
    target_right = 0;
    pid_reset_state();
    Motor_Stop();
  }

  if (ps2_btn_just_pressed(PS2_BTN_TRIANGLE)) {
    bal_enabled ^= 1U;
    bal_test_mode = 0U;
    bal_fallen = 0U;
    if (bal_enabled != 0U) {
      bal_auto_zero_pending = 1U;
      bal_stable_since = 0U;
      /* 立刻用当前姿态作零点，避免只按 △ 后约 0.5s 内晃 H30 无反应（STA=6） */
      if (balance_h30_online() != 0U) {
        balance_zero_now();
      } else {
        bal_zero_valid = 0U;
      }
    }
  }

  if (ps2_btn_just_pressed(PS2_BTN_SQUARE)) {
    bal_test_mode ^= 1U;
    bal_fallen = 0U;
    if (bal_test_mode != 0U) {
      bal_enabled = 1U;
      bal_auto_zero_pending = 1U;
      bal_zero_valid = 0U;
      bal_stable_since = 0U;
    }
  }

  if (ps2_btn_just_pressed(PS2_BTN_L1)) {
    char lamp_msg[16];
    int lamp_n;
    Lamp_Toggle();
    lamp_n = snprintf(lamp_msg, sizeof(lamp_msg), "LAMP:%u\r\n", (unsigned int)Lamp_IsOn());
    if (lamp_n > 0) {
      HAL_UART_Transmit(&huart3, (uint8_t *)lamp_msg, (uint16_t)lamp_n, 20);
    }
  }

  ps2_buttons_prev = ps2_buttons;
  return 0U;
}

void ps2_poll(void)
{
  uint32_t now = HAL_GetTick();

  if (ps2_read_frame(ps2_frame) != 0U &&
      (((ps2_frame[1] & 0xF0U) == 0x70U) || ps2_frame[1] == 0x41U)) {
    ps2_online = 1U;
    ps2_mode = ps2_frame[1];
    ps2_buttons = (uint16_t)(((uint16_t)ps2_frame[4] << 8) | ps2_frame[3]);
    if ((ps2_frame[1] & 0xF0U) == 0x70U) {
      ps2_rx = ps2_frame[5];
      ps2_ry = ps2_frame[6];
      ps2_lx = ps2_frame[7];
      ps2_ly = ps2_frame[8];
    } else {
      ps2_rx = PS2_AXIS_CENTER;
      ps2_ry = PS2_AXIS_CENTER;
      ps2_lx = PS2_AXIS_CENTER;
      ps2_ly = PS2_AXIS_CENTER;
    }
    ps2_last_ok_tick = now;

    if (ps2_handle_buttons() != 0U) {
      return;
    }

    ps2_update_drive();
    return;
  }

  ps2_mode = ps2_frame[1];
  if ((now - ps2_last_ok_tick) > PS2_FAILSAFE_MS) {
    ps2_online = 0U;
    ps2_cmd_left = 0;
    ps2_cmd_right = 0;
  }

  if ((now - ps2_last_config_tick) >= PS2_RECONFIG_PERIOD_MS) {
    (void)ps2_configure();
  }
}

void ps2_disable(void)
{
  ps2_enabled = 0U;
  ps2_online = 0U;
  ps2_debug_enabled = 0U;
  ps2_mode = 0xFFU;
  ps2_lx = PS2_AXIS_CENTER;
  ps2_ly = PS2_AXIS_CENTER;
  ps2_rx = PS2_AXIS_CENTER;
  ps2_ry = PS2_AXIS_CENTER;
  ps2_buttons = 0xFFFFU;
  ps2_buttons_prev = 0xFFFFU;
  ps2_cmd_left = 0;
  ps2_cmd_right = 0;
}

void ps2_clear_commands(void)
{
  ps2_cmd_left = 0;
  ps2_cmd_right = 0;
}
