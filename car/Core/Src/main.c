/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdint.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#pragma pack(1)
typedef struct {
  uint8_t dataId;
  uint8_t dataLen;
  int32_t ax;
  int32_t ay;
  int32_t az;
} H30_AccelRawTypeDef;

typedef struct {
  uint8_t dataId;
  uint8_t dataLen;
  int32_t gx;
  int32_t gy;
  int32_t gz;
} H30_GyroRawTypeDef;

typedef struct {
  uint8_t dataId;
  uint8_t dataLen;
  int32_t pitch;
  int32_t roll;
  int32_t yaw;
} H30_AttitudeTypeDef;

typedef struct {
  uint8_t dataId;
  uint8_t dataLen;
  int32_t q0;
  int32_t q1;
  int32_t q2;
  int32_t q3;
} H30_QuaternionTypeDef;

typedef struct {
  uint8_t head1;
  uint8_t head2;
  uint16_t frameNum;
  uint8_t packLen;
  H30_AccelRawTypeDef accel;
  H30_GyroRawTypeDef gyro;
  H30_AttitudeTypeDef attitude;
  H30_QuaternionTypeDef quaternion;
  uint8_t ck1;
  uint8_t ck2;
} H30_FrameTypeDef;
#pragma pack()

typedef enum {
  H30_WAIT_HEAD = 0,
  H30_RECV_FRAME
} H30_StateTypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PWM_MAX 4199
/* GitHub 2500/2200/2800；本车 MG540 起转需更高占空比 */
#define PWM_START_MIN 4000
#define PWM_RUN_MIN   4000
#define PWM_START_BOOST 4199
#define PWM_START_BOOST_MS 200
#define PID_PERIOD_MS 10
/* 串口目标按 0~80 发，内部映射为编码器约 0~40（满速实测） */
#define PID_USER_FULL 80
#define PID_SPEED_FULL 40
#define PID_I_SEP_ERR 8.0f
#define PLOT_PERIOD_MS 100
#define PLOT_SEND_DIV (PLOT_PERIOD_MS / PID_PERIOD_MS)
#define PID_PARAM_MAGIC 0x50494431UL
/* F407VE 512KB：最后一扇区 Sector7 @ 0x08060000（勿用 0x080E0000，仅 1MB 芯片有效） */
#define PID_PARAM_FLASH_ADDR 0x08060000UL
#define PID_PARAM_FLASH_SECTOR FLASH_SECTOR_7
#define PID_FLAG_ENABLED 0x00000001UL
#define LEFT_FORWARD_CH   TIM_CHANNEL_1
#define LEFT_BACKWARD_CH  TIM_CHANNEL_2
/* 丝印 BIN1→PA11(TIM1_CH4)、BIN2→PA10(CH3)；与 GitHub CH3 正转不同 */
#define RIGHT_FORWARD_CH  TIM_CHANNEL_4
#define RIGHT_BACKWARD_CH TIM_CHANNEL_3
#define CTRL_SRC_UART 0U
#define CTRL_SRC_PS2  1U
#define PS2_SHIFT_DELAY_US 5U
#define PS2_BYTE_DELAY_US  4U
#define PS2_POLL_PERIOD_MS 20U
#define PS2_DEBUG_PERIOD_MS 200U
#define PS2_FAILSAFE_MS 300U
#define PS2_RECONFIG_PERIOD_MS 500U
#define H30_DEBUG_PERIOD_MS 200U
#define H30_FRAME_LEN 67U
#define H30_ONLINE_TIMEOUT_MS 500U
#define H30_UART_BAUD_DEFAULT 460800U
/* H30 姿态/角速度：原始值 ×1e-6 → 弧度 / 弧度每秒（与 H30? 一致） */
#define BAL_DEBUG_PERIOD_MS 200U
/* 增益单位：PWM/弧度；MG540 起转约 4000，0.2rad(约11°)≈满修正 */
#define BAL_PITCH_KP_DEFAULT 20000.0f
#define BAL_PITCH_KD_DEFAULT 500.0f
#define BAL_ROLL_KP_DEFAULT 10000.0f
#define BAL_ROLL_KD_DEFAULT 250.0f
#define BAL_MAX_CORR_DEFAULT 12000.0f
#define BAL_U_MIN_ACTIVE 8.0f
/* 小角度不输出，避免 MG540 在起转边缘反复嗡鸣；约 2.3° 后再介入 */
#define BAL_ANGLE_ACTIVE_RAD 0.040f
#define BAL_TEST_ANGLE_RAD 0.012f
#define BAL_PWM_MIN PWM_START_BOOST
#define BAL_AUTO_ENABLE_DEFAULT 1U
#define BAL_AUTO_ZERO_DELAY_MS 500U
#define BAL_MAX_PITCH_RAD_DEFAULT 0.785398f
#define BAL_MAX_ROLL_RAD_DEFAULT 0.610865f
#define PS2_AXIS_CENTER 128
#define PS2_AXIS_DEADZONE 18
#define PS2_PWM_MAX PWM_MAX
/* 转向增益：资料/ GitHub 为差速 left=f+t；1/1 避免左转偏弱 */
#define PS2_TURN_GAIN_NUM 1
#define PS2_TURN_GAIN_DEN 1
/* 手柄方向测试阶段：过死区即给同等幅值，保证前/后/左/右速度一致 */
#define PS2_AXIS_BINARY_OUTPUT 1
/* 二值模式只取主方向，避免摇杆串扰导致 forward/turn 互相抵消 */
#define PS2_AXIS_CARDINAL_ONLY 1
/* PS2 按键位为低有效：按下时对应位为 0 */
#define PS2_BTN_CROSS  0x4000U
#define PS2_BTN_CIRCLE 0x2000U
#define PS2_BTN_PRESSED(btn, mask) (((btn) & (mask)) == 0U)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define PS2_CLK_HIGH() HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, GPIO_PIN_SET)
#define PS2_CLK_LOW()  HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, GPIO_PIN_RESET)
#define PS2_CMD_HIGH() HAL_GPIO_WritePin(PS2_CMD_GPIO_Port, PS2_CMD_Pin, GPIO_PIN_SET)
#define PS2_CMD_LOW()  HAL_GPIO_WritePin(PS2_CMD_GPIO_Port, PS2_CMD_Pin, GPIO_PIN_RESET)
#define PS2_CS_HIGH()  HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_SET)
#define PS2_CS_LOW()   HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_RESET)
#define PS2_DAT_READ() HAL_GPIO_ReadPin(PS2_DAT_GPIO_Port, PS2_DAT_Pin)

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t estop_triggered = 0;
int16_t cmd_left = 0;
int16_t cmd_right = 0;
static uint8_t uart3_buf[32];
static char uart3_line_buf[96];
static uint8_t uart3_line_len = 0;
static volatile uint8_t uart3_cmd_ready = 0;
static char uart3_cmd_buf[96];
static uint8_t left_running = 0;
static uint8_t right_running = 0;
static int8_t left_dir = 0;
static int8_t right_dir = 0;
static uint32_t left_boost_until = 0;
static uint32_t right_boost_until = 0;
static uint8_t pid_enabled = 0;
static int16_t target_left = 0;
static int16_t target_right = 0;
static int16_t speed_left = 0;
static int16_t speed_right = 0;
static int16_t pid_out_left = 0;
static int16_t pid_out_right = 0;
static int32_t enc_last_left = 0;
static int32_t enc_last_right = 0;
static float pid_kp = 18.0f;
static float pid_ki = 0.8f;
static float pid_kd = 0.0f;
static float pid_kff = 55.0f;
static float pid_kff_high = 105.0f;
static float pid_kff_split = 20.0f;
static float pid_i_left = 0.0f;
static float pid_i_right = 0.0f;
static float pid_prev_e_left = 0.0f;
static float pid_prev_e_right = 0.0f;
static volatile uint8_t control_tick = 0;
static uint8_t plot_enabled = 0;
static uint8_t plot_div = 0;
static uint32_t plot_pause_until = 0;
static volatile uint32_t uart3_last_arm_tick = 0;
static volatile uint32_t uart3_last_rx_tick = 0;
static volatile uint32_t uart3_last_recover_tick = 0;
static volatile uint8_t uart3_need_rearm = 0;
static uint8_t ctrl_source = CTRL_SRC_UART;
static uint8_t ps2_enabled = 0;
static uint8_t ps2_online = 0;
static uint8_t ps2_debug_enabled = 0;
static uint8_t ps2_mode = 0xFF;
static uint8_t ps2_frame[9];
static uint8_t ps2_lx = PS2_AXIS_CENTER;
static uint8_t ps2_ly = PS2_AXIS_CENTER;
static uint8_t ps2_rx = PS2_AXIS_CENTER;
static uint8_t ps2_ry = PS2_AXIS_CENTER;
static uint16_t ps2_buttons = 0xFFFFU;
static int16_t ps2_cmd_left = 0;
static int16_t ps2_cmd_right = 0;
static uint32_t ps2_last_poll_tick = 0;
static uint32_t ps2_last_ok_tick = 0;
static uint32_t ps2_last_debug_tick = 0;
static uint32_t ps2_last_config_tick = 0;
static uint8_t h30_rx_byte = 0;
static uint8_t h30_frame_buf[H30_FRAME_LEN];
static uint16_t h30_frame_index = 0;
static uint8_t h30_last_rx = 0;
static volatile uint8_t h30_frame_ready = 0;
static volatile uint8_t h30_online = 0;
static uint8_t h30_debug_enabled = 0;
static H30_StateTypeDef h30_state = H30_WAIT_HEAD;
static H30_FrameTypeDef h30_frame;
static uint32_t h30_last_frame_tick = 0;
static uint32_t h30_last_debug_tick = 0;
static uint32_t h30_frame_count = 0;
static uint32_t h30_checksum_errors = 0;
static volatile uint32_t h30_rx_bytes = 0;
static uint32_t h30_sync_count = 0;
static uint8_t h30_raw_ring[16];
static uint8_t h30_raw_w = 0;
static uint32_t h30_uart_baud = H30_UART_BAUD_DEFAULT;
static uint8_t bal_enabled = 0U;
static uint8_t bal_debug_enabled = 0U;
static uint8_t bal_test_mode = 0U;
static uint8_t bal_output_active = 0U;
static uint8_t bal_fallen = 0U;
static uint8_t bal_zero_valid = 0U;
static uint8_t bal_auto_zero_pending = 1U;
static uint8_t bal_last_sta = 0U;
static float bal_last_pitch_err = 0.0f;
static float bal_last_roll_err = 0.0f;
static float bal_pitch_zero = 0.0f;
static float bal_roll_zero = 0.0f;
static float bal_pitch_kp = BAL_PITCH_KP_DEFAULT;
static float bal_pitch_kd = BAL_PITCH_KD_DEFAULT;
static float bal_roll_kp = BAL_ROLL_KP_DEFAULT;
static float bal_roll_kd = BAL_ROLL_KD_DEFAULT;
static float bal_max_corr = BAL_MAX_CORR_DEFAULT;
static float bal_max_pitch_rad = BAL_MAX_PITCH_RAD_DEFAULT;
static float bal_max_roll_rad = BAL_MAX_ROLL_RAD_DEFAULT;
static float bal_pitch_sign = 1.0f;
static float bal_roll_sign = 1.0f;
static float bal_last_pitch = 0.0f;
static float bal_last_roll = 0.0f;
static float bal_last_u_pitch = 0.0f;
static float bal_last_u_roll = 0.0f;
static int16_t bal_last_motor_l = 0;
static int16_t bal_last_motor_r = 0;
static uint32_t bal_stable_since = 0U;
static uint32_t bal_last_debug_tick = 0U;
static uint8_t h30_boot_reported = 0U;
static uint32_t h30_watch_tick = 0U;
static uint8_t dwt_delay_ready = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static uint16_t clamp_pwm(int16_t pwm);
static int16_t apply_deadzone_comp(
  int16_t pwm,
  uint8_t *running_flag,
  int8_t *dir_flag,
  uint32_t *boost_until
);
static int16_t pid_step(
  int16_t target,
  int16_t measured,
  float *i_term,
  float *prev_e
);
static float pid_effective_kff(int16_t target);
static void pid_reset_state(void);
static int16_t pid_map_user_speed(int16_t user_spd);
static int16_t ps2_pwm_to_pid_target(int16_t pwm);
static void pid_load_params(void);
static HAL_StatusTypeDef pid_save_params(void);
static HAL_StatusTypeDef uart3_start_rx_to_idle(void);
static HAL_StatusTypeDef h30_start_rx(void);
static HAL_StatusTypeDef h30_uart_set_baud(uint32_t baud);
static uint16_t h30_checksum(const uint8_t *data, uint16_t len);
static uint8_t h30_process_byte(uint8_t byte);
static void h30_reset_parser(void);
static void h30_push_raw_byte(uint8_t byte);
static void h30_ensure_rx(void);
static void h30_boot_report(void);
static void h30_scan_baud(void);
static void uart3_soft_recover(void);
static void Motor_SetLeft(int16_t pwm);
static void Motor_SetRight(int16_t pwm);
static void Motor_Set(int16_t left, int16_t right);
static void Motor_Stop(void);
static int parse_two_ints(const char *s, int *out_l, int *out_r);
static void VOFA_HandleLine(const char *line);
static void dwt_delay_init(void);
static void delay_us(uint32_t us);
static uint8_t ps2_shift_byte(uint8_t tx);
static void ps2_transfer(const uint8_t *tx, uint8_t *rx, uint8_t len);
static uint8_t ps2_axis_at_rest(uint8_t raw);
static uint8_t ps2_read_frame(uint8_t *frame);
static uint8_t ps2_configure(void);
static int16_t ps2_axis_to_pwm(uint8_t raw, uint8_t invert);
static void ps2_scale_forward_turn(int32_t *forward, int32_t *turn);
static void ps2_update_drive(void);
static void ps2_poll(void);
static void ps2_disable(void);
static const char *ctrl_source_name(void);
static uint8_t balance_h30_online(void);
static void balance_snapshot(
  float *pitch_rad,
  float *roll_rad,
  float *gyro_pitch,
  float *gyro_roll
);
static void balance_zero_now(void);
static void balance_auto_update(void);
static void balance_apply(int16_t *left, int16_t *right);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static const uint8_t ps2_cmd_read_data[9] = {0x01U, 0x42U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
static const uint8_t ps2_cmd_enter_config[5] = {0x01U, 0x43U, 0x00U, 0x01U, 0x00U};
static const uint8_t ps2_cmd_set_mode[9] = {0x01U, 0x44U, 0x00U, 0x01U, 0x03U, 0x00U, 0x00U, 0x00U, 0x00U};
static const uint8_t ps2_cmd_exit_config[9] = {0x01U, 0x43U, 0x00U, 0x00U, 0x5AU, 0x5AU, 0x5AU, 0x5AU, 0x5AU};

static void dwt_delay_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  dwt_delay_ready = ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U) ? 1U : 0U;
}

static void delay_us(uint32_t us)
{
  if ((us == 0U) || (SystemCoreClock == 0U)) {
    return;
  }

  if (dwt_delay_ready != 0U) {
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = (SystemCoreClock / 1000000U) * us;
    while ((DWT->CYCCNT - start) < ticks) {
    }
  } else {
    volatile uint32_t loops = (SystemCoreClock / 8000000U) * us;
    while (loops-- > 0U) {
      __NOP();
    }
  }
}

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

static uint8_t ps2_read_frame(uint8_t *frame)
{
  memset(frame, 0xFF, 9U);
  ps2_transfer(ps2_cmd_read_data, frame, 9U);
  return (frame[1] == 0x41U || frame[1] == 0x73U || frame[1] == 0x79U) ? 1U : 0U;
}

static uint8_t ps2_configure(void)
{
  uint8_t frame[9];

  ps2_last_config_tick = HAL_GetTick();
  ps2_online = 0U;
  ps2_mode = 0xFFU;

  (void)ps2_read_frame(frame);
  HAL_Delay(1);
  if (ps2_read_frame(frame) != 0U && ((frame[1] & 0xF0U) == 0x70U)) {
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

    if (ps2_read_frame(frame) != 0U && ((frame[1] & 0xF0U) == 0x70U)) {
      ps2_online = 1U;
      ps2_mode = frame[1];
      ps2_last_ok_tick = HAL_GetTick();
      return 1U;
    }
  }

  return 0U;
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

  /* LX 前进、LY 转向（GitHub d89467d 差速模型） */
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

static void ps2_poll(void)
{
  if (ps2_read_frame(ps2_frame) != 0U && ((ps2_frame[1] & 0xF0U) == 0x70U)) {
    ps2_online = 1U;
    ps2_mode = ps2_frame[1];
    ps2_buttons = (uint16_t)(((uint16_t)ps2_frame[4] << 8) | ps2_frame[3]);
    ps2_rx = ps2_frame[5];
    ps2_ry = ps2_frame[6];
    ps2_lx = ps2_frame[7];
    ps2_ly = ps2_frame[8];
    ps2_last_ok_tick = HAL_GetTick();

    if (PS2_BTN_PRESSED(ps2_buttons, PS2_BTN_CIRCLE)) {
      estop_triggered = 0U;
      balance_zero_now();
      cmd_left = 0;
      cmd_right = 0;
      target_left = 0;
      target_right = 0;
      pid_reset_state();
      Motor_Stop();
    } else if (PS2_BTN_PRESSED(ps2_buttons, PS2_BTN_CROSS)) {
      estop_triggered = 1U;
      cmd_left = 0;
      cmd_right = 0;
      target_left = 0;
      target_right = 0;
      ps2_cmd_left = 0;
      ps2_cmd_right = 0;
      pid_reset_state();
      Motor_Stop();
      return;
    }

    ps2_update_drive();
    return;
  }

  ps2_online = 0U;
  ps2_mode = ps2_frame[1];
  ps2_cmd_left = 0;
  ps2_cmd_right = 0;

  if ((HAL_GetTick() - ps2_last_config_tick) >= PS2_RECONFIG_PERIOD_MS) {
    (void)ps2_configure();
  }
}

static void ps2_disable(void)
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
  ps2_cmd_left = 0;
  ps2_cmd_right = 0;
}

static const char *ctrl_source_name(void)
{
  return (ctrl_source == CTRL_SRC_PS2) ? "PS2" : "UART";
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  /* TIM1 须开主输出 MOE，否则 PA8~PA11 无波形（万用表仅 ~0.1V） */
  __HAL_TIM_MOE_ENABLE(&htim1);
  htim1.Instance->BDTR |= TIM_BDTR_MOE;
  htim1.Instance->CR1 |= TIM_CR1_CEN;
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim3);
  Motor_Stop();
  pid_load_params();
  enc_last_left = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
  enc_last_right = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
  uart3_last_rx_tick = HAL_GetTick();
  uart3_last_recover_tick = HAL_GetTick();
  dwt_delay_init();
  ps2_last_ok_tick = HAL_GetTick();
  ps2_last_config_tick = HAL_GetTick();
  if (uart3_start_rx_to_idle() != HAL_OK) {
    uart3_need_rearm = 1;
  }
  if (h30_uart_set_baud(H30_UART_BAUD_DEFAULT) != HAL_OK) {
    (void)MX_USART2_UART_Init();
    (void)h30_start_rx();
  }
#if BAL_AUTO_ENABLE_DEFAULT
  bal_enabled = 1U;
  bal_test_mode = 0U;
  bal_auto_zero_pending = 1U;
  bal_zero_valid = 0U;
  bal_stable_since = 0U;
#endif
  {
    char boot_msg[128];
    int boot_n = snprintf(
      boot_msg,
      sizeof(boot_msg),
      "BOOT:USART3,115200,H30=%lu,READY,RST=%s%s%s%s%s%s%s\r\n",
      (unsigned long)H30_UART_BAUD_DEFAULT,
      __HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) ? "PIN|" : "",
      __HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) ? "POR|" : "",
      __HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) ? "SFT|" : "",
      __HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) ? "IWDG|" : "",
      __HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) ? "WWDG|" : "",
      __HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) ? "BOR|" : "",
      __HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) ? "LPWR|" : ""
    );
    if (boot_n > 0) {
      HAL_UART_Transmit(&huart3, (uint8_t *)boot_msg, (uint16_t)boot_n, 30);
    }
    __HAL_RCC_CLEAR_RESET_FLAGS();
  }

  /* 上电默认手柄：握手成功则 CTRL=PS2（等价手动 PS2EN=1 + CTRL=PS2）；失败保持 UART */
  if (ps2_configure() != 0U) {
    ps2_enabled = 1U;
    ctrl_source = CTRL_SRC_PS2;
    pid_enabled = 0U;
    pid_reset_state();
    ps2_last_poll_tick = 0U;
    Motor_Stop();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (ps2_enabled == 0U &&
        (HAL_GetTick() - ps2_last_config_tick) >= PS2_RECONFIG_PERIOD_MS) {
      if (ps2_configure() != 0U) {
        ps2_enabled = 1U;
        ctrl_source = CTRL_SRC_PS2;
        pid_enabled = 0U;
        pid_reset_state();
        ps2_last_poll_tick = 0U;
        Motor_Stop();
      }
    }

    if (ps2_enabled != 0U &&
        (HAL_GetTick() - ps2_last_poll_tick) >= PS2_POLL_PERIOD_MS) {
      ps2_last_poll_tick = HAL_GetTick();
      ps2_poll();
    }

    if (ps2_enabled != 0U &&
        ps2_debug_enabled != 0U &&
        !plot_enabled &&
        ((HAL_GetTick() - ps2_last_debug_tick) >= PS2_DEBUG_PERIOD_MS))
    {
      char ps2_line[128];
      int ps2_n = snprintf(
        ps2_line,
        sizeof(ps2_line),
        "PS2:EN=%u,ONLINE=%u,MODE=0x%02X,LX=%u,LY=%u,RX=%u,RY=%u,BTN=0x%04X,CTRL=%s,CMD=%d,%d\r\n",
        (unsigned int)ps2_enabled,
        (unsigned int)ps2_online,
        (unsigned int)ps2_mode,
        (unsigned int)ps2_lx,
        (unsigned int)ps2_ly,
        (unsigned int)ps2_rx,
        (unsigned int)ps2_ry,
        (unsigned int)ps2_buttons,
        ctrl_source_name(),
        ps2_cmd_left,
        ps2_cmd_right
      );
      ps2_last_debug_tick = HAL_GetTick();
      if (ps2_n > 0) {
        HAL_UART_Transmit(&huart3, (uint8_t *)ps2_line, (uint16_t)ps2_n, 40);
      }
    }

    if (h30_online != 0U &&
        ((HAL_GetTick() - h30_last_frame_tick) > H30_ONLINE_TIMEOUT_MS)) {
      h30_online = 0U;
    }

    if (h30_boot_reported == 0U && (HAL_GetTick() >= 800U)) {
      h30_boot_reported = 1U;
      h30_boot_report();
    }

    if ((HAL_GetTick() - h30_watch_tick) >= 1000U) {
      h30_watch_tick = HAL_GetTick();
      h30_ensure_rx();
    }

    balance_auto_update();

    if (h30_debug_enabled != 0U &&
        !plot_enabled &&
        ((HAL_GetTick() - h30_last_debug_tick) >= H30_DEBUG_PERIOD_MS))
    {
      H30_FrameTypeDef frame_local;
      char h30_line[160];
      int h30_n = 0;

      __disable_irq();
      memcpy(&frame_local, &h30_frame, sizeof(frame_local));
      h30_frame_ready = 0U;
      __enable_irq();

      h30_last_debug_tick = HAL_GetTick();
      if (h30_online != 0U) {
        h30_n = snprintf(
          h30_line,
          sizeof(h30_line),
          "H30:1,CNT:%lu,ERR:%lu,PYR:%.3f,%.3f,%.3f,GYR:%.3f,%.3f,%.3f,ACC:%.3f,%.3f,%.3f\r\n",
          (unsigned long)h30_frame_count,
          (unsigned long)h30_checksum_errors,
          (double)frame_local.attitude.pitch * 0.000001,
          (double)frame_local.attitude.roll * 0.000001,
          (double)frame_local.attitude.yaw * 0.000001,
          (double)frame_local.gyro.gx * 0.000001,
          (double)frame_local.gyro.gy * 0.000001,
          (double)frame_local.gyro.gz * 0.000001,
          (double)frame_local.accel.ax * 0.000001,
          (double)frame_local.accel.ay * 0.000001,
          (double)frame_local.accel.az * 0.000001
        );
      } else {
        uint32_t rx_total = h30_rx_bytes;
        h30_n = snprintf(
          h30_line,
          sizeof(h30_line),
          "H30:0,RX:%lu,CNT:%lu,ERR:%lu,SYNC:%lu,BAUD:%lu (无有效帧,查线/YIS=460800)\r\n",
          (unsigned long)rx_total,
          (unsigned long)h30_frame_count,
          (unsigned long)h30_checksum_errors,
          (unsigned long)h30_sync_count,
          (unsigned long)h30_uart_baud
        );
      }
      if (h30_n > 0) {
        uint16_t tx_len = (h30_n < (int)sizeof(h30_line)) ? (uint16_t)h30_n : (uint16_t)(sizeof(h30_line) - 1U);
        HAL_UART_Transmit(&huart3, (uint8_t *)h30_line, tx_len, 40);
      }
    }

    if (bal_debug_enabled != 0U &&
        bal_enabled != 0U &&
        !plot_enabled &&
        ((HAL_GetTick() - bal_last_debug_tick) >= BAL_DEBUG_PERIOD_MS))
    {
      char bal_line[160];
      int bal_n = 0;

      bal_last_debug_tick = HAL_GetTick();
      bal_n = snprintf(
        bal_line,
        sizeof(bal_line),
        "BAL:1,STA:%u,ZV:%u,FALL:%u,PE:%.3f,RE:%.3f,PIT:%.3f,ROL:%.3f,MOT:%d,%d\r\n",
        (unsigned int)bal_last_sta,
        (unsigned int)bal_zero_valid,
        (unsigned int)bal_fallen,
        (double)bal_last_pitch_err,
        (double)bal_last_roll_err,
        (double)bal_last_pitch,
        (double)bal_last_roll,
        bal_last_motor_l,
        bal_last_motor_r
      );
      if (bal_n > 0) {
        uint16_t tx_len = (bal_n < (int)sizeof(bal_line)) ? (uint16_t)bal_n : (uint16_t)(sizeof(bal_line) - 1U);
        HAL_UART_Transmit(&huart3, (uint8_t *)bal_line, tx_len, 40);
      }
    }

    if (uart3_cmd_ready != 0U) {
      char line_local[96];

      __disable_irq();
      memcpy(line_local, uart3_cmd_buf, sizeof(line_local));
      uart3_cmd_ready = 0;
      __enable_irq();

      VOFA_HandleLine(line_local);
    }

    if (control_tick) {
      control_tick = 0;
      {
        int32_t enc_now_left = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
        int32_t enc_now_right = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
        speed_left = (int16_t)(enc_now_left - enc_last_left);
        speed_right = (int16_t)(enc_now_right - enc_last_right);
        enc_last_left = enc_now_left;
        enc_last_right = enc_now_right;
      }

      if (!estop_triggered) {
        if (ctrl_source == CTRL_SRC_PS2) {
          if ((ps2_enabled == 0U) ||
              (ps2_online == 0U) ||
              ((HAL_GetTick() - ps2_last_ok_tick) > PS2_FAILSAFE_MS))
          {
            ps2_cmd_left = 0;
            ps2_cmd_right = 0;
            cmd_left = 0;
            cmd_right = 0;
            target_left = 0;
            target_right = 0;
            pid_reset_state();
          } else {
            if (pid_enabled) {
              target_left = ps2_pwm_to_pid_target(ps2_cmd_left);
              target_right = ps2_pwm_to_pid_target(ps2_cmd_right);

              if (target_left == 0 && target_right == 0) {
                pid_i_left = 0.0f;
                pid_i_right = 0.0f;
                pid_prev_e_left = 0.0f;
                pid_prev_e_right = 0.0f;
                cmd_left = 0;
                cmd_right = 0;
              } else {
                pid_out_left = pid_step(target_left, speed_left, &pid_i_left, &pid_prev_e_left);
                pid_out_right = pid_step(target_right, speed_right, &pid_i_right, &pid_prev_e_right);
                cmd_left = pid_out_left;
                cmd_right = pid_out_right;
              }
            } else {
              cmd_left = ps2_cmd_left;
              cmd_right = ps2_cmd_right;
              target_left = 0;
              target_right = 0;
            }
          }
        } else if (pid_enabled) {
          if (target_left == 0 && target_right == 0) {
            pid_i_left = 0.0f;
            pid_i_right = 0.0f;
            pid_prev_e_left = 0.0f;
            pid_prev_e_right = 0.0f;
            cmd_left = 0;
            cmd_right = 0;
          } else {
            pid_out_left = pid_step(target_left, speed_left, &pid_i_left, &pid_prev_e_left);
            pid_out_right = pid_step(target_right, speed_right, &pid_i_right, &pid_prev_e_right);
            cmd_left = pid_out_left;
            cmd_right = pid_out_right;
          }
        }
        {
          int16_t motor_l = cmd_left;
          int16_t motor_r = cmd_right;
          uint8_t balance_allowed = 1U;

          /*
           * 手柄命令优先：摇杆正在给前进/后退/转向时，保持原差速比例。
           * H30 在摇杆回中时介入，避免把已调好的左转/后退速度重新拉偏。
           */
          if (ctrl_source == CTRL_SRC_PS2 &&
              bal_test_mode == 0U &&
              (cmd_left != 0 || cmd_right != 0))
          {
            balance_allowed = 0U;
            bal_last_sta = 7U;
            bal_last_u_pitch = 0.0f;
            bal_last_u_roll = 0.0f;
            bal_last_motor_l = motor_l;
            bal_last_motor_r = motor_r;
          }

          if (balance_allowed != 0U) {
            balance_apply(&motor_l, &motor_r);
          }
          bal_output_active = (motor_l != cmd_left || motor_r != cmd_right) ? 1U : 0U;
          Motor_Set(motor_l, motor_r);
          bal_output_active = 0U;
        }
      } else {
        Motor_Stop();
      }

      if (plot_enabled) {
        if (HAL_GetTick() >= plot_pause_until) {
          plot_div++;
          if (plot_div >= PLOT_SEND_DIV) {
            char plot_line[64];
            int plot_n = 0;
            plot_div = 0;
            plot_n = snprintf(
              plot_line,
              sizeof(plot_line),
              "%d,%d,%d,%d\r\n",
              target_left,
              speed_left,
              target_right,
              speed_right
            );
            if (plot_n > 0) {
              HAL_UART_Transmit(&huart3, (uint8_t *)plot_line, (uint16_t)plot_n, 20);
            }
          }
        }
      }
    }

    /* 串口接收自恢复：避免偶发状态机卡死必须按RST */
    if ((uart3_need_rearm != 0U) ||
        ((huart3.RxState == HAL_UART_STATE_READY) && ((HAL_GetTick() - uart3_last_arm_tick) > 100U)))
    {
      if (uart3_start_rx_to_idle() == HAL_OK) {
        uart3_need_rearm = 0;
      }
    }

    /* 仅在接收启动失败或硬件报错时软恢复，避免空闲时反复重初始化 USART */
    if (((HAL_GetTick() - uart3_last_recover_tick) > 1000U) &&
        ((uart3_need_rearm != 0U) || (huart3.ErrorCode != HAL_UART_ERROR_NONE)))
    {
      uart3_soft_recover();
    }

    HAL_Delay(1);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static uint16_t clamp_pwm(int16_t pwm)
{
  if (pwm < 0) {
    pwm = -pwm;
  }
  if (pwm > PWM_MAX) {
    pwm = PWM_MAX;
  }
  return (uint16_t)pwm;
}

static int16_t apply_deadzone_comp(
  int16_t pwm,
  uint8_t *running_flag,
  int8_t *dir_flag,
  uint32_t *boost_until
)
{
  int16_t sign = 1;
  int16_t mag = 0;
  uint16_t min_pwm = 0;
  uint32_t now = HAL_GetTick();

  if (pwm == 0) {
    *running_flag = 0;
    *dir_flag = 0;
    *boost_until = 0;
    return 0;
  }

  if (pwm < 0) {
    sign = -1;
    mag = (int16_t)(-pwm);
  } else {
    mag = pwm;
  }

  if ((*running_flag == 0) || (*dir_flag != (int8_t)sign)) {
    *running_flag = 1;
    *dir_flag = (int8_t)sign;
    *boost_until = now + PWM_START_BOOST_MS;
  }

  if (now < *boost_until) {
    min_pwm = PWM_START_BOOST;
  } else {
    min_pwm = PWM_RUN_MIN;
  }

  if (min_pwm < PWM_START_MIN) {
    min_pwm = PWM_START_MIN;
  }

  if (mag < (int16_t)min_pwm) {
    mag = (int16_t)min_pwm;
  }
  if (mag > PWM_MAX) {
    mag = PWM_MAX;
  }
  return (int16_t)(sign * mag);
}

static int16_t pid_step(
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

static void pid_reset_state(void)
{
  pid_i_left = 0.0f;
  pid_i_right = 0.0f;
  pid_prev_e_left = 0.0f;
  pid_prev_e_right = 0.0f;
  pid_out_left = 0;
  pid_out_right = 0;
}

static int16_t pid_map_user_speed(int16_t user_spd)
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

static int16_t ps2_pwm_to_pid_target(int16_t pwm)
{
  int32_t user_spd;

  if (pwm == 0) {
    return 0;
  }

  user_spd = ((int32_t)pwm * (int32_t)PID_USER_FULL) / (int32_t)PS2_PWM_MAX;
  if (user_spd > PID_USER_FULL) {
    user_spd = PID_USER_FULL;
  } else if (user_spd < -PID_USER_FULL) {
    user_spd = -PID_USER_FULL;
  }

  return pid_map_user_speed((int16_t)user_spd);
}

static void pid_load_params(void)
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

static HAL_StatusTypeDef pid_save_params(void)
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

static HAL_StatusTypeDef uart3_start_rx_to_idle(void)
{
  HAL_StatusTypeDef st;

  st = HAL_UARTEx_ReceiveToIdle_IT(&huart3, uart3_buf, sizeof(uart3_buf));
  if (st == HAL_OK) {
    uart3_last_arm_tick = HAL_GetTick();
  }
  return st;
}

static HAL_StatusTypeDef h30_start_rx(void)
{
  if (huart2.gState != HAL_UART_STATE_READY) {
    (void)HAL_UART_AbortReceive_IT(&huart2);
  }
  return HAL_UART_Receive_IT(&huart2, &h30_rx_byte, 1U);
}

static void h30_ensure_rx(void)
{
  if (huart2.RxState == HAL_UART_STATE_READY) {
    (void)h30_start_rx();
  }
}

static void h30_boot_report(void)
{
  char line[96];
  int n = snprintf(
    line,
    sizeof(line),
    "H30BOOT:RX:%lu,CNT:%lu,SYNC:%lu,BAUD:%lu,RXST:%u (无数据发H30SCAN)\r\n",
    (unsigned long)h30_rx_bytes,
    (unsigned long)h30_frame_count,
    (unsigned long)h30_sync_count,
    (unsigned long)h30_uart_baud,
    (unsigned int)huart2.RxState
  );
  if (n > 0) {
    uint16_t tx_len = (n < (int)sizeof(line)) ? (uint16_t)n : (uint16_t)(sizeof(line) - 1U);
    HAL_UART_Transmit(&huart3, (uint8_t *)line, tx_len, 40);
  }
}

static void h30_scan_baud(void)
{
  static const uint32_t rates[] = {921600U, 460800U, 115200U, 9600U};
  char line[128];
  uint32_t best_baud = 0U;
  uint32_t best_frames = 0U;
  uint8_t i = 0U;
  int n = 0;

  for (i = 0U; i < (uint8_t)(sizeof(rates) / sizeof(rates[0])); i++) {
    uint32_t baud = rates[i];
    uint32_t rx0 = h30_rx_bytes;
    uint32_t cnt0 = h30_frame_count;
    uint32_t sync0 = h30_sync_count;

    if (h30_uart_set_baud(baud) != HAL_OK) {
      continue;
    }
    HAL_Delay(350);

    {
      uint32_t drx = h30_rx_bytes - rx0;
      uint32_t dcnt = h30_frame_count - cnt0;
      uint32_t dsync = h30_sync_count - sync0;

      n = snprintf(
        line,
        sizeof(line),
        "H30SCAN:%lu,RX+%lu,CNT+%lu,SYNC+%lu\r\n",
        (unsigned long)baud,
        (unsigned long)drx,
        (unsigned long)dcnt,
        (unsigned long)dsync
      );
      if (n > 0) {
        HAL_UART_Transmit(&huart3, (uint8_t *)line, (uint16_t)n, 40);
      }

      if (dcnt > best_frames) {
        best_frames = dcnt;
        best_baud = baud;
      } else if (dcnt == best_frames && dcnt > 0U && dsync > 0U) {
        best_baud = baud;
      }
    }
  }

  if (best_baud != 0U) {
    (void)h30_uart_set_baud(best_baud);
  }

  n = snprintf(
    line,
    sizeof(line),
    "H30SCAN:DONE,BEST:%lu,CNT+%lu\r\n",
    (unsigned long)best_baud,
    (unsigned long)best_frames
  );
  if (n > 0) {
    HAL_UART_Transmit(&huart3, (uint8_t *)line, (uint16_t)n, 40);
  }
}

static HAL_StatusTypeDef h30_uart_set_baud(uint32_t baud)
{
  if (baud < 9600U || baud > 921600U) {
    return HAL_ERROR;
  }

  (void)HAL_UART_AbortReceive_IT(&huart2);
  (void)HAL_UART_DeInit(&huart2);

  huart2.Init.BaudRate = baud;
  if (HAL_UART_Init(&huart2) != HAL_OK) {
    return HAL_ERROR;
  }

  h30_uart_baud = baud;
  h30_last_rx = 0U;
  h30_reset_parser();
  return h30_start_rx();
}

static uint16_t h30_checksum(const uint8_t *data, uint16_t len)
{
  uint8_t ck1 = 0U;
  uint8_t ck2 = 0U;

  for (uint16_t i = 0; i < len; i++) {
    ck1 = (uint8_t)(ck1 + data[i]);
    ck2 = (uint8_t)(ck2 + ck1);
  }

  return (uint16_t)(((uint16_t)ck1 << 8) | ck2);
}

static void h30_reset_parser(void)
{
  h30_state = H30_WAIT_HEAD;
  h30_frame_index = 0U;
}

static void h30_push_raw_byte(uint8_t byte)
{
  h30_raw_ring[h30_raw_w] = byte;
  h30_raw_w = (uint8_t)((h30_raw_w + 1U) % (uint8_t)sizeof(h30_raw_ring));
}

static uint8_t h30_process_byte(uint8_t byte)
{
  uint16_t checksum_val = 0U;

  switch (h30_state) {
    case H30_WAIT_HEAD:
      if (h30_last_rx == 0x59U && byte == 0x53U) {
        h30_frame_buf[0] = 0x59U;
        h30_frame_buf[1] = 0x53U;
        h30_frame_index = 2U;
        h30_state = H30_RECV_FRAME;
        h30_sync_count++;
      }
      break;

    case H30_RECV_FRAME:
      if (h30_frame_index < H30_FRAME_LEN) {
        h30_frame_buf[h30_frame_index++] = byte;
      }

      if (h30_frame_index >= H30_FRAME_LEN) {
        checksum_val = h30_checksum(&h30_frame_buf[2], (uint16_t)(H30_FRAME_LEN - 4U));
        if (((checksum_val >> 8) & 0xFFU) == h30_frame_buf[H30_FRAME_LEN - 2U] &&
            (checksum_val & 0xFFU) == h30_frame_buf[H30_FRAME_LEN - 1U])
        {
          memcpy(&h30_frame, h30_frame_buf, sizeof(h30_frame));
          h30_last_frame_tick = HAL_GetTick();
          h30_frame_count++;
          h30_online = 1U;
          h30_frame_ready = 1U;
        } else {
          h30_checksum_errors++;
        }
        h30_reset_parser();
      }
      break;

    default:
      h30_reset_parser();
      break;
  }

  h30_last_rx = byte;
  return h30_frame_ready;
}

static uint8_t balance_h30_online(void)
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

static void balance_zero_now(void)
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

static void balance_auto_update(void)
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

/* 平衡环输出映射到 MG540 可转的 PWM（本车平衡修正必须直接给起转满占空比） */
static int16_t balance_u_to_pwm(float u)
{
  int32_t mag;
  int16_t sign;

  if (u > -BAL_U_MIN_ACTIVE && u < BAL_U_MIN_ACTIVE) {
    return 0;
  }

  sign = (u >= 0.0f) ? 1 : -1;
  mag = (int32_t)fabsf(u);
  if (mag < (int32_t)BAL_PWM_MIN) {
    mag = (int32_t)BAL_PWM_MIN;
  }
  if (mag > (int32_t)PWM_MAX) {
    mag = (int32_t)PWM_MAX;
  }
  if (bal_test_mode != 0U) {
    mag = (int32_t)PWM_START_BOOST;
  }
  return (int16_t)(sign * mag);
}

static void balance_apply(int16_t *left, int16_t *right)
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
    bal_last_sta = 6U;
    return;
  }

  balance_snapshot(&pitch_rad, &roll_rad, &gyro_pitch, &gyro_roll);
  bal_last_pitch = pitch_rad;
  bal_last_roll = roll_rad;

  pitch_err = (pitch_rad - bal_pitch_zero) * bal_pitch_sign;
  roll_err = (roll_rad - bal_roll_zero) * bal_roll_sign;
  bal_last_pitch_err = pitch_err;
  bal_last_roll_err = roll_err;

  if (bal_test_mode == 0U &&
      ((fabsf(pitch_err) > bal_max_pitch_rad) || (fabsf(roll_err) > bal_max_roll_rad)))
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

  if (bal_test_mode != 0U) {
  if ((fabsf(pitch_err) > BAL_TEST_ANGLE_RAD) || (fabsf(roll_err) > BAL_TEST_ANGLE_RAD)) {
      int16_t mp = 0;
      int16_t mr = 0;

      if (fabsf(pitch_err) > BAL_TEST_ANGLE_RAD) {
        mp = (pitch_err > 0.0f) ? (int16_t)PWM_START_BOOST : (int16_t)(-PWM_START_BOOST);
      }
      if (fabsf(roll_err) > BAL_TEST_ANGLE_RAD) {
        mr = (roll_err > 0.0f) ? (int16_t)PWM_START_BOOST : (int16_t)(-PWM_START_BOOST);
      }
      *left = (int16_t)(mp + mr);
      *right = (int16_t)(mp - mr);
      bal_last_u_pitch = (float)mp;
      bal_last_u_roll = (float)mr;
      bal_last_motor_l = *left;
      bal_last_motor_r = *right;
      bal_last_sta = 5U;
    } else {
      *left = 0;
      *right = 0;
      bal_last_sta = 4U;
    }
    return;
  }

  if ((fabsf(pitch_err) < BAL_ANGLE_ACTIVE_RAD) &&
      (fabsf(roll_err) < BAL_ANGLE_ACTIVE_RAD))
  {
    bal_last_sta = 4U;
    bal_last_u_pitch = 0.0f;
    bal_last_u_roll = 0.0f;
    bal_last_motor_l = *left;
    bal_last_motor_r = *right;
    return;
  }

  u_pitch = bal_pitch_kp * pitch_err + bal_pitch_kd * gyro_pitch;
  u_roll = bal_roll_kp * roll_err + bal_roll_kd * gyro_roll;
  u_pitch = balance_clampf(u_pitch, -bal_max_corr, bal_max_corr);
  u_roll = balance_clampf(u_roll, -bal_max_corr, bal_max_corr);
  bal_last_u_pitch = u_pitch;
  bal_last_u_roll = u_roll;

  {
    int16_t pwm_pitch_l = balance_u_to_pwm(u_pitch);
    int16_t pwm_pitch_r = pwm_pitch_l;
    int16_t pwm_roll_l = balance_u_to_pwm(u_roll);
    int16_t pwm_roll_r = balance_u_to_pwm(-u_roll);

    corr_l = (float)(pwm_pitch_l + pwm_roll_l);
    corr_r = (float)(pwm_pitch_r + pwm_roll_r);
  }

  out_l = (int32_t)(*left) + (int32_t)corr_l;
  out_r = (int32_t)(*right) + (int32_t)corr_r;

  if (out_l > PWM_MAX) {
    out_l = PWM_MAX;
  } else if (out_l < -PWM_MAX) {
    out_l = -PWM_MAX;
  }
  if (out_r > PWM_MAX) {
    out_r = PWM_MAX;
  } else if (out_r < -PWM_MAX) {
    out_r = -PWM_MAX;
  }

  *left = (int16_t)out_l;
  *right = (int16_t)out_r;
  bal_last_motor_l = *left;
  bal_last_motor_r = *right;
  bal_last_sta = 5U;
}

static void uart3_soft_recover(void)
{
  uart3_last_recover_tick = HAL_GetTick();

  HAL_UART_AbortReceive(&huart3);
  HAL_UART_Abort(&huart3);

  __HAL_UART_CLEAR_OREFLAG(&huart3);
  __HAL_UART_CLEAR_NEFLAG(&huart3);
  __HAL_UART_CLEAR_FEFLAG(&huart3);
  __HAL_UART_CLEAR_PEFLAG(&huart3);
  __HAL_UART_CLEAR_IDLEFLAG(&huart3);

  HAL_UART_DeInit(&huart3);
  HAL_UART_Init(&huart3);

  uart3_need_rearm = 0;
  uart3_line_len = 0;
  uart3_start_rx_to_idle();
}

static void Motor_SetLeft(int16_t pwm)
{
  __HAL_TIM_MOE_ENABLE(&htim1);

  if (!pid_enabled && bal_output_active == 0U) {
    pwm = apply_deadzone_comp(pwm, &left_running, &left_dir, &left_boost_until);
  } else if (bal_output_active != 0U && pwm != 0) {
    int16_t abs_p = (pwm > 0) ? pwm : (int16_t)(-pwm);
    if (abs_p < (int16_t)BAL_PWM_MIN) {
      pwm = (pwm > 0) ? (int16_t)BAL_PWM_MIN : (int16_t)(-BAL_PWM_MIN);
    }
  }
  uint16_t duty = clamp_pwm(pwm);

  if (pwm > 0) {
    __HAL_TIM_SET_COMPARE(&htim1, LEFT_FORWARD_CH, duty);
    __HAL_TIM_SET_COMPARE(&htim1, LEFT_BACKWARD_CH, 0);
  } else if (pwm < 0) {
    __HAL_TIM_SET_COMPARE(&htim1, LEFT_FORWARD_CH, 0);
    __HAL_TIM_SET_COMPARE(&htim1, LEFT_BACKWARD_CH, duty);
  } else {
    __HAL_TIM_SET_COMPARE(&htim1, LEFT_FORWARD_CH, 0);
    __HAL_TIM_SET_COMPARE(&htim1, LEFT_BACKWARD_CH, 0);
  }
}

static void Motor_SetRight(int16_t pwm)
{
  __HAL_TIM_MOE_ENABLE(&htim1);

  if (!pid_enabled && bal_output_active == 0U) {
    pwm = apply_deadzone_comp(pwm, &right_running, &right_dir, &right_boost_until);
  } else if (bal_output_active != 0U && pwm != 0) {
    int16_t abs_p = (pwm > 0) ? pwm : (int16_t)(-pwm);
    if (abs_p < (int16_t)BAL_PWM_MIN) {
      pwm = (pwm > 0) ? (int16_t)BAL_PWM_MIN : (int16_t)(-BAL_PWM_MIN);
    }
  }
  uint16_t duty = clamp_pwm(pwm);

  if (pwm > 0) {
    __HAL_TIM_SET_COMPARE(&htim1, RIGHT_FORWARD_CH, duty);
    __HAL_TIM_SET_COMPARE(&htim1, RIGHT_BACKWARD_CH, 0);
  } else if (pwm < 0) {
    __HAL_TIM_SET_COMPARE(&htim1, RIGHT_FORWARD_CH, 0);
    __HAL_TIM_SET_COMPARE(&htim1, RIGHT_BACKWARD_CH, duty);
  } else {
    __HAL_TIM_SET_COMPARE(&htim1, RIGHT_FORWARD_CH, 0);
    __HAL_TIM_SET_COMPARE(&htim1, RIGHT_BACKWARD_CH, 0);
  }
}

static void Motor_Set(int16_t left, int16_t right)
{
  if (estop_triggered) {
    Motor_Stop();
    return;
  }
  Motor_SetLeft(left);
  Motor_SetRight(right);
}

static void Motor_Stop(void)
{
  left_running = 0;
  right_running = 0;
  left_dir = 0;
  right_dir = 0;
  left_boost_until = 0;
  right_boost_until = 0;
  __HAL_TIM_SET_COMPARE(&htim1, LEFT_FORWARD_CH, 0);
  __HAL_TIM_SET_COMPARE(&htim1, LEFT_BACKWARD_CH, 0);
  __HAL_TIM_SET_COMPARE(&htim1, RIGHT_FORWARD_CH, 0);
  __HAL_TIM_SET_COMPARE(&htim1, RIGHT_BACKWARD_CH, 0);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == ESTOP_Pin) {
    estop_triggered = 1;
    cmd_left = 0;
    cmd_right = 0;
    target_left = 0;
    target_right = 0;
    ps2_cmd_left = 0;
    ps2_cmd_right = 0;
    pid_reset_state();
    Motor_Stop();
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == &huart3)
  {
    uart3_last_rx_tick = HAL_GetTick();
    if (Size > sizeof(uart3_buf)) {
      Size = sizeof(uart3_buf);
    }

    for (uint16_t i = 0; i < Size; i++)
    {
      char ch = (char)uart3_buf[i];

      if (ch == '\r' || ch == '\n')
      {
        if (uart3_line_len > 0)
        {
          uart3_line_buf[uart3_line_len] = '\0';
          if (uart3_cmd_ready == 0U) {
            memcpy(uart3_cmd_buf, uart3_line_buf, sizeof(uart3_cmd_buf));
            uart3_cmd_ready = 1;
          }
          uart3_line_len = 0;
        }
      }
      else
      {
        if (uart3_line_len < (sizeof(uart3_line_buf) - 1U))
        {
          uart3_line_buf[uart3_line_len++] = ch;
        }
        else
        {
          /* 行过长则丢弃本行，避免缓冲区污染 */
          uart3_line_len = 0;
        }
      }
    }

    /* 空闲帧且无换行：提交当前缓存为一行（适配单次突发、末尾不带 \\r\\n 的上位机） */
    if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_IDLE &&
        uart3_line_len > 0U)
    {
      uart3_line_buf[uart3_line_len] = '\0';
      if (uart3_cmd_ready == 0U) {
        memcpy(uart3_cmd_buf, uart3_line_buf, sizeof(uart3_cmd_buf));
        uart3_cmd_ready = 1;
      }
      uart3_line_len = 0;
    }

    /* 重新开启中断接收 */
    if (uart3_start_rx_to_idle() == HAL_OK) {
      uart3_need_rearm = 0;
    } else {
      uart3_need_rearm = 1;
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2)
  {
    h30_rx_bytes++;
    h30_push_raw_byte(h30_rx_byte);
    (void)h30_process_byte(h30_rx_byte);
    (void)h30_start_rx();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart3)
  {
    /* 发生错误时重新开启接收，避免挂死 */
    uart3_last_rx_tick = HAL_GetTick();
    if (uart3_start_rx_to_idle() == HAL_OK) {
      uart3_need_rearm = 0;
    } else {
      uart3_need_rearm = 1;
    }
  }
  else if (huart == &huart2)
  {
    h30_reset_parser();
    (void)h30_start_rx();
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3) {
    control_tick = 1;
  }
}

/* 从字符串里提取第一个 "-?\d+,-?\d+" 子串，前缀杂讯会被跳过 */
static int parse_two_ints(const char *s, int *out_l, int *out_r)
{
  const char *p = s;
  while (*p)
  {
    if (*p == '-' || isdigit((unsigned char)*p))
    {
      char *end1;
      long l = strtol(p, &end1, 10);
      if (end1 == p) { p++; continue; }

      const char *q = end1;
      while (*q == ' ' || *q == '\t') q++;
      if (*q != ',') { p = end1; continue; }
      q++; /* skip comma */
      while (*q == ' ' || *q == '\t') q++;

      char *end2;
      long r = strtol(q, &end2, 10);
      if (end2 == q) { p = end1; continue; }

      *out_l = (int)l;
      *out_r = (int)r;
      return 1;
    }
    p++;
  }
  return 0;
}

static void VOFA_HandleLine(const char *line)
{
  int left = 0;
  int right = 0;
  int tmp_left = 0;
  int tmp_right = 0;
  int ch = 0;
  int duty_cmd = 0;
  static char ack[160];
  static char reply[384];
  char clean[32];
  size_t len = 0;
  const char *cmd = clean;
  int n = 0;
  uint8_t boost_l = 0;
  uint8_t boost_r = 0;
  uint32_t now = HAL_GetTick();

  if (line[0] == '\0') {
    return;
  }

  /* 兼容 \r\n / 前后空白：统一裁剪后再做命令匹配 */
  len = strlen(line);
  if (len >= sizeof(clean)) {
    len = sizeof(clean) - 1;
  }
  memcpy(clean, line, len);
  clean[len] = '\0';

  while (*cmd == ' ' || *cmd == '\t' || *cmd == '\r' || *cmd == '\n') {
    cmd++;
  }

  len = strlen(cmd);
  while (len > 0) {
    char tail = cmd[len - 1];
    if (tail == ' ' || tail == '\t' || tail == '\r' || tail == '\n') {
      ((char *)cmd)[len - 1] = '\0';
      len--;
    } else {
      break;
    }
  }

  if (cmd[0] == '\0') {
    return;
  }

  {
    char rx_echo[48];
    int rx_n = snprintf(rx_echo, sizeof(rx_echo), "RX:%s\r\n", cmd);
    if (rx_n > 0 && !plot_enabled) {
      HAL_UART_Transmit(&huart3, (uint8_t *)rx_echo, (uint16_t)rx_n, 20);
    }
  }

  /* 绘图模式下，仅文本命令暂停数字流；纯数值目标命令不暂停 */
  if (!plot_enabled || !parse_two_ints(cmd, &tmp_left, &tmp_right)) {
    plot_pause_until = HAL_GetTick() + 1000U;
  }

  if (strcmp(cmd, "1") == 0)
  {
    const char reply[] = "2\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t *)reply, sizeof(reply) - 1, 20);
    return;
  }

  if (strcmp(cmd, "CLR") == 0 || strcmp(cmd, "ESTOP=0") == 0)
  {
    estop_triggered = 0;
    bal_fallen = 0U;
    cmd_left = 0;
    cmd_right = 0;
    target_left = 0;
    target_right = 0;
    pid_reset_state();
    Motor_Stop();
    {
      const char reply[] = "ESTOP:0\r\n";
      HAL_UART_Transmit(&huart3, (uint8_t *)reply, sizeof(reply) - 1, 20);
    }
    return;
  }

  if (strcmp(cmd, "PID=1") == 0)
  {
    pid_enabled = 1;
    target_left = 0;
    target_right = 0;
    cmd_left = 0;
    cmd_right = 0;
    pid_reset_state();
    Motor_Stop();
    {
      const char reply[] = "PID:1\r\n";
      HAL_UART_Transmit(&huart3, (uint8_t *)reply, sizeof(reply) - 1, 20);
    }
    return;
  }

  if (strcmp(cmd, "PID=0") == 0)
  {
    pid_enabled = 0;
    target_left = 0;
    target_right = 0;
    cmd_left = 0;
    cmd_right = 0;
    pid_reset_state();
    Motor_Stop();
    {
      const char reply[] = "PID:0\r\n";
      HAL_UART_Transmit(&huart3, (uint8_t *)reply, sizeof(reply) - 1, 20);
    }
    return;
  }

  if (strcmp(cmd, "PLOT=1") == 0)
  {
    plot_enabled = 1;
    plot_div = 0;
    {
      const char reply[] = "PLOT:1\r\n";
      HAL_UART_Transmit(&huart3, (uint8_t *)reply, sizeof(reply) - 1, 20);
    }
    return;
  }

  if (strcmp(cmd, "PLOT=0") == 0)
  {
    plot_enabled = 0;
    plot_div = 0;
    {
      const char reply[] = "PLOT:0\r\n";
      HAL_UART_Transmit(&huart3, (uint8_t *)reply, sizeof(reply) - 1, 20);
    }
    return;
  }

  if (strcmp(cmd, "PS2EN=1") == 0)
  {
    ps2_enabled = (ps2_configure() != 0U) ? 1U : 0U;
    ps2_last_poll_tick = 0U;
    n = snprintf(
      reply,
      sizeof(reply),
      "PS2EN:%u,PS2:%u,MODE:0x%02X\r\n",
      (unsigned int)ps2_enabled,
      (unsigned int)ps2_online,
      (unsigned int)ps2_mode
    );
    if (n > 0) {
      HAL_UART_Transmit(&huart3, (uint8_t *)reply, (uint16_t)n, 30);
    }
    return;
  }

  if (strcmp(cmd, "PS2EN=0") == 0)
  {
    if (ctrl_source == CTRL_SRC_PS2) {
      ctrl_source = CTRL_SRC_UART;
      cmd_left = 0;
      cmd_right = 0;
      target_left = 0;
      target_right = 0;
      pid_reset_state();
      Motor_Stop();
    }
    ps2_disable();
    const char reply_ps2_en[] = "PS2EN:0\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t *)reply_ps2_en, sizeof(reply_ps2_en) - 1U, 20);
    return;
  }

  if (strcmp(cmd, "PS2DBG=1") == 0)
  {
    ps2_debug_enabled = 1U;
    const char reply_ps2_dbg[] = "PS2DBG:1\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t *)reply_ps2_dbg, sizeof(reply_ps2_dbg) - 1U, 20);
    return;
  }

  if (strcmp(cmd, "PS2DBG=0") == 0)
  {
    ps2_debug_enabled = 0U;
    const char reply_ps2_dbg[] = "PS2DBG:0\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t *)reply_ps2_dbg, sizeof(reply_ps2_dbg) - 1U, 20);
    return;
  }

  if (strcmp(cmd, "PS2RAW") == 0)
  {
    uint8_t raw[9];
    (void)ps2_read_frame(raw);
    n = snprintf(
      reply,
      sizeof(reply),
      "PS2RAW:%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X,%02X\r\n",
      (unsigned int)raw[0],
      (unsigned int)raw[1],
      (unsigned int)raw[2],
      (unsigned int)raw[3],
      (unsigned int)raw[4],
      (unsigned int)raw[5],
      (unsigned int)raw[6],
      (unsigned int)raw[7],
      (unsigned int)raw[8]
    );
    if (n > 0) {
      uint16_t tx_len = (n < (int)sizeof(reply)) ? (uint16_t)n : (uint16_t)(sizeof(reply) - 1U);
      HAL_UART_Transmit(&huart3, (uint8_t *)reply, tx_len, 40);
    }
    return;
  }

  if (strcmp(cmd, "CTRL=PS2") == 0)
  {
    if (ps2_enabled == 0U) {
      ps2_enabled = (ps2_configure() != 0U) ? 1U : 0U;
    }
    if (ps2_enabled == 0U) {
      const char reply_ps2_fail[] = "CTRL:PS2,ERR:NO_PAD\r\n";
      HAL_UART_Transmit(&huart3, (uint8_t *)reply_ps2_fail, sizeof(reply_ps2_fail) - 1U, 30);
      return;
    }
    ctrl_source = CTRL_SRC_PS2;
    cmd_left = 0;
    cmd_right = 0;
    target_left = 0;
    target_right = 0;
    ps2_cmd_left = 0;
    ps2_cmd_right = 0;
    left_running = 0;
    right_running = 0;
    left_dir = 0;
    right_dir = 0;
    left_boost_until = 0;
    right_boost_until = 0;
    pid_reset_state();
    Motor_Stop();
    n = snprintf(
      reply,
      sizeof(reply),
      "CTRL:PS2,PS2:%u,MODE:0x%02X\r\n",
      (unsigned int)ps2_online,
      (unsigned int)ps2_mode
    );
    if (n > 0) {
      HAL_UART_Transmit(&huart3, (uint8_t *)reply, (uint16_t)n, 30);
    }
    return;
  }

  if (strcmp(cmd, "CTRL=UART") == 0)
  {
    ctrl_source = CTRL_SRC_UART;
    cmd_left = 0;
    cmd_right = 0;
    target_left = 0;
    target_right = 0;
    pid_reset_state();
    Motor_Stop();
    const char reply_ctrl_uart[] = "CTRL:UART\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t *)reply_ctrl_uart, sizeof(reply_ctrl_uart) - 1U, 20);
    return;
  }

  if (strcmp(cmd, "CTRL?") == 0)
  {
    n = snprintf(
      reply,
      sizeof(reply),
      "CTRL:%s,PS2EN:%u,PS2:%u,MODE:0x%02X,LX:%u,LY:%u,CMD:%d,%d\r\n",
      ctrl_source_name(),
      (unsigned int)ps2_enabled,
      (unsigned int)ps2_online,
      (unsigned int)ps2_mode,
      (unsigned int)ps2_lx,
      (unsigned int)ps2_ly,
      ps2_cmd_left,
      ps2_cmd_right
    );
    if (n > 0) {
      HAL_UART_Transmit(&huart3, (uint8_t *)reply, (uint16_t)n, 30);
    }
    return;
  }

  if (strcmp(cmd, "PS2?") == 0)
  {
    n = snprintf(
      reply,
      sizeof(reply),
      "PS2EN:%u,PS2:%u,CTRL:%s,MODE:0x%02X,BTN:0x%04X,LX:%u,LY:%u,RX:%u,RY:%u,CMD:%d,%d\r\n",
      (unsigned int)ps2_enabled,
      (unsigned int)ps2_online,
      ctrl_source_name(),
      (unsigned int)ps2_mode,
      (unsigned int)ps2_buttons,
      (unsigned int)ps2_lx,
      (unsigned int)ps2_ly,
      (unsigned int)ps2_rx,
      (unsigned int)ps2_ry,
      ps2_cmd_left,
      ps2_cmd_right
    );
    if (n > 0) {
      uint16_t tx_len = (n < (int)sizeof(reply)) ? (uint16_t)n : (uint16_t)(sizeof(reply) - 1U);
      HAL_UART_Transmit(&huart3, (uint8_t *)reply, tx_len, 40);
    }
    return;
  }

  if (strcmp(cmd, "H30DBG=1") == 0)
  {
    h30_debug_enabled = 1U;
    const char reply_h30_dbg[] = "H30DBG:1\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t *)reply_h30_dbg, sizeof(reply_h30_dbg) - 1U, 20);
    return;
  }

  if (strcmp(cmd, "H30DBG=0") == 0)
  {
    h30_debug_enabled = 0U;
    const char reply_h30_dbg[] = "H30DBG:0\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t *)reply_h30_dbg, sizeof(reply_h30_dbg) - 1U, 20);
    return;
  }

  if (strcmp(cmd, "H30SCAN") == 0)
  {
    const char scan_start[] = "H30SCAN:START (约2秒)\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t *)scan_start, sizeof(scan_start) - 1U, 30);
    h30_scan_baud();
    return;
  }

  if (strcmp(cmd, "H30RESTART") == 0)
  {
    h30_reset_parser();
    (void)h30_uart_set_baud(h30_uart_baud);
    n = snprintf(
      reply,
      sizeof(reply),
      "H30RESTART:OK,BAUD:%lu,RX:%lu\r\n",
      (unsigned long)h30_uart_baud,
      (unsigned long)h30_rx_bytes
    );
    if (n > 0) {
      HAL_UART_Transmit(&huart3, (uint8_t *)reply, (uint16_t)n, 30);
    }
    return;
  }

  if (strncmp(cmd, "H30BAUD=", 8) == 0)
  {
    uint32_t baud = (uint32_t)strtoul(cmd + 8, NULL, 10);
    HAL_StatusTypeDef st = h30_uart_set_baud(baud);

    n = snprintf(
      reply,
      sizeof(reply),
      "H30BAUD:%lu,%s\r\n",
      (unsigned long)h30_uart_baud,
      (st == HAL_OK) ? "OK" : "ERR"
    );
    if (n > 0) {
      uint16_t tx_len = (n < (int)sizeof(reply)) ? (uint16_t)n : (uint16_t)(sizeof(reply) - 1U);
      HAL_UART_Transmit(&huart3, (uint8_t *)reply, tx_len, 40);
    }
    return;
  }

  if (strcmp(cmd, "H30STAT") == 0 || strcmp(cmd, "H30RAW") == 0)
  {
    if (strcmp(cmd, "H30RAW") == 0) {
      const char hint[] = "H30RAW:->H30STAT\r\n";
      HAL_UART_Transmit(&huart3, (uint8_t *)hint, sizeof(hint) - 1U, 20);
    }
    uint32_t rx_total = h30_rx_bytes;
    uint8_t online = h30_online;
    uint8_t i = 0U;

    if ((HAL_GetTick() - h30_last_frame_tick) > H30_ONLINE_TIMEOUT_MS) {
      online = 0U;
    }

    n = snprintf(
      reply,
      sizeof(reply),
      "H30STAT:ON=%u,RX:%lu,CNT:%lu,ERR:%lu,SYNC:%lu,FRM:%u,BAUD:%lu,PA2=TX,PA3=RX\r\n",
      (unsigned int)online,
      (unsigned long)rx_total,
      (unsigned long)h30_frame_count,
      (unsigned long)h30_checksum_errors,
      (unsigned long)h30_sync_count,
      (unsigned int)(uint8_t)sizeof(H30_FrameTypeDef),
      (unsigned long)h30_uart_baud
    );
    if (n > 0) {
      uint16_t tx_len = (n < (int)sizeof(reply)) ? (uint16_t)n : (uint16_t)(sizeof(reply) - 1U);
      HAL_UART_Transmit(&huart3, (uint8_t *)reply, tx_len, 40);
    }

    n = snprintf(reply, sizeof(reply), "H30RAW:");
    if (n < 0) {
      n = 0;
    }
    for (i = 0U; i < (uint8_t)sizeof(h30_raw_ring); i++) {
      uint8_t idx = (uint8_t)((h30_raw_w + i) % (uint8_t)sizeof(h30_raw_ring));
      int part = 0;
      if (n >= (int)sizeof(reply) - 4) {
        break;
      }
      part = snprintf(&reply[(size_t)n], sizeof(reply) - (size_t)n, "%02X", (unsigned int)h30_raw_ring[idx]);
      if (part > 0) {
        n += part;
      }
    }
    if (n > 0 && n < (int)sizeof(reply) - 2) {
      reply[(size_t)n++] = '\r';
      reply[(size_t)n++] = '\n';
      HAL_UART_Transmit(&huart3, (uint8_t *)reply, (uint16_t)n, 40);
    }
    return;
  }

  if (strcmp(cmd, "H30?") == 0)
  {
    H30_FrameTypeDef frame_local;
    uint8_t online = h30_online;

    if ((HAL_GetTick() - h30_last_frame_tick) > H30_ONLINE_TIMEOUT_MS) {
      online = 0U;
    }

    __disable_irq();
    memcpy(&frame_local, &h30_frame, sizeof(frame_local));
    __enable_irq();

    n = snprintf(
      reply,
      sizeof(reply),
      "H30:%u,CNT:%lu,ERR:%lu,NUM:%u,PITCH:%.3f,ROLL:%.3f,YAW:%.3f,GX:%.3f,GY:%.3f,GZ:%.3f,AX:%.3f,AY:%.3f,AZ:%.3f,Q:%.6f,%.6f,%.6f,%.6f\r\n",
      (unsigned int)online,
      (unsigned long)h30_frame_count,
      (unsigned long)h30_checksum_errors,
      (unsigned int)frame_local.frameNum,
      (double)frame_local.attitude.pitch * 0.000001,
      (double)frame_local.attitude.roll * 0.000001,
      (double)frame_local.attitude.yaw * 0.000001,
      (double)frame_local.gyro.gx * 0.000001,
      (double)frame_local.gyro.gy * 0.000001,
      (double)frame_local.gyro.gz * 0.000001,
      (double)frame_local.accel.ax * 0.000001,
      (double)frame_local.accel.ay * 0.000001,
      (double)frame_local.accel.az * 0.000001,
      (double)frame_local.quaternion.q0 * 0.000001,
      (double)frame_local.quaternion.q1 * 0.000001,
      (double)frame_local.quaternion.q2 * 0.000001,
      (double)frame_local.quaternion.q3 * 0.000001
    );
    if (n > 0) {
      uint16_t tx_len = (n < (int)sizeof(reply)) ? (uint16_t)n : (uint16_t)(sizeof(reply) - 1U);
      HAL_UART_Transmit(&huart3, (uint8_t *)reply, tx_len, 40);
    }
    return;
  }

  if (strcmp(cmd, "BAL=1") == 0)
  {
    bal_enabled = 1U;
    bal_test_mode = 0U;
    bal_fallen = 0U;
    bal_auto_zero_pending = 1U;
    bal_zero_valid = 0U;
    bal_stable_since = 0U;
    ps2_cmd_left = 0;
    ps2_cmd_right = 0;
    cmd_left = 0;
    cmd_right = 0;
    target_left = 0;
    target_right = 0;
    pid_reset_state();
    n = snprintf(
      reply,
      sizeof(reply),
      "BAL:1,H30:%u,ZERO_P:%.4f,ZERO_R:%.4f,PID:%u,AUTOZERO\r\n",
      (unsigned int)balance_h30_online(),
      (double)bal_pitch_zero,
      (double)bal_roll_zero,
      (unsigned int)pid_enabled
    );
    if (n > 0) {
      HAL_UART_Transmit(&huart3, (uint8_t *)reply, (uint16_t)n, 30);
    }
    return;
  }

  if (strcmp(cmd, "BALTEST=1") == 0)
  {
    bal_enabled = 1U;
    bal_test_mode = 1U;
    bal_fallen = 0U;
    ctrl_source = CTRL_SRC_UART;
    cmd_left = 0;
    cmd_right = 0;
    const char reply_bal_test[] =
      "BALTEST:1,满PWM测电机,倾斜>0.7deg应转,发BALTEST=0关\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t *)reply_bal_test, sizeof(reply_bal_test) - 1U, 40);
    return;
  }

  if (strcmp(cmd, "BALTEST=0") == 0)
  {
    bal_test_mode = 0U;
    cmd_left = 0;
    cmd_right = 0;
    Motor_Stop();
    const char reply_bal_test[] = "BALTEST:0\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t *)reply_bal_test, sizeof(reply_bal_test) - 1U, 20);
    return;
  }

  if (strcmp(cmd, "BAL=0") == 0)
  {
    bal_enabled = 0U;
    bal_test_mode = 0U;
    const char reply_bal_off[] = "BAL:0\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t *)reply_bal_off, sizeof(reply_bal_off) - 1U, 20);
    return;
  }

  if (strcmp(cmd, "BAL?") == 0)
  {
    n = snprintf(
      reply,
      sizeof(reply),
      "BAL:%u,ZV:%u,AZ:%u,FALL:%u,H30:%u,PIT:%.4f,ROL:%.4f,ZP:%.4f,ZR:%.4f,PKP:%.1f,PKD:%.1f,RKP:%.1f,RKD:%.1f,MAX:%.0f,PS:%.0f,RS:%.0f\r\n",
      (unsigned int)bal_enabled,
      (unsigned int)bal_zero_valid,
      (unsigned int)bal_auto_zero_pending,
      (unsigned int)bal_fallen,
      (unsigned int)balance_h30_online(),
      (double)bal_last_pitch,
      (double)bal_last_roll,
      (double)bal_pitch_zero,
      (double)bal_roll_zero,
      (double)bal_pitch_kp,
      (double)bal_pitch_kd,
      (double)bal_roll_kp,
      (double)bal_roll_kd,
      (double)bal_max_corr,
      (double)bal_pitch_sign,
      (double)bal_roll_sign
    );
    if (n > 0) {
      HAL_UART_Transmit(&huart3, (uint8_t *)reply, (uint16_t)n, 40);
    }
    return;
  }

  if (strcmp(cmd, "BALZERO") == 0)
  {
    if (balance_h30_online() == 0U) {
      const char reply_bal_err[] = "BALZERO:ERR,NO_H30\r\n";
      HAL_UART_Transmit(&huart3, (uint8_t *)reply_bal_err, sizeof(reply_bal_err) - 1U, 30);
      return;
    }
    balance_zero_now();
    n = snprintf(
      reply,
      sizeof(reply),
      "BALZERO:OK,P:%.4f,R:%.4f\r\n",
      (double)bal_pitch_zero,
      (double)bal_roll_zero
    );
    if (n > 0) {
      HAL_UART_Transmit(&huart3, (uint8_t *)reply, (uint16_t)n, 30);
    }
    return;
  }

  if (strcmp(cmd, "BALDBG=1") == 0)
  {
    bal_debug_enabled = 1U;
    const char reply_bal_dbg[] = "BALDBG:1\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t *)reply_bal_dbg, sizeof(reply_bal_dbg) - 1U, 20);
    return;
  }

  if (strcmp(cmd, "BALDBG=0") == 0)
  {
    bal_debug_enabled = 0U;
    const char reply_bal_dbg[] = "BALDBG:0\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t *)reply_bal_dbg, sizeof(reply_bal_dbg) - 1U, 20);
    return;
  }

  {
    float bal_f = 0.0f;
    if (sscanf(cmd, "BALP=%f", &bal_f) == 1)
    {
      bal_pitch_kp = bal_f;
      n = snprintf(reply, sizeof(reply), "BALP:%.1f\r\n", (double)bal_pitch_kp);
      if (n > 0) {
        HAL_UART_Transmit(&huart3, (uint8_t *)reply, (uint16_t)n, 20);
      }
      return;
    }
    if (sscanf(cmd, "BALD=%f", &bal_f) == 1)
    {
      bal_pitch_kd = bal_f;
      n = snprintf(reply, sizeof(reply), "BALD:%.1f\r\n", (double)bal_pitch_kd);
      if (n > 0) {
        HAL_UART_Transmit(&huart3, (uint8_t *)reply, (uint16_t)n, 20);
      }
      return;
    }
    if (sscanf(cmd, "BALR=%f", &bal_f) == 1)
    {
      bal_roll_kp = bal_f;
      n = snprintf(reply, sizeof(reply), "BALR:%.1f\r\n", (double)bal_roll_kp);
      if (n > 0) {
        HAL_UART_Transmit(&huart3, (uint8_t *)reply, (uint16_t)n, 20);
      }
      return;
    }
    if (sscanf(cmd, "BALRD=%f", &bal_f) == 1)
    {
      bal_roll_kd = bal_f;
      n = snprintf(reply, sizeof(reply), "BALRD:%.1f\r\n", (double)bal_roll_kd);
      if (n > 0) {
        HAL_UART_Transmit(&huart3, (uint8_t *)reply, (uint16_t)n, 20);
      }
      return;
    }
    if (sscanf(cmd, "BALMAX=%f", &bal_f) == 1)
    {
      if (bal_f < 100.0f) {
        bal_f = 100.0f;
      }
      bal_max_corr = bal_f;
      n = snprintf(reply, sizeof(reply), "BALMAX:%.0f\r\n", (double)bal_max_corr);
      if (n > 0) {
        HAL_UART_Transmit(&huart3, (uint8_t *)reply, (uint16_t)n, 20);
      }
      return;
    }
    if (sscanf(cmd, "BALPSIGN=%f", &bal_f) == 1)
    {
      bal_pitch_sign = (bal_f >= 0.0f) ? 1.0f : -1.0f;
      n = snprintf(reply, sizeof(reply), "BALPSIGN:%.0f\r\n", (double)bal_pitch_sign);
      if (n > 0) {
        HAL_UART_Transmit(&huart3, (uint8_t *)reply, (uint16_t)n, 20);
      }
      return;
    }
    if (sscanf(cmd, "BALRSIGN=%f", &bal_f) == 1)
    {
      bal_roll_sign = (bal_f >= 0.0f) ? 1.0f : -1.0f;
      n = snprintf(reply, sizeof(reply), "BALRSIGN:%.0f\r\n", (double)bal_roll_sign);
      if (n > 0) {
        HAL_UART_Transmit(&huart3, (uint8_t *)reply, (uint16_t)n, 20);
      }
      return;
    }
    if (sscanf(cmd, "BALPITLIM=%f", &bal_f) == 1)
    {
      if (bal_f < 5.0f) {
        bal_f = 5.0f;
      }
      bal_max_pitch_rad = bal_f * 0.017453292f;
      n = snprintf(reply, sizeof(reply), "BALPITLIM:%.1f\r\n", (double)bal_f);
      if (n > 0) {
        HAL_UART_Transmit(&huart3, (uint8_t *)reply, (uint16_t)n, 20);
      }
      return;
    }
    if (sscanf(cmd, "BALROLLIM=%f", &bal_f) == 1)
    {
      if (bal_f < 5.0f) {
        bal_f = 5.0f;
      }
      bal_max_roll_rad = bal_f * 0.017453292f;
      n = snprintf(reply, sizeof(reply), "BALROLLIM:%.1f\r\n", (double)bal_f);
      if (n > 0) {
        HAL_UART_Transmit(&huart3, (uint8_t *)reply, (uint16_t)n, 20);
      }
      return;
    }
  }

  if (strcmp(cmd, "PID?") == 0)
  {
    n = snprintf(
      reply,
      sizeof(reply),
      "PID:%u,USERFULL:%d,SPDFULL:%d,KP:%.3f,KI:%.3f,KD:%.3f,KFF:%.3f,KFFH:%.3f,KFFS:%.3f,TGT:%d,%d,SPD:%d,%d\r\n",
      (unsigned int)pid_enabled,
      (int)PID_USER_FULL,
      (int)PID_SPEED_FULL,
      (double)pid_kp,
      (double)pid_ki,
      (double)pid_kd,
      (double)pid_kff,
      (double)pid_kff_high,
      (double)pid_kff_split,
      target_left,
      target_right,
      speed_left,
      speed_right
    );
    if (n > 0) {
      HAL_UART_Transmit(&huart3, (uint8_t *)reply, (uint16_t)n, 30);
    }
    return;
  }

  if (strcmp(cmd, "STATUS") == 0 || strcmp(cmd, "MODE?") == 0)
  {
    uint32_t c1 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1);
    uint32_t c2 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2);
    uint32_t c3 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_3);
    uint32_t c4 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_4);
    boost_l = (left_boost_until > now) ? 1u : 0u;
    boost_r = (right_boost_until > now) ? 1u : 0u;
    n = snprintf(
      reply,
      sizeof(reply),
      "STATUS:CTRL=%s,PID=%u,BAL=%u,BALZ=%u,BALF=%u,PLOT=%u,PS2EN=%u,PS2=%u,MODE=0x%02X,LX=%u,LY=%u,H30=%u,H30CNT=%lu,PIT:%.3f,ROL:%.3f,ESTOP=%u,CMD=%d,%d,TGT=%d,%d,SPD=%d,%d,CCR=%lu,%lu,%lu,%lu,BOOST=%u,%u\r\n",
      ctrl_source_name(),
      (unsigned int)pid_enabled,
      (unsigned int)bal_enabled,
      (unsigned int)bal_zero_valid,
      (unsigned int)bal_fallen,
      (unsigned int)plot_enabled,
      (unsigned int)ps2_enabled,
      (unsigned int)ps2_online,
      (unsigned int)ps2_mode,
      (unsigned int)ps2_lx,
      (unsigned int)ps2_ly,
      (unsigned int)h30_online,
      (unsigned long)h30_frame_count,
      (double)bal_last_pitch,
      (double)bal_last_roll,
      (unsigned int)estop_triggered,
      cmd_left,
      cmd_right,
      target_left,
      target_right,
      speed_left,
      speed_right,
      (unsigned long)c1,
      (unsigned long)c2,
      (unsigned long)c3,
      (unsigned long)c4,
      (unsigned int)boost_l,
      (unsigned int)boost_r
    );
    if (n > 0) {
      uint16_t tx_len = (n < (int)sizeof(reply)) ? (uint16_t)n : (uint16_t)(sizeof(reply) - 1U);
      HAL_UART_Transmit(&huart3, (uint8_t *)reply, tx_len, 40);
    }
    return;
  }

  if (sscanf(cmd, "CH%d=%d", &ch, &duty_cmd) == 2)
  {
    uint32_t tim_ch = 0U;

    if (duty_cmd < 0) duty_cmd = 0;
    if (duty_cmd > PWM_MAX) duty_cmd = PWM_MAX;

    switch (ch) {
      case 1: tim_ch = TIM_CHANNEL_1; break; /* PA8  -> AIN1 */
      case 2: tim_ch = TIM_CHANNEL_2; break; /* PA9  -> AIN2 */
      case 3: tim_ch = TIM_CHANNEL_3; break; /* PA10 -> BIN2 */
      case 4: tim_ch = TIM_CHANNEL_4; break; /* PA11 -> BIN1 */
      default: tim_ch = 0U; break;
    }

    cmd_left = 0;
    cmd_right = 0;
    target_left = 0;
    target_right = 0;
    pid_reset_state();
    __HAL_TIM_MOE_ENABLE(&htim1);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0U);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 0U);

    if (tim_ch != 0U && estop_triggered == 0U) {
      __HAL_TIM_SET_COMPARE(&htim1, tim_ch, (uint32_t)duty_cmd);
    }

    n = snprintf(
      reply,
      sizeof(reply),
      "CH:%d,DUTY:%d,ESTOP:%u,MAP:1=PA8/AIN1,2=PA9/AIN2,3=PA10/BIN2,4=PA11/BIN1\r\n",
      ch,
      duty_cmd,
      (unsigned int)estop_triggered
    );
    if (n > 0) {
      uint16_t tx_len = (n < (int)sizeof(reply)) ? (uint16_t)n : (uint16_t)(sizeof(reply) - 1U);
      HAL_UART_Transmit(&huart3, (uint8_t *)reply, tx_len, 40);
    }
    return;
  }

  if (sscanf(cmd, "KP=%f", &pid_kp) == 1)
  {
    n = snprintf(reply, sizeof(reply), "KP:%.3f\r\n", (double)pid_kp);
    if (n > 0) {
      HAL_UART_Transmit(&huart3, (uint8_t *)reply, (uint16_t)n, 20);
    }
    return;
  }

  if (strcmp(cmd, "SAVE") == 0)
  {
    HAL_StatusTypeDef st = pid_save_params();
    const char *reply_ok = (st == HAL_OK) ? "SAVE:OK\r\n" : "SAVE:ERR\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t *)reply_ok, (uint16_t)strlen(reply_ok), 30);
    return;
  }

  if (sscanf(cmd, "KI=%f", &pid_ki) == 1)
  {
    n = snprintf(reply, sizeof(reply), "KI:%.3f\r\n", (double)pid_ki);
    if (n > 0) {
      HAL_UART_Transmit(&huart3, (uint8_t *)reply, (uint16_t)n, 20);
    }
    return;
  }

  if (sscanf(cmd, "KD=%f", &pid_kd) == 1)
  {
    n = snprintf(reply, sizeof(reply), "KD:%.3f\r\n", (double)pid_kd);
    if (n > 0) {
      HAL_UART_Transmit(&huart3, (uint8_t *)reply, (uint16_t)n, 20);
    }
    return;
  }

  if (sscanf(cmd, "KFF=%f", &pid_kff) == 1)
  {
    n = snprintf(reply, sizeof(reply), "KFF:%.3f\r\n", (double)pid_kff);
    if (n > 0) {
      HAL_UART_Transmit(&huart3, (uint8_t *)reply, (uint16_t)n, 20);
    }
    return;
  }

  if (sscanf(cmd, "KFFH=%f", &pid_kff_high) == 1)
  {
    n = snprintf(reply, sizeof(reply), "KFFH:%.3f\r\n", (double)pid_kff_high);
    if (n > 0) {
      HAL_UART_Transmit(&huart3, (uint8_t *)reply, (uint16_t)n, 20);
    }
    return;
  }

  if (sscanf(cmd, "KFFS=%f", &pid_kff_split) == 1)
  {
    if (pid_kff_split < 1.0f) {
      pid_kff_split = 1.0f;
    }
    n = snprintf(reply, sizeof(reply), "KFFS:%.3f\r\n", (double)pid_kff_split);
    if (n > 0) {
      HAL_UART_Transmit(&huart3, (uint8_t *)reply, (uint16_t)n, 20);
    }
    return;
  }

  if (strncmp(cmd, "H30", 3) == 0)
  {
    const char h30_help[] =
      "H30?: H30STAT H30SCAN H30? H30DBG=1 H30BAUD=460800 H30RESTART\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t *)h30_help, sizeof(h30_help) - 1U, 40);
    return;
  }

  if (parse_two_ints(cmd, &left, &right))
  {
    ctrl_source = CTRL_SRC_UART;
    if (left > PWM_MAX) left = PWM_MAX;
    if (left < -PWM_MAX) left = -PWM_MAX;
    if (right > PWM_MAX) right = PWM_MAX;
    if (right < -PWM_MAX) right = -PWM_MAX;

    if (!estop_triggered) {
      if (pid_enabled) {
        target_left = pid_map_user_speed((int16_t)left);
        target_right = pid_map_user_speed((int16_t)right);
        if (target_left == 0 && target_right == 0) {
          pid_i_left = 0.0f;
          pid_i_right = 0.0f;
          pid_prev_e_left = 0.0f;
          pid_prev_e_right = 0.0f;
          cmd_left = 0;
          cmd_right = 0;
          Motor_Stop();
        }
      } else {
        cmd_left = (int16_t)left;
        cmd_right = (int16_t)right;
        /* 开环模式立刻更新 PWM，避免 ACK 显示上一个周期的 CCR */
        Motor_Set(cmd_left, cmd_right);
      }
    } else {
      Motor_Stop();
    }

    uint32_t c1 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1);
    uint32_t c2 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2);
    uint32_t c3 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_3);
    uint32_t c4 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_4);
    boost_l = (left_boost_until > now) ? 1u : 0u;
    boost_r = (right_boost_until > now) ? 1u : 0u;
    n = snprintf(
      ack,
      sizeof(ack),
      "ACK:%d,%d,%lu,%lu,%lu,%lu,ESTOP:%u,BOOST:%u,%u,PID:%u,TGT:%d,%d,SPD:%d,%d\r\n",
      cmd_left,
      cmd_right,
      (unsigned long)c1,
      (unsigned long)c2,
      (unsigned long)c3,
      (unsigned long)c4,
      (unsigned int)estop_triggered,
      (unsigned int)boost_l,
      (unsigned int)boost_r,
      (unsigned int)pid_enabled,
      target_left,
      target_right,
      speed_left,
      speed_right
    );
    if (n > 0 && !plot_enabled) {
      HAL_UART_Transmit(&huart3, (uint8_t *)ack, (uint16_t)n, 30);
    }
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
