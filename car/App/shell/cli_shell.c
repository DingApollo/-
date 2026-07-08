/**
 * @file    cli_shell.c
 * @brief   串口命令行：USART3(115200) 文本行协议，调试与控制入口
 * @note    命令示例: "左,右"目标值 / PID=1 / CTRL=PS2 / BAL=1 / H30STAT /
 *          LAMP=1 / STATUS / KP= KI= KD= / SAVE(存Flash) / PLOT=1(绘图流)
 *          接收采用 ReceiveToIdle 中断 + 主循环兜底重挂与错误恢复
 *          依赖: usart.h(USART3) 及各功能模块
 */
#include "cli_shell.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "pid.h"
#include "motor.h"
#include "h30.h"
#include "balance.h"
#include "ps2.h"
#include "app_ctrl.h"
#include "app_state.h"
#include "lamp.h"

static uint8_t uart3_buf[32];
static char uart3_line_buf[96];
static uint8_t uart3_line_len = 0;
volatile uint8_t uart3_cmd_ready = 0;
char uart3_cmd_buf[96];
volatile uint32_t uart3_last_arm_tick = 0;
volatile uint32_t uart3_last_rx_tick = 0;
volatile uint32_t uart3_last_recover_tick = 0;
volatile uint8_t uart3_need_rearm = 0;
uint8_t plot_enabled = 0;
uint8_t plot_div = 0;
uint32_t plot_pause_until = 0;

HAL_StatusTypeDef cli_uart3_start_rx_to_idle(void)
{
  HAL_StatusTypeDef st;
  st = HAL_UARTEx_ReceiveToIdle_IT(&huart3, uart3_buf, sizeof(uart3_buf));
  if (st == HAL_OK) {
    uart3_last_arm_tick = HAL_GetTick();
  }
  return st;
}

void cli_uart3_soft_recover(void)
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
  (void)cli_uart3_start_rx_to_idle();
}

void cli_uart3_on_rx_event(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart != &huart3) {
    return;
  }
  uart3_last_rx_tick = HAL_GetTick();
  if (Size > sizeof(uart3_buf)) {
    Size = sizeof(uart3_buf);
  }
  for (uint16_t i = 0; i < Size; i++) {
    char ch = (char)uart3_buf[i];
    if (ch == '\r' || ch == '\n') {
      if (uart3_line_len > 0) {
        uart3_line_buf[uart3_line_len] = '\0';
        if (uart3_cmd_ready == 0U) {
          memcpy(uart3_cmd_buf, uart3_line_buf, sizeof(uart3_cmd_buf));
          uart3_cmd_ready = 1;
        }
        uart3_line_len = 0;
      }
    } else {
      if (uart3_line_len < (sizeof(uart3_line_buf) - 1U)) {
        uart3_line_buf[uart3_line_len++] = (uint8_t)ch;
      } else {
        uart3_line_len = 0;
      }
    }
  }
  if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_IDLE &&
      uart3_line_len > 0U) {
    uart3_line_buf[uart3_line_len] = '\0';
    if (uart3_cmd_ready == 0U) {
      memcpy(uart3_cmd_buf, uart3_line_buf, sizeof(uart3_cmd_buf));
      uart3_cmd_ready = 1;
    }
    uart3_line_len = 0;
  }
  if (cli_uart3_start_rx_to_idle() == HAL_OK) {
    uart3_need_rearm = 0;
  } else {
    uart3_need_rearm = 1;
  }
}

void cli_uart3_on_error(UART_HandleTypeDef *huart)
{
  if (huart != &huart3) {
    return;
  }
  uart3_last_rx_tick = HAL_GetTick();
  if (cli_uart3_start_rx_to_idle() == HAL_OK) {
    uart3_need_rearm = 0;
  } else {
    uart3_need_rearm = 1;
  }
}

static int parse_two_ints(const char *s, int *out_l, int *out_r)
{
  const char *p = s;
  while (*p) {
    if (*p == '-' || isdigit((unsigned char)*p)) {
      char *end1;
      long l = strtol(p, &end1, 10);
      if (end1 == p) {
        p++;
        continue;
      }
      const char *q = end1;
      while (*q == ' ' || *q == '\t') {
        q++;
      }
      if (*q != ',') {
        p = end1;
        continue;
      }
      q++;
      while (*q == ' ' || *q == '\t') {
        q++;
      }
      char *end2;
      long r = strtol(q, &end2, 10);
      if (end2 == q) {
        p = end1;
        continue;
      }
      *out_l = (int)l;
      *out_r = (int)r;
      return 1;
    }
    p++;
  }
  return 0;
}


void cli_shell_handle_line(const char *line)
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
    motor_reset_ramp_state();
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
      "BALTEST:1,平衡车连续模式,摇杆回中,发BALTEST=0关\r\n";
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

  if (strcmp(cmd, "LAMP=1") == 0)
  {
    Lamp_Set(1U);
    const char reply[] = "LAMP:1\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t *)reply, sizeof(reply) - 1U, 20);
    return;
  }

  if (strcmp(cmd, "LAMP=0") == 0)
  {
    Lamp_Set(0U);
    const char reply[] = "LAMP:0\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t *)reply, sizeof(reply) - 1U, 20);
    return;
  }

  if (strcmp(cmd, "LAMP?") == 0)
  {
    n = snprintf(reply, sizeof(reply), "LAMP:%u\r\n", (unsigned int)Lamp_IsOn());
    if (n > 0) {
      HAL_UART_Transmit(&huart3, (uint8_t *)reply, (uint16_t)n, 20);
    }
    return;
  }

  if (strcmp(cmd, "STATUS") == 0 || strcmp(cmd, "MODE?") == 0)
  {
    uint32_t c1 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1);
    uint32_t c2 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2);
    uint32_t c3 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_3);
    uint32_t c4 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_4);
    boost_l = (motor_left_boost_deadline() > now) ? 1u : 0u;
    boost_r = (motor_right_boost_deadline() > now) ? 1u : 0u;
    n = snprintf(
      reply,
      sizeof(reply),
      "STATUS:CTRL=%s,PID=%u,BAL=%u,BALZ=%u,BALF=%u,PLOT=%u,PS2EN=%u,PS2=%u,MODE=0x%02X,LX=%u,LY=%u,LAMP=%u,H30=%u,H30CNT=%lu,PIT:%.3f,ROL:%.3f,ESTOP=%u,CMD=%d,%d,TGT=%d,%d,SPD=%d,%d,CCR=%lu,%lu,%lu,%lu,BOOST=%u,%u\r\n",
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
      (unsigned int)Lamp_IsOn(),
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
    int16_t test_left = 0;
    int16_t test_right = 0;

    if (duty_cmd < 0) duty_cmd = 0;
    if (duty_cmd > PWM_MAX) duty_cmd = PWM_MAX;

    switch (ch) {
      case 1: tim_ch = TIM_CHANNEL_1; break; /* PA8  -> AIN1 */
      case 2: tim_ch = TIM_CHANNEL_2; break; /* PA9  -> AIN2 */
      case 3: tim_ch = TIM_CHANNEL_3; break; /* PA10 -> BIN2 */
      case 4: tim_ch = TIM_CHANNEL_4; break; /* PA11 -> BIN1 */
      default: tim_ch = 0U; break;
    }

    ctrl_source = CTRL_SRC_UART;
    pid_enabled = 0U;
    if (ch == 1) {
      test_left = (int16_t)duty_cmd;
    } else if (ch == 2) {
      test_left = (int16_t)(-duty_cmd);
    } else if (ch == 4) {
      test_right = (int16_t)duty_cmd;
    } else if (ch == 3) {
      test_right = (int16_t)(-duty_cmd);
    }
    cmd_left = test_left;
    cmd_right = test_right;
    target_left = 0;
    target_right = 0;
    pid_reset_state();

    if (tim_ch != 0U && estop_triggered == 0U) {
      Motor_Set(cmd_left, cmd_right);
    } else {
      Motor_Stop();
    }

    n = snprintf(
      reply,
      sizeof(reply),
      "CH:%d,DUTY:%d,ESTOP:%u,CMD:%d,%d,MAP:1=PA8/AIN1,2=PA9/AIN2,3=PA10/BIN2,4=PA11/BIN1\r\n",
      ch,
      duty_cmd,
      (unsigned int)estop_triggered,
      cmd_left,
      cmd_right
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
    boost_l = (motor_left_boost_deadline() > now) ? 1u : 0u;
    boost_r = (motor_right_boost_deadline() > now) ? 1u : 0u;
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
