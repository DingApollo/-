#include "h30.h"
#include <string.h>
#include <stdio.h>
#include "usart.h"

uint8_t h30_rx_byte = 0;
uint8_t h30_frame_buf[H30_FRAME_LEN];
uint16_t h30_frame_index = 0;
uint8_t h30_last_rx = 0;
volatile uint8_t h30_frame_ready = 0;
volatile uint8_t h30_online = 0;
uint8_t h30_debug_enabled = 0;
H30_StateTypeDef h30_state = H30_WAIT_HEAD;
H30_FrameTypeDef h30_frame;
uint32_t h30_last_frame_tick = 0;
uint32_t h30_last_debug_tick = 0;
uint32_t h30_frame_count = 0;
uint32_t h30_checksum_errors = 0;
volatile uint32_t h30_rx_bytes = 0;
uint32_t h30_sync_count = 0;
uint8_t h30_raw_ring[16];
uint8_t h30_raw_w = 0;
uint32_t h30_uart_baud = H30_UART_BAUD_DEFAULT;
uint8_t h30_boot_reported = 0U;
uint32_t h30_watch_tick = 0U;

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

void h30_reset_parser(void)
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
HAL_StatusTypeDef h30_start_rx(void)
{
  if (huart2.gState != HAL_UART_STATE_READY) {
    (void)HAL_UART_AbortReceive_IT(&huart2);
  }
  return HAL_UART_Receive_IT(&huart2, &h30_rx_byte, 1U);
}

HAL_StatusTypeDef h30_uart_set_baud(uint32_t baud)
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

void h30_scan_baud(void)
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

void h30_ensure_rx(void)
{
  if (huart2.RxState == HAL_UART_STATE_READY) {
    (void)h30_start_rx();
  }
}

void h30_boot_report(void)
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


void h30_on_uart_rx_cplt(void)
{
  h30_rx_bytes++;
  h30_push_raw_byte(h30_rx_byte);
  (void)h30_process_byte(h30_rx_byte);
  (void)h30_start_rx();
}

void h30_uart2_error_recover(void)
{
  h30_reset_parser();
  (void)h30_start_rx();
}
