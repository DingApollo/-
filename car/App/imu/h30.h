/**
 * @file    h30.h
 * @brief   WHEELTEC H30 惯导接口：帧结构定义、解析状态、波特率/恢复控制
 */
#ifndef H30_H
#define H30_H

#include <stdint.h>
#include "main.h"

#define H30_DEBUG_PERIOD_MS 200U
#define H30_FRAME_LEN 67U
#define H30_ONLINE_TIMEOUT_MS 500U
#define H30_UART_BAUD_DEFAULT 460800U

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

extern uint8_t h30_rx_byte;
extern uint8_t h30_frame_buf[H30_FRAME_LEN];
extern uint16_t h30_frame_index;
extern uint8_t h30_last_rx;
extern volatile uint8_t h30_frame_ready;
extern volatile uint8_t h30_online;
extern uint8_t h30_debug_enabled;
extern H30_StateTypeDef h30_state;
extern H30_FrameTypeDef h30_frame;
extern uint32_t h30_last_frame_tick;
extern uint32_t h30_last_debug_tick;
extern uint32_t h30_frame_count;
extern uint32_t h30_checksum_errors;
extern volatile uint32_t h30_rx_bytes;
extern uint32_t h30_sync_count;
extern uint8_t h30_raw_ring[16];
extern uint8_t h30_raw_w;
extern uint32_t h30_uart_baud;
extern uint8_t h30_boot_reported;
extern uint32_t h30_watch_tick;

HAL_StatusTypeDef h30_start_rx(void);
void h30_ensure_rx(void);
void h30_boot_report(void);
void h30_scan_baud(void);
HAL_StatusTypeDef h30_uart_set_baud(uint32_t baud);
void h30_reset_parser(void);
void h30_on_uart_rx_cplt(void);
void h30_uart2_error_recover(void);

#endif
