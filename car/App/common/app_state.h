/**
 * @file    app_state.h
 * @brief   全局运行状态：急停标志与左右轮当前指令值（main.c 中定义）
 */
#ifndef APP_STATE_H
#define APP_STATE_H

#include <stdint.h>

extern volatile uint8_t estop_triggered;
extern int16_t cmd_left;
extern int16_t cmd_right;

#endif
