/**
 * @file    app_ctrl.h
 * @brief   控制源管理：当前指令来自串口(UART)还是手柄(PS2)
 */
#ifndef APP_CTRL_H
#define APP_CTRL_H

#include <stdint.h>

#define CTRL_SRC_UART 0U
#define CTRL_SRC_PS2  1U

extern uint8_t ctrl_source;

const char *ctrl_source_name(void);

#endif
