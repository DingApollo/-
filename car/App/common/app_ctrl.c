/**
 * @file    app_ctrl.c
 * @brief   控制源管理：记录当前指令来源（UART 串口 / PS2 手柄）
 */
#include "app_ctrl.h"

uint8_t ctrl_source = CTRL_SRC_UART;

const char *ctrl_source_name(void)
{
  return (ctrl_source == CTRL_SRC_PS2) ? "PS2" : "UART";
}
