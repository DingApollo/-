/**
 * @file    cli_shell.h
 * @brief   串口命令行接口：USART3 接收状态、绘图模式开关、命令分发入口
 */
#ifndef CLI_SHELL_H
#define CLI_SHELL_H

#include <stdint.h>
#include "main.h"

extern volatile uint8_t uart3_cmd_ready;
extern char uart3_cmd_buf[96];
extern volatile uint32_t uart3_last_arm_tick;
extern volatile uint32_t uart3_last_rx_tick;
extern volatile uint32_t uart3_last_recover_tick;
extern volatile uint8_t uart3_need_rearm;
extern uint8_t plot_enabled;
extern uint8_t plot_div;
extern uint32_t plot_pause_until;

HAL_StatusTypeDef cli_uart3_start_rx_to_idle(void);
void cli_uart3_soft_recover(void);
void cli_uart3_on_rx_event(UART_HandleTypeDef *huart, uint16_t Size);
void cli_uart3_on_error(UART_HandleTypeDef *huart);
void cli_shell_handle_line(const char *line);

#endif
