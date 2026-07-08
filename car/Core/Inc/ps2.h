#ifndef PS2_H
#define PS2_H

#include <stdint.h>
#include "main.h"

#define PS2_SHIFT_DELAY_US 5U
#define PS2_BYTE_DELAY_US  4U
#define PS2_POLL_PERIOD_MS 20U
#define PS2_DEBUG_PERIOD_MS 200U
#define PS2_FAILSAFE_MS 300U
#define PS2_RECONFIG_PERIOD_MS 500U
#define PS2_BAL_QUIET_DEADZONE 32
#define PS2_AXIS_CENTER 128
#define PS2_AXIS_DEADZONE 18
#define PS2_PWM_MAX PWM_MAX
#define PS2_TURN_GAIN_NUM 1
#define PS2_TURN_GAIN_DEN 1
#define PS2_AXIS_BINARY_OUTPUT 0
#define PS2_AXIS_CARDINAL_ONLY 0
#define PS2_BTN_UP     0x0010U
#define PS2_BTN_RIGHT  0x0020U
#define PS2_BTN_DOWN   0x0040U
#define PS2_BTN_LEFT   0x0080U
#define PS2_BTN_CROSS     0x4000U
#define PS2_BTN_CIRCLE    0x2000U
#define PS2_BTN_TRIANGLE  0x1000U
#define PS2_BTN_SQUARE    0x8000U
#define PS2_BTN_L1        0x0400U
#define PS2_BTN_PRESSED(btn, mask) (((btn) & (mask)) == 0U)

#define PS2_CLK_HIGH() HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, GPIO_PIN_SET)
#define PS2_CLK_LOW()  HAL_GPIO_WritePin(PS2_CLK_GPIO_Port, PS2_CLK_Pin, GPIO_PIN_RESET)
#define PS2_CMD_HIGH() HAL_GPIO_WritePin(PS2_CMD_GPIO_Port, PS2_CMD_Pin, GPIO_PIN_SET)
#define PS2_CMD_LOW()  HAL_GPIO_WritePin(PS2_CMD_GPIO_Port, PS2_CMD_Pin, GPIO_PIN_RESET)
#define PS2_CS_HIGH()  HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_SET)
#define PS2_CS_LOW()   HAL_GPIO_WritePin(PS2_CS_GPIO_Port, PS2_CS_Pin, GPIO_PIN_RESET)
#define PS2_DAT_READ() HAL_GPIO_ReadPin(PS2_DAT_GPIO_Port, PS2_DAT_Pin)

extern uint8_t ps2_enabled;
extern uint8_t ps2_online;
extern uint8_t ps2_debug_enabled;
extern uint8_t ps2_mode;
extern uint8_t ps2_frame[9];
extern uint8_t ps2_lx;
extern uint8_t ps2_ly;
extern uint8_t ps2_rx;
extern uint8_t ps2_ry;
extern uint16_t ps2_buttons;
extern uint16_t ps2_buttons_prev;
extern int16_t ps2_cmd_left;
extern int16_t ps2_cmd_right;
extern uint32_t ps2_last_poll_tick;
extern uint32_t ps2_last_ok_tick;
extern uint32_t ps2_last_debug_tick;
extern uint32_t ps2_last_config_tick;

uint8_t ps2_read_frame(uint8_t *frame);
uint8_t ps2_configure(void);
uint8_t ps2_drive_quiet(void);
void ps2_poll(void);
void ps2_disable(void);
void ps2_clear_commands(void);

#endif
