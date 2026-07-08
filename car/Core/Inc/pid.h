#ifndef PID_H
#define PID_H

#include <stdint.h>
#include "main.h"

#define PID_PERIOD_MS 10
/* 串口目标按 0~80 发，内部映射为编码器约 0~40（满速实测） */
#define PID_USER_FULL 80
#define PID_SPEED_FULL 40
#define PID_I_SEP_ERR 8.0f
#define PID_PARAM_MAGIC 0x50494431UL
/* F407VE 512KB：最后一扇区 Sector7 @ 0x08060000（勿用 0x080E0000，仅 1MB 芯片有效） */
#define PID_PARAM_FLASH_ADDR 0x08060000UL
#define PID_PARAM_FLASH_SECTOR FLASH_SECTOR_7
#define PID_FLAG_ENABLED 0x00000001UL

extern uint8_t pid_enabled;
extern int16_t target_left;
extern int16_t target_right;
extern int16_t speed_left;
extern int16_t speed_right;
extern int16_t pid_out_left;
extern int16_t pid_out_right;
extern int32_t enc_last_left;
extern int32_t enc_last_right;
extern float pid_kp;
extern float pid_ki;
extern float pid_kd;
extern float pid_kff;
extern float pid_kff_high;
extern float pid_kff_split;
extern float pid_i_left;
extern float pid_i_right;
extern float pid_prev_e_left;
extern float pid_prev_e_right;

int16_t pid_step(int16_t target, int16_t measured, float *i_term, float *prev_e);
void pid_reset_state(void);
int16_t pid_map_user_speed(int16_t user_spd);
void pid_load_params(void);
HAL_StatusTypeDef pid_save_params(void);

#endif /* PID_H */
