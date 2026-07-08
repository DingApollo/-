/**
 * @file    motor.h
 * @brief   电机驱动接口：PWM 设置/停止/方向映射宏/死区补偿参数
 */
#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include "main.h"
#include "tim.h"

#define PWM_START_MIN 4200
#define PWM_RUN_MIN   4200
#define PWM_START_BOOST 4200
#define PWM_START_BOOST_MS 400
/* 低于此幅值走比例 PWM，不抬到 PWM_MAX 起转（供 H30 平衡用） */
#define MOTOR_PROP_PWM_MAX 1500

/*
 * 1：丝印左轮在 BIN(PA10/11)、右轮在 AIN(PA8/9) 时开启。
 * 映射到 TIM 通道，不在 Motor_Set 里再对调参数（避免与 PS2 转向修正叠两层）。
 */
#define MOTOR_SWAP_LR 1
/* 1：前进/后退与当前相反；左右通道与 MOTOR_SWAP_LR 无关 */
#define MOTOR_SWAP_FB 1

#if MOTOR_SWAP_LR
/* 逻辑左轮 → BIN1/BIN2 */
#define LEFT_FORWARD_CH   TIM_CHANNEL_4
#define LEFT_BACKWARD_CH  TIM_CHANNEL_3
/* 逻辑右轮 → AIN1/AIN2 */
#define RIGHT_FORWARD_CH  TIM_CHANNEL_1
#define RIGHT_BACKWARD_CH TIM_CHANNEL_2
#else
#define LEFT_FORWARD_CH   TIM_CHANNEL_1
#define LEFT_BACKWARD_CH  TIM_CHANNEL_2
#define RIGHT_FORWARD_CH  TIM_CHANNEL_4
#define RIGHT_BACKWARD_CH TIM_CHANNEL_3
#endif

#include "app_state.h"

void Motor_SetLeft(int16_t pwm);
void Motor_SetRight(int16_t pwm);
void Motor_Set(int16_t left, int16_t right);
void Motor_SetProportional(int16_t left, int16_t right);
void Motor_Stop(void);
void motor_reset_ramp_state(void);
uint32_t motor_left_boost_deadline(void);
uint32_t motor_right_boost_deadline(void);

#endif
