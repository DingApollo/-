/**
 * @file    battery.h
 * @brief   电池电压监测（D157B 自带 1/11 分压 → ADC）+ 欠压分级
 * @note    硬件: D157B 的 ADC 脚 → 杜邦线 → 主控 PA4 (ADC1_IN4)
 *          分压: D157B 板上 10k + 1k = 1/11, 故 V_bat = V_adc × 11
 *          ★ 需先在 CubeMX 使能 ADC1 + PA4，步骤见 battery.c 顶部
 *
 * 用法(main.c):
 *   Battery_Init();                 // 上电一次
 *   每 ~100ms: Battery_Update();    // 采样+滤波+分级
 *   Battery_GetVoltage() / GetLevel()  // 读结果
 */
#ifndef BATTERY_H
#define BATTERY_H

#include <stdint.h>

/* 欠压分级（磷酸铁锂 4 串，标称 12.8V，满充 14.6V） */
typedef enum {
    BAT_NORMAL = 0,   /* 正常 */
    BAT_WARN,         /* 低电量告警（灯闪/上报） */
    BAT_LIMIT,        /* 限速 */
    BAT_CUTOFF        /* 禁止启动 / 停机 */
} Bat_Level;

void      Battery_Init(void);
void      Battery_Update(void);      /* 主循环每 ~100ms 调一次 */
float     Battery_GetVoltage(void);  /* 最新电池电压(V) */
uint16_t  Battery_GetRaw(void);      /* 最新 ADC 原始值(标定/调试用) */
Bat_Level Battery_GetLevel(void);    /* 当前分级 */

#endif /* BATTERY_H */
