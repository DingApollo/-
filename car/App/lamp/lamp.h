/**
 * @file    lamp.h
 * @brief   示警灯接口：开/关/翻转/查询
 */
#ifndef LAMP_H
#define LAMP_H

#include <stdint.h>

/* 示警灯：PC8 → MOS 的 SIGN。高电平=亮；若不亮可改 LAMP_GPIO_ACTIVE_HIGH 为 0 */
#ifndef LAMP_GPIO_ACTIVE_HIGH
#define LAMP_GPIO_ACTIVE_HIGH 1U
#endif

extern uint8_t lamp_on;

void Lamp_Init(void);
void Lamp_Set(uint8_t on);
void Lamp_Toggle(void);
uint8_t Lamp_IsOn(void);

#endif
