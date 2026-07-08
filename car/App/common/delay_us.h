/**
 * @file    delay_us.h
 * @brief   微秒级延时接口（PS2 时序等场合使用）
 */
#ifndef DELAY_US_H
#define DELAY_US_H

#include <stdint.h>

void delay_us_init(void);
void delay_us(uint32_t us);

#endif
