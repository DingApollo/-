/**
 * @file    delay_us.c
 * @brief   微秒延时：优先 DWT 周期计数器，失败退化为 NOP 循环估时
 * @note    供 PS2 软件时序等使用；依赖 SystemCoreClock
 */
#include "delay_us.h"
#include "main.h"

static uint8_t s_dwt_ready = 0U;

void delay_us_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  s_dwt_ready = ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U) ? 1U : 0U;
}

void delay_us(uint32_t us)
{
  if ((us == 0U) || (SystemCoreClock == 0U)) {
    return;
  }

  if (s_dwt_ready != 0U) {
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = (SystemCoreClock / 1000000U) * us;
    while ((DWT->CYCCNT - start) < ticks) {
    }
  } else {
    volatile uint32_t loops = (SystemCoreClock / 8000000U) * us;
    while (loops-- > 0U) {
      __NOP();
    }
  }
}
