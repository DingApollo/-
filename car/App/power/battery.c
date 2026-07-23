/**
 * @file    battery.c
 * @brief   电池电压监测：D157B 分压 → ADC1_IN4(PA4) → 滑动平均 → 分级+迟滞
 *
 * ============ CubeMX 配置（必做，否则编译缺 hadc1）============
 *   1. Pinout：把 PA4 设成 ADC1_IN4
 *   2. Categories → Analog → ADC1：
 *        - 勾选 IN4
 *        - Resolution = 12 bits
 *        - Continuous Conversion Mode = Disabled（软件触发轮询）
 *        - Scan Conversion Mode = Disabled（单通道）
 *        - Rank1 Sampling Time 选最大（480 Cycles）★分压源内阻~900Ω，采样要慢
 *   3. Project → Generate Code（USER CODE 块会保留，放心生成）
 *      生成后会得到 Core/Inc/adc.h、MX_ADC1_Init()、全局 hadc1
 *   4. main.c 里 MX_ADC1_Init() 由 CubeMX 自动加进初始化序列，无需手加
 *   （可选）PA4 对地并一颗 100nF 电容，进一步稳读数
 *
 * 标定：接好后用万用表量真实电池电压，与 BAT? 命令读数比，
 *       BAT_CAL = 万用表值 / 程序读数，填进下面再编译。
 */
#include "battery.h"
#include "adc.h"     /* CubeMX 生成；若报找不到，说明 ADC1 还没在 CubeMX 使能 */
#include "main.h"

/* ===================== 可调参数 ===================== */
#define BAT_DIV_RATIO   11.0f    /* D157B 分压 1/11 → 还原乘 11 */
#define BAT_VREF        3.3f     /* ADC 参考电压 */
#define BAT_ADC_MAX     4095.0f  /* 12bit 满量程 */
#define BAT_CAL         1.000f   /* ★标定系数，实测后微调 */
#define BAT_FILTER_N    8        /* 滑动平均点数 */

/* 阈值(V) + 迟滞（下降立即降级，回升要 +HYST 才升级，防抖） */
#define BAT_WARN_V      11.8f
#define BAT_LIMIT_V     11.2f
#define BAT_CUTOFF_V    10.5f
#define BAT_HYST_V      0.3f

/* ===================== 内部状态 ===================== */
static uint16_t  s_ring[BAT_FILTER_N];
static uint8_t   s_idx;
static uint8_t   s_filled;
static float     s_voltage;
static uint16_t  s_raw;
static Bat_Level s_level;

void Battery_Init(void)
{
    uint8_t i;
    for (i = 0; i < BAT_FILTER_N; i++) s_ring[i] = 0;
    s_idx = 0;
    s_filled = 0;
    s_voltage = 0.0f;
    s_raw = 0;
    s_level = BAT_NORMAL;
}

static uint16_t adc_read_once(void)
{
    uint16_t v = 0;
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
        v = (uint16_t)HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);
    return v;
}

void Battery_Update(void)
{
    uint32_t sum = 0;
    uint8_t  i, n;
    float    avg, v_adc;

    s_raw = adc_read_once();
    s_ring[s_idx] = s_raw;
    s_idx = (uint8_t)((s_idx + 1U) % BAT_FILTER_N);
    if (s_idx == 0U) s_filled = 1U;

    n = s_filled ? (uint8_t)BAT_FILTER_N : s_idx;
    if (n == 0U) n = 1U;
    for (i = 0; i < n; i++) sum += s_ring[i];

    avg   = (float)sum / (float)n;
    v_adc = avg / BAT_ADC_MAX * BAT_VREF;
    s_voltage = v_adc * BAT_DIV_RATIO * BAT_CAL;

    /* 分级 + 迟滞 */
    switch (s_level) {
    case BAT_NORMAL:
        if      (s_voltage < BAT_CUTOFF_V) s_level = BAT_CUTOFF;
        else if (s_voltage < BAT_LIMIT_V)  s_level = BAT_LIMIT;
        else if (s_voltage < BAT_WARN_V)   s_level = BAT_WARN;
        break;
    case BAT_WARN:
        if      (s_voltage < BAT_CUTOFF_V)            s_level = BAT_CUTOFF;
        else if (s_voltage < BAT_LIMIT_V)             s_level = BAT_LIMIT;
        else if (s_voltage > BAT_WARN_V + BAT_HYST_V) s_level = BAT_NORMAL;
        break;
    case BAT_LIMIT:
        if      (s_voltage < BAT_CUTOFF_V)             s_level = BAT_CUTOFF;
        else if (s_voltage > BAT_LIMIT_V + BAT_HYST_V) s_level = BAT_WARN;
        break;
    case BAT_CUTOFF:
        if (s_voltage > BAT_CUTOFF_V + BAT_HYST_V) s_level = BAT_LIMIT;
        break;
    default:
        s_level = BAT_NORMAL;
        break;
    }
}

float     Battery_GetVoltage(void) { return s_voltage; }
uint16_t  Battery_GetRaw(void)     { return s_raw; }
Bat_Level Battery_GetLevel(void)   { return s_level; }
