/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PWM_MAX 4199
#define PWM_START_MIN 2500
#define PWM_RUN_MIN   2200
#define PWM_START_BOOST 2800
#define PWM_START_BOOST_MS 200
#define PID_PERIOD_MS 10
#define PLOT_PERIOD_MS 500
#define PLOT_SEND_DIV (PLOT_PERIOD_MS / PID_PERIOD_MS)
#define PID_PARAM_MAGIC 0x50494431UL
#define PID_PARAM_FLASH_ADDR 0x080E0000UL
#define PID_FLAG_ENABLED 0x00000001UL
#define LEFT_FORWARD_CH   TIM_CHANNEL_1
#define LEFT_BACKWARD_CH  TIM_CHANNEL_2
#define RIGHT_FORWARD_CH  TIM_CHANNEL_3
#define RIGHT_BACKWARD_CH TIM_CHANNEL_4
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t estop_triggered = 0;
int16_t cmd_left = 0;
int16_t cmd_right = 0;
static uint8_t uart3_buf[32];
static char uart3_line_buf[96];
static uint8_t uart3_line_len = 0;
static volatile uint8_t uart3_cmd_ready = 0;
static char uart3_cmd_buf[96];
static uint8_t left_running = 0;
static uint8_t right_running = 0;
static int8_t left_dir = 0;
static int8_t right_dir = 0;
static uint32_t left_boost_until = 0;
static uint32_t right_boost_until = 0;
static uint8_t pid_enabled = 0;
static int16_t target_left = 0;
static int16_t target_right = 0;
static int16_t speed_left = 0;
static int16_t speed_right = 0;
static int16_t pid_out_left = 0;
static int16_t pid_out_right = 0;
static int32_t enc_last_left = 0;
static int32_t enc_last_right = 0;
static float pid_kp = 18.0f;
static float pid_ki = 0.8f;
static float pid_kd = 0.0f;
static float pid_kff = 55.0f;
static float pid_i_left = 0.0f;
static float pid_i_right = 0.0f;
static float pid_prev_e_left = 0.0f;
static float pid_prev_e_right = 0.0f;
static volatile uint8_t control_tick = 0;
static uint8_t plot_enabled = 0;
static uint8_t plot_div = 0;
static uint32_t plot_pause_until = 0;
static volatile uint32_t uart3_last_arm_tick = 0;
static volatile uint32_t uart3_last_rx_tick = 0;
static volatile uint32_t uart3_last_recover_tick = 0;
static volatile uint8_t uart3_need_rearm = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static uint16_t clamp_pwm(int16_t pwm);
static int16_t apply_deadzone_comp(
  int16_t pwm,
  uint8_t *running_flag,
  int8_t *dir_flag,
  uint32_t *boost_until
);
static int16_t pid_step(
  int16_t target,
  int16_t measured,
  float *i_term,
  float *prev_e
);
static void pid_reset_state(void);
static void pid_load_params(void);
static HAL_StatusTypeDef pid_save_params(void);
static HAL_StatusTypeDef uart3_start_rx_to_idle(void);
static void uart3_soft_recover(void);
static void Motor_SetLeft(int16_t pwm);
static void Motor_SetRight(int16_t pwm);
static void Motor_Set(int16_t left, int16_t right);
static void Motor_Stop(void);
static int parse_two_ints(const char *s, int *out_l, int *out_r);
static void VOFA_HandleLine(const char *line);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim3);
  Motor_Stop();
  pid_load_params();
  enc_last_left = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
  enc_last_right = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
  uart3_last_rx_tick = HAL_GetTick();
  uart3_last_recover_tick = HAL_GetTick();
  if (uart3_start_rx_to_idle() != HAL_OK) {
    uart3_need_rearm = 1;
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (uart3_cmd_ready != 0U) {
      char line_local[96];

      __disable_irq();
      memcpy(line_local, uart3_cmd_buf, sizeof(line_local));
      uart3_cmd_ready = 0;
      __enable_irq();

      VOFA_HandleLine(line_local);
    }

    if (control_tick) {
      control_tick = 0;
      {
        int32_t enc_now_left = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
        int32_t enc_now_right = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
        speed_left = (int16_t)(enc_now_left - enc_last_left);
        speed_right = (int16_t)(enc_now_right - enc_last_right);
        enc_last_left = enc_now_left;
        enc_last_right = enc_now_right;
      }

      if (!estop_triggered) {
        if (pid_enabled) {
          if (target_left == 0 && target_right == 0) {
            pid_i_left = 0.0f;
            pid_i_right = 0.0f;
            pid_prev_e_left = 0.0f;
            pid_prev_e_right = 0.0f;
            cmd_left = 0;
            cmd_right = 0;
          } else {
            pid_out_left = pid_step(target_left, speed_left, &pid_i_left, &pid_prev_e_left);
            pid_out_right = pid_step(target_right, speed_right, &pid_i_right, &pid_prev_e_right);
            cmd_left = pid_out_left;
            cmd_right = pid_out_right;
          }
        }
        Motor_Set(cmd_left, cmd_right);
      } else {
        Motor_Stop();
      }

      if (plot_enabled) {
        if (HAL_GetTick() >= plot_pause_until) {
          plot_div++;
          if (plot_div >= PLOT_SEND_DIV) {
            char plot_line[64];
            int plot_n = 0;
            plot_div = 0;
            plot_n = snprintf(
              plot_line,
              sizeof(plot_line),
              "%d,%d,%d,%d\r\n",
              target_left,
              speed_left,
              target_right,
              speed_right
            );
            if (plot_n > 0) {
              HAL_UART_Transmit(&huart3, (uint8_t *)plot_line, (uint16_t)plot_n, 20);
            }
          }
        }
      }
    }

    /* 串口接收自恢复：避免偶发状态机卡死必须按RST */
    if ((uart3_need_rearm != 0U) ||
        ((huart3.RxState == HAL_UART_STATE_READY) && ((HAL_GetTick() - uart3_last_arm_tick) > 100U)))
    {
      if (uart3_start_rx_to_idle() == HAL_OK) {
        uart3_need_rearm = 0;
      }
    }

    /* 若串口状态长期异常，执行一次完整软恢复，避免必须按硬件RST */
    if (((HAL_GetTick() - uart3_last_recover_tick) > 1000U) &&
        (((HAL_GetTick() - uart3_last_rx_tick) > 1500U) || (uart3_need_rearm != 0U)))
    {
      uart3_soft_recover();
    }

    HAL_Delay(1);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static uint16_t clamp_pwm(int16_t pwm)
{
  if (pwm < 0) {
    pwm = -pwm;
  }
  if (pwm > PWM_MAX) {
    pwm = PWM_MAX;
  }
  return (uint16_t)pwm;
}

static int16_t apply_deadzone_comp(
  int16_t pwm,
  uint8_t *running_flag,
  int8_t *dir_flag,
  uint32_t *boost_until
)
{
  int16_t sign = 1;
  int16_t mag = 0;
  uint16_t min_pwm = 0;
  uint32_t now = HAL_GetTick();

  if (pwm == 0) {
    *running_flag = 0;
    *dir_flag = 0;
    *boost_until = 0;
    return 0;
  }

  if (pwm < 0) {
    sign = -1;
    mag = (int16_t)(-pwm);
  } else {
    mag = pwm;
  }

  if ((*running_flag == 0) || (*dir_flag != (int8_t)sign)) {
    *running_flag = 1;
    *dir_flag = (int8_t)sign;
    *boost_until = now + PWM_START_BOOST_MS;
  }

  if (now < *boost_until) {
    min_pwm = PWM_START_BOOST;
  } else {
    min_pwm = PWM_RUN_MIN;
  }

  if (min_pwm < PWM_START_MIN) {
    min_pwm = PWM_START_MIN;
  }

  if (mag < (int16_t)min_pwm) {
    mag = (int16_t)min_pwm;
  }
  if (mag > PWM_MAX) {
    mag = PWM_MAX;
  }
  return (int16_t)(sign * mag);
}

static int16_t pid_step(
  int16_t target,
  int16_t measured,
  float *i_term,
  float *prev_e
)
{
  const float dt_s = (float)PID_PERIOD_MS / 1000.0f;
  float e = (float)target - (float)measured;
  float p_term = 0.0f;
  float d_term = 0.0f;
  float ff_term = 0.0f;
  float out = 0.0f;

  p_term = pid_kp * e;

  *i_term += pid_ki * e * dt_s;
  if (*i_term > (float)PWM_MAX) *i_term = (float)PWM_MAX;
  if (*i_term < -(float)PWM_MAX) *i_term = -(float)PWM_MAX;

  if (dt_s > 0.0f) {
    d_term = pid_kd * (e - *prev_e) / dt_s;
  }

  ff_term = pid_kff * (float)target;
  out = ff_term + p_term + (*i_term) + d_term;
  *prev_e = e;

  if (out > (float)PWM_MAX) out = (float)PWM_MAX;
  if (out < -(float)PWM_MAX) out = -(float)PWM_MAX;
  return (int16_t)out;
}

static void pid_reset_state(void)
{
  pid_i_left = 0.0f;
  pid_i_right = 0.0f;
  pid_prev_e_left = 0.0f;
  pid_prev_e_right = 0.0f;
  pid_out_left = 0;
  pid_out_right = 0;
}

static void pid_load_params(void)
{
  const uint32_t *flash = (const uint32_t *)PID_PARAM_FLASH_ADDR;

  if (flash[0] == PID_PARAM_MAGIC) {
    memcpy(&pid_kp, &flash[1], sizeof(float));
    memcpy(&pid_ki, &flash[2], sizeof(float));
    memcpy(&pid_kd, &flash[3], sizeof(float));
    if (flash[4] != 0xFFFFFFFFUL) {
      pid_enabled = (flash[4] & PID_FLAG_ENABLED) ? 1U : 0U;
    } else {
      pid_enabled = 0;
    }
  }
}

static HAL_StatusTypeDef pid_save_params(void)
{
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t sector_error = 0;
  uint32_t words[5];
  HAL_StatusTypeDef st;

  words[0] = PID_PARAM_MAGIC;
  memcpy(&words[1], &pid_kp, sizeof(float));
  memcpy(&words[2], &pid_ki, sizeof(float));
  memcpy(&words[3], &pid_kd, sizeof(float));
  words[4] = pid_enabled ? PID_FLAG_ENABLED : 0UL;

  HAL_FLASH_Unlock();

  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  erase.Sector = FLASH_SECTOR_11;
  erase.NbSectors = 1;

  st = HAL_FLASHEx_Erase(&erase, &sector_error);
  if (st == HAL_OK) {
    uint32_t addr = PID_PARAM_FLASH_ADDR;
    for (uint32_t i = 0; i < 5; i++) {
      st = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, words[i]);
      if (st != HAL_OK) {
        break;
      }
      addr += 4U;
    }
  }

  HAL_FLASH_Lock();
  return st;
}

static HAL_StatusTypeDef uart3_start_rx_to_idle(void)
{
  HAL_StatusTypeDef st;

  st = HAL_UARTEx_ReceiveToIdle_IT(&huart3, uart3_buf, sizeof(uart3_buf));
  if (st == HAL_OK) {
    uart3_last_arm_tick = HAL_GetTick();
  }
  return st;
}

static void uart3_soft_recover(void)
{
  uart3_last_recover_tick = HAL_GetTick();

  HAL_UART_AbortReceive(&huart3);
  HAL_UART_Abort(&huart3);

  __HAL_UART_CLEAR_OREFLAG(&huart3);
  __HAL_UART_CLEAR_NEFLAG(&huart3);
  __HAL_UART_CLEAR_FEFLAG(&huart3);
  __HAL_UART_CLEAR_PEFLAG(&huart3);
  __HAL_UART_CLEAR_IDLEFLAG(&huart3);

  HAL_UART_DeInit(&huart3);
  HAL_UART_Init(&huart3);

  uart3_need_rearm = 0;
  uart3_line_len = 0;
  uart3_start_rx_to_idle();
}

static void Motor_SetLeft(int16_t pwm)
{
  if (!pid_enabled) {
    pwm = apply_deadzone_comp(pwm, &left_running, &left_dir, &left_boost_until);
  }
  uint16_t duty = clamp_pwm(pwm);

  if (pwm > 0)
  {
    __HAL_TIM_SET_COMPARE(&htim1, LEFT_FORWARD_CH, duty);
    __HAL_TIM_SET_COMPARE(&htim1, LEFT_BACKWARD_CH, 0);
  }
  else if (pwm < 0)
  {
    __HAL_TIM_SET_COMPARE(&htim1, LEFT_FORWARD_CH, 0);
    __HAL_TIM_SET_COMPARE(&htim1, LEFT_BACKWARD_CH, duty);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim1, LEFT_FORWARD_CH, 0);
    __HAL_TIM_SET_COMPARE(&htim1, LEFT_BACKWARD_CH, 0);
  }
}

static void Motor_SetRight(int16_t pwm)
{
  if (!pid_enabled) {
    pwm = apply_deadzone_comp(pwm, &right_running, &right_dir, &right_boost_until);
  }
  uint16_t duty = clamp_pwm(pwm);

  if (pwm > 0)
  {
    __HAL_TIM_SET_COMPARE(&htim1, RIGHT_FORWARD_CH, duty);
    __HAL_TIM_SET_COMPARE(&htim1, RIGHT_BACKWARD_CH, 0);
  }
  else if (pwm < 0)
  {
    __HAL_TIM_SET_COMPARE(&htim1, RIGHT_FORWARD_CH, 0);
    __HAL_TIM_SET_COMPARE(&htim1, RIGHT_BACKWARD_CH, duty);
  }
  else
  {
    __HAL_TIM_SET_COMPARE(&htim1, RIGHT_FORWARD_CH, 0);
    __HAL_TIM_SET_COMPARE(&htim1, RIGHT_BACKWARD_CH, 0);
  }
}

static void Motor_Set(int16_t left, int16_t right)
{
  if (estop_triggered) {
    Motor_Stop();
    return;
  }
  Motor_SetLeft(left);
  Motor_SetRight(right);
}

static void Motor_Stop(void)
{
  left_running = 0;
  right_running = 0;
  left_dir = 0;
  right_dir = 0;
  left_boost_until = 0;
  right_boost_until = 0;
  __HAL_TIM_SET_COMPARE(&htim1, LEFT_FORWARD_CH, 0);
  __HAL_TIM_SET_COMPARE(&htim1, LEFT_BACKWARD_CH, 0);
  __HAL_TIM_SET_COMPARE(&htim1, RIGHT_FORWARD_CH, 0);
  __HAL_TIM_SET_COMPARE(&htim1, RIGHT_BACKWARD_CH, 0);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == ESTOP_Pin) {
    estop_triggered = 1;
    cmd_left = 0;
    cmd_right = 0;
    Motor_Stop();
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == &huart3)
  {
    uart3_last_rx_tick = HAL_GetTick();
    if (Size > sizeof(uart3_buf)) {
      Size = sizeof(uart3_buf);
    }

    for (uint16_t i = 0; i < Size; i++)
    {
      char ch = (char)uart3_buf[i];

      if (ch == '\r' || ch == '\n')
      {
        if (uart3_line_len > 0)
        {
          uart3_line_buf[uart3_line_len] = '\0';
          if (uart3_cmd_ready == 0U) {
            memcpy(uart3_cmd_buf, uart3_line_buf, sizeof(uart3_cmd_buf));
            uart3_cmd_ready = 1;
          }
          uart3_line_len = 0;
        }
      }
      else
      {
        if (uart3_line_len < (sizeof(uart3_line_buf) - 1U))
        {
          uart3_line_buf[uart3_line_len++] = ch;
        }
        else
        {
          /* 行过长则丢弃本行，避免缓冲区污染 */
          uart3_line_len = 0;
        }
      }
    }

    /* 重新开启中断接收 */
    if (uart3_start_rx_to_idle() == HAL_OK) {
      uart3_need_rearm = 0;
    } else {
      uart3_need_rearm = 1;
    }
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart3)
  {
    /* 发生错误时重新开启接收，避免挂死 */
    uart3_last_rx_tick = HAL_GetTick();
    if (uart3_start_rx_to_idle() == HAL_OK) {
      uart3_need_rearm = 0;
    } else {
      uart3_need_rearm = 1;
    }
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3) {
    control_tick = 1;
  }
}

/* 从字符串里提取第一个 "-?\d+,-?\d+" 子串，前缀杂讯会被跳过 */
static int parse_two_ints(const char *s, int *out_l, int *out_r)
{
  const char *p = s;
  while (*p)
  {
    if (*p == '-' || isdigit((unsigned char)*p))
    {
      char *end1;
      long l = strtol(p, &end1, 10);
      if (end1 == p) { p++; continue; }

      const char *q = end1;
      while (*q == ' ' || *q == '\t') q++;
      if (*q != ',') { p = end1; continue; }
      q++; /* skip comma */
      while (*q == ' ' || *q == '\t') q++;

      char *end2;
      long r = strtol(q, &end2, 10);
      if (end2 == q) { p = end1; continue; }

      *out_l = (int)l;
      *out_r = (int)r;
      return 1;
    }
    p++;
  }
  return 0;
}

static void VOFA_HandleLine(const char *line)
{
  int left = 0;
  int right = 0;
  int tmp_left = 0;
  int tmp_right = 0;
  char ack[128];
  char reply[160];
  char clean[32];
  size_t len = 0;
  const char *cmd = clean;
  int n = 0;
  uint8_t boost_l = 0;
  uint8_t boost_r = 0;
  uint32_t now = HAL_GetTick();

  if (line[0] == '\0') {
    return;
  }

  /* 兼容 \r\n / 前后空白：统一裁剪后再做命令匹配 */
  len = strlen(line);
  if (len >= sizeof(clean)) {
    len = sizeof(clean) - 1;
  }
  memcpy(clean, line, len);
  clean[len] = '\0';

  while (*cmd == ' ' || *cmd == '\t' || *cmd == '\r' || *cmd == '\n') {
    cmd++;
  }

  len = strlen(cmd);
  while (len > 0) {
    char tail = cmd[len - 1];
    if (tail == ' ' || tail == '\t' || tail == '\r' || tail == '\n') {
      ((char *)cmd)[len - 1] = '\0';
      len--;
    } else {
      break;
    }
  }

  if (cmd[0] == '\0') {
    return;
  }

  /* 绘图模式下，仅文本命令暂停数字流；纯数值目标命令不暂停 */
  if (!plot_enabled || !parse_two_ints(cmd, &tmp_left, &tmp_right)) {
    plot_pause_until = HAL_GetTick() + 1000U;
  }

  if (strcmp(cmd, "1") == 0)
  {
    const char reply[] = "2\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t *)reply, sizeof(reply) - 1, 20);
    return;
  }

  if (strcmp(cmd, "CLR") == 0 || strcmp(cmd, "ESTOP=0") == 0)
  {
    estop_triggered = 0;
    cmd_left = 0;
    cmd_right = 0;
    target_left = 0;
    target_right = 0;
    pid_reset_state();
    Motor_Stop();
    {
      const char reply[] = "ESTOP:0\r\n";
      HAL_UART_Transmit(&huart3, (uint8_t *)reply, sizeof(reply) - 1, 20);
    }
    return;
  }

  if (strcmp(cmd, "PID=1") == 0)
  {
    pid_enabled = 1;
    target_left = 0;
    target_right = 0;
    cmd_left = 0;
    cmd_right = 0;
    pid_reset_state();
    Motor_Stop();
    {
      const char reply[] = "PID:1\r\n";
      HAL_UART_Transmit(&huart3, (uint8_t *)reply, sizeof(reply) - 1, 20);
    }
    return;
  }

  if (strcmp(cmd, "PID=0") == 0)
  {
    pid_enabled = 0;
    target_left = 0;
    target_right = 0;
    cmd_left = 0;
    cmd_right = 0;
    pid_reset_state();
    Motor_Stop();
    {
      const char reply[] = "PID:0\r\n";
      HAL_UART_Transmit(&huart3, (uint8_t *)reply, sizeof(reply) - 1, 20);
    }
    return;
  }

  if (strcmp(cmd, "PLOT=1") == 0)
  {
    plot_enabled = 1;
    plot_div = 0;
    {
      const char reply[] = "PLOT:1\r\n";
      HAL_UART_Transmit(&huart3, (uint8_t *)reply, sizeof(reply) - 1, 20);
    }
    return;
  }

  if (strcmp(cmd, "PLOT=0") == 0)
  {
    plot_enabled = 0;
    plot_div = 0;
    {
      const char reply[] = "PLOT:0\r\n";
      HAL_UART_Transmit(&huart3, (uint8_t *)reply, sizeof(reply) - 1, 20);
    }
    return;
  }

  if (strcmp(cmd, "PID?") == 0)
  {
    n = snprintf(
      reply,
      sizeof(reply),
      "PID:%u,KP:%.3f,KI:%.3f,KD:%.3f,KFF:%.3f,TGT:%d,%d,SPD:%d,%d\r\n",
      (unsigned int)pid_enabled,
      (double)pid_kp,
      (double)pid_ki,
      (double)pid_kd,
      (double)pid_kff,
      target_left,
      target_right,
      speed_left,
      speed_right
    );
    if (n > 0) {
      HAL_UART_Transmit(&huart3, (uint8_t *)reply, (uint16_t)n, 30);
    }
    return;
  }

  if (strcmp(cmd, "STATUS") == 0 || strcmp(cmd, "MODE?") == 0)
  {
    uint32_t c1 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1);
    uint32_t c2 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2);
    uint32_t c3 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_3);
    uint32_t c4 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_4);
    boost_l = (left_boost_until > now) ? 1u : 0u;
    boost_r = (right_boost_until > now) ? 1u : 0u;
    n = snprintf(
      reply,
      sizeof(reply),
      "STATUS:PID=%u,PLOT=%u,ESTOP=%u,CMD=%d,%d,TGT=%d,%d,SPD=%d,%d,CCR=%lu,%lu,%lu,%lu,BOOST=%u,%u,KP=%.3f,KI=%.3f,KD=%.3f,KFF=%.3f\r\n",
      (unsigned int)pid_enabled,
      (unsigned int)plot_enabled,
      (unsigned int)estop_triggered,
      cmd_left,
      cmd_right,
      target_left,
      target_right,
      speed_left,
      speed_right,
      (unsigned long)c1,
      (unsigned long)c2,
      (unsigned long)c3,
      (unsigned long)c4,
      (unsigned int)boost_l,
      (unsigned int)boost_r,
      (double)pid_kp,
      (double)pid_ki,
      (double)pid_kd,
      (double)pid_kff
    );
    if (n > 0) {
      HAL_UART_Transmit(&huart3, (uint8_t *)reply, (uint16_t)n, 40);
    }
    return;
  }

  if (sscanf(cmd, "KP=%f", &pid_kp) == 1)
  {
    n = snprintf(reply, sizeof(reply), "KP:%.3f\r\n", (double)pid_kp);
    if (n > 0) {
      HAL_UART_Transmit(&huart3, (uint8_t *)reply, (uint16_t)n, 20);
    }
    return;
  }

  if (strcmp(cmd, "SAVE") == 0)
  {
    HAL_StatusTypeDef st = pid_save_params();
    const char *reply_ok = (st == HAL_OK) ? "SAVE:OK\r\n" : "SAVE:ERR\r\n";
    HAL_UART_Transmit(&huart3, (uint8_t *)reply_ok, (uint16_t)strlen(reply_ok), 30);
    return;
  }

  if (sscanf(cmd, "KI=%f", &pid_ki) == 1)
  {
    n = snprintf(reply, sizeof(reply), "KI:%.3f\r\n", (double)pid_ki);
    if (n > 0) {
      HAL_UART_Transmit(&huart3, (uint8_t *)reply, (uint16_t)n, 20);
    }
    return;
  }

  if (sscanf(cmd, "KD=%f", &pid_kd) == 1)
  {
    n = snprintf(reply, sizeof(reply), "KD:%.3f\r\n", (double)pid_kd);
    if (n > 0) {
      HAL_UART_Transmit(&huart3, (uint8_t *)reply, (uint16_t)n, 20);
    }
    return;
  }

  if (sscanf(cmd, "KFF=%f", &pid_kff) == 1)
  {
    n = snprintf(reply, sizeof(reply), "KFF:%.3f\r\n", (double)pid_kff);
    if (n > 0) {
      HAL_UART_Transmit(&huart3, (uint8_t *)reply, (uint16_t)n, 20);
    }
    return;
  }

  if (parse_two_ints(cmd, &left, &right))
  {
    if (left > PWM_MAX) left = PWM_MAX;
    if (left < -PWM_MAX) left = -PWM_MAX;
    if (right > PWM_MAX) right = PWM_MAX;
    if (right < -PWM_MAX) right = -PWM_MAX;

    if (!estop_triggered) {
      if (pid_enabled) {
        target_left = (int16_t)left;
        target_right = (int16_t)right;
        if (target_left == 0 && target_right == 0) {
          pid_i_left = 0.0f;
          pid_i_right = 0.0f;
          pid_prev_e_left = 0.0f;
          pid_prev_e_right = 0.0f;
          cmd_left = 0;
          cmd_right = 0;
          Motor_Stop();
        }
      } else {
        cmd_left = (int16_t)left;
        cmd_right = (int16_t)right;
        /* 开环模式立刻更新 PWM，避免 ACK 显示上一个周期的 CCR */
        Motor_Set(cmd_left, cmd_right);
      }
    } else {
      Motor_Stop();
    }

    uint32_t c1 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1);
    uint32_t c2 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2);
    uint32_t c3 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_3);
    uint32_t c4 = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_4);
    boost_l = (left_boost_until > now) ? 1u : 0u;
    boost_r = (right_boost_until > now) ? 1u : 0u;
    n = snprintf(
      ack,
      sizeof(ack),
      "ACK:%d,%d,%lu,%lu,%lu,%lu,ESTOP:%u,BOOST:%u,%u,PID:%u,TGT:%d,%d,SPD:%d,%d\r\n",
      cmd_left,
      cmd_right,
      (unsigned long)c1,
      (unsigned long)c2,
      (unsigned long)c3,
      (unsigned long)c4,
      (unsigned int)estop_triggered,
      (unsigned int)boost_l,
      (unsigned int)boost_r,
      (unsigned int)pid_enabled,
      target_left,
      target_right,
      speed_left,
      speed_right
    );
    if (n > 0 && !plot_enabled) {
      HAL_UART_Transmit(&huart3, (uint8_t *)ack, (uint16_t)n, 30);
    }
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
