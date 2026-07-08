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
#include <stdint.h>
#include "pid.h"
#include "delay_us.h"
#include "motor.h"
#include "h30.h"
#include "balance.h"
#include "ps2.h"
#include "app_ctrl.h"
#include "app_state.h"
#include "cli_shell.h"
#include "lamp.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PLOT_PERIOD_MS 100
#define PLOT_SEND_DIV (PLOT_PERIOD_MS / PID_PERIOD_MS)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t estop_triggered = 0;
int16_t cmd_left = 0;
int16_t cmd_right = 0;
static volatile uint8_t control_tick = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
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
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  MX_GPIO_Init();
  Lamp_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  __HAL_TIM_MOE_ENABLE(&htim1);
  htim1.Instance->BDTR |= TIM_BDTR_MOE;
  htim1.Instance->CR1 |= TIM_CR1_CEN;
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT(&htim3);
  Motor_Stop();
  pid_load_params();
#if MOTOR_SWAP_LR
  enc_last_left = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
  enc_last_right = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
#else
  enc_last_left = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
  enc_last_right = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
#endif
  uart3_last_rx_tick = HAL_GetTick();
  uart3_last_recover_tick = HAL_GetTick();
  delay_us_init();
  ps2_last_ok_tick = HAL_GetTick();
  ps2_last_config_tick = HAL_GetTick();
  if (cli_uart3_start_rx_to_idle() != HAL_OK) {
    uart3_need_rearm = 1;
  }
  if (h30_uart_set_baud(H30_UART_BAUD_DEFAULT) != HAL_OK) {
    (void)MX_USART2_UART_Init();
    (void)h30_start_rx();
  }
#if BAL_AUTO_ENABLE_DEFAULT
  bal_enabled = 1U;
  bal_test_mode = 0U;
  bal_auto_zero_pending = 1U;
  bal_zero_valid = 0U;
  bal_stable_since = 0U;
#endif
  {
    char boot_msg[128];
    int boot_n = snprintf(
      boot_msg,
      sizeof(boot_msg),
      "BOOT:USART3,115200,LAMP=PC8,L1=0x0400,H30=%lu,READY,RST=%s%s%s%s%s%s%s\r\n",
      (unsigned long)H30_UART_BAUD_DEFAULT,
      __HAL_RCC_GET_FLAG(RCC_FLAG_PINRST) ? "PIN|" : "",
      __HAL_RCC_GET_FLAG(RCC_FLAG_PORRST) ? "POR|" : "",
      __HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST) ? "SFT|" : "",
      __HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) ? "IWDG|" : "",
      __HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST) ? "WWDG|" : "",
      __HAL_RCC_GET_FLAG(RCC_FLAG_BORRST) ? "BOR|" : "",
      __HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST) ? "LPWR|" : ""
    );
    if (boot_n > 0) {
      HAL_UART_Transmit(&huart3, (uint8_t *)boot_msg, (uint16_t)boot_n, 30);
    }
    __HAL_RCC_CLEAR_RESET_FLAGS();
  }

  if (ps2_configure() != 0U) {
    ps2_enabled = 1U;
    ctrl_source = CTRL_SRC_PS2;
    pid_enabled = 0U;
    pid_reset_state();
    ps2_last_poll_tick = 0U;
    Motor_Stop();
  }
  /* USER CODE END 2 */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (ps2_enabled == 0U &&
        (HAL_GetTick() - ps2_last_config_tick) >= PS2_RECONFIG_PERIOD_MS) {
      if (ps2_configure() != 0U) {
        ps2_enabled = 1U;
        ctrl_source = CTRL_SRC_PS2;
        pid_enabled = 0U;
        pid_reset_state();
        ps2_last_poll_tick = 0U;
        Motor_Stop();
      }
    }

    if (ps2_enabled != 0U &&
        (HAL_GetTick() - ps2_last_poll_tick) >= PS2_POLL_PERIOD_MS) {
      ps2_last_poll_tick = HAL_GetTick();
      ps2_poll();
    }

    if (ps2_enabled != 0U &&
        ps2_debug_enabled != 0U &&
        !plot_enabled &&
        ((HAL_GetTick() - ps2_last_debug_tick) >= PS2_DEBUG_PERIOD_MS))
    {
      char ps2_line[128];
      int ps2_n = snprintf(
        ps2_line,
        sizeof(ps2_line),
        "PS2:EN=%u,ONLINE=%u,MODE=0x%02X,LX=%u,LY=%u,RX=%u,RY=%u,BTN=0x%04X,CTRL=%s,CMD=%d,%d\r\n",
        (unsigned int)ps2_enabled,
        (unsigned int)ps2_online,
        (unsigned int)ps2_mode,
        (unsigned int)ps2_lx,
        (unsigned int)ps2_ly,
        (unsigned int)ps2_rx,
        (unsigned int)ps2_ry,
        (unsigned int)ps2_buttons,
        ctrl_source_name(),
        ps2_cmd_left,
        ps2_cmd_right
      );
      ps2_last_debug_tick = HAL_GetTick();
      if (ps2_n > 0) {
        HAL_UART_Transmit(&huart3, (uint8_t *)ps2_line, (uint16_t)ps2_n, 40);
      }
    }

    if (h30_online != 0U &&
        ((HAL_GetTick() - h30_last_frame_tick) > H30_ONLINE_TIMEOUT_MS)) {
      h30_online = 0U;
    }

    if (h30_boot_reported == 0U && (HAL_GetTick() >= 800U)) {
      h30_boot_reported = 1U;
      h30_boot_report();
    }

    if ((HAL_GetTick() - h30_watch_tick) >= 1000U) {
      h30_watch_tick = HAL_GetTick();
      h30_ensure_rx();
    }

    balance_auto_update();

    if (h30_debug_enabled != 0U &&
        !plot_enabled &&
        ((HAL_GetTick() - h30_last_debug_tick) >= H30_DEBUG_PERIOD_MS))
    {
      H30_FrameTypeDef frame_local;
      char h30_line[160];
      int h30_n = 0;

      __disable_irq();
      memcpy(&frame_local, &h30_frame, sizeof(frame_local));
      h30_frame_ready = 0U;
      __enable_irq();

      h30_last_debug_tick = HAL_GetTick();
      if (h30_online != 0U) {
        h30_n = snprintf(
          h30_line,
          sizeof(h30_line),
          "H30:1,CNT:%lu,ERR:%lu,PYR:%.3f,%.3f,%.3f,GYR:%.3f,%.3f,%.3f,ACC:%.3f,%.3f,%.3f\r\n",
          (unsigned long)h30_frame_count,
          (unsigned long)h30_checksum_errors,
          (double)frame_local.attitude.pitch * 0.000001,
          (double)frame_local.attitude.roll * 0.000001,
          (double)frame_local.attitude.yaw * 0.000001,
          (double)frame_local.gyro.gx * 0.000001,
          (double)frame_local.gyro.gy * 0.000001,
          (double)frame_local.gyro.gz * 0.000001,
          (double)frame_local.accel.ax * 0.000001,
          (double)frame_local.accel.ay * 0.000001,
          (double)frame_local.accel.az * 0.000001
        );
      } else {
        uint32_t rx_total = h30_rx_bytes;
        h30_n = snprintf(
          h30_line,
          sizeof(h30_line),
          "H30:0,RX:%lu,CNT:%lu,ERR:%lu,SYNC:%lu,BAUD:%lu (无有效帧,查线/YIS=460800)\r\n",
          (unsigned long)rx_total,
          (unsigned long)h30_frame_count,
          (unsigned long)h30_checksum_errors,
          (unsigned long)h30_sync_count,
          (unsigned long)h30_uart_baud
        );
      }
      if (h30_n > 0) {
        uint16_t tx_len = (h30_n < (int)sizeof(h30_line)) ? (uint16_t)h30_n : (uint16_t)(sizeof(h30_line) - 1U);
        HAL_UART_Transmit(&huart3, (uint8_t *)h30_line, tx_len, 40);
      }
    }

    if (bal_debug_enabled != 0U &&
        bal_enabled != 0U &&
        !plot_enabled &&
        ((HAL_GetTick() - bal_last_debug_tick) >= BAL_DEBUG_PERIOD_MS))
    {
      char bal_line[160];
      int bal_n = 0;

      bal_last_debug_tick = HAL_GetTick();
      bal_n = snprintf(
        bal_line,
        sizeof(bal_line),
        "BAL:1,STA:%u,ZV:%u,FALL:%u,PE:%.3f,RE:%.3f,PIT:%.3f,ROL:%.3f,MOT:%d,%d\r\n",
        (unsigned int)bal_last_sta,
        (unsigned int)bal_zero_valid,
        (unsigned int)bal_fallen,
        (double)bal_last_pitch_err,
        (double)bal_last_roll_err,
        (double)bal_last_pitch,
        (double)bal_last_roll,
        bal_last_motor_l,
        bal_last_motor_r
      );
      if (bal_n > 0) {
        uint16_t tx_len = (bal_n < (int)sizeof(bal_line)) ? (uint16_t)bal_n : (uint16_t)(sizeof(bal_line) - 1U);
        HAL_UART_Transmit(&huart3, (uint8_t *)bal_line, tx_len, 40);
      }
    }

    if (uart3_cmd_ready != 0U) {
      char line_local[96];

      __disable_irq();
      memcpy(line_local, uart3_cmd_buf, sizeof(line_local));
      uart3_cmd_ready = 0;
      __enable_irq();

      cli_shell_handle_line(line_local);
    }

    if (control_tick) {
      control_tick = 0;
      {
        int32_t enc_now_left;
        int32_t enc_now_right;
#if MOTOR_SWAP_LR
        enc_now_left = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
        enc_now_right = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
#else
        enc_now_left = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
        enc_now_right = (int32_t)__HAL_TIM_GET_COUNTER(&htim4);
#endif
        speed_left = (int16_t)(enc_now_left - enc_last_left);
        speed_right = (int16_t)(enc_now_right - enc_last_right);
#if MOTOR_SWAP_FB
        speed_left = (int16_t)(-speed_left);
        speed_right = (int16_t)(-speed_right);
#endif
        enc_last_left = enc_now_left;
        enc_last_right = enc_now_right;
      }

      if (!estop_triggered) {
        if (ctrl_source == CTRL_SRC_PS2) {
          if ((ps2_enabled == 0U) ||
              (ps2_online == 0U) ||
              ((HAL_GetTick() - ps2_last_ok_tick) > PS2_FAILSAFE_MS))
          {
            ps2_cmd_left = 0;
            ps2_cmd_right = 0;
            cmd_left = 0;
            cmd_right = 0;
            target_left = 0;
            target_right = 0;
            pid_reset_state();
          } else {
            cmd_left = ps2_cmd_left;
            cmd_right = ps2_cmd_right;
            target_left = 0;
            target_right = 0;
            pid_reset_state();
          }
        } else if (pid_enabled) {
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
        {
          int16_t motor_l = cmd_left;
          int16_t motor_r = cmd_right;
          uint8_t balance_allowed = 1U;

          if (ctrl_source == CTRL_SRC_PS2 &&
              bal_test_mode == 0U &&
              ps2_drive_quiet() == 0U)
          {
            balance_allowed = 0U;
            bal_last_sta = 7U;
            bal_last_u_pitch = 0.0f;
            bal_last_u_roll = 0.0f;
            bal_last_motor_l = motor_l;
            bal_last_motor_r = motor_r;
          }

          if (balance_allowed != 0U) {
            balance_apply(&motor_l, &motor_r);
          }
          bal_output_active = (motor_l != cmd_left || motor_r != cmd_right) ? 1U : 0U;
          /* 仅 H30 平衡修正、无手柄/串口指令时用比例 PWM，避免被起转死区抬到满速 */
          if (bal_output_active != 0U && cmd_left == 0 && cmd_right == 0) {
            Motor_SetProportional(motor_l, motor_r);
          } else {
            Motor_Set(motor_l, motor_r);
          }
          bal_output_active = 0U;
        }
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

    if ((uart3_need_rearm != 0U) ||
        ((huart3.RxState == HAL_UART_STATE_READY) && ((HAL_GetTick() - uart3_last_arm_tick) > 100U)))
    {
      if (cli_uart3_start_rx_to_idle() == HAL_OK) {
        uart3_need_rearm = 0;
      }
    }

    if (((HAL_GetTick() - uart3_last_recover_tick) > 1000U) &&
        ((uart3_need_rearm != 0U) || (huart3.ErrorCode != HAL_UART_ERROR_NONE)))
    {
      cli_uart3_soft_recover();
    }

    HAL_Delay(1);
    /* USER CODE END 3 */
  }
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
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == ESTOP_Pin) {
    estop_triggered = 1;
    cmd_left = 0;
    cmd_right = 0;
    target_left = 0;
    target_right = 0;
    ps2_clear_commands();
    pid_reset_state();
    Motor_Stop();
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  cli_uart3_on_rx_event(huart, Size);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2) {
    h30_on_uart_rx_cplt();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart3) {
    cli_uart3_on_error(huart);
  } else if (huart == &huart2) {
    h30_uart2_error_recover();
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM3) {
    control_tick = 1;
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
