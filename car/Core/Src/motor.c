#include "motor.h"

static uint8_t left_running = 0;
static uint8_t right_running = 0;
static int8_t left_dir = 0;
static int8_t right_dir = 0;
static uint32_t left_boost_until = 0;
static uint32_t right_boost_until = 0;

static int16_t motor_map_fb(int16_t pwm)
{
#if MOTOR_SWAP_FB
  return (int16_t)(-pwm);
#else
  return pwm;
#endif
}

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

static void motor_write_pwm(
  int16_t pwm,
  uint32_t forward_ch,
  uint32_t backward_ch
)
{
  uint16_t duty = clamp_pwm(pwm);

  if (pwm > 0) {
    __HAL_TIM_SET_COMPARE(&htim1, forward_ch, duty);
    __HAL_TIM_SET_COMPARE(&htim1, backward_ch, 0);
  } else if (pwm < 0) {
    __HAL_TIM_SET_COMPARE(&htim1, forward_ch, 0);
    __HAL_TIM_SET_COMPARE(&htim1, backward_ch, duty);
  } else {
    __HAL_TIM_SET_COMPARE(&htim1, forward_ch, 0);
    __HAL_TIM_SET_COMPARE(&htim1, backward_ch, 0);
  }
}

void Motor_SetLeft(int16_t pwm)
{
  pwm = motor_map_fb(pwm);
  __HAL_TIM_MOE_ENABLE(&htim1);

  if (pwm != 0) {
    pwm = apply_deadzone_comp(pwm, &left_running, &left_dir, &left_boost_until);
  }
  motor_write_pwm(pwm, LEFT_FORWARD_CH, LEFT_BACKWARD_CH);
}

void Motor_SetRight(int16_t pwm)
{
  pwm = motor_map_fb(pwm);
  __HAL_TIM_MOE_ENABLE(&htim1);

  if (pwm != 0) {
    pwm = apply_deadzone_comp(pwm, &right_running, &right_dir, &right_boost_until);
  }
  motor_write_pwm(pwm, RIGHT_FORWARD_CH, RIGHT_BACKWARD_CH);
}

void Motor_SetProportional(int16_t left, int16_t right)
{
  if (estop_triggered) {
    Motor_Stop();
    return;
  }

  left = motor_map_fb(left);
  right = motor_map_fb(right);
  __HAL_TIM_MOE_ENABLE(&htim1);

  if (left != 0) {
    int16_t mag = left;
    if (mag < 0) {
      mag = (int16_t)(-mag);
    }
    if (mag > (int16_t)MOTOR_PROP_PWM_MAX) {
      left = (left > 0) ? (int16_t)MOTOR_PROP_PWM_MAX : (int16_t)(-MOTOR_PROP_PWM_MAX);
    }
  }
  if (right != 0) {
    int16_t mag = right;
    if (mag < 0) {
      mag = (int16_t)(-mag);
    }
    if (mag > (int16_t)MOTOR_PROP_PWM_MAX) {
      right = (right > 0) ? (int16_t)MOTOR_PROP_PWM_MAX : (int16_t)(-MOTOR_PROP_PWM_MAX);
    }
  }

  motor_write_pwm(left, LEFT_FORWARD_CH, LEFT_BACKWARD_CH);
  motor_write_pwm(right, RIGHT_FORWARD_CH, RIGHT_BACKWARD_CH);
}

void Motor_Set(int16_t left, int16_t right)
{
  if (estop_triggered) {
    Motor_Stop();
    return;
  }
  Motor_SetLeft(left);
  Motor_SetRight(right);
}

void Motor_Stop(void)
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

uint32_t motor_left_boost_deadline(void)
{
  return left_boost_until;
}

uint32_t motor_right_boost_deadline(void)
{
  return right_boost_until;
}

void motor_reset_ramp_state(void)
{
  left_running = 0;
  right_running = 0;
  left_dir = 0;
  right_dir = 0;
  left_boost_until = 0;
  right_boost_until = 0;
}
