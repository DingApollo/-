#include "lamp.h"
#include "main.h"

uint8_t lamp_on = 0U;

void Lamp_Init(void)
{
  lamp_on = 0U;
  HAL_GPIO_WritePin(LAMP_GPIO_Port, LAMP_Pin, GPIO_PIN_RESET);
}

void Lamp_Set(uint8_t on)
{
  GPIO_PinState pin_on;
  GPIO_PinState pin_off;

#if LAMP_GPIO_ACTIVE_HIGH
  pin_on = GPIO_PIN_SET;
  pin_off = GPIO_PIN_RESET;
#else
  pin_on = GPIO_PIN_RESET;
  pin_off = GPIO_PIN_SET;
#endif

  lamp_on = (on != 0U) ? 1U : 0U;
  HAL_GPIO_WritePin(LAMP_GPIO_Port, LAMP_Pin, lamp_on ? pin_on : pin_off);
}

void Lamp_Toggle(void)
{
  Lamp_Set(lamp_on ^ 1U);
}

uint8_t Lamp_IsOn(void)
{
  return lamp_on;
}
