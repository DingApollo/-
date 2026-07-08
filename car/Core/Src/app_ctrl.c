#include "app_ctrl.h"

uint8_t ctrl_source = CTRL_SRC_UART;

const char *ctrl_source_name(void)
{
  return (ctrl_source == CTRL_SRC_PS2) ? "PS2" : "UART";
}
