#include "stm32f1xx_hal.h"

// make pid.c happy
#define TICK_SECOND 1000
#define tick_get HAL_GetTick