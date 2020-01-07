#ifndef _LOW_POWER_CTL_H_
#define _LOW_POWER_CTL_H_
#include "stm32l0xx_hal.h"
extern void Low_Power_Enter(void);
extern void Low_Power_Exit(void);
extern void SetDeepSleepMode(void);
extern uint8_t GetDeepSleepMode(void);

#endif




