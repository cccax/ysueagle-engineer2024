#ifndef TIMER_H
#define TIMER_H
#include "stm32f4xx.h"
#include "main.h"

extern void TIM1_Init(uint16_t arr, uint16_t psc);
extern void TIM3_Init(uint16_t arr, uint16_t psc);
extern void TIM6_Init(void);

#endif
