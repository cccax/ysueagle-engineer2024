#ifndef BUZZER_H
#define BUZZER_H

#include "stm32f4xx.h"
#include "delay.h"

extern void buzzer_configuration(void);
extern void buzzerCtrl(void);
extern void buzzerOn(uint16_t timeInMs, uint16_t repeatNum, uint8_t freq);

#endif
