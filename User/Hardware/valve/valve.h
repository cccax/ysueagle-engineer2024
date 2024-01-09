#ifndef __VALVE_H__
#define __VALVE_H__

#include "stm32f4xx.h"

//#define PUMP_ORE_ON GPIO_SetBits(GPIOF,GPIO_Pin_10)
//#define PUMP_ORE_OFF GPIO_ResetBits(GPIOF,GPIO_Pin_10)

#define PUMP_ORE_ON GPIO_SetBits(GPIOC,GPIO_Pin_5)
#define PUMP_ORE_OFF GPIO_ResetBits(GPIOC,GPIO_Pin_5)
#define PUMP_ORE_TOGGLE GPIO_ToggleBits(GPIOC,GPIO_Pin_5)
void valve_Init(void);

#endif
