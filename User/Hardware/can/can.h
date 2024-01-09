#ifndef CAN_H
#define CAN_H
#include "stm32f4xx.h"
#include "valve.h"

void CAN1_Init(void);
void CAN2_Init(void);
void djiMotorCurrentSendQueue(CAN_TypeDef* CANx, uint32_t stdId, s16 *current, u8 len);

#endif
