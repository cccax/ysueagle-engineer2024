#ifndef __SERVO_H__
#define __SERVO_H__

#include "stm32f4xx.h"

#define CAM_SERVO 0
#define DEPO_SERVO 1

#define CAM_ANGLE_NORMAL 178
#define CAM_ANGLE_GRAB 0
#define CAM_ANGLE_VERTICAL 90

void servo_configuration(void);
void setServoAngle(uint8_t ID, uint16_t angle);
void setScreenChannel(uint8_t newState);

#endif
