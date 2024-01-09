#include "stm32f4xx.h"
#include "timer_send_task.h"
#include "grab_task.h"

#define CAN_DATA_HEADER_SERVO 0x00
#define CAN_DATA_HEADER_VALVE 0x01



void Board_B_ServoAngleSendQueue(CAN_TypeDef* CANx , uint8_t angleSet);


