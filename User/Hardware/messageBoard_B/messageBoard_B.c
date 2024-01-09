#include "messageBoard_B.h"

void Board_B_ServoAngleSendQueue(CAN_TypeDef* CANx , uint8_t ServoOre_angleSet)
{
  CanTxMsg can_msg;

  can_msg.StdId = 0x100;
  can_msg.IDE = CAN_ID_STD;
  can_msg.RTR = CAN_RTR_DATA;	// 消息类型为数据帧，一帧8位
  can_msg.DLC = 8;				// 发送8帧信息

  for(uint8_t i = 0; i < 8; i++)
		can_msg.Data[i] = 0;
	
	can_msg.Data[0]=CAN_DATA_HEADER_SERVO;
	can_msg.Data[1]=ServoOre_angleSet;

  if(CANx == CAN1) {
    xQueueSend(CAN1_Queue, &can_msg, 1); //向队列中填充内容
  } else if(CANx == CAN2) {
    xQueueSend(CAN2_Queue, &can_msg, 1); //向队列中填充内容
  }
}	


