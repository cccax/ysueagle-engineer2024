#include "stm32f4xx.h"
#include <stdio.h>
#include <can.h>
#include <mf9025.h>
#include "kalman.h"
#include "can.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "grab_task.h"

extern QueueHandle_t	CAN1_Queue;					//CAN1消息队列句柄
extern QueueHandle_t	CAN2_Queue;					//CAN2消息队列句柄

//位置闭环控制命令1
//返回值 ：0 成功。其他 失败

void MF9025_Ecd1_CtrlAll(CAN_TypeDef* CANx ,float angleControl)
{
	MF9025_Motor_Ecd1Ctrl(CANx,1,angleControl);
	MF9025_Motor_Ecd1Ctrl(CANx,2,angleControl);
}


void MF9025_Motor_Ecd1Ctrl(CAN_TypeDef* CANx , uint8_t id , float angleControl) //angleControl 0~35999 对应0~359.99度 
{
//	if(angleControl>35999)			 //角度限幅，由mf9025手册定义
//			return ;
	
	CanTxMsg can_msg; //定义CAN结构体
	int32_t angleNum_Send;
	//自动根据安装的id判断转向正负
	if(id == 1) angleNum_Send=angleControl*100;
	else if(id == 2) angleNum_Send=-angleControl*100;
	
	//CAN发送↓
  can_msg.StdId = GRAB_CANBUS_SEND_MF9025+id;  //MF9025标识符
  can_msg.IDE = CAN_ID_STD;  //帧类型
  can_msg.RTR = CAN_RTR_DATA;	// 帧格式为数据帧，一帧8位
  can_msg.DLC = 8;				// 发送8帧信息
	
  u8 i = 0;
  for(i = 0; i < 8; i++) can_msg.Data[i] = 0; //数组清零
	
	
	
	can_msg.Data[0]=0xA3;
	can_msg.Data[1]=0x00;
	can_msg.Data[2]=0x00;
	can_msg.Data[3]=0x00;
	can_msg.Data[4]=angleNum_Send&0xFF;
	can_msg.Data[5]=(angleNum_Send&0xFF00)>>8;
	can_msg.Data[6]=(angleNum_Send&0xFF0000)>>16;
	can_msg.Data[7]=(angleNum_Send&0xFF000000)>>24;
	
  if(CANx == CAN1) {
    xQueueSend(CAN1_Queue, &can_msg, 1); //向队列中填充内容
  } else if(CANx == CAN2) {
    xQueueSend(CAN2_Queue, &can_msg, 1); //向队列中填充内容
  }

}


//位置闭环控制命令3
//返回值 ：0 成功。其他 失败
void MF9025_Motor_Ecd3Ctrl(CAN_TypeDef* CANx , uint8_t spinDirection , uint16_t angleControl) //spinDirection 0顺1逆；angleControl 0~35999 对应0~359.99度
{
	if(angleControl>35999)			 //角度限幅，由mf9025手册定义
			return ;
	
	CanTxMsg can_msg; //定义CAN结构体
	//CAN发送↓
  can_msg.StdId = GRAB_CANBUS_SEND_MF9025+1;  //MF9025标识符
  can_msg.IDE = CAN_ID_STD;  //帧类型
  can_msg.RTR = CAN_RTR_DATA;	// 帧格式为数据帧，一帧8位
  can_msg.DLC = 8;				// 发送8帧信息
	
  u8 i = 0;
  for(i = 0; i < 8; i++) can_msg.Data[i] = 0; //数组清零
	
	can_msg.Data[0]=0xA5;
	can_msg.Data[1]=spinDirection;
	can_msg.Data[2]=0x00;
	can_msg.Data[3]=0x00;
	can_msg.Data[4]=angleControl&0xFF;
	can_msg.Data[5]=(angleControl&0xFF00)>>8;
	can_msg.Data[6]=0x00;
	can_msg.Data[7]=0x00;
	
  if(CANx == CAN1) {
    xQueueSend(CAN1_Queue, &can_msg, 1); //向队列中填充内容
  } else if(CANx == CAN2) {
    xQueueSend(CAN2_Queue, &can_msg, 1); //向队列中填充内容
  }

}
void MF9025_Motor_Ecd_SetZero(CAN_TypeDef* CANx , uint8_t id)
{
	CanTxMsg can_msg; //定义CAN结构体
	//CAN发送↓
  can_msg.StdId = GRAB_CANBUS_SEND_MF9025+id;  //MF9025标识符
  can_msg.IDE = CAN_ID_STD;  //帧类型
  can_msg.RTR = CAN_RTR_DATA;	// 帧格式为数据帧，一帧8位
  can_msg.DLC = 8;				// 发送8帧信息
	
  u8 i = 0;
  for(i = 0; i < 8; i++) can_msg.Data[i] = 0; //数组清零
	
	can_msg.Data[0]=0x19;
	can_msg.Data[1]=0x00;
	can_msg.Data[2]=0x00;
	can_msg.Data[3]=0x00;
	can_msg.Data[4]=0x00;
	can_msg.Data[5]=0x00;
	can_msg.Data[6]=0x00;
	can_msg.Data[7]=0x00;
	
  if(CANx == CAN1) {
    xQueueSend(CAN1_Queue, &can_msg, 1); //向队列中填充内容
  } else if(CANx == CAN2) {
    xQueueSend(CAN2_Queue, &can_msg, 1); //向队列中填充内容
  }
}

//pid设置命令
void MF9025_Motor_Pid_Set(CAN_TypeDef* CANx , uint8_t id)
{
	CanTxMsg can_msg; //定义CAN结构体
	//CAN发送↓
  can_msg.StdId = GRAB_CANBUS_SEND_MF9025+id;  //MF9025标识符
  can_msg.IDE = CAN_ID_STD;  //帧类型
  can_msg.RTR = CAN_RTR_DATA;	// 帧格式为数据帧，一帧8位
  can_msg.DLC = 8;				// 发送8帧信息
	
  u8 i = 0;
  for(i = 0; i < 8; i++) can_msg.Data[i] = 0; //数组清零
	
	can_msg.Data[0]=0x32;
	can_msg.Data[1]=0x00;
	can_msg.Data[2]=100; //angle kp
	can_msg.Data[3]=100; //angle ki
	can_msg.Data[4]=165; //speed kp
	can_msg.Data[5]=40; //speed ki
	can_msg.Data[6]=50; //iq kp
	can_msg.Data[7]=50; //iq ki
	
	
  if(CANx == CAN1) {
    xQueueSend(CAN1_Queue, &can_msg, 1); //向队列中填充内容
  } else if(CANx == CAN2) {
    xQueueSend(CAN2_Queue, &can_msg, 1); //向队列中填充内容
  }
}
//电机关闭命令
void MF9025_Motor_Stop(CAN_TypeDef* CANx , uint8_t id)
{
	CanTxMsg can_msg; //定义CAN结构体
	//CAN发送↓
  can_msg.StdId = GRAB_CANBUS_SEND_MF9025+id;  //MF9025标识符
  can_msg.IDE = CAN_ID_STD;  //帧类型
  can_msg.RTR = CAN_RTR_DATA;	// 帧格式为数据帧，一帧8位
  can_msg.DLC = 8;				// 发送8帧信息
	
  u8 i = 0;
  for(i = 0; i < 8; i++) can_msg.Data[i] = 0; //数组清零
	
	can_msg.Data[0]=0x80;
	can_msg.Data[1]=0x00;
	can_msg.Data[2]=0x00; 
	can_msg.Data[3]=0x00; 
	can_msg.Data[4]=0x00; 
	can_msg.Data[5]=0x00; 
	can_msg.Data[6]=0x00; 
	can_msg.Data[7]=0x00; 
	
	
  if(CANx == CAN1) {
    xQueueSend(CAN1_Queue, &can_msg, 1); //向队列中填充内容
  } else if(CANx == CAN2) {
    xQueueSend(CAN2_Queue, &can_msg, 1); //向队列中填充内容
  }
}

//电机恢复命令
void MF9025_Motor_Resume(CAN_TypeDef* CANx , uint8_t id)
{
	CanTxMsg can_msg; //定义CAN结构体
	//CAN发送↓
  can_msg.StdId = GRAB_CANBUS_SEND_MF9025+id;  //MF9025标识符
  can_msg.IDE = CAN_ID_STD;  //帧类型
  can_msg.RTR = CAN_RTR_DATA;	// 帧格式为数据帧，一帧8位
  can_msg.DLC = 8;				// 发送8帧信息
	
  u8 i = 0;
  for(i = 0; i < 8; i++) can_msg.Data[i] = 0; //数组清零
	
	can_msg.Data[0]=0x88;
	can_msg.Data[1]=0x00;
	can_msg.Data[2]=0x00; 
	can_msg.Data[3]=0x00; 
	can_msg.Data[4]=0x00; 
	can_msg.Data[5]=0x00; 
	can_msg.Data[6]=0x00; 
	can_msg.Data[7]=0x00; 
	
	
  if(CANx == CAN1) {
    xQueueSend(CAN1_Queue, &can_msg, 1); //向队列中填充内容
  } else if(CANx == CAN2) {
    xQueueSend(CAN2_Queue, &can_msg, 1); //向队列中填充内容
  }
}

void MF9025_CurrentSendQueue(CAN_TypeDef* CANx , uint8_t id, int16_t iqControl)
{
	CanTxMsg can_msg; //定义CAN结构体
	//CAN发送↓
  can_msg.StdId = GRAB_CANBUS_SEND_MF9025+id;  //MF9025标识符
  can_msg.IDE = CAN_ID_STD;  //帧类型
  can_msg.RTR = CAN_RTR_DATA;	// 帧格式为数据帧，一帧8位
  can_msg.DLC = 8;				// 发送8帧信息
	
  u8 i = 0;
  for(i = 0; i < 8; i++) can_msg.Data[i] = 0; //数组清零
	
	can_msg.Data[0]=0xA1; 
	can_msg.Data[1]=0x00; 
	can_msg.Data[2]=0x00; 
	can_msg.Data[3]=0x00; 
	can_msg.Data[4]=iqControl&0xFF;
	can_msg.Data[5]=(iqControl&0xFF00)>>8;
	can_msg.Data[6]=0x00; 
	can_msg.Data[7]=0x00; 
	
	
  if(CANx == CAN1) {
    xQueueSend(CAN1_Queue, &can_msg, 1); //向队列中填充内容
  } else if(CANx == CAN2) {
    xQueueSend(CAN2_Queue, &can_msg, 1); //向队列中填充内容
  }
}

