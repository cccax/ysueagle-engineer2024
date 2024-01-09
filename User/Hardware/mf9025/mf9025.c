#include "stm32f4xx.h"
#include <stdio.h>
#include <can.h>
#include <mf9025.h>
#include "kalman.h"
#include "can.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "grab_task.h"

extern QueueHandle_t	CAN1_Queue;					//CAN1��Ϣ���о��
extern QueueHandle_t	CAN2_Queue;					//CAN2��Ϣ���о��

//λ�ñջ���������1
//����ֵ ��0 �ɹ������� ʧ��

void MF9025_Ecd1_CtrlAll(CAN_TypeDef* CANx ,float angleControl)
{
	MF9025_Motor_Ecd1Ctrl(CANx,1,angleControl);
	MF9025_Motor_Ecd1Ctrl(CANx,2,angleControl);
}


void MF9025_Motor_Ecd1Ctrl(CAN_TypeDef* CANx , uint8_t id , float angleControl) //angleControl 0~35999 ��Ӧ0~359.99�� 
{
//	if(angleControl>35999)			 //�Ƕ��޷�����mf9025�ֲᶨ��
//			return ;
	
	CanTxMsg can_msg; //����CAN�ṹ��
	int32_t angleNum_Send;
	//�Զ����ݰ�װ��id�ж�ת������
	if(id == 1) angleNum_Send=angleControl*100;
	else if(id == 2) angleNum_Send=-angleControl*100;
	
	//CAN���͡�
  can_msg.StdId = GRAB_CANBUS_SEND_MF9025+id;  //MF9025��ʶ��
  can_msg.IDE = CAN_ID_STD;  //֡����
  can_msg.RTR = CAN_RTR_DATA;	// ֡��ʽΪ����֡��һ֡8λ
  can_msg.DLC = 8;				// ����8֡��Ϣ
	
  u8 i = 0;
  for(i = 0; i < 8; i++) can_msg.Data[i] = 0; //��������
	
	
	
	can_msg.Data[0]=0xA3;
	can_msg.Data[1]=0x00;
	can_msg.Data[2]=0x00;
	can_msg.Data[3]=0x00;
	can_msg.Data[4]=angleNum_Send&0xFF;
	can_msg.Data[5]=(angleNum_Send&0xFF00)>>8;
	can_msg.Data[6]=(angleNum_Send&0xFF0000)>>16;
	can_msg.Data[7]=(angleNum_Send&0xFF000000)>>24;
	
  if(CANx == CAN1) {
    xQueueSend(CAN1_Queue, &can_msg, 1); //��������������
  } else if(CANx == CAN2) {
    xQueueSend(CAN2_Queue, &can_msg, 1); //��������������
  }

}


//λ�ñջ���������3
//����ֵ ��0 �ɹ������� ʧ��
void MF9025_Motor_Ecd3Ctrl(CAN_TypeDef* CANx , uint8_t spinDirection , uint16_t angleControl) //spinDirection 0˳1�棻angleControl 0~35999 ��Ӧ0~359.99��
{
	if(angleControl>35999)			 //�Ƕ��޷�����mf9025�ֲᶨ��
			return ;
	
	CanTxMsg can_msg; //����CAN�ṹ��
	//CAN���͡�
  can_msg.StdId = GRAB_CANBUS_SEND_MF9025+1;  //MF9025��ʶ��
  can_msg.IDE = CAN_ID_STD;  //֡����
  can_msg.RTR = CAN_RTR_DATA;	// ֡��ʽΪ����֡��һ֡8λ
  can_msg.DLC = 8;				// ����8֡��Ϣ
	
  u8 i = 0;
  for(i = 0; i < 8; i++) can_msg.Data[i] = 0; //��������
	
	can_msg.Data[0]=0xA5;
	can_msg.Data[1]=spinDirection;
	can_msg.Data[2]=0x00;
	can_msg.Data[3]=0x00;
	can_msg.Data[4]=angleControl&0xFF;
	can_msg.Data[5]=(angleControl&0xFF00)>>8;
	can_msg.Data[6]=0x00;
	can_msg.Data[7]=0x00;
	
  if(CANx == CAN1) {
    xQueueSend(CAN1_Queue, &can_msg, 1); //��������������
  } else if(CANx == CAN2) {
    xQueueSend(CAN2_Queue, &can_msg, 1); //��������������
  }

}
void MF9025_Motor_Ecd_SetZero(CAN_TypeDef* CANx , uint8_t id)
{
	CanTxMsg can_msg; //����CAN�ṹ��
	//CAN���͡�
  can_msg.StdId = GRAB_CANBUS_SEND_MF9025+id;  //MF9025��ʶ��
  can_msg.IDE = CAN_ID_STD;  //֡����
  can_msg.RTR = CAN_RTR_DATA;	// ֡��ʽΪ����֡��һ֡8λ
  can_msg.DLC = 8;				// ����8֡��Ϣ
	
  u8 i = 0;
  for(i = 0; i < 8; i++) can_msg.Data[i] = 0; //��������
	
	can_msg.Data[0]=0x19;
	can_msg.Data[1]=0x00;
	can_msg.Data[2]=0x00;
	can_msg.Data[3]=0x00;
	can_msg.Data[4]=0x00;
	can_msg.Data[5]=0x00;
	can_msg.Data[6]=0x00;
	can_msg.Data[7]=0x00;
	
  if(CANx == CAN1) {
    xQueueSend(CAN1_Queue, &can_msg, 1); //��������������
  } else if(CANx == CAN2) {
    xQueueSend(CAN2_Queue, &can_msg, 1); //��������������
  }
}

//pid��������
void MF9025_Motor_Pid_Set(CAN_TypeDef* CANx , uint8_t id)
{
	CanTxMsg can_msg; //����CAN�ṹ��
	//CAN���͡�
  can_msg.StdId = GRAB_CANBUS_SEND_MF9025+id;  //MF9025��ʶ��
  can_msg.IDE = CAN_ID_STD;  //֡����
  can_msg.RTR = CAN_RTR_DATA;	// ֡��ʽΪ����֡��һ֡8λ
  can_msg.DLC = 8;				// ����8֡��Ϣ
	
  u8 i = 0;
  for(i = 0; i < 8; i++) can_msg.Data[i] = 0; //��������
	
	can_msg.Data[0]=0x32;
	can_msg.Data[1]=0x00;
	can_msg.Data[2]=100; //angle kp
	can_msg.Data[3]=100; //angle ki
	can_msg.Data[4]=165; //speed kp
	can_msg.Data[5]=40; //speed ki
	can_msg.Data[6]=50; //iq kp
	can_msg.Data[7]=50; //iq ki
	
	
  if(CANx == CAN1) {
    xQueueSend(CAN1_Queue, &can_msg, 1); //��������������
  } else if(CANx == CAN2) {
    xQueueSend(CAN2_Queue, &can_msg, 1); //��������������
  }
}
//����ر�����
void MF9025_Motor_Stop(CAN_TypeDef* CANx , uint8_t id)
{
	CanTxMsg can_msg; //����CAN�ṹ��
	//CAN���͡�
  can_msg.StdId = GRAB_CANBUS_SEND_MF9025+id;  //MF9025��ʶ��
  can_msg.IDE = CAN_ID_STD;  //֡����
  can_msg.RTR = CAN_RTR_DATA;	// ֡��ʽΪ����֡��һ֡8λ
  can_msg.DLC = 8;				// ����8֡��Ϣ
	
  u8 i = 0;
  for(i = 0; i < 8; i++) can_msg.Data[i] = 0; //��������
	
	can_msg.Data[0]=0x80;
	can_msg.Data[1]=0x00;
	can_msg.Data[2]=0x00; 
	can_msg.Data[3]=0x00; 
	can_msg.Data[4]=0x00; 
	can_msg.Data[5]=0x00; 
	can_msg.Data[6]=0x00; 
	can_msg.Data[7]=0x00; 
	
	
  if(CANx == CAN1) {
    xQueueSend(CAN1_Queue, &can_msg, 1); //��������������
  } else if(CANx == CAN2) {
    xQueueSend(CAN2_Queue, &can_msg, 1); //��������������
  }
}

//����ָ�����
void MF9025_Motor_Resume(CAN_TypeDef* CANx , uint8_t id)
{
	CanTxMsg can_msg; //����CAN�ṹ��
	//CAN���͡�
  can_msg.StdId = GRAB_CANBUS_SEND_MF9025+id;  //MF9025��ʶ��
  can_msg.IDE = CAN_ID_STD;  //֡����
  can_msg.RTR = CAN_RTR_DATA;	// ֡��ʽΪ����֡��һ֡8λ
  can_msg.DLC = 8;				// ����8֡��Ϣ
	
  u8 i = 0;
  for(i = 0; i < 8; i++) can_msg.Data[i] = 0; //��������
	
	can_msg.Data[0]=0x88;
	can_msg.Data[1]=0x00;
	can_msg.Data[2]=0x00; 
	can_msg.Data[3]=0x00; 
	can_msg.Data[4]=0x00; 
	can_msg.Data[5]=0x00; 
	can_msg.Data[6]=0x00; 
	can_msg.Data[7]=0x00; 
	
	
  if(CANx == CAN1) {
    xQueueSend(CAN1_Queue, &can_msg, 1); //��������������
  } else if(CANx == CAN2) {
    xQueueSend(CAN2_Queue, &can_msg, 1); //��������������
  }
}

void MF9025_CurrentSendQueue(CAN_TypeDef* CANx , uint8_t id, int16_t iqControl)
{
	CanTxMsg can_msg; //����CAN�ṹ��
	//CAN���͡�
  can_msg.StdId = GRAB_CANBUS_SEND_MF9025+id;  //MF9025��ʶ��
  can_msg.IDE = CAN_ID_STD;  //֡����
  can_msg.RTR = CAN_RTR_DATA;	// ֡��ʽΪ����֡��һ֡8λ
  can_msg.DLC = 8;				// ����8֡��Ϣ
	
  u8 i = 0;
  for(i = 0; i < 8; i++) can_msg.Data[i] = 0; //��������
	
	can_msg.Data[0]=0xA1; 
	can_msg.Data[1]=0x00; 
	can_msg.Data[2]=0x00; 
	can_msg.Data[3]=0x00; 
	can_msg.Data[4]=iqControl&0xFF;
	can_msg.Data[5]=(iqControl&0xFF00)>>8;
	can_msg.Data[6]=0x00; 
	can_msg.Data[7]=0x00; 
	
	
  if(CANx == CAN1) {
    xQueueSend(CAN1_Queue, &can_msg, 1); //��������������
  } else if(CANx == CAN2) {
    xQueueSend(CAN2_Queue, &can_msg, 1); //��������������
  }
}

