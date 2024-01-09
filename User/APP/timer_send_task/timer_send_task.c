#include "timer_send_task.h"
#include "stm32f4xx.h"
#include "can.h"
#include "usart6.h"
#include "grab_task.h"

TimerHandle_t	CAN1_Timer_Handle; 			//���ڶ�ʱ�����
TimerHandle_t	CAN2_Timer_Handle; 			//���ڶ�ʱ�����
TimerHandle_t	VISION_Timer_Handle; 		//���ڶ�ʱ�����

QueueHandle_t	CAN1_Queue;					//CAN1��Ϣ���о��
QueueHandle_t	CAN2_Queue;					//CAN2��Ϣ���о��
QueueHandle_t	VISION_Send_Queue;			//�Ӿ���Ϣ���о��

void timerSendCreate(void) {
  taskENTER_CRITICAL(); 	//�����ٽ���

  /*--------------------------���ݽ���--------------------------*/

  //����can1���ն���
  //can1���յ��ı��Ĵ���ڴ˶�����
  CAN1_Queue = xQueueCreate( 128, sizeof(CanTxMsg));//���ɱ���64��CanTxMsg

  //����can2���ն���
  //can2���յ��ı��Ĵ���ڴ˶�����
  CAN2_Queue = xQueueCreate( 128, sizeof(CanTxMsg));//

  //�����Ӿ����Ͷ���
  //�Ӿ����͵��ı��Ĵ���ڴ˶�����
  VISION_Send_Queue = xQueueCreate( 128, sizeof(VisionSendData_t));//

  //����can1���Ͷ�ʱ��
  CAN1_Timer_Handle = xTimerCreate((const char*	)"CAN1_Timer",
                                   (TickType_t 	)TIME_STAMP_1MS,//1ms
                                   (UBaseType_t	)pdTRUE, 		//����ִ��
                                   (void *		)0,				//���һ���0
                                   (TimerCallbackFunction_t)CAN1_Timer_Callback);//�ص�����

  //����CAN1��ʱ��,���ҽ��ܿ���һ�Σ����������������򲻻ᷢ����
  if( CAN1_Timer_Handle != NULL ) {
    xTimerStart(CAN1_Timer_Handle, 0); //���ȴ�
  }

  //����can2���Ͷ�ʱ��
  CAN2_Timer_Handle = xTimerCreate((const char*	)"CAN2_Timer",
                                   (TickType_t 	)TIME_STAMP_1MS,//1ms
                                   (UBaseType_t	)pdTRUE, 		//����ִ��
                                   (void *		)1,				//���һ���0
                                   (TimerCallbackFunction_t)CAN2_Timer_Callback);//�ص�����

  //����CAN2��ʱ��,���ҽ��ܿ���һ�Σ����������������򲻻ᷢ����
  if( CAN2_Timer_Handle != NULL ) {
    xTimerStart(CAN2_Timer_Handle, 0); //���ȴ�
  }

  //�����Ӿ����ܶ�ʱ��
  VISION_Timer_Handle = xTimerCreate((const char*	)"VISION_Timer",
                                     (TickType_t 	)TIME_STAMP_5MS,//5ms
                                     (UBaseType_t	)pdTRUE, 		//����ִ��
                                     (void *		)1,				//���һ���0
                                     (TimerCallbackFunction_t)Vision_Timer_Callback);//�ص�����

  //�����Ӿ���ʱ��,���ҽ��ܿ���һ�Σ����������������򲻻ᷢ����
  if( VISION_Timer_Handle != NULL ) {
    xTimerStart(VISION_Timer_Handle, 0); //���ȴ�
  }


  taskEXIT_CRITICAL();	//�˳��ٽ���
}

/**
  * @brief  �����ʱ�ص�����
  * @param  void
  * @retval void
  * @attention can���н��õȴ�������delay
  */
void CAN1_Timer_Callback( TimerHandle_t xTimer ) {
  CanTxMsg SendCanTxMsg;

  while(xQueueReceive(CAN1_Queue, &SendCanTxMsg, 0)) { // �Ӷ��н��գ����ȴ�
    uint8_t res;
    res = CAN_Transmit(CAN1, &SendCanTxMsg); // ���Է���
    if(res == CAN_TxStatus_NoMailBox) { // CAN������������û�з��ͳɹ�
      xQueueSend(CAN1_Queue, &SendCanTxMsg, 0); // ������������������
      break; // �´����������ٷ���
    }
  }
}

/**
  * @brief  �����ʱ�ص�����
  * @param  void
  * @retval void
  * @attention can���н��õȴ�������delay
  */
void CAN2_Timer_Callback( TimerHandle_t xTimer ) {
  CanTxMsg SendCanTxMsg;

  while(xQueueReceive(CAN2_Queue, &SendCanTxMsg, 0)) { // �Ӷ��н��գ����ȴ�
    uint8_t res;
    res = CAN_Transmit(CAN2, &SendCanTxMsg); // ���Է���
    if(res == CAN_TxStatus_NoMailBox) { // CAN������������û�з��ͳɹ�
      xQueueSend(CAN2_Queue, &SendCanTxMsg, 0); // ������������������
      break; // �´����������ٷ���
    }
  }
}

/**
  * @brief  �����ʱ�ص�����
  * @param  void
  * @retval void
  * @attention can���н��õȴ�������delay
  */
void Vision_Timer_Callback( TimerHandle_t xTimer ) {
  VisionSendData_t VisionSendData_Struct;

  while(xQueueReceive(VISION_Send_Queue, &VisionSendData_Struct, 0)) { //���ն�����Ϣ,��ֹ�ȴ�������
  }

}
