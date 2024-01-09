#include "start_task.h"

#define START_TASK_PRIO 1
#define START_STK_SIZE 512
static TaskHandle_t StartTask_Handler;

#define INS_TASK_PRIO 5
#define INS_STK_SIZE 512
static TaskHandle_t INSTask_Handler;

#define CHASSIS_TASK_PRIO 10
#define CHASSIS_STK_SIZE 512
static TaskHandle_t CHASSISTask_Handler;

#define GRAB_TASK_PRIO 15
#define GRAB_STK_SIZE 512
static TaskHandle_t GRABTask_Handler;

#define DECT_TASK_PRIO 3
#define DECT_STK_SIZE 256
static TaskHandle_t DECTTask_Handler;

//#define REFEREE_USART_TASK_PRIO 5    //add
//#define REFEREE_USART_STK_SIZE 256
//static TaskHandle_t Referee_USARTTask_Handler;

#define OSCILLOGRAPHY_TASK_PRIO 5
#define OSCILLOGRAPHY_STK_SIZE 256
static TaskHandle_t Oscillography_Handler;

#define BASIC_TASK_PRIO 20
#define BASIC_STK_SIZE 128
static TaskHandle_t BasicTask_Handler;

void start_task(void *pvParameters) {
  taskENTER_CRITICAL();

  xTaskCreate((TaskFunction_t)INS_task,          		//������     //freertos
              (const char *)"INS_task",          		//��������
              (uint16_t)INS_STK_SIZE,            		//�����ջ��С
              (void *)NULL,                        	//���ݸ��������Ĳ���
              (UBaseType_t)INS_TASK_PRIO,        		//�������ȼ�
              (TaskHandle_t *)&INSTask_Handler); 		//������

  xTaskCreate((TaskFunction_t)chassisTask,			//������
              (const char *)"chassisTask",			//��������
              (uint16_t)CHASSIS_STK_SIZE,				//�����ջ��С
              (void *)NULL,							//���ݸ��������Ĳ���
              (UBaseType_t)CHASSIS_TASK_PRIO,			//�������ȼ�
              (TaskHandle_t *)&CHASSISTask_Handler);	//������

  xTaskCreate((TaskFunction_t)grabTask,				//������
              (const char *)"grabTask",				//��������
              (uint16_t)GRAB_STK_SIZE,				//�����ջ��С
              (void *)NULL,							//���ݸ��������Ĳ���
              (UBaseType_t)GRAB_TASK_PRIO,			//�������ȼ�
              (TaskHandle_t *)&GRABTask_Handler);		//������

  xTaskCreate((TaskFunction_t)DetectTask,				//������
              (const char *)"DetectTask",				//��������
              (uint16_t)DECT_STK_SIZE,				//�����ջ��С
              (void *)NULL,							//���ݸ��������Ĳ���
              (UBaseType_t)DECT_TASK_PRIO,			//�������ȼ�
              (TaskHandle_t *)&DECTTask_Handler);		//������

  xTaskCreate((TaskFunction_t)basic_task,				//������
              (const char *)"basic_task",				//��������
              (uint16_t)BASIC_STK_SIZE,				//�����ջ��С
              (void *)NULL,							//���ݸ��������Ĳ���
              (UBaseType_t)BASIC_TASK_PRIO,			//�������ȼ�
              (TaskHandle_t *)&BasicTask_Handler);	//������

  xTaskCreate((TaskFunction_t)oscillography_task,		//������
              (const char *)"oscillography_task",	//��������
              (uint16_t)OSCILLOGRAPHY_STK_SIZE,		//�����ջ��С
              (void *)NULL,							//���ݸ��������Ĳ���
              (UBaseType_t)OSCILLOGRAPHY_TASK_PRIO,	//�������ȼ�
              (TaskHandle_t *)&Oscillography_Handler);//������

//							//add
//  xTaskCreate((TaskFunction_t)referee_usart_task,		//������
//              (const char *)"referee_usart_task",	//��������
//              (uint16_t)REFEREE_USART_STK_SIZE,		//�����ջ��С
//              (void *)NULL,							//���ݸ��������Ĳ���
//              (UBaseType_t)REFEREE_USART_TASK_PRIO,	//�������ȼ�
//              (TaskHandle_t *)&Referee_USARTTask_Handler);//������
							
  vTaskDelete(StartTask_Handler); //ɾ����ʼ����

  taskEXIT_CRITICAL();            //�˳��ٽ���
}

void startTask(void) {
  xTaskCreate((TaskFunction_t)start_task,          //������
              (const char *)"start_task",          //��������
              (uint16_t)START_STK_SIZE,            //�����ջ��С
              (void *)NULL,                        //���ݸ��������Ĳ���
              (UBaseType_t)START_TASK_PRIO,        //�������ȼ�
              (TaskHandle_t *)&StartTask_Handler); //������
}
