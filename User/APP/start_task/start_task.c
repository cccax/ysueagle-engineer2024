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

  xTaskCreate((TaskFunction_t)INS_task,          		//任务函数     //freertos
              (const char *)"INS_task",          		//任务名称
              (uint16_t)INS_STK_SIZE,            		//任务堆栈大小
              (void *)NULL,                        	//传递给任务函数的参数
              (UBaseType_t)INS_TASK_PRIO,        		//任务优先级
              (TaskHandle_t *)&INSTask_Handler); 		//任务句柄

  xTaskCreate((TaskFunction_t)chassisTask,			//任务函数
              (const char *)"chassisTask",			//任务名称
              (uint16_t)CHASSIS_STK_SIZE,				//任务堆栈大小
              (void *)NULL,							//传递给任务函数的参数
              (UBaseType_t)CHASSIS_TASK_PRIO,			//任务优先级
              (TaskHandle_t *)&CHASSISTask_Handler);	//任务句柄

  xTaskCreate((TaskFunction_t)grabTask,				//任务函数
              (const char *)"grabTask",				//任务名称
              (uint16_t)GRAB_STK_SIZE,				//任务堆栈大小
              (void *)NULL,							//传递给任务函数的参数
              (UBaseType_t)GRAB_TASK_PRIO,			//任务优先级
              (TaskHandle_t *)&GRABTask_Handler);		//任务句柄

  xTaskCreate((TaskFunction_t)DetectTask,				//任务函数
              (const char *)"DetectTask",				//任务名称
              (uint16_t)DECT_STK_SIZE,				//任务堆栈大小
              (void *)NULL,							//传递给任务函数的参数
              (UBaseType_t)DECT_TASK_PRIO,			//任务优先级
              (TaskHandle_t *)&DECTTask_Handler);		//任务句柄

  xTaskCreate((TaskFunction_t)basic_task,				//任务函数
              (const char *)"basic_task",				//任务名称
              (uint16_t)BASIC_STK_SIZE,				//任务堆栈大小
              (void *)NULL,							//传递给任务函数的参数
              (UBaseType_t)BASIC_TASK_PRIO,			//任务优先级
              (TaskHandle_t *)&BasicTask_Handler);	//任务句柄

  xTaskCreate((TaskFunction_t)oscillography_task,		//任务函数
              (const char *)"oscillography_task",	//任务名称
              (uint16_t)OSCILLOGRAPHY_STK_SIZE,		//任务堆栈大小
              (void *)NULL,							//传递给任务函数的参数
              (UBaseType_t)OSCILLOGRAPHY_TASK_PRIO,	//任务优先级
              (TaskHandle_t *)&Oscillography_Handler);//任务句柄

//							//add
//  xTaskCreate((TaskFunction_t)referee_usart_task,		//任务函数
//              (const char *)"referee_usart_task",	//任务名称
//              (uint16_t)REFEREE_USART_STK_SIZE,		//任务堆栈大小
//              (void *)NULL,							//传递给任务函数的参数
//              (UBaseType_t)REFEREE_USART_TASK_PRIO,	//任务优先级
//              (TaskHandle_t *)&Referee_USARTTask_Handler);//任务句柄
							
  vTaskDelete(StartTask_Handler); //删除开始任务

  taskEXIT_CRITICAL();            //退出临界区
}

void startTask(void) {
  xTaskCreate((TaskFunction_t)start_task,          //任务函数
              (const char *)"start_task",          //任务名称
              (uint16_t)START_STK_SIZE,            //任务堆栈大小
              (void *)NULL,                        //传递给任务函数的参数
              (UBaseType_t)START_TASK_PRIO,        //任务优先级
              (TaskHandle_t *)&StartTask_Handler); //任务句柄
}
