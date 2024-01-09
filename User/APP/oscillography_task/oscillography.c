/**************************************************************************
 * @file     	oscillography.h/.c
 * @brief    	通过串口2连接上位机显示波形调参使用
 **************************************************************************/
#include "oscillography.h"
#include "stm32f4xx.h"
#include "usart2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart2.h"
#include "chassis_task.h"

s16 oscillographyData[10] = {0};

//向上位机发送一组数据，用于显示
void oscillography_sendData(s16 *dataSend, u8 len) {
//    u8 i = 0;
//    u8 data[20];
//	if(len > 10)
//        len = 10;
//    for(; i < len; i++) {
//        data[2*i] = dataSend[i];
//		data[2*i+1]=(u8)(dataSend[i]>>8);
//    }
//    Send_data8(data, len*2, 1);
}

void oscillography_task(void *pvParameters) {

  USART2_Init();
  vTaskDelay(100);

  while(1) {

    oscillographyData[0] =  chassisTaskStructure.motor[CHASSIS_CM1].speedSet;
    oscillographyData[1] =  chassisTaskStructure.motor[CHASSIS_CM1].filterSpeed;
    oscillographyData[2] =  chassisTaskStructure.motor[CHASSIS_CM2].speedSet;
    oscillographyData[3] =  chassisTaskStructure.motor[CHASSIS_CM2].filterSpeed;
    oscillographyData[4] =  chassisTaskStructure.motor[CHASSIS_CM3].speedSet;
    oscillographyData[5] =  chassisTaskStructure.motor[CHASSIS_CM3].filterSpeed;
    oscillographyData[6] =  chassisTaskStructure.motor[CHASSIS_CM4].speedSet;
    oscillographyData[7] =  chassisTaskStructure.motor[CHASSIS_CM4].filterSpeed;

    oscillographyData[8] =  (s32) * (chassisTaskStructure.relativeAngle);
    oscillographyData[9] =  (s32)chassisTaskStructure.relativeAngleSet;

    oscillography_sendData(oscillographyData, 10);
    vTaskDelay(5);
  }
}
