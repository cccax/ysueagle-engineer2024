/**************************************************************************
* @file     	oscillography.h/.c
* @brief    	通过串口2连接上位机显示波形调参使用
**************************************************************************/
#ifndef OSCILLOGRAPHY_H
#define OSCILLOGRAPHY_H

#include "stm32f4xx.h"
void oscillography_task(void *pvParameters);
extern s16 oscillographyData[10];	//用于显示波形

#endif
