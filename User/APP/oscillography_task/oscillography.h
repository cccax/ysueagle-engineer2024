/**************************************************************************
* @file     	oscillography.h/.c
* @brief    	ͨ������2������λ����ʾ���ε���ʹ��
**************************************************************************/
#ifndef OSCILLOGRAPHY_H
#define OSCILLOGRAPHY_H

#include "stm32f4xx.h"
void oscillography_task(void *pvParameters);
extern s16 oscillographyData[10];	//������ʾ����

#endif
