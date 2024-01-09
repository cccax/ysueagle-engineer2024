/**************************************************************************
 * @file     	usart3_dma.h
 * @brief    	�����������ⲿ���������Ƕ�ȡ
 * @writer		������
 * @Q Q			2296054658
 **************************************************************************/
#ifndef _USART3_DMA_H
#define _USART3_DMA_H
#include "stm32f4xx.h"
#include "main.h"

void USART3_Init(void);	//����3��ʼ��
void USART3_Send_Data(u8 data);
void USART3_Send_Buf(u8 *DataToSend, u8 data_num);


#endif /*_USART3_DMA_H*/
