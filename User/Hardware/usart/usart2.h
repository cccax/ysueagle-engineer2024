#ifndef _USART2_H
#define _USART2_H
#include "stm32f4xx.h"
#include "main.h"

void USART2_Init(void);	//串口二初始化函数
void USART2_Send_Buf(u8 *DataToSend, u8 data_num);	//通过串口2发送一组数据，用于上位机显示
void Send_data8(u8 *dataddr, u8 len, u8 func_word);
void USART2_Send_Data(u8 data);

#endif
