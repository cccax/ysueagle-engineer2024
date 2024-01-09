#include "usart6.h"
#include "stm32f4xx.h"
#include "oscillography.h"
/**********************************中间变量、函数声明************************************************/
uint8_t  Send_Data_Buf[34];
void USART6_Send_Data(u8 data);	//通过串口2发送一个八位字符
/*********************************************************************************************/

/**
  * @name 	USART2_Init()
  * @brief 	串口2初始化
  * @param	None
  * @return None
  */
void USART6_Init() {
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);	//使能USART2_GPIO时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);	//使能USART2时钟

  GPIO_PinAFConfig(GPIOG, GPIO_PinSource9, GPIO_AF_USART6); //GPIOD_10复用为USART2
  GPIO_PinAFConfig(GPIOG, GPIO_PinSource14, GPIO_AF_USART6); //GPIOD_11复用为USART2

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			//复用功能
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		//速度50MHz
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			//推挽复用输出
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			//上拉
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_14; 	//TX RX
  GPIO_Init(GPIOG, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = 115200;										//波特率设置
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;							//字长为8位数据格式
  USART_InitStructure.USART_StopBits = USART_StopBits_1;								//一个停止位
  USART_InitStructure.USART_Parity = USART_Parity_No;									//无奇偶校验位
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;		//无硬件数据流控制
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;						//收发模式
  USART_Init(USART6, &USART_InitStructure);											//初始化串口2

  USART_Cmd(USART6, ENABLE);  //使能串口2

  USART_ClearFlag(USART6, USART_FLAG_TC);

  USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);	//开启相关中断

  NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;			//串口3中断通道
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USART6_NVIC;		//抢占优先级3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			//子优先级0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//IRQ通道使能
  NVIC_Init(&NVIC_InitStructure);								//根据指定的参数初始化NVIC寄存器
}

/**
  * @name 	USART2_Init()
  * @brief 	串口2接收中断，用于接收上位机发送来的数据
  * @param	None
  * @return None
  */
void USART6_IRQHandler(void) {
  if(USART_GetITStatus(USART6, USART_IT_RXNE) != RESET) {
    USART_ClearITPendingBit(USART6, USART_IT_RXNE);
  }
}

/**
  * @name 	USART2_Send_Buf()
  * @brief 	串口2发送一组数据
  * @param	DataToSend:发送的数据首指针
  * @param	data_num:发送的数据数量
  * @return None
  */
void USART6_Send_Buf(u8 *DataToSend, u8 data_num) {
  int data_i = 0;

  for(data_i = 0; data_i < data_num; data_i++)
    USART6_Send_Data(*(DataToSend + data_i));
}

/**
  * @name 	USART2_Send_Data()
  * @brief 	串口2发送一个字节7
  * @param	发送的字节内容
  * @return None
  */
void USART6_Send_Data(u8 data) {
  while(USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET) {};

  USART6->DR = (data & (uint16_t)0x01FF);
}

//void Send_data8(u8 *dataddr,u8 len,u8 func_word)    //常用
//{
//	u8 i, count;
//	u8 now_word;
//	u8 sc, ac;
//	ac = sc = 0;
//	if(len > 20) len = 20;
//	if((func_word >= 1) && (func_word <= 10))
//		now_word = 0xF0 | func_word;
//	Send_Data_Buf[0] = 0xAA;
//	Send_Data_Buf[1] = 0xFF;
//	Send_Data_Buf[2] = now_word;
//	Send_Data_Buf[3] = len;
//	for(i=0; i<len; i++)
//		Send_Data_Buf[4+i] = *(dataddr + i);
//	for(i=0; i<Send_Data_Buf[3]+4; i++){
//		ac += Send_Data_Buf[i];
//		ac += sc;
//	}
//	Send_Data_Buf[4+len] = sc;
//	Send_Data_Buf[5+len] = ac;
//	count = 6+len;
//	USART6_Send_Buf(Send_Data_Buf, count);
//}

void Send_data8(u8 *dataddr, u8 len, u8 func_word) { //常用
  /*更改了匿名上位机版本型号使用这个更方便*/
  u8 i, count;
  u8 now_word;
  u8 sc, ac;
  ac = sc = 0;

  if(len > 20) len = 20;

  if((func_word >= 1) && (func_word <= 10))
    now_word = 0xF0 | func_word;

  Send_Data_Buf[0] = 0xAA;
  Send_Data_Buf[1] = 0xFF;
  Send_Data_Buf[2] = now_word;
  Send_Data_Buf[3] = len;

  for(i = 0; i < len; i++)
    Send_Data_Buf[4 + i] = *(dataddr + i);   //发送数据

  for(i = 0; i < Send_Data_Buf[3] + 4; i++) {
    sc += Send_Data_Buf[i];
    ac += sc;
  }

  Send_Data_Buf[4 + len] = sc;
  Send_Data_Buf[5 + len] = ac;
  count = 6 + len;
  USART6_Send_Buf(Send_Data_Buf, count);
}

