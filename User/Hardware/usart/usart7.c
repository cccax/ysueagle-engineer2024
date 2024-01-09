


void USART7_Init() {
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);	//ʹ��USART2_GPIOʱ��
  RCC_APB2PeriphClockCmd(RCC_APB1Periph_UART7, ENABLE);	//ʹ��USART2ʱ��


  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;			//���ù���
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		//�ٶ�50MHz
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;			//���츴�����
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;			//����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_14; 	//TX RX
  GPIO_Init(GPIOG, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = 115200;										//����������
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;							//�ֳ�Ϊ8λ���ݸ�ʽ
  USART_InitStructure.USART_StopBits = USART_StopBits_1;								//һ��ֹͣλ
  USART_InitStructure.USART_Parity = USART_Parity_No;									//����żУ��λ
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;		//��Ӳ������������
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;						//�շ�ģʽ
  USART_Init(USART6, &USART_InitStructure);											//��ʼ������2

  USART_Cmd(USART6, ENABLE);  //ʹ�ܴ���2

  USART_ClearFlag(USART6, USART_FLAG_TC);

  USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);	//��������ж�

  NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;			//����3�ж�ͨ��
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = USART6_NVIC;		//��ռ���ȼ�3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			//�����ȼ�0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//IRQͨ��ʹ��
  NVIC_Init(&NVIC_InitStructure);								//����ָ���Ĳ�����ʼ��NVIC�Ĵ���
}

/**
  * @name 	USART2_Init()
  * @brief 	����2�����жϣ����ڽ�����λ��������������
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
  * @brief 	����2����һ������
  * @param	DataToSend:���͵�������ָ��
  * @param	data_num:���͵���������
  * @return None
  */
void USART6_Send_Buf(u8 *DataToSend, u8 data_num) {
  int data_i = 0;

  for(data_i = 0; data_i < data_num; data_i++)
    USART6_Send_Data(*(DataToSend + data_i));
}

/**
  * @name 	USART2_Send_Data()
  * @brief 	����2����һ���ֽ�
  * @param	���͵��ֽ�����
  * @return None
  */
void USART6_Send_Data(u8 data) {
  while(USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET) {};

  USART6->DR = (data & (uint16_t)0x01FF);
}

//void Send_data8(u8 *dataddr,u8 len,u8 func_word)    //����
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