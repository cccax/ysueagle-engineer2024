/**
  **********************************2022 CKYF***********************************
  * @file    referee_usart_task.c
  * @brief   ����ϵͳͨѶ����
  ******************************************************************************
	* @team    ��������
	* @author  ������
	*�����ֲ������21�����ܲ���
  ******************************************************************************
  * @attention
  * 
  * ���������ںͲ���ϵͳ�������ݣ�����������UI����Ļ��ƣ���������A�巢����ز���ϵͳ����
  *
  **********************************2022 CKYF***********************************
  */

#include "referee_usart_task.h"
#include "usart.h"
#include "crcs.h"
#include "fifo.h"
#include "protocol.h"
#include "referee.h"
#include "boardA.h"
/* Private define ------------------------------------------------------------*/
#define Max(a,b) ((a) > (b) ? (a) : (b))
#define Robot_ID_Current Robot_ID_Blue_Infantry3

/* Private variables ---------------------------------------------------------*/
/* ����ϵͳ����˫������ */
uint8_t Referee_Buffer[2][REFEREE_USART_RX_BUF_LENGHT];//ֻ����һ��������

extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

/* ����ϵͳ�������ݶ��� */
fifo_s_t Referee_FIFO;
uint8_t Referee_FIFO_Buffer[REFEREE_FIFO_BUF_LENGTH];

/* protocol�������ṹ�� */
unpack_data_t Referee_Unpack_OBJ;

/* ��̬UI���ݱ��� */
uint8_t UI_AutoAim_Flag = 0;    //�Ƿ��������־λ
float   UI_Kalman_Speed = 0;    //������Ԥ���ٶ�
float   UI_Gimbal_Pitch = 0.0f; //��̨Pitch��Ƕ�
float   UI_Gimbal_Yaw   = 0.0f; //��̨Yaw��Ƕ�
float		UI_Chassis_Angle= 0.0f;
uint8_t UI_Capacitance  = 10;   //����ʣ������
uint8_t UI_fric_is_on   = 0;    //Ħ�����Ƿ���

/* �����߸߶ȱ��� */
uint16_t y01 = 455;
uint16_t y02 = 420;
uint16_t y03 = 280;
uint16_t y04 = 230;

uint8_t autoaim_mode;//2:normal,3:small energy,4:big energy
uint8_t autoaim_armor;//0x10:auto,0x20:big,0x30:small
uint8_t if_predict;


void ui_update_data(void)
{
	UI_Gimbal_Pitch=uiStruct.pitchAngle;
	UI_Gimbal_Yaw=uiStruct.yawAngle;
	UI_Chassis_Angle=uiStruct.chassisAngle;
	UI_fric_is_on=uiStruct.fire_mode;
}
void referee_usart_task(void const * argument)
{
	/* ��̬UI���Ʊ��� */
	uint16_t UI_PushUp_Counter = 261;
	
	/* ����ϵͳ��ʼ�� */
	vTaskDelay(300);
	
	/* new UI */
	while(1)
	{
		/* ��������ϵͳ���� */
		vTaskDelay(10);
		Referee_UnpackFifoData(&Referee_Unpack_OBJ, &Referee_FIFO);
		/* ����A������ */
		boardA_Task();//���͸�A��ĺ���
		/* UI���� */
		ui_update_data();//����ui��̬����
		UI_PushUp_Counter++;
		if(UI_PushUp_Counter % 301 == 0) //��̬UIԤ���� ������1
		{
			UI_Draw_Line(&UI_Graph7.Graphic[0], "001", UI_Graph_Add, 0, UI_Color_Green, 1,  840,   y01,  920,   y01); //��һ�������
			UI_Draw_Line(&UI_Graph7.Graphic[1], "002", UI_Graph_Add, 0, UI_Color_Green, 1,  950,   y01,  970,   y01); //��һ��ʮ�ֺ�
			UI_Draw_Line(&UI_Graph7.Graphic[2], "003", UI_Graph_Add, 0, UI_Color_Green, 1, 1000,   y01, 1080,   y01); //��һ���Һ���
			UI_Draw_Line(&UI_Graph7.Graphic[3], "004", UI_Graph_Add, 0, UI_Color_Green, 1,  960,y01-10,  960,y01+10); //��һ��ʮ����
			UI_Draw_Line(&UI_Graph7.Graphic[4], "005", UI_Graph_Add, 0, UI_Color_Green, 1,  870,   y02,  930,   y02); //�ڶ��������
			UI_Draw_Line(&UI_Graph7.Graphic[5], "006", UI_Graph_Add, 0, UI_Color_Green, 5,  959,   y02,  960,   y02); //�ڶ������ĵ�
			UI_Draw_Line(&UI_Graph7.Graphic[6], "007", UI_Graph_Add, 0, UI_Color_Green, 1,  990,   y02, 1050,   y02); //�ڶ����Һ���
			UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);
			continue;
		}
		if(UI_PushUp_Counter % 311 == 0) //��̬UIԤ���� ������2
		{
			UI_Draw_Line(&UI_Graph7.Graphic[0], "008", UI_Graph_Add, 0, UI_Color_Green, 1,  900,   y03,  940,   y03); //�����������
			UI_Draw_Line(&UI_Graph7.Graphic[1], "009", UI_Graph_Add, 0, UI_Color_Green, 5,  959,   y03,  960,   y03); //���������ĵ�
			UI_Draw_Line(&UI_Graph7.Graphic[2], "010", UI_Graph_Add, 0, UI_Color_Green, 1,  980,   y03, 1020,   y03); //�������Һ���
			UI_Draw_Line(&UI_Graph7.Graphic[3], "011", UI_Graph_Add, 0, UI_Color_Green, 1,  930,   y04,  950,   y04); //�����������
			UI_Draw_Line(&UI_Graph7.Graphic[4], "012", UI_Graph_Add, 0, UI_Color_Green, 5,  959,   y04,  960,   y04); //���������ĵ�
			UI_Draw_Line(&UI_Graph7.Graphic[5], "013", UI_Graph_Add, 0, UI_Color_Green, 1,  970,   y04,  990,   y04); //�������Һ���
			UI_Draw_Line(&UI_Graph7.Graphic[6], "014", UI_Graph_Add, 0, UI_Color_Green, 1,  960,y04-10,  960,y04-30); //������������
			UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);
			continue;
		}
		if(UI_PushUp_Counter % 321 == 0) //��̬UIԤ���� С����Ԥ����
		{
			UI_Draw_Line(&UI_Graph5.Graphic[0], "101", UI_Graph_Add, 1, UI_Color_Yellow, 2,  630,   30,  780,  100);
			UI_Draw_Line(&UI_Graph5.Graphic[1], "102", UI_Graph_Add, 1, UI_Color_Yellow, 2,  780,  100,  930,  100);
			UI_Draw_Line(&UI_Graph5.Graphic[2], "103", UI_Graph_Add, 1, UI_Color_Yellow, 2,  990,  100, 1140,  100);
			UI_Draw_Line(&UI_Graph5.Graphic[3], "104", UI_Graph_Add, 1, UI_Color_Yellow, 2, 1140,  100, 1290,   30);
			UI_Draw_Line(&UI_Graph5.Graphic[4], "105", UI_Graph_Add, 1, UI_Color_Yellow, 5,  959,  100,  960,  100);
			UI_PushUp_Graphs(5, &UI_Graph5, Robot_ID_Current);
			continue;
		}
		if(UI_PushUp_Counter % 331 == 0) //��̬UIԤ���� ͼ��
		{
			UI_Draw_Float (&UI_Graph5.Graphic[0], "201", UI_Graph_Add, 2, UI_Color_Yellow, 22, 3, 3, 1355, 632, 0.000f);   //Pith��Ƕ�
			UI_Draw_Line  (&UI_Graph5.Graphic[1], "202", UI_Graph_Add, 2, UI_Color_Orange, 20, 1829, 330, 1870, 334);      //��������
			UI_PushUp_Graphs(5, &UI_Graph5, Robot_ID_Current);
			continue;
		}
		if(UI_PushUp_Counter % 341 == 0) //��̬UIԤ���� �ַ���1
		{
			UI_Draw_String(&UI_String.String,     "203", UI_Graph_Add, 2, UI_Color_Black,  22, 8, 3,  400, 632, "Fric OFF"); //Ħ�����Ƿ���
			UI_PushUp_String(&UI_String, Robot_ID_Current);
			continue;
		}
		if(UI_PushUp_Counter % 21 == 0) //��̬UI���� �ַ���1
		{
			if(UI_fric_is_on == 1) 
			{
				if(autoaim_mode==0x02&&autoaim_armor==0x10&&if_predict==0)
				{
					UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+4+9+8, 3,  100, 700, "Fric  ON\nNor\nArm Auto\nPre  NO");
				}
				else if(autoaim_mode==0x02&&autoaim_armor==0x20&&if_predict==0)
				{
					UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+4+9+8, 3,  100, 700, "Fric  ON\nNor\nArm  Big\nPre  NO");
				}
				else if(autoaim_mode==0x02&&autoaim_armor==0x30&&if_predict==0)
				{
					UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+4+9+8, 3,  100, 700, "Fric  ON\nNor\nArm Smal\nPre  NO");
				}		
				else if(autoaim_mode==0x02&&autoaim_armor==0x10&&if_predict==1)
				{
					UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+4+9+8, 3,  100, 700, "Fric  ON\nNor\nArm Auto\nPre YES");
				}
				else if(autoaim_mode==0x02&&autoaim_armor==0x20&&if_predict==1)
				{
					UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+4+9+8, 3,  100, 700, "Fric  ON\nNor\nArm  Big\nPre YES");
				}
				else if(autoaim_mode==0x02&&autoaim_armor==0x30&&if_predict==1)
				{
					UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+4+9+8, 3,  100, 700, "Fric  ON\nNor\nArm Smal\nPre YES");
				}		
				
				else if(autoaim_mode==0x03)
				{
					UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+4+9+8, 3,  100, 700, "Fric  ON\nXFu\n        \n       ");
				}
				else if(autoaim_mode==0x04)
				{
					UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Main,  22, 8+4+9+8, 3,  100, 700, "Fric  ON\nDFu\n        \n       ");
				}
			}
			if(UI_fric_is_on == 0) UI_Draw_String(&UI_String.String, "203", UI_Graph_Change, 2, UI_Color_Black, 22, 8+4+9+8, 3,  100, 700, "Fric OFF\n   \n        \n       ");
			UI_PushUp_String(&UI_String, Robot_ID_Current);
			continue;
		}
		if(UI_PushUp_Counter % 10 == 0)  //��̬UI���� ͼ��
		{
			/* Pitch�ᵱǰ�Ƕ� */
			UI_Draw_Float(&UI_Graph5.Graphic[0], "201", UI_Graph_Change, 2, UI_Color_Yellow, 22, 3, 3, 1355, 632, UI_Gimbal_Pitch);
			
			UI_PushUp_Graphs(5, &UI_Graph5, Robot_ID_Current);
			continue;
		}
	}
}

uint16_t this_time_rx_len = 0;
void USART2_IRQHandler_1(void)
{
	if(USART2->SR & UART_FLAG_IDLE)
    {
       static uint16_t this_time_rx_len = 0;
			__HAL_UART_CLEAR_IDLEFLAG(&huart2);  //��������жϱ�־λ
			HAL_UART_DMAStop(&huart2);//�ر�DMA
			this_time_rx_len = REFEREE_USART_RX_BUF_LENGHT - hdma_usart2_rx.Instance->CNDTR;
			hdma_usart2_rx.Instance->CNDTR = REFEREE_USART_RX_BUF_LENGHT;
			fifo_s_puts(&Referee_FIFO, (char*)Referee_Buffer[1], this_time_rx_len);
			HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_1);
			HAL_UART_Receive_DMA(&huart2,Referee_Buffer[1],REFEREE_USART_RX_BUF_LENGHT);//����ʹ��
//        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET)
//        {
//            __HAL_DMA_DISABLE(&hdma_usart6_rx);
//            this_time_rx_len = REFEREE_USART_RX_BUF_LENGHT - hdma_usart6_rx.Instance->NDTR;
//            hdma_usart6_rx.Instance->NDTR = REFEREE_USART_RX_BUF_LENGHT;
//            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;
//            __HAL_DMA_ENABLE(&hdma_usart6_rx);
//						fifo_s_puts(&Referee_FIFO, (char*)Referee_Buffer[1], this_time_rx_len);
//        }
//        else
//        {
//            __HAL_DMA_DISABLE(&hdma_usart6_rx);
//            this_time_rx_len = REFEREE_USART_RX_BUF_LENGHT - hdma_usart6_rx.Instance->NDTR;
//            hdma_usart6_rx.Instance->NDTR = REFEREE_USART_RX_BUF_LENGHT;
//            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
//            __HAL_DMA_ENABLE(&hdma_usart6_rx);
//						fifo_s_puts(&Referee_FIFO, (char*)Referee_Buffer[1], this_time_rx_len);

//        }
//if(witchbuf)                        						//֮ǰ�õ�u2rxbuf���л�Ϊu1rxbuf
//		{
//			p=u2rxbuf;												//�ȱ���ǰһ�����ݵ�ַ���л�������
//			DMA1_Channel6->CMAR=(u32)u1rxbuf;						//�л�Ϊu1rxbuf��������ַ
//			witchbuf=0;                     						//��һ���л�Ϊu2rxbuf
//		}else                               						//֮ǰ�õ�u1rxbuf���л�Ϊu2rxbuf
//		{
//			p=u1rxbuf;												//�ȱ���ǰһ�����ݵ�ַ���л�������
//			DMA1_Channel6->CMAR=(u32)u2rxbuf;						//�л�Ϊu2rxbuf��������ַ
//			witchbuf=1;                     						//��һ���л�Ϊu1rxbuf
//		}
    }
}
