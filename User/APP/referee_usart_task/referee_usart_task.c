/**
  **********************************2022 CKYF***********************************
  * @file    referee_usart_task.c
  * @brief   裁判系统通讯任务
  ******************************************************************************
	* @team    长空御风
	* @author  曾新宇
	*相关移植工作：21电控组曹博鑫
  ******************************************************************************
  * @attention
  * 
  * 此任务用于和裁判系统交换数据，包括操作手UI界面的绘制，还包括向A板发送相关裁判系统数据
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
/* 裁判系统串口双缓冲区 */
uint8_t Referee_Buffer[2][REFEREE_USART_RX_BUF_LENGHT];//只用了一个缓冲区

extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;

/* 裁判系统接收数据队列 */
fifo_s_t Referee_FIFO;
uint8_t Referee_FIFO_Buffer[REFEREE_FIFO_BUF_LENGTH];

/* protocol解析包结构体 */
unpack_data_t Referee_Unpack_OBJ;

/* 动态UI数据变量 */
uint8_t UI_AutoAim_Flag = 0;    //是否开启自瞄标志位
float   UI_Kalman_Speed = 0;    //卡尔曼预测速度
float   UI_Gimbal_Pitch = 0.0f; //云台Pitch轴角度
float   UI_Gimbal_Yaw   = 0.0f; //云台Yaw轴角度
float		UI_Chassis_Angle= 0.0f;
uint8_t UI_Capacitance  = 10;   //电容剩余容量
uint8_t UI_fric_is_on   = 0;    //摩擦轮是否开启

/* 中央标尺高度变量 */
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
	/* 动态UI控制变量 */
	uint16_t UI_PushUp_Counter = 261;
	
	/* 裁判系统初始化 */
	vTaskDelay(300);
	
	/* new UI */
	while(1)
	{
		/* 解析裁判系统数据 */
		vTaskDelay(10);
		Referee_UnpackFifoData(&Referee_Unpack_OBJ, &Referee_FIFO);
		/* 发送A板数据 */
		boardA_Task();//发送给A板的函数
		/* UI更新 */
		ui_update_data();//更新ui动态数据
		UI_PushUp_Counter++;
		if(UI_PushUp_Counter % 301 == 0) //静态UI预绘制 中央标尺1
		{
			UI_Draw_Line(&UI_Graph7.Graphic[0], "001", UI_Graph_Add, 0, UI_Color_Green, 1,  840,   y01,  920,   y01); //第一行左横线
			UI_Draw_Line(&UI_Graph7.Graphic[1], "002", UI_Graph_Add, 0, UI_Color_Green, 1,  950,   y01,  970,   y01); //第一行十字横
			UI_Draw_Line(&UI_Graph7.Graphic[2], "003", UI_Graph_Add, 0, UI_Color_Green, 1, 1000,   y01, 1080,   y01); //第一行右横线
			UI_Draw_Line(&UI_Graph7.Graphic[3], "004", UI_Graph_Add, 0, UI_Color_Green, 1,  960,y01-10,  960,y01+10); //第一行十字竖
			UI_Draw_Line(&UI_Graph7.Graphic[4], "005", UI_Graph_Add, 0, UI_Color_Green, 1,  870,   y02,  930,   y02); //第二行左横线
			UI_Draw_Line(&UI_Graph7.Graphic[5], "006", UI_Graph_Add, 0, UI_Color_Green, 5,  959,   y02,  960,   y02); //第二行中心点
			UI_Draw_Line(&UI_Graph7.Graphic[6], "007", UI_Graph_Add, 0, UI_Color_Green, 1,  990,   y02, 1050,   y02); //第二行右横线
			UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);
			continue;
		}
		if(UI_PushUp_Counter % 311 == 0) //静态UI预绘制 中央标尺2
		{
			UI_Draw_Line(&UI_Graph7.Graphic[0], "008", UI_Graph_Add, 0, UI_Color_Green, 1,  900,   y03,  940,   y03); //第三行左横线
			UI_Draw_Line(&UI_Graph7.Graphic[1], "009", UI_Graph_Add, 0, UI_Color_Green, 5,  959,   y03,  960,   y03); //第三行中心点
			UI_Draw_Line(&UI_Graph7.Graphic[2], "010", UI_Graph_Add, 0, UI_Color_Green, 1,  980,   y03, 1020,   y03); //第三行右横线
			UI_Draw_Line(&UI_Graph7.Graphic[3], "011", UI_Graph_Add, 0, UI_Color_Green, 1,  930,   y04,  950,   y04); //第四行左横线
			UI_Draw_Line(&UI_Graph7.Graphic[4], "012", UI_Graph_Add, 0, UI_Color_Green, 5,  959,   y04,  960,   y04); //第四行中心点
			UI_Draw_Line(&UI_Graph7.Graphic[5], "013", UI_Graph_Add, 0, UI_Color_Green, 1,  970,   y04,  990,   y04); //第四行右横线
			UI_Draw_Line(&UI_Graph7.Graphic[6], "014", UI_Graph_Add, 0, UI_Color_Green, 1,  960,y04-10,  960,y04-30); //第四行下竖线
			UI_PushUp_Graphs(7, &UI_Graph7, Robot_ID_Current);
			continue;
		}
		if(UI_PushUp_Counter % 321 == 0) //静态UI预绘制 小陀螺预警线
		{
			UI_Draw_Line(&UI_Graph5.Graphic[0], "101", UI_Graph_Add, 1, UI_Color_Yellow, 2,  630,   30,  780,  100);
			UI_Draw_Line(&UI_Graph5.Graphic[1], "102", UI_Graph_Add, 1, UI_Color_Yellow, 2,  780,  100,  930,  100);
			UI_Draw_Line(&UI_Graph5.Graphic[2], "103", UI_Graph_Add, 1, UI_Color_Yellow, 2,  990,  100, 1140,  100);
			UI_Draw_Line(&UI_Graph5.Graphic[3], "104", UI_Graph_Add, 1, UI_Color_Yellow, 2, 1140,  100, 1290,   30);
			UI_Draw_Line(&UI_Graph5.Graphic[4], "105", UI_Graph_Add, 1, UI_Color_Yellow, 5,  959,  100,  960,  100);
			UI_PushUp_Graphs(5, &UI_Graph5, Robot_ID_Current);
			continue;
		}
		if(UI_PushUp_Counter % 331 == 0) //动态UI预绘制 图形
		{
			UI_Draw_Float (&UI_Graph5.Graphic[0], "201", UI_Graph_Add, 2, UI_Color_Yellow, 22, 3, 3, 1355, 632, 0.000f);   //Pith轴角度
			UI_Draw_Line  (&UI_Graph5.Graphic[1], "202", UI_Graph_Add, 2, UI_Color_Orange, 20, 1829, 330, 1870, 334);      //电容容量
			UI_PushUp_Graphs(5, &UI_Graph5, Robot_ID_Current);
			continue;
		}
		if(UI_PushUp_Counter % 341 == 0) //动态UI预绘制 字符串1
		{
			UI_Draw_String(&UI_String.String,     "203", UI_Graph_Add, 2, UI_Color_Black,  22, 8, 3,  400, 632, "Fric OFF"); //摩擦轮是否开启
			UI_PushUp_String(&UI_String, Robot_ID_Current);
			continue;
		}
		if(UI_PushUp_Counter % 21 == 0) //动态UI更新 字符串1
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
		if(UI_PushUp_Counter % 10 == 0)  //动态UI更新 图形
		{
			/* Pitch轴当前角度 */
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
			__HAL_UART_CLEAR_IDLEFLAG(&huart2);  //清除空闲中断标志位
			HAL_UART_DMAStop(&huart2);//关闭DMA
			this_time_rx_len = REFEREE_USART_RX_BUF_LENGHT - hdma_usart2_rx.Instance->CNDTR;
			hdma_usart2_rx.Instance->CNDTR = REFEREE_USART_RX_BUF_LENGHT;
			fifo_s_puts(&Referee_FIFO, (char*)Referee_Buffer[1], this_time_rx_len);
			HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_1);
			HAL_UART_Receive_DMA(&huart2,Referee_Buffer[1],REFEREE_USART_RX_BUF_LENGHT);//重新使能
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
//if(witchbuf)                        						//之前用的u2rxbuf，切换为u1rxbuf
//		{
//			p=u2rxbuf;												//先保存前一次数据地址再切换缓冲区
//			DMA1_Channel6->CMAR=(u32)u1rxbuf;						//切换为u1rxbuf缓冲区地址
//			witchbuf=0;                     						//下一次切换为u2rxbuf
//		}else                               						//之前用的u1rxbuf，切换为u2rxbuf
//		{
//			p=u1rxbuf;												//先保存前一次数据地址再切换缓冲区
//			DMA1_Channel6->CMAR=(u32)u2rxbuf;						//切换为u2rxbuf缓冲区地址
//			witchbuf=1;                     						//下一次切换为u1rxbuf
//		}
    }
}
