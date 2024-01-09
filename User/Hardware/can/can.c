#include "can.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "timer_send_task.h"
#include "motor.h"
#include "grab_task.h"
#include "chassis_task.h"
#include "Detect_Task.h"
#include "basic_task.h"
#include "mf9025.h"
/**
  * @name	CAN1_Init()
  * @brief 	CAN1初始化函数
  * @param  None
  */
void CAN1_Init() {
  GPIO_InitTypeDef GPIO_InitStructure;
  CAN_InitTypeDef        CAN_InitStructure;
  CAN_FilterInitTypeDef  CAN_FilterInitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;

  //使能相关时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);//使能PORTA时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟

  //初始化GPIO
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOD, &GPIO_InitStructure);//初始化PA11,PA12

  //引脚复用映射配置
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource1, GPIO_AF_CAN1); //GPIOA11复用为CAN1
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource0, GPIO_AF_CAN1); //GPIOA12复用为CAN1


  //CAN单元设置
  CAN_InitStructure.CAN_TTCM = DISABLE;	//非时间触发通信模式
  CAN_InitStructure.CAN_ABOM = DISABLE;	//软件自动离线管理
  CAN_InitStructure.CAN_AWUM = DISABLE; //睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
  CAN_InitStructure.CAN_NART = ENABLE;	//禁止报文自动传送
  CAN_InitStructure.CAN_RFLM = DISABLE;	//报文不锁定,新的覆盖旧的
  CAN_InitStructure.CAN_TXFP = DISABLE;	//优先级由报文标识符决定
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;	 //模式设置
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
  CAN_InitStructure.CAN_BS1 = CAN_BS1_8tq; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
  CAN_InitStructure.CAN_BS2 = CAN_BS2_6tq; //Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
  CAN_InitStructure.CAN_Prescaler = 3; //分频系数(Fdiv)为brp+1
  CAN_Init(CAN1, &CAN_InitStructure);   // 初始化CAN1

  //配置过滤器
  CAN_FilterInitStructure.CAN_FilterNumber = 0;	 //过滤器0
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit; //32位
  CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000; ////32位ID
  CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000; //32位MASK
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0; //过滤器0关联到FIFO0
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE; //激活过滤器0
  CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化

  CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE); //FIFO0消息挂号中断允许.

  NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;     // 主优先级为1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

}

/**
  * @name	CAN2_Init()
  * @brief 	CAN2初始化函数
  * @param  None
  */
void CAN2_Init(void) {
  GPIO_InitTypeDef GPIO_InitStructure;
  CAN_InitTypeDef        CAN_InitStructure;
  CAN_FilterInitTypeDef  CAN_FilterInitStructure;
  NVIC_InitTypeDef  NVIC_InitStructure;

  //使能相关时钟
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能PORTA时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);//使能CAN2时钟

  //初始化GPIO
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化PA11,PA12

  //引脚复用映射配置
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2); //GPIOB12复用为CAN2
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2); //GPIOB13复用为CAN2


  //CAN单元设置
  CAN_InitStructure.CAN_TTCM = DISABLE;	//非时间触发通信模式
  CAN_InitStructure.CAN_ABOM = DISABLE;	//软件自动离线管理
  CAN_InitStructure.CAN_AWUM = DISABLE; //睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
  CAN_InitStructure.CAN_NART = ENABLE;	//禁止报文自动传送
  CAN_InitStructure.CAN_RFLM = DISABLE;	//报文不锁定,新的覆盖旧的
  CAN_InitStructure.CAN_TXFP = DISABLE;	//优先级由报文标识符决定
  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;	 //模式设置
  CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
  CAN_InitStructure.CAN_BS1 = CAN_BS1_8tq; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
  CAN_InitStructure.CAN_BS2 = CAN_BS2_6tq; //Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
  CAN_InitStructure.CAN_Prescaler = 3; //分频系数(Fdiv)为brp+1
  CAN_Init(CAN2, &CAN_InitStructure);   // 初始化CAN2

  //配置过滤器
  CAN_FilterInitStructure.CAN_FilterNumber = 27;	 //过滤器0
  CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
  CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit; //32位
  CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000; ////32位ID
  CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000; //32位MASK
  CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
  CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0; //过滤器0关联到FIFO0
  CAN_FilterInitStructure.CAN_FilterActivation = ENABLE; //激活过滤器0
  CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化

  CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE); //FIFO0消息挂号中断允许.
  CAN_ITConfig(CAN2, CAN_IT_TME, ENABLE);	//开启can2中断

  NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;     // 主优先级为1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}


/**
  * @name	CAN1_RX0_IRQHandler()
  * @brief 	CAN1接受中断
  * @param  None
  */

int32_t motorCurrentReal, motorCurrentSet;

void CAN1_RX0_IRQHandler(void) {
  CanRxMsg rx_message;

  if(CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET) {
    CAN_ClearITPendingBit(CAN1, CAN_IT_FF0);
    CAN_ClearFlag(CAN1, CAN_FLAG_FF0);
    CAN_Receive(CAN1, CAN_FIFO0, &rx_message);//读取数据

    switch(rx_message.StdId) {
			
      case 0x203: { //矿仓提升电机1
        reductMotorDataRecieve(&rx_message, &(grabTaskStructure.liftoreMotor1.baseInf));
        DetectHook(DETECT_LIFT_LM1_MOTOR);
        break;
      }

      case 0x204: { //矿仓提升电机2
        reductMotorDataRecieve(&rx_message, &(grabTaskStructure.liftoreMotor2.baseInf));
        DetectHook(DETECT_LIFT_LM2_MOTOR);
        break;
      }

      case 0x205: { //底盘电机1
        chassisMotorDataRecieve(&rx_message, &(chassisTaskStructure.motor[CHASSIS_CM1].baseInf));
        DetectHook(DETECT_CHASSIS_CM1_MOTOR);
        break;
      }

      case 0x206: { //底盘电机2
        chassisMotorDataRecieve(&rx_message, &(chassisTaskStructure.motor[CHASSIS_CM2].baseInf));
        DetectHook(DETECT_CHASSIS_CM2_MOTOR);
        break;
      }

      case 0x207: { //底盘电机3
        chassisMotorDataRecieve(&rx_message, &(chassisTaskStructure.motor[CHASSIS_CM3].baseInf));
        DetectHook(DETECT_CHASSIS_CM3_MOTOR);
        break;
      }

      case 0x208: { //底盘电机4
        chassisMotorDataRecieve(&rx_message, &(chassisTaskStructure.motor[CHASSIS_CM4].baseInf));
        DetectHook(DETECT_CHASSIS_CM4_MOTOR);
        break;
      }

      default:
        break;
    }
  }
}

/**
  * @name	CAN2_RX0_IRQHandler()
  * @brief 	CAN2接受中断
  * @param  None
  * @return None
  */
void CAN2_RX0_IRQHandler(void) {
  CanRxMsg rx_message;

  if (CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET) {
    CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
    CAN_Receive(CAN2, CAN_FIFO0, &rx_message);

    switch(rx_message.StdId) {
      case 0x101: { //B板
        for(int i = 0; i < LIMSW_LENGTH; i++) {
					grabTaskStructure.limsw[i]=rx_message.Data[i];
					
        }
				DetectHook(DETECT_BOARD_B);
        break;
      }
//			case 0x141:{ //MF9025 id1
//				motor_measure_MF9025(&(grabTaskStructure.armbasepMotor1.baseInf), rx_message.Data,1);
//				DetectHook(DETECT_ARMBASE_P_MOTOR1);
//				break;
//			}
//			case 0x142:{ //MF9025 id2
//				motor_measure_MF9025(&(grabTaskStructure.armbasepMotor2.baseInf), rx_message.Data,2);
//				DetectHook(DETECT_ARMBASE_P_MOTOR2);
//				break;
//			}
//      case 0x201: { //夹子旋转电机
//        reductMotorDataRecieve(&rx_message, &(grabTaskStructure.singlerotateMotor.baseInf));
//        DetectHook(DETECT_SINGLERO_RM_MOTOR);
//        break;
//      }
			
//      case 0x202: { //机械臂底座yaw轴旋转电机
//        reductMotorDataRecieve(&rx_message, &(grabTaskStructure.armbaseyMotor.baseInf));
//        DetectHook(DETECT_ARMBASE_Y_MOTOR);
//        break;
//      }
      case 0x201: { //夹子旋转电机
        reductMotorDataRecieve(&rx_message, &(grabTaskStructure.parallelMotor1.baseInf));
        DetectHook(DETECT_PARALLElLMOTOR1);
        break;
      }
			case 0x202: { //夹子旋转电机
        reductMotorDataRecieve(&rx_message, &(grabTaskStructure.parallelMotor2.baseInf));
        DetectHook(DETECT_PARALLElLMOTOR2);
        break;
      }
      case 0x203: { //机械臂中间pitch轴旋转电机//yaw cjh
        reductMotorDataRecieve(&rx_message, &(grabTaskStructure.armmidyMotor.baseInf));
        DetectHook(DETECT_ARMMID_P_MOTOR);
        break;
      }
			
      case 0x204: { //平台X方向平移电机
        reductMotorDataRecieve(&rx_message, &(grabTaskStructure.xplatMotor1.baseInf));
        DetectHook(DETECT_XPLAT_MOTOR);
        break;
      }
      case 0x205: { //平台Y方向平移电机
        reductMotorDataRecieve(&rx_message, &(grabTaskStructure.yplatMotor1.baseInf));
        DetectHook(DETECT_YPLAT_MOTOR1);
        break;
      }
			case 0x206: { //平台Y方向平移电机2
        reductMotorDataRecieve(&rx_message, &(grabTaskStructure.yplatMotor2.baseInf));
        DetectHook(DETECT_YPLAT_MOTOR2);
        break;
      }
			case 0x207: { //机械臂底座电机1 cjh
        reductMotorDataRecieve(&rx_message, &(grabTaskStructure.armbasepMotor1.baseInf));
        DetectHook(DETECT_ARMBASE_P_MOTOR1);
        break;
      }
			case 0x208: { //机械臂底座电机2 cjh
        reductMotorDataRecieve(&rx_message, &(grabTaskStructure.armbasepMotor2.baseInf));
        DetectHook(DETECT_ARMBASE_P_MOTOR2);
        break;
      }
//			case 0x206: {//图传旋转电机
//        reductMotorDataRecieve(&rx_message, &(basicTaskStructure.camtopMotor.baseInf));
//        DetectHook(DETECT_CAMTOP_MOTOR);
//			}
      default:
        break;
    }
  }
}

void djiMotorCurrentSendQueue(CAN_TypeDef* CANx, uint32_t stdId, s16 *current, u8 len) {
  CanTxMsg can_msg;

  can_msg.StdId = stdId;  //标识符
  can_msg.IDE = CAN_ID_STD;
  can_msg.RTR = CAN_RTR_DATA;	// 消息类型为数据帧，一帧8位
  can_msg.DLC = 8;				// 发送8帧信息

  u8 i = 0;

  for(i = 0; i < 8; i++) can_msg.Data[i] = 0;

  i = 0;

  for(; i < len; i++) {
    can_msg.Data[2 * i] = (u8)((int16_t)current[i] >> 8);  //控制电流值高8位
    can_msg.Data[2 * i + 1] = (u8)((int16_t)current[i]);   //控制电流值低8位
  }

  if(CANx == CAN1) {
    xQueueSend(CAN1_Queue, &can_msg, 1); //向队列中填充内容
  } else if(CANx == CAN2) {
    xQueueSend(CAN2_Queue, &can_msg, 1); //向队列中填充内容
  }
}
