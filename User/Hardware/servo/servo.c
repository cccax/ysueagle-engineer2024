#include "servo.h"
#include "grab_task.h"

extern u8 DCF_CTRLl;
extern void USART6_Send_Data(u8 data);

void TIM5_CH1_Init(uint16_t arr, uint16_t psc) { //H10 ͼ�����
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);

  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;

  GPIO_PinAFConfig(GPIOH, GPIO_PinSource10, GPIO_AF_TIM5);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOH, &GPIO_InitStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC1Init(TIM5, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM5, ENABLE);

  TIM_TimeBaseInitStructure.TIM_Prescaler = psc; //TIM5ʱ��Ƶ��90Mhz
  TIM_TimeBaseInitStructure.TIM_Period = arr;
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStructure);

  TIM_Cmd(TIM5, ENABLE);
}

void TIM4_CH4_Init(uint16_t arr, uint16_t psc) { //D15 ���ոǶ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;

  GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);

  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM4, ENABLE);

  TIM_TimeBaseInitStructure.TIM_Prescaler = psc; //TIM4ʱ��Ƶ��
  TIM_TimeBaseInitStructure.TIM_Period = arr;
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

  TIM_Cmd(TIM4, ENABLE);
}

void GPIO_D15_Init(void) { //ͨ��ѡ���ź�
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_WriteBit(GPIOD, GPIO_Pin_15, Bit_RESET);
  //GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_RESET);
  GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_RESET);


  GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_SET);
  GPIO_WriteBit(GPIOA, GPIO_Pin_3, Bit_RESET);
}

double map(double smallData, uint16_t bigStart, uint16_t bigTo) { //smallData:�����Ƕ�(��ƫ180��),bigStart:�����Χ���,bigTo:�����Χ�յ�
  double rateDat;
  rateDat = smallData / 180;
  uint16_t res = rateDat * (bigTo - bigStart) + bigStart;
  return res;
}

void setServoAngle(uint8_t ID, uint16_t angle) {
  uint16_t pwmValue = 0;
  pwmValue = map(angle, 24, 124); //0-180�Ƕ�ӳ������ ռ�ձ�2.5%-12.5%

  switch(ID) {
    case CAM_SERVO:
      TIM_SetCompare1(TIM5, pwmValue);
      break;

    case DEPO_SERVO:
      TIM_SetCompare4(TIM4, pwmValue);
      break;
  }
}

void setScreenChannel(uint8_t newState) {
//	          *armbaseyExp = RAMP_float(grabTaskStructure.armbaseyMotor.fixedEcd[ARM_BASE_Y_BIG_ISLAND], *armbaseyExp, 1850/1000.0*GRAB_TASK_MS);//������Ŀ��߶�
//            *armmidpExp = RAMP_float(grabTaskStructure.armmidpMotor.fixedEcd[ROTATE_BIG_ISLAND_Y], *armmidpExp, 1420/1000.0*GRAB_TASK_MS);//���͵�Ŀ�곤��
// 		grabTaskStructure.armmidpMotor.totalEcdSet = grabTaskStructure.armbaseyMotor.fixedEcd[ROTATE_MAX_STEP];
//		grabTaskStructure.armbaseyMotor.totalEcdSet = grabTaskStructure.armmidpMotor.fixedEcd[ROTATE_MAX_STEP_Y];
  switch (newState) { //?????
//			grabTaskStructure.armbaseyMotor.totalEcdSet = 1200;//rotateExp, 1850/1000.0*GRAB_TASK_MS);//������Ŀ��߶�
//           grabTaskStructure.armmidpMotor.totalEcdSet = - 500 ;
    case 0: {
      GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_RESET);
      GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_RESET);
      GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_RESET);
      break;
    }

    case 1: {
      GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_RESET);
      GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_SET);
      GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_RESET);
      break;
    }

    case 2: {
      GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_RESET);
      GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_RESET);
      GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_SET);
      break;
    }

    case 3: {
      GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET);
      GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_RESET);
      GPIO_WriteBit(GPIOA, GPIO_Pin_0, Bit_RESET);
      break;
    }

    default:
      break;
  }

//	if(newState == 0)
//		GPIO_WriteBit(GPIOD, GPIO_Pin_15, Bit_RESET);
//	else
//		GPIO_WriteBit(GPIOD, GPIO_Pin_15, Bit_SET);
}

void servo_configuration(void) {
  TIM5_CH1_Init(999, 1799); //ͼ�����
  GPIO_D15_Init(); //ͨ��ѡ���ź�
  setServoAngle(CAM_SERVO, CAM_ANGLE_VERTICAL);
  setScreenChannel(0);
}
