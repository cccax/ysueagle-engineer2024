#include "buzzer.h"

#define BUZZER_PERIOD 899

volatile uint16_t buzzerInitTime, buzzerRemainTime;// 10ms为单位
volatile uint8_t buzzerRemainNum, buzzerStat, buzzerFreq;
void buzzer_set(uint16_t psc);
void buzzer_reset(void);

void buzzerOn(uint16_t timeInMs, uint16_t repeatNum, uint8_t freq) {
  buzzerInitTime = timeInMs / 10;
  buzzerRemainNum = repeatNum;
  buzzerFreq = freq;
}

void buzzer_configuration() {
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);//90Mhz
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);

  RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM12, ENABLE);
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM12, DISABLE);

  GPIO_PinAFConfig(GPIOH, GPIO_PinSource6, GPIO_AF_TIM12);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  GPIO_Init(GPIOH, &GPIO_InitStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC1Init(TIM12, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM12, TIM_OCPreload_Enable);

  TIM_ARRPreloadConfig(TIM12, ENABLE);

  TIM_TimeBaseInitStructure.TIM_Period = BUZZER_PERIOD;
  TIM_TimeBaseInitStructure.TIM_Prescaler = 20;//5kHz
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

  TIM_TimeBaseInit(TIM12, &TIM_TimeBaseInitStructure);
  TIM_Cmd(TIM12, ENABLE);

  buzzer_set(30);
  delay_ms(200);
  buzzer_set(25);
  delay_ms(200);
  buzzer_set(20);
  delay_ms(200);
  buzzer_reset();
}

void buzzer_set(uint16_t psc) {
  TIM12->PSC = psc;
  TIM_SetCompare1(TIM12, (int)(BUZZER_PERIOD + 1) / 2);
}
void buzzer_reset(void) {
  TIM_SetCompare1(TIM12, 0);
}

void buzzerCtrl(void) {
  if(buzzerRemainNum > 0 && buzzerRemainTime == 0) {// 正开始或者进repeat间隔 即状态改变时刻
    buzzerRemainTime = buzzerInitTime * 2;// 响或不响的时长 注意修改

    if(buzzerStat != 1) {// 正开始
      buzzerRemainNum --;//响次数-1
      buzzerStat = 1;
    } else buzzerStat = 0; // 进repeat间隔
  } else if(buzzerRemainNum == 0 && buzzerRemainTime == 0) {// 停止
    buzzerStat = 0;
    buzzerInitTime = 0;
  }

  if(buzzerRemainTime > 0) {// 减时间
    buzzerRemainTime --;
  }

  if(buzzerStat) buzzer_set(buzzerFreq);// 19: 5kHz 24：4kHz
  else buzzer_reset();
}
