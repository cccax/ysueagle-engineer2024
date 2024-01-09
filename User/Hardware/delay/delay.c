#include "delay.h"
#include "stm32f4xx.h"
#include "core_cm4.h"

void delay_init(void) {
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE); //APB1����ʱ������Ƶ��90Mhz

  RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM6, ENABLE);
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_TIM6, DISABLE);

  TIM_DeInit(TIM6);
  TIM_TimeBaseInitStructure.TIM_Period = 0xFFFF; //���������ʱֵ65535us
  TIM_TimeBaseInitStructure.TIM_Prescaler = 89; //90Mhz -> 1Mhz
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //TIM���¼���ģʽ
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; //ʱ�Ӳ��ָ���벶���˲��ã�
  TIM_TimeBaseInit(TIM6, &TIM_TimeBaseInitStructure);

  TIM_ClearFlag(TIM6, TIM_FLAG_Update); //������±�־λ

  uint32_t reload = 0;
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);

  reload = SystemCoreClock / 1000 / 8;
  reload--;

  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
  SysTick->LOAD = reload;
  SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

void delay_us(uint16_t us) { //����ָ��Ĵ�Լ0.5us��ʱ
  TIM_SetCounter(TIM6, 0xFFFF - us + 1); //���ö�ʱֵ
  TIM_Cmd(TIM6, ENABLE); //ʹ�ܶ�ʱ��

  while(TIM_GetFlagStatus(TIM6, TIM_FLAG_Update) != SET);

  TIM_ClearFlag(TIM6, TIM_FLAG_Update); //������±�־λ
  TIM_Cmd(TIM6, DISABLE); //ʧ�ܶ�ʱ��
}

void delay_ms(uint16_t ms) {
  TIM_SetCounter(TIM6, 0xFFFF - 1000 + 1); //���ö�ʱֵ1ms
  TIM_Cmd(TIM6, ENABLE); //ʹ�ܶ�ʱ��

  for(uint16_t i = 0; i < ms; i++) { //ѭ��nms
    while(TIM_GetFlagStatus(TIM6, TIM_FLAG_Update) != SET);

    TIM_SetCounter(TIM6, 0xFFFF - 1000 + 1); //���ö�ʱֵ1ms
    TIM_ClearFlag(TIM6, TIM_FLAG_Update); //������±�־λ
  }

  TIM_Cmd(TIM6, DISABLE); //ʧ�ܶ�ʱ��
}

void smartDelayMs(uint16_t ms) {
  if(xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED)
    delay_ms(ms);
  else
    vTaskDelay(ms);
}
