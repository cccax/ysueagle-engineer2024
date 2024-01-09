#include "laser.h"

void laser_configuration(void) {
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); //

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;      //��ͨ���ģʽ
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; //100MHz
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;       //����
  GPIO_Init(GPIOD, &GPIO_InitStructure);             //��ʼ��

}

void laser_on(void) {
  //GPIO_SetBits(GPIOG, GPIO_Pin_13);
}

void laser_off(void) {
  //GPIO_ResetBits(GPIOG, GPIO_Pin_13);
}
