#include "sensor.h"

void sensor_configuration() { //光电传感器
  GPIO_InitTypeDef GPIO_InitStructure;

  RCC_AHB1PeriphClockCmd(SENSORL_PORT_CLK | SENSORM_PORT_CLK | SENSORR_PORT_CLK, ENABLE);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Fast_Speed;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

  GPIO_InitStructure.GPIO_Pin = SENSORL_PIN;
  GPIO_Init(SENSORL_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = SENSORM_PIN;
  GPIO_Init(SENSORM_PORT, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = SENSORR_PIN;
  GPIO_Init(SENSORR_PORT, &GPIO_InitStructure);
}
