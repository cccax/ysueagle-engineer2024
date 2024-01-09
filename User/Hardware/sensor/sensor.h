#ifndef __SENSOR_H__
#define __SENSOR_H__

#include "stm32f4xx.h"

void sensor_configuration(void);

#define SENSORL_PORT_CLK RCC_AHB1Periph_GPIOI
#define SENSORL_PORT GPIOI
#define SENSORL_PIN GPIO_Pin_0
#define SENSORM_PORT_CLK RCC_AHB1Periph_GPIOH
#define SENSORM_PORT GPIOH
#define SENSORM_PIN GPIO_Pin_12
#define SENSORR_PORT_CLK RCC_AHB1Periph_GPIOH
#define SENSORR_PORT GPIOH
#define SENSORR_PIN GPIO_Pin_11

#endif
