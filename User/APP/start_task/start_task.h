#ifndef START_TASK_H
#define START_TASK_H

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "chassis_task.h"
#include "grab_task.h"
#include "detect_task.h"
#include "basic_task.h"
#include "oscillography.h"
//#include "referee_usart_task.h"

void startTask(void);

#endif
