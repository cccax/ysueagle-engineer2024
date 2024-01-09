/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       main.c/h
  * @brief      stm32初始化以及开始任务freeRTOS。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef MAIN_H
#define MAIN_H

typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned char u8;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

#define SysCoreClock 180

#define RC_NVIC 4
#define CAN1_NVIC 4
#define CAN2_NVIC 4
#define TIM3_NVIC 5
#define TIM6_NVIC 3
#define SPI5_RX_NVIC 5
#define MPU_INT_NVIC 5
#define USART6_NVIC 10
#define USART2_NVIC 10
#define USART3_NVIC 10

#define Latitude_At_ShenZhen 22.57025f

#ifndef NULL
#define NULL 0
#endif

#ifndef PI
#define PI 3.14159265358979f
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#endif /* __MAIN_H */
