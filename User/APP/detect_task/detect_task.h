/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       detect_task.c/h
  * @brief
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
#ifndef DETECT_Task_H
#define DETECT_Task_H
#include "main.h"
//错误码以及对应设备顺序
enum errorList {
  DBUSTOE = 0,//遥控器

  DETECT_XPLAT_MOTOR,//平台平移电机
	DETECT_YPLAT_MOTOR1,
	DETECT_YPLAT_MOTOR2,
  DETECT_LIFT_LM1_MOTOR,//平台提升两个电机
  DETECT_LIFT_LM2_MOTOR,

  //DETECT_ARMBASE_Y_MOTOR,//机械臂底座yaw轴旋转电机
	DETECT_ARMBASE_P_MOTOR1,
	DETECT_ARMBASE_P_MOTOR2,
  DETECT_ARMMID_P_MOTOR,//机械臂中部pitch轴旋转电机
  //DETECT_SINGLERO_RM_MOTOR,//夹子旋转一个电机
  DETECT_PARALLElLMOTOR1,
	DETECT_PARALLElLMOTOR2,
	DETECT_BOARD_B,
//	DETECT_CAMTOP_MOTOR,
	
  DETECT_CHASSIS_CM1_MOTOR,//底盘四个电机
  DETECT_CHASSIS_CM2_MOTOR,
  DETECT_CHASSIS_CM3_MOTOR,
  DETECT_CHASSIS_CM4_MOTOR,
	
  errorListLength,
};

typedef __packed struct {
  //数据丢失周期检测，数据错误逻辑检测
  uint32_t newTime;		//上次更新的时间
  uint32_t lastTime;
  uint32_t Losttime;
  uint32_t worktime;		//上电时间
  uint16_t setOfflineTime : 12;
  uint16_t setOnlineTime : 12;
  uint8_t enable : 1;
  uint8_t Priority : 4;
  uint8_t errorExist : 1;
  uint8_t isLost : 1;
  uint8_t dataIsError : 1;

  fp32 frequency;
  bool_t (*dataIsErrorFun)(void);		//数据错误检查函数
  void (*solveLostFun)(void);			//数据丢失处理函数
  void (*solveDataErrorFun)(void);	//数据错误处理函数
} error_t;

extern bool_t toe_is_error(uint8_t err);
extern void DetectTask(void *pvParameters);
void DetectHook(uint8_t toe);
extern const error_t *getErrorListPoint(void);
extern void DetectHook(uint8_t toe);
#endif
