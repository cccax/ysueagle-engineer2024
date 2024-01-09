/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       detect_task.c/h
  * @brief
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
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
//�������Լ���Ӧ�豸˳��
enum errorList {
  DBUSTOE = 0,//ң����

  DETECT_XPLAT_MOTOR,//ƽ̨ƽ�Ƶ��
	DETECT_YPLAT_MOTOR1,
	DETECT_YPLAT_MOTOR2,
  DETECT_LIFT_LM1_MOTOR,//ƽ̨�����������
  DETECT_LIFT_LM2_MOTOR,

  //DETECT_ARMBASE_Y_MOTOR,//��е�۵���yaw����ת���
	DETECT_ARMBASE_P_MOTOR1,
	DETECT_ARMBASE_P_MOTOR2,
  DETECT_ARMMID_P_MOTOR,//��е���в�pitch����ת���
  //DETECT_SINGLERO_RM_MOTOR,//������תһ�����
  DETECT_PARALLElLMOTOR1,
	DETECT_PARALLElLMOTOR2,
	DETECT_BOARD_B,
//	DETECT_CAMTOP_MOTOR,
	
  DETECT_CHASSIS_CM1_MOTOR,//�����ĸ����
  DETECT_CHASSIS_CM2_MOTOR,
  DETECT_CHASSIS_CM3_MOTOR,
  DETECT_CHASSIS_CM4_MOTOR,
	
  errorListLength,
};

typedef __packed struct {
  //���ݶ�ʧ���ڼ�⣬���ݴ����߼����
  uint32_t newTime;		//�ϴθ��µ�ʱ��
  uint32_t lastTime;
  uint32_t Losttime;
  uint32_t worktime;		//�ϵ�ʱ��
  uint16_t setOfflineTime : 12;
  uint16_t setOnlineTime : 12;
  uint8_t enable : 1;
  uint8_t Priority : 4;
  uint8_t errorExist : 1;
  uint8_t isLost : 1;
  uint8_t dataIsError : 1;

  fp32 frequency;
  bool_t (*dataIsErrorFun)(void);		//���ݴ����麯��
  void (*solveLostFun)(void);			//���ݶ�ʧ������
  void (*solveDataErrorFun)(void);	//���ݴ�������
} error_t;

extern bool_t toe_is_error(uint8_t err);
extern void DetectTask(void *pvParameters);
void DetectHook(uint8_t toe);
extern const error_t *getErrorListPoint(void);
extern void DetectHook(uint8_t toe);
#endif
