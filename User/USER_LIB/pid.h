/**************************************************************************
 * @ file     	PID.h
 * @ brief    	PID�㷨
 * @ writer		������
 * @ Q Q		2296054658
 **************************************************************************/
#ifndef _PID_H
#define _PID_H

#include "stm32f4xx.h"

typedef enum {
  SPEED = 0,
  POSITION_180,
  POSITION_360,
} pid_mode_e;

typedef struct {
  //PID ������
  float kKp;			//����ϵ������Ҫ������Ӧ�ٶȣ��������׳���
  float kKi;			//����ϵ��������������̬������С��Χ�ڵķ�����
  float Kd;			//΢��ϵ����ʵ�־���Ԥ�⹦�ܣ���С����
  float Ka;			//��ͨ�˲�ϵ�� ����[0,1),��Խ���˲�Լ���ԣ������������𵴵�ʱ�򽫴˲�������

  float max_out;  		//������
  float dead_band;		//PIDƫ������
  float intergral_band;	//������
  float intergral_maxOut;	//����������
  float max_input;		//�������
  pid_mode_e model;			//PID�������ͣ�0Ϊ�ٶȿ��ƣ�1Ϊλ�ÿ�������������Ӧ180�ȣ�ʵ�ʶ��������и�����2Ϊλ�ÿ�������������Ӧ360�ȣ�ʵ�ʶ�����Ϊ����

  //PID���ֵ
  float output;		//���
  float P_output;		//P���
  float I_output;		//I���
  float D_output;		//D���

  //���
  float err;				//��ǰ���
  float err_last;			//��һ�ε����
  float d_last;			//��һ�ε�ϵ��D
} PidTypeDef;

void PIDInit(PidTypeDef *pid, float kp, float ki, float kd, float ka, float max_out, float dead_band, float i_band, float max_input, float i_maxout, pid_mode_e model);
void PIDOutputClear(PidTypeDef *pid);
void PID_Calc(PidTypeDef *pid, float rel_val, float set_val);	//PID���㺯��

#endif
