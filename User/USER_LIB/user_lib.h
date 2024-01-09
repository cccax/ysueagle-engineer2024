#ifndef USER_LIB_H
#define USER_LIB_H
#include "main.h"

typedef __packed struct {
  fp32 input;        //��������
  fp32 out;          //�˲����������
  fp32 num[1];       //�˲�����
  fp32 frame_period; //�˲���ʱ���� ��λ s
} first_order_filter_type_t;

typedef struct {
  int delay_cnt;//����������֡Ŀ�겻�����ʱ��,�����ж��ٶ��Ƿ�Ϊ0
  int freq;
  int last_time;//�ϴ��ܵ�Ŀ��Ƕȵ�ʱ��
  float last_position;//�ϸ�Ŀ��Ƕ�
  float speed;//�ٶ�
  float last_speed;//�ϴ��ٶ�
  float processed_speed;//�ٶȼ�����
} speed_calc_data_t;

//���ٿ���
extern fp32 invSqrt(fp32 num);
//���ݽǶȽ���Ŀ���ٶ�
float Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position);
//б�º���,ʹĿ�����ֵ��������ָ������ֵ
float RampInc_float( float *buffer, float now, float ramp);
//б�º���,ʹĿ�����ֵ������������ֵ
float RAMP_float( float final, float now, float ramp);

//һ���˲���ʼ��
extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, const fp32 num);
//һ���˲�����
extern void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input);
//��������
extern void abs_limit(fp32 *num, fp32 Limit);
//�жϷ���λ
extern fp32 sign(fp32 value);
//��������
extern fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue);
//int26����
extern int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
//�޷�����
extern fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue);
//�޷�����
extern int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
//ѭ���޷�����
extern fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue);
//�Ƕ� ���޷� 180 ~ -180
extern fp32 theta_format(fp32 Ang);
float loop_ramp_float(float final, float now, float ramp, float minVal, float maxVal);
float f_abs(float num);
int16_t s16_abs(int16_t value);
//���ȸ�ʽ��Ϊ-PI~PI
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)

#endif
