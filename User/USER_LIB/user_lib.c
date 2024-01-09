#include "user_lib.h"
#include "arm_math.h"
#include "main.h"

float Target_Speed_Calc(speed_calc_data_t *S, uint32_t time, float position) {
  S->delay_cnt++;

  if (time != S->last_time) {
    S->speed = (position - S->last_position) / (time - S->last_time) * 2;//�����ٶ�

    S->processed_speed = S->speed;

    S->last_time = time;
    S->last_position = position;
    S->last_speed = S->speed;
    S->delay_cnt = 0;
  }

  if(S->delay_cnt > 300/*100*/) { // delay 200ms speed = 0
    S->processed_speed = 0;//ʱ���������Ϊ�ٶȲ���
  }

  return S->processed_speed;//��������ٶ�
}


/**
  * @brief  б�º���,ʹĿ�����ֵ������������ֵ
  * @param  �����������,��ǰ���,�仯�ٶ�(Խ��Խ��)������Ӧ����360�ȵĵ��б����
  * @retval ��ǰ���
  * @attention
  */
float RAMP_float( float final, float now, float ramp) {
  float buffer = 0;

  buffer = final - now;

  if (buffer > 0) {
    if (buffer > ramp) {
      now += ramp;
    } else {
      now += buffer;
    }
  } else {
    if (buffer < -ramp) {
      now += -ramp;
    } else {
      now += buffer;
    }
  }

  return now;
}

//ѭ��б�º�����������̨����б�£�maxVal��minVal�ֱ�����̨λ��ʵ��ֵ���������ֵ����Сֵ
float loop_ramp_float(float final, float now, float ramp, float minVal, float maxVal) {
  float err = final - now;

  if(err > (maxVal - minVal) / 2) {
    err -= maxVal - minVal;
  } else if(err < -(maxVal - minVal) / 2)
    err += maxVal - minVal;

  if(err > ramp)
    return now + ramp;
  else if(err < -ramp)
    return now - ramp;
  else
    return final;
}

float f_abs(float num) {
  if(num < 0)
    num = -num;

  return num;
}
/**
  * @brief  б�º���,ʹĿ�����ֵ��������ָ������ֵ
  * @param  Ҫ�ڵ�ǰ��������ۼӵ�ֵ,Ŀ�������,��������
  * @retval Ŀ�������
  * @attention
  *
*/
float RampInc_float( float *buffer, float now, float ramp ) {

  if (*buffer > 0) {
    if (*buffer > ramp) {
      now     += ramp;
      *buffer -= ramp;
    } else {
      now     += *buffer;
      *buffer  = 0;
    }
  } else {
    if (*buffer < -ramp) {
      now += -ramp;
      *buffer -= -ramp;
    } else {
      now     += *buffer;
      *buffer  = 0;
    }
  }

  return now;
}

int16_t s16_abs(int16_t value) {
  if(value > 0)
    return value;
  else
    return -value;
}

//���ٿ���
fp32 invSqrt(fp32 num) {
  fp32 halfnum = 0.5f * num;
  fp32 y = num;
  long i = *(long *)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(fp32 *)&i;
  y = y * (1.5f - (halfnum * y * y));
  return y;
}


/**
  * @brief          һ�׵�ͨ�˲���ʼ��
  * @author         RM
  * @param[in]      һ�׵�ͨ�˲��ṹ��
  * @param[in]      �����ʱ�䣬��λ s
  * @param[in]      �˲�����
  * @retval         ���ؿ�
  */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, fp32 frame_period, fp32 num) {
  first_order_filter_type->frame_period = frame_period;
  first_order_filter_type->num[0] = num;
  first_order_filter_type->input = 0.0f;
  first_order_filter_type->out = 0.0f;
}

/**
  * @brief          һ�׵�ͨ�˲�����
  * @author         RM
  * @param[in]      һ�׵�ͨ�˲��ṹ��
  * @param[in]      �����ʱ�䣬��λ s
  * @retval         ���ؿ�
  */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, fp32 input) {
  //��ͨ�˲� out = a*last_out + ��1-a��*input
  first_order_filter_type->input = input;
  first_order_filter_type->out =
    first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out +
    first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
}

//��������
void abs_limit(fp32 *num, fp32 Limit) {
  if (*num > Limit) {
    *num = Limit;
  } else if (*num < -Limit) {
    *num = -Limit;
  }
}

//�жϷ���λ
fp32 sign(fp32 value) {
  if (value >= 0.0f) {
    return 1.0f;
  } else {
    return -1.0f;
  }
}

//��������
fp32 fp32_deadline(fp32 Value, fp32 minValue, fp32 maxValue) {
  if (Value < maxValue && Value > minValue) {
    Value = 0.0f;
  }

  return Value;
}

//int26����
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue) {
  if (Value < maxValue && Value > minValue) {
    Value = 0;
  }

  return Value;
}

//�޷�����
fp32 fp32_constrain(fp32 Value, fp32 minValue, fp32 maxValue) {
  if (Value < minValue)
    return minValue;
  else if (Value > maxValue)
    return maxValue;
  else
    return Value;
}

//�޷�����
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue) {
  if (Value < minValue)
    return minValue;
  else if (Value > maxValue)
    return maxValue;
  else
    return Value;
}

//ѭ���޷�����
fp32 loop_fp32_constrain(fp32 Input, fp32 minValue, fp32 maxValue) {
  if (maxValue < minValue) {
    return Input;
  }

  if (Input > maxValue) {
    fp32 len = maxValue - minValue;

    while (Input > maxValue) {
      Input -= len;
    }
  } else if (Input < minValue) {
    fp32 len = maxValue - minValue;

    while (Input < minValue) {
      Input += len;
    }
  }

  return Input;
}

//���ȸ�ʽ��Ϊ-PI~PI

//�Ƕȸ�ʽ��Ϊ-180~180
fp32 theta_format(fp32 Ang) {
  return loop_fp32_constrain(Ang, -180.0f, 180.0f);
}

void AHRS_get_height(fp32 *high) {
  if (high != NULL) {
    *high = 0.0f;
  }
}

/**
  * @brief          ���ڻ�ȡ��ǰγ��
  * @author         RM
  * @param[in]      γ�ȵ�ָ�룬fp32
  * @retval         ���ؿ�
  */

void AHRS_get_latitude(fp32 *latitude) {
  if (latitude != NULL) {
    *latitude = Latitude_At_ShenZhen;
  }
}

/**
  * @brief          ���ٿ���������
  * @author         RM
  * @param[in]      ������Ҫ�����ĸ�������fp32
  * @retval         ����1/sqrt ������ĵ���
  */

fp32 AHRS_invSqrt(fp32 num) {
  fp32 halfnum = 0.5f * num;
  fp32 y = num;
  long i = *(long *)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(fp32 *)&i;
  y = y * (1.5f - (halfnum * y * y));
  return y;
}

/**
  * @brief          sin����
  * @author         RM
  * @param[in]      �Ƕ� ��λ rad
  * @retval         ���ض�Ӧ�Ƕȵ�sinֵ
  */

fp32 AHRS_sinf(fp32 angle) {
  return arm_sin_f32(angle);
}
/**
  * @brief          cos����
  * @author         RM
  * @param[in]      �Ƕ� ��λ rad
  * @retval         ���ض�Ӧ�Ƕȵ�cosֵ
  */

fp32 AHRS_cosf(fp32 angle) {
  return arm_cos_f32(angle);
}

/**
  * @brief          tan����
  * @author         RM
  * @param[in]      �Ƕ� ��λ rad
  * @retval         ���ض�Ӧ�Ƕȵ�tanֵ
  */

fp32 AHRS_tanf(fp32 angle) {
  return tanf(angle);
}
/**
  * @brief          ����32λ�������ķ����Ǻ��� asin����
  * @author         RM
  * @param[in]      ����sinֵ�����1.0f����С-1.0f
  * @retval         ���ؽǶ� ��λ����
  */

fp32 AHRS_asinf(fp32 sin) {

  return asinf(sin);
}

/**
  * @brief          �����Ǻ���acos����
  * @author         RM
  * @param[in]      ����cosֵ�����1.0f����С-1.0f
  * @retval         ���ض�Ӧ�ĽǶ� ��λ����
  */

fp32 AHRS_acosf(fp32 cos) {

  return acosf(cos);
}

/**
  * @brief          �����Ǻ���atan����
  * @author         RM
  * @param[in]      ����tanֵ�е�yֵ ����������С������
  * @param[in]      ����tanֵ�е�xֵ ����������С������
  * @retval         ���ض�Ӧ�ĽǶ� ��λ����
  */

fp32 AHRS_atan2f(fp32 y, fp32 x) {
  return atan2f(y, x);
}
