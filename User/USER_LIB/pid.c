/**************************************************************************
 * @ file     	PID.c
 * @ brief    	PID�㷨
 * @ writer		������
 * @ Q Q		2296054658
 **************************************************************************/
#include "pid.h"
#include "user_lib.h"

/**
  * @name 	PIDInit()
  * @brief 	PID������ʼ��
  * @param	pid:��Ҫ��ʼ���Ľṹ��
  * @param	kp:ϵ��p
  * @param	ki:ϵ��i
  * @param	kd:ϵ��d
  * @param	ka:ϵ��a
  * @param	max_out:������
  * @param	dead_band:�������ڴ˷�Χ�ڲ�����PID����ֱ�����0
  * @param	i_band:�������䣬�ڴ˷�Χ�ڽ��л���
  * @param	max_input:����ֵ���������
  * @param	e_max:����ʱ��������
  * @param	i_maxout:i��������
  * @return None
  */
void PIDInit(PidTypeDef *pid, float kp, float ki, float kd, float ka, float max_out, float dead_band, float i_band, float max_input, float i_maxout, pid_mode_e model) {
  pid->kKp = kp;
  pid->kKi = ki;
  pid->Kd = kd;

  if(ka < 0 || ka >= 1)
    pid->Ka = 0;
  else
    pid->Ka = ka;

  if(max_out < 0)
    pid->max_out = 65535 / 2 - 1;
  else
    pid->max_out = max_out;

  if(max_input < 0)
    pid->max_input = 65535 / 2 - 1;
  else
    pid->max_input = max_input;

  if(i_maxout < 0)
    pid->intergral_maxOut = pid->max_out;
  else
    pid->intergral_maxOut = i_maxout;

  if(dead_band < 0)
    pid->dead_band = 0;
  else
    pid->dead_band = dead_band;

  if(i_band < 0)
    pid->intergral_band = 2 * pid->max_input;
  else
    pid->intergral_band = i_band;

  pid->model = model;
}

/**
  * @name 	PIDOutputClear()
  * @brief 	���PID�ṹ������
  * @param	pid:��Ҫ��յĽṹ��
  * @return None
  */
void PIDOutputClear(PidTypeDef *pid) {
  pid->output = 0;
  pid->I_output = 0;
}

/**
  * @name 	PID_Calc()
  * @brief 	PID���㺯������������������pid->output������
  * @param	pid:����PID��������Ľṹ��
  * @param	rel_val:PID�����ʵ��ֵֵ
  * @param	set_val:PID���������ֵ
  * @return None
  */
void PID_Calc(PidTypeDef *pid, float rel_val, float set_val) {
//    if(set_val > pid->max_input)
//        set_val = pid->max_input;
//    else if(set_val < -(pid->max_input))
//        set_val = -(pid->max_input);

  pid->err = set_val - rel_val; //��ǰ���

  /*360ת��PIDλ�ü����0����*/
  if(pid->model == POSITION_360) {
    if(pid->err > (pid->max_input / 2)) {
      //��̨����
      pid->err = pid->err - pid->max_input;
    } else if((-pid->err) > (pid->max_input / 2)) {
      pid->err = pid->max_input + pid->err;
    }
  } else if(pid->model == POSITION_180) {
    if(pid->err > (pid->max_input)) {
      pid->err = pid->err - 2 * pid->max_input;     //��̨����
    } else if((-pid->err) > (pid->max_input)) {
      pid->err = pid->max_input * 2 + pid->err;
    }
  }

  if(f_abs(pid->err) > pid->dead_band) {
    //�ж��Ƿ����������û�н��������PID���㣬���������0

    //�ж��Ƿ��ڻ��������ڣ�����ھͽ��л��֣����ھ�ʹ�û����𽥹�0
    if(f_abs(pid->err) <= pid->intergral_band)
      pid->I_output = pid->I_output + (pid->kKi) * (pid->err);
    else
      pid->I_output = pid->I_output * 0.99f;

    //���ƻ������ֵ
    if(pid->I_output > pid->intergral_maxOut)
      pid->I_output = pid->intergral_maxOut;
    else if(pid->I_output < -pid->intergral_maxOut)
      pid->I_output = -pid->intergral_maxOut;

    //����PID���
    pid->P_output = pid->kKp * (pid->err);
    pid->D_output = pid->Kd * (1 - pid->Ka) * (pid->err - pid->err_last) + (pid->Ka) * (pid->d_last);
    pid->d_last = pid->D_output;
    pid->output = pid->P_output + pid->I_output + pid->D_output;

    //�Ƿ񳬳�������
    if(pid->output > pid->max_out)
      pid->output = pid->max_out;

    if(pid->output < -(pid->max_out))
      pid->output = -(pid->max_out);

  } else {
    pid->output *= 0.99f;
    pid->I_output *= 0.99f;
  }

  pid->err_last = pid->err;
}
