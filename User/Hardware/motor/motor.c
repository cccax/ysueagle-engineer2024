#include "motor.h"
/*
float getMotorSpeedByPosition(motor_speed_calc_t* calcS, int16_t nowEcd, uint32_t nowTime)
{
    if(calcS == NULL)
        return 0;
    if(nowTime - calcS->lastTime > calcS->minCalcTime)
    {
        int16_t posInc = nowEcd - calcS->lastEcd;
        if(posInc > calcS->ecdRange/2)
            posInc -= calcS->ecdRange;
        if(posInc < -calcS->ecdRange/2)
            posInc += calcS->ecdRange;
        float rpInc = ((float)posInc)/calcS->ecdRange;
        *(calcS->speed_rpm) = rpInc/(nowTime - calcS->lastTime)*60000;

        calcS->lastTime = nowTime;
        calcS->lastEcd = nowEcd;
    }
    return *(calcS->speed_rpm);
}

void motorSpeedCalcInit(motor_speed_calc_t* calcS, motor_measure_t *motorS, int16_t ecdMinVal, int16_t ecdMaxVal, uint32_t time, uint16_t minCalcTime)
{
    if(calcS == NULL || motorS == NULL)
        return;
    calcS->ecdMaxVal = ecdMaxVal;
    calcS->ecdMinVal = ecdMinVal;
    calcS->lastTime = time;
    calcS->minCalcTime = minCalcTime;
    calcS->lastEcd = motorS->real_ecd;
    calcS->speed_rpm = &(motorS->real_speed_rpm);
    calcS->ecdRange = ecdMaxVal - ecdMinVal;
}*/

void reductMotorDataRecieve(CanRxMsg *canMsg, motor_measure_t *motor) {
  //速度处理
  motor->speed_rpm = (canMsg->Data[2] << 8) | canMsg->Data[3];

  if(motor->reduction_ratio == 0) {
    return;
  }

  motor->real_speed_rpm = (motor->speed_rpm) / motor->reduction_ratio;

  //位置处理
  motor->rotor_ecd = (((canMsg->Data[0] << 8) | canMsg->Data[1]) / motor->reduction_ratio); //接收到的真实数据值

  if((motor->rotor_ecd - motor->lastRotor_ecd) > 4095 / motor->reduction_ratio) {
    motor->position_cnt--;
  } else if((motor->rotor_ecd - motor->lastRotor_ecd) < -4095 / motor->reduction_ratio) {
    motor->position_cnt++;
  }

  if(motor->position_cnt >= motor->reduction_ratio)
    motor->position_cnt = 0;
  else if(motor->position_cnt < 0)
    motor->position_cnt = motor->reduction_ratio - 1;

  motor->real_ecd = motor->position_cnt * (8191 / motor->reduction_ratio) + motor->rotor_ecd;

  //电机轴相对初始位置处理
  if((motor->real_ecd - motor->lastReal_ecd) > 4095) {
    //反转过界
    motor->relativeEcd -= 8192 - motor->real_ecd + motor->lastReal_ecd;
  } else if((motor->real_ecd - motor->lastReal_ecd) < -4095) {
    //正转过界
    motor->relativeEcd += motor->real_ecd + 8192 - motor->lastReal_ecd;
  } else {
    //不过界
    motor->relativeEcd += motor->real_ecd - motor->lastReal_ecd;
  }

  if(!motor->isOnlined) { //上电位置为初始位置
    motor->isOnlined = 1;
    motor->relativeEcd = 0;
  }

  motor->lastReal_ecd = motor->real_ecd;
  motor->lastRotor_ecd = motor->rotor_ecd;

  //电流处理
  motor->real_current = (canMsg->Data[4] << 8) | canMsg->Data[5];
}

void chassisMotorDataRecieve(CanRxMsg *canMsg, motor_measure_t *motor) {
  motor->speed_rpm = (canMsg->Data[2] << 8) | canMsg->Data[3];

  if(motor->reduction_ratio != 0)
    motor->real_speed_rpm = ((float)motor->speed_rpm) / motor->reduction_ratio;
  else
    motor->real_speed_rpm = motor->speed_rpm;
}

void motorInit(motor_measure_t *motor, u16 ratio) {
  if(ratio < 1)
    ratio = 1;

  motor->reduction_ratio = ratio;
  motor->given_current = 0;
}
