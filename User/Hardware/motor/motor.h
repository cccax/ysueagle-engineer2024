#ifndef MOTOR_H
#define MOTOR_H
#include "stm32f4xx.h"
#include "stm32f4xx_can.h"
#include "main.h"

#define MOTOR_ECD_FEEDBACL_RANGE 8191

typedef struct {
  fp32 *speed_rpm;
  uint32_t lastTime;
  uint16_t minCalcTime;
  int16_t lastEcd;
  int16_t ecdMaxVal;
  int16_t ecdMinVal;
  uint16_t ecdRange;
} motor_speed_calc_t;

typedef struct {
  int8_t position_cnt;
  uint16_t rotor_ecd;		    //转子角度
  int16_t lastRotor_ecd;		//上次转子角度
  int32_t real_ecd;		    //电机轴角度
  int32_t lastReal_ecd;		//上次电机轴角度
  int32_t relativeEcd;		//相对上电位置电机轴角度
  uint8_t reduction_ratio;    //电机减速比
  int16_t speed_rpm;		    //转子转速
  float real_speed_rpm;	    //电机轴转速
  int16_t given_current;	    //发送的电流值
  int16_t real_current;		//实际的电流值
  uint8_t isOnlined;
} motor_measure_t;

//float getMotorSpeedByPosition(motor_speed_calc_t* calcS, int16_t nowEcd, uint32_t nowTime);
//void motorSpeedCalcInit(motor_speed_calc_t* calcS, motor_measure_t *motorS, int16_t ecdMinVal, int16_t ecdMaxVal, uint32_t time, uint16_t minCalcTime);
void motorInit(motor_measure_t *motor, uint16_t ratio);
void reductMotorDataRecieve(CanRxMsg *canMsg, motor_measure_t *motor);
void chassisMotorDataRecieve(CanRxMsg *canMsg, motor_measure_t *motor);

#endif
