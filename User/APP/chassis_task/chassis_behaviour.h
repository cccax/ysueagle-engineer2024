#ifndef CHASSIS_BEHAVIOUR_H
#define CHASSIS_BEHAVIOUR_H
#include "stm32f4xx.h"
#include "main.h"

//CHASSIS_DEBUG
bool_t enterChassisDebugCondition(void);
bool_t outChassisDebugCondition(void);
void chassisDebugHandleFun(float *vx, float *vy, float *vz, float *none);

//CHASSIS_ZERO_FORCE
void zeroForceHandleFun(float *raw_cm1, float *raw_cm2, float *raw_cm3, float *raw_cm4);
bool_t enterZeroForceCondition(void);
bool_t outZeroForceCondition(void);

//CHASSIS_INIT
void chassisInitHandleFun(float *raw_cm1, float *raw_cm2, float *raw_cm3, float *raw_cm4);
bool_t outChassisInitCondition(void);
bool_t enterChassisInitCondition(void);

//CHASSIS_SLEEP
void chassisSleepHandleFun(float* vx, float* vy, float* vz, float *none);
bool_t outChassisSleepCondition(void);
bool_t enterChassisSleepCondition(void);

//CHASSIS_ALONE
bool_t enterChassisAloneCondition(void);
bool_t outChassisAloneCondition(void);
void chassisAloneHandleFun(float* vx, float *vy, float* vz, float* none);

#endif
