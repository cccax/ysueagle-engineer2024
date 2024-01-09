#ifndef GRAB_BEHAVIOUR_H
#define GRAB_BEHAVIOUR_H
#include "main.h"
#include "robot.h"
#include "buzzer.h"
#include "sensor.h"
#include "user_lib.h"
#include "valve.h"

#define SENSOR_VALID_LEVEL Bit_SET//挡光为高电平
#define JUDGE_SENSOR(PORT,PIN) (GPIO_ReadInputDataBit(PORT,PIN) == SENSOR_VALID_LEVEL ? 1 : 0)
#define JUDGE_SENSORL JUDGE_SENSOR(SENSORL_PORT,SENSORL_PIN)
#define JUDGE_SENSORM JUDGE_SENSOR(SENSORM_PORT,SENSORM_PIN)
#define JUDGE_SENSORR JUDGE_SENSOR(SENSORR_PORT,SENSORR_PIN)

#define JUDGE_ALLEY_OPP GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_13)


//debug
void grabDebugBehaviourHandleFun(float *armbaseyExp, float *armbasepExp, float *armmidpExp, float *singlerotateExp, float *xplatExp, float *yplatExp, float *liftoreExp);
bool_t grabDebugBehaviourEnterCondition(void);
bool_t grabDebugBehaviourOutCondition(void);

//zeroforce
void grabZeroForceBehaviourHandleFun(float *armbaseyExp, float *armbasepExp, float *armmidpExp, float *singlerotateExp, float *xplatExp, float *yplatExp, float *liftoreExp);
bool_t grabZeroForceBehaviourEnterCondition(void);
bool_t grabZeroForceBehaviourOutCondition(void);

//init
void grabInitBehaviourHandleFun(float *armbaseyExp, float *armbasepExp, float *armmidpExp, float *singlerotateExp, float *xplatExp, float *yplatExp, float *liftoreExp);
bool_t grabInitBehaviourEnterCondition(void);
bool_t grabInitBehaviourOutCondition(void);

//prepare
void grabPrepareBehaviourHandleFun(float *armbaseyExp, float *armbasepExp, float *armmidpExp, float *singlerotateExp, float *xplatExp, float *yplatExp, float *liftoreExp);
bool_t grabPrepareBehaviourEnterCondition(void);
bool_t grabPrepareBehaviourOutCondition(void);

//take
void grabTakeBehaviourHandleFun(float *armbaseyExp, float *armbasepExp, float *armmidpExp, float *singlerotateExp, float *xplatExp, float *yplatExp, float *liftoreExp);
bool_t grabTakeBehaviourEnterCondition(void);
bool_t grabTakeBehaviourOutCondition(void);

//exchange
void grabExchangeBehaviourHandleFun(float *armbaseyExp, float *armbasepExp, float *armmidpExp, float *singlerotateExp, float *xplatExp, float *yplatExp, float *liftoreExp);
bool_t grabExchangeBehaviourEnterCondition(void);
bool_t grabExchangeBehaviourOutCondition(void);

//normal
void grabNormalBehaviourHandleFun(float *armbaseyExp, float *armbasepExp, float *armmidpExp, float *singlerotateExp, float *xplatExp, float *yplatExp, float *liftoreExp);
bool_t grabNormalBehaviourEnterCondition(void);
bool_t grabNormalBehaviourOutCondition(void);

#endif
