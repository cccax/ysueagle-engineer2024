#include "chassis_task.h"
#include "INS_task.h"
#include  "IMU_ext.h"
#include "stm32f4xx.h"

//#define CHASSIS_PID_DEBUG  //底盘调参

ChassisCtrl_t chassisTaskStructure;


extern int16_t realAngle, setAngle;


void chassisTask(void *pvParameters) {
  portTickType currentTime;
  chassisTaskInit();
  vTaskDelay(5);
  currentTime = xTaskGetTickCount();

  while(1) {

		//		chassisCanbuscCtrlMotors(2000,0,0,0);
		
   //选择底盘控制模式
    chassisBehaviorSelect();

    //电机模式过渡
    chassisModeChangeHandle();

    //更新实际值，并对实际值进行滤波
    chassisFeedBackUpdate();


    //设置vx,vy,vz三个速度
    if(chassisTaskStructure.behaviorNow->mode == CHASSIS_RAW)
      chassisTaskStructure.behaviorNow->behaviorHandleFun(&(chassisTaskStructure.motor[CHASSIS_CM1].rawCmdCurrent),
          &(chassisTaskStructure.motor[CHASSIS_CM2].rawCmdCurrent),
          &(chassisTaskStructure.motor[CHASSIS_CM3].rawCmdCurrent),
          &(chassisTaskStructure.motor[CHASSIS_CM4].rawCmdCurrent));
    else if(chassisTaskStructure.behaviorNow->mode == CHASSIS_ANGLE || chassisTaskStructure.behaviorNow->mode == CHASSIS_SPEED)
      chassisTaskStructure.behaviorNow->behaviorHandleFun(chassisTaskStructure.relativeSpeedSet + VX, chassisTaskStructure.relativeSpeedSet + VY, chassisTaskStructure.relativeSpeedSet + VZ, NULL);

    //本次遥控器数据作为历史数据
    rcDataCopy(&(chassisTaskStructure.rc.last));

    //速度限幅
    // chassisSpeedLimit();

    //vx vy相对位置转化，并进行pid计算
    chassisPidCalc();

    //队列发送
    chassisCanbuscCtrlMotors(chassisTaskStructure.motor[CHASSIS_CM1].baseInf.given_current,//chassisTaskStructure.motor[CHASSIS_CM1].baseInf.given_current
                             chassisTaskStructure.motor[CHASSIS_CM2].baseInf.given_current,
                             chassisTaskStructure.motor[CHASSIS_CM3].baseInf.given_current,
                             chassisTaskStructure.motor[CHASSIS_CM4].baseInf.given_current);
		
		//chassisCanbuscCtrlMotors(2000,0,0,0);
		
    vTaskDelayUntil(&currentTime, CHASSIS_TASK_MS);
  }
}

void chassisTaskInit(void) {
  //获取遥控器数据
  chassisTaskStructure.rc.now = get_remote_control_point();

  //初始化速度限制变量
  chassisTaskStructure.relativeSpeedMax[VX] = CHASSIS_NORMAL_MAX_VX;
  chassisTaskStructure.relativeSpeedMax[VY] = CHASSIS_NORMAL_MAX_VY;
  chassisTaskStructure.relativeSpeedMax[VZ] = CHASSIS_NORMAL_MAX_VZ;

  motorInit(&(chassisTaskStructure.motor[CHASSIS_CM1].baseInf), CHASSIS_MOTOR_REDUCTION_RATIO);
  motorInit(&(chassisTaskStructure.motor[CHASSIS_CM2].baseInf), CHASSIS_MOTOR_REDUCTION_RATIO);
  motorInit(&(chassisTaskStructure.motor[CHASSIS_CM3].baseInf), CHASSIS_MOTOR_REDUCTION_RATIO);
  motorInit(&(chassisTaskStructure.motor[CHASSIS_CM4].baseInf), CHASSIS_MOTOR_REDUCTION_RATIO);

  //初始化底盘角度
  chassisTaskStructure.relativeAngle = &imu_.yaw;
  chassisTaskStructure.relativeAngleSet = imu_.yaw;
  chassisTaskStructure.maxRelativeAngle = 180;
  chassisTaskStructure.minRelativeAngle = -180;

  //void PISDInit(PidTypeDef *pid,float kp,float ki,float kd, ka,float max_out,float dead_band,float i_band,float max_input, float i_maxout, pid_mode_e model)
  PIDInit(&(chassisTaskStructure.anglePidParameter), 10, 0.04, 100, 0, CHASSIS_NORMAL_MAX_VZ, 0, 10, chassisTaskStructure.maxRelativeAngle, -1, POSITION_180);
  PIDInit(&(chassisTaskStructure.motor[CHASSIS_CM1].pidParameter), 0, 0, 0, 0, 16384, -1, -1, 200, -1, SPEED); //180 1 200
  PIDInit(&(chassisTaskStructure.motor[CHASSIS_CM2].pidParameter), 0, 0, 0, 0, 16384, -1, -1, 200, -1, SPEED);
  PIDInit(&(chassisTaskStructure.motor[CHASSIS_CM3].pidParameter), 0, 0, 0, 0, 16384, -1, -1, 200, -1, SPEED);
  PIDInit(&(chassisTaskStructure.motor[CHASSIS_CM4].pidParameter), 0, 0, 0, 0, 16384, -1, -1, 200, -1, SPEED); //限幅


  //卡尔曼电机速度滤波
  KalmanCreate(&(chassisTaskStructure.motor[CHASSIS_CM1].klmFiller), 1, 40);
  KalmanCreate(&(chassisTaskStructure.motor[CHASSIS_CM2].klmFiller), 1, 40);
  KalmanCreate(&(chassisTaskStructure.motor[CHASSIS_CM3].klmFiller), 1, 40);
  KalmanCreate(&(chassisTaskStructure.motor[CHASSIS_CM4].klmFiller), 1, 40);


  //初始化底盘电机行为
  chassisBehaviourInit(chassisTaskStructure.behaviorList + CHASSIS_DEBUG, CHASSIS_DEBUG, DEBUG_CHSSIS_TYPE, enterChassisDebugCondition, outChassisDebugCondition, NULL, NULL, chassisDebugHandleFun);
  chassisBehaviourInit(chassisTaskStructure.behaviorList + CHASSIS_ZERO_FORCE, CHASSIS_ZERO_FORCE, CHASSIS_RAW, enterZeroForceCondition, outZeroForceCondition, NULL, NULL, zeroForceHandleFun);
  chassisBehaviourInit(chassisTaskStructure.behaviorList + CHASSIS_INIT, CHASSIS_INIT, CHASSIS_RAW, enterChassisInitCondition, outChassisInitCondition, NULL, NULL, chassisInitHandleFun);
  chassisBehaviourInit(chassisTaskStructure.behaviorList + CHASSIS_SLEEP, CHASSIS_SLEEP, CHASSIS_SPEED, enterChassisSleepCondition, outChassisSleepCondition, NULL, NULL, chassisSleepHandleFun);
  chassisBehaviourInit(chassisTaskStructure.behaviorList + CHASSIS_ALONE, CHASSIS_ALONE, CHASSIS_SPEED, enterChassisAloneCondition, outChassisAloneCondition, NULL, NULL, chassisAloneHandleFun);
  chassisTaskStructure.behaviorNow = chassisTaskStructure.behaviorList + CHASSIS_ZERO_FORCE;
}

void chassisBehaviourInit(chassis_behaviour_t *initBehavior, chassis_behaviour_e num, chassis_mode_e mode, 	bool_t (*enterBehaviorCondition)(void),
                          bool_t (*outBehaviorCondition)(void), void (*enterBehaviorFun)(void), void (*outBehaviorFun)(void), void (*behaviorHandleFun)(float* vx, float* vy, float* vz, float* raw)) {
  if(initBehavior == NULL || num >= CHASSIS_BEHAVOUR_LENGTH || enterBehaviorCondition == NULL || outBehaviorCondition == NULL || behaviorHandleFun == NULL)
    return;

  initBehavior->num = num;
  initBehavior->behaviorHandleFun = behaviorHandleFun;
  initBehavior->enterBehaviorCondition = enterBehaviorCondition;
  initBehavior->outBehaviorCondition = outBehaviorCondition;
  initBehavior->enterBehaviorFun = enterBehaviorFun;
  initBehavior->outBehaviorFun = outBehaviorFun;
  initBehavior->mode = mode;

}

void chassisBehaviorSelect() {
  //查看有没有优先级比当行为高的行为的进入条件得到满足
  for(chassis_behaviour_t *iterator = chassisTaskStructure.behaviorList;
      iterator < chassisTaskStructure.behaviorNow; iterator++) {
    //如果有优先级高的进入行为得到满足，则进入那个行为的模式
    if(iterator->enterBehaviorCondition()) {
      chassisBehaviorChange(iterator);
      break;
    }
  }

  if(chassisTaskStructure.behaviorNow->outBehaviorCondition()) {
    //查看当前行为是否已经达到了退出的条件，如果达到，则寻找下一个可以进入的行为
    for(chassis_behaviour_t *iterator = chassisTaskStructure.behaviorNow;    iterator < chassisTaskStructure.behaviorList + CHASSIS_BEHAVOUR_LENGTH;    iterator++) {
      //按照优先级，如果有满足的行为，那么进入这个行为
      if(iterator->enterBehaviorCondition()) {
        chassisBehaviorChange(iterator);
        return;
      }
    }
  } else
    return;

  //如果没有满足所有行为的进入条件，那么底盘进入无力模式
  chassisBehaviorChange(chassisTaskStructure.behaviorList + CHASSIS_ZERO_FORCE);
}

void chassisBehaviorChange(chassis_behaviour_t *next) {
  //执行上一个behavior的退出函数
  if(chassisTaskStructure.behaviorNow->outBehaviorFun != NULL)
    chassisTaskStructure.behaviorNow->outBehaviorFun();

  //将当前行为切换成函数传入的行为
  chassisTaskStructure.behaviorNow = next;
  chassisTaskStructure.behaviourStep = 0;
  chassisTaskStructure.mode = chassisTaskStructure.behaviorNow->mode;

  //如果有进入该行为的处理函数，则执行
  if(chassisTaskStructure.behaviorNow->enterBehaviorFun != NULL)
    chassisTaskStructure.behaviorNow->enterBehaviorFun();
}

void chassisModeChangeHandle() {
  if(chassisTaskStructure.mode != chassisTaskStructure.lastMode) {
    chassisTaskStructure.lastMode = chassisTaskStructure.mode;

    //切换到跟随模式的时候，将当前的角度偏差作为期望，防止切换到这个模式后底盘甩头
    //if(chassisTaskStructure.mode == CHASSIS_ANGLE)
    chassisTaskStructure.relativeAngleSet = *(chassisTaskStructure.relativeAngle);
  }

}

void chassisSpeedLimit() {
  //限制平移方向的输出
  chassisTaskStructure.relativeSpeedSet[VX] = fp32_constrain(chassisTaskStructure.relativeSpeedSet[VX], -chassisTaskStructure.relativeSpeedMax[VX], chassisTaskStructure.relativeSpeedMax[VX]);
  chassisTaskStructure.relativeSpeedSet[VY] = fp32_constrain(chassisTaskStructure.relativeSpeedSet[VY], -chassisTaskStructure.relativeSpeedMax[VY], chassisTaskStructure.relativeSpeedMax[VY]);

  //限制旋转方向的输出
  chassisTaskStructure.relativeSpeedSet[VZ] = fp32_constrain(chassisTaskStructure.relativeSpeedSet[VZ], -chassisTaskStructure.relativeSpeedMax[VZ], chassisTaskStructure.relativeSpeedMax[VZ]);
}
void chassisFeedBackUpdate() {
  //底盘速度滤波
  chassisTaskStructure.motor[CHASSIS_CM1].filterSpeed = KalmanFilter(&(chassisTaskStructure.motor[CHASSIS_CM1].klmFiller), chassisTaskStructure.motor[CHASSIS_CM1].baseInf.real_speed_rpm);
  chassisTaskStructure.motor[CHASSIS_CM2].filterSpeed = KalmanFilter(&(chassisTaskStructure.motor[CHASSIS_CM2].klmFiller), chassisTaskStructure.motor[CHASSIS_CM2].baseInf.real_speed_rpm);
  chassisTaskStructure.motor[CHASSIS_CM3].filterSpeed = KalmanFilter(&(chassisTaskStructure.motor[CHASSIS_CM3].klmFiller), chassisTaskStructure.motor[CHASSIS_CM3].baseInf.real_speed_rpm);
  chassisTaskStructure.motor[CHASSIS_CM4].filterSpeed = KalmanFilter(&(chassisTaskStructure.motor[CHASSIS_CM4].klmFiller), chassisTaskStructure.motor[CHASSIS_CM4].baseInf.real_speed_rpm);
}

int16_t setspeed_chassis, realspeed_chassis;
int16_t set_speed_1,real_speed_1,set_speed_2,real_speed_2,set_speed_3,real_speed_3,set_speed_4,real_speed_4;


void chassisPidCalc() {
  //如果是直接给电机发送电流，不需要进行PID计算，直接发送即可
  if(chassisTaskStructure.mode == CHASSIS_RAW)
    for(u8 i = CHASSIS_CM1; i <= CHASSIS_CM4; i++)
      chassisTaskStructure.motor[i].currentSet = chassisTaskStructure.motor[i].rawCmdCurrent;

  else if(chassisTaskStructure.mode == CHASSIS_ANGLE || chassisTaskStructure.mode == CHASSIS_SPEED) {
    #ifndef CHASSIS_PID_DEBUG//PID调参时定义
    //底盘速度（无需）分解
    chassisTaskStructure.chassisSpeedSet[VX] = chassisTaskStructure.relativeSpeedSet[VX];
    chassisTaskStructure.chassisSpeedSet[VY] = chassisTaskStructure.relativeSpeedSet[VY];
    chassisTaskStructure.chassisSpeedSet[VZ] = chassisTaskStructure.relativeSpeedSet[VZ];

    //将三个方向的速度分解到轮子上
    chassisTaskStructure.motor[CHASSIS_CM1].speedSet = -chassisTaskStructure.chassisSpeedSet[VX] + chassisTaskStructure.chassisSpeedSet[VY] + chassisTaskStructure.chassisSpeedSet[VZ];
    chassisTaskStructure.motor[CHASSIS_CM2].speedSet = -chassisTaskStructure.chassisSpeedSet[VX] - chassisTaskStructure.chassisSpeedSet[VY] + chassisTaskStructure.chassisSpeedSet[VZ];
    chassisTaskStructure.motor[CHASSIS_CM3].speedSet = chassisTaskStructure.chassisSpeedSet[VX] + chassisTaskStructure.chassisSpeedSet[VY] + chassisTaskStructure.chassisSpeedSet[VZ];
    chassisTaskStructure.motor[CHASSIS_CM4].speedSet = chassisTaskStructure.chassisSpeedSet[VX] - chassisTaskStructure.chassisSpeedSet[VY] + chassisTaskStructure.chassisSpeedSet[VZ];
    #else
    chassisTaskStructure.motor[CHASSIS_CM1].speedSet = chassisTaskStructure.relativeSpeedSet[VX];
    chassisTaskStructure.motor[CHASSIS_CM2].speedSet = chassisTaskStructure.relativeSpeedSet[VX];
    chassisTaskStructure.motor[CHASSIS_CM3].speedSet = chassisTaskStructure.relativeSpeedSet[VX];
    chassisTaskStructure.motor[CHASSIS_CM4].speedSet = chassisTaskStructure.relativeSpeedSet[VX];
    //PID调参
    setspeed_chassis = chassisTaskStructure.relativeSpeedSet[VX];
    realspeed_chassis = chassisTaskStructure.motor[CHASSIS_CM1].filterSpeed;
    #endif
    //PID计算
    PID_Calc(&(chassisTaskStructure.motor[CHASSIS_CM1].pidParameter), chassisTaskStructure.motor[CHASSIS_CM1].filterSpeed, chassisTaskStructure.motor[CHASSIS_CM1].speedSet); // RAMP_float(chassisTaskStructure.motor[CHASSIS_CM1].speedSet,chassisTaskStructure.motor[CHASSIS_CM1].filterSpeed,100));
    PID_Calc(&(chassisTaskStructure.motor[CHASSIS_CM2].pidParameter), chassisTaskStructure.motor[CHASSIS_CM2].filterSpeed, chassisTaskStructure.motor[CHASSIS_CM2].speedSet); // RAMP_float(chassisTaskStructure.motor[CHASSIS_CM2].speedSet,chassisTaskStructure.motor[CHASSIS_CM2].filterSpeed,100));
    PID_Calc(&(chassisTaskStructure.motor[CHASSIS_CM3].pidParameter), chassisTaskStructure.motor[CHASSIS_CM3].filterSpeed, chassisTaskStructure.motor[CHASSIS_CM3].speedSet); // RAMP_float(chassisTaskStructure.motor[CHASSIS_CM3].speedSet,chassisTaskStructure.motor[CHASSIS_CM3].filterSpeed,100));
    PID_Calc(&(chassisTaskStructure.motor[CHASSIS_CM4].pidParameter), chassisTaskStructure.motor[CHASSIS_CM4].filterSpeed, chassisTaskStructure.motor[CHASSIS_CM4].speedSet); // RAMP_float(chassisTaskStructure.motor[CHASSIS_CM4].speedSet,chassisTaskStructure.motor[CHASSIS_CM4].filterSpeed,100));

    for(u8 i = CHASSIS_CM1; i <= CHASSIS_CM4; i++)
      chassisTaskStructure.motor[i].currentSet = chassisTaskStructure.motor[i].pidParameter.output;
			
  } else
    for(u8 i = CHASSIS_CM1; i <= CHASSIS_CM4; i++)
      chassisTaskStructure.motor[i].currentSet = 0;

  for(u8 i = CHASSIS_CM1; i <= CHASSIS_CM4; i++)
    chassisTaskStructure.motor[i].baseInf.given_current = (s16)chassisTaskStructure.motor[i].currentSet;

//调参
	real_speed_1=chassisTaskStructure.motor[CHASSIS_CM1].filterSpeed;
	set_speed_1=chassisTaskStructure.motor[CHASSIS_CM1].speedSet;
	
	real_speed_2=chassisTaskStructure.motor[CHASSIS_CM2].filterSpeed;
	set_speed_2=chassisTaskStructure.motor[CHASSIS_CM2].speedSet;
	
	real_speed_3=chassisTaskStructure.motor[CHASSIS_CM3].filterSpeed;
	set_speed_3=chassisTaskStructure.motor[CHASSIS_CM3].speedSet;
	
	real_speed_4=chassisTaskStructure.motor[CHASSIS_CM4].filterSpeed;
	set_speed_4=chassisTaskStructure.motor[CHASSIS_CM4].speedSet;
}

void chassisCanbuscCtrlMotors(s16 cm1Current, s16 cm2Current, s16 cm3Current, s16 cm4Current ) //传递电机电流值。控制电流值范围-16384~0~16384，对应电调输出的转矩电流范围-20~0~20A。
	{
  //底盘电机0x205-0x208
  s16 chassisCurrent[4];
  chassisCurrent[0] = cm1Current;
  chassisCurrent[1] = cm2Current;
  chassisCurrent[2] = cm3Current;
  chassisCurrent[3] = cm4Current;

  if(toe_is_error(DBUSTOE)) {
    chassisCurrent[0] = 0;
    chassisCurrent[1] = 0;
    chassisCurrent[2] = 0;
    chassisCurrent[3] = 0;
  }

  djiMotorCurrentSendQueue(CAN1, CHASSIS_CANBUS_SEND_HEADER58, chassisCurrent, 4);

}

