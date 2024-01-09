#include "chassis_behaviour.h"
#include "detect_task.h"
#include "robot.h"
#include "chassis_task.h"
#include "basic_task.h"
#include "Remote_Control.h"
#include "grab_task.h"

//----------CHASSIS_DEBUG----------
int oretest1, oretest2, oretest3;

bool_t enterChassisDebugCondition() {
  #ifdef DEBUG_CHASSIS
  return TRUE;
  #endif

  #ifndef DEBUG_CHASSIS
  return FALSE;
  #endif
}

bool_t outChassisDebugCondition() {
  #ifdef DEBUG_CHASSIS
  return FALSE;
  #endif

  #ifndef DEBUG_CHASSIS
  return TRUE;
  #endif
}

void chassisDebugHandleFun(float *vx, float *vy, float *vz, float *none) {
  if(vx == NULL || vy == NULL || vz == NULL)
    return;

  if(toe_is_error(DBUSTOE)) {
    *vx = 0;
    *vy = 0;
    *vz = 0;
    return;
  }

}

//----------CHASSIS_ZERO_FORCE----------
bool_t enterZeroForceCondition() {
  if(toe_is_error(DBUSTOE))
		return TRUE;
	
//	if(toe_is_error(DETECT_CHASSIS_CM1_MOTOR) || toe_is_error(DETECT_CHASSIS_CM2_MOTOR) || toe_is_error(DETECT_CHASSIS_CM3_MOTOR) || toe_is_error(DETECT_CHASSIS_CM4_MOTOR)) {
//    //底盘4电机离线
//		buzzerOn(30, 1, 50);
//    return TRUE;
//  } 
	if(robotInf.modeStep == ROBOT_INIT_IMU && robotInf.robotMode == ROBOT_INIT) {
    //初始化前
    return TRUE;
  } else if(robotInf.robotMode == ROBOT_ZERO_FORCE)
    return TRUE;

  return FALSE;
}

bool_t outZeroForceCondition() {
  if(enterZeroForceCondition())
    return FALSE;

  return TRUE;
}

void zeroForceHandleFun(float *raw_cm1, float *raw_cm2, float *raw_cm3, float *raw_cm4) {
  *raw_cm1 = 0;
  *raw_cm2 = 0;
  *raw_cm3 = 0;
  *raw_cm4 = 0;
	
}

//----------CHASSIS_INIT----------
bool_t enterChassisInitCondition() {
  if(robotInf.modeStep == ROBOT_INIT_CHASSIS && robotInf.robotMode == ROBOT_INIT)
    return TRUE;

  return FALSE;
}

bool_t outChassisInitCondition() {
  if(enterChassisInitCondition())
    return FALSE;

  return TRUE;
}

void chassisInitHandleFun(float *raw_cm1, float *raw_cm2, float *raw_cm3, float *raw_cm4) {
  *raw_cm1 = 0;
  *raw_cm2 = 0;
  *raw_cm3 = 0;
  *raw_cm4 = 0;
  chassisTaskStructure.relativeAngleSet = *(chassisTaskStructure.relativeAngle);
  robotInf.modeStep = (robot_init_step)(ROBOT_INIT_CHASSIS + 1);
}

//----------CHASSIS_SLEEP----------
bool_t enterChassisSleepCondition() {
  if(grabTaskStructure.nowBehaviorName == GRAB_TAKE || grabTaskStructure.nowBehaviorName == GRAB_EXCHANGE) {
    //return TRUE;
  }

  return FALSE;
}

bool_t outChassisSleepCondition() {
  if(enterChassisSleepCondition()) {
    return FALSE;
  }

  return TRUE;
}

void chassisSleepHandleFun(float* vx, float* vy, float* vz, float *none) {
  if(vx == NULL || vy == NULL || vz == NULL)
    return;

  *vx = 0;
  *vy = 0;
  *vz = 0;
}

//----------CHASSIS_ALONE----------
bool_t enterChassisAloneCondition() {
  return TRUE;
}
bool_t outChassisAloneCondition() {
  return FALSE;
}

int16_t realAngle;
int16_t setAngle;
void chassisAloneHandleFun(float* vx, float *vy, float* vz, float* none) { //vz自旋转速度为陀螺仪yaw速度
  static float vxExp = 0.0, vyExp = 0.0, vzExp = 0.0;//默认状态速度为零
	

	if(chassisTaskStructure.chassis_STA == SERVO_CAM_GRAB)
	{
		vxExp = CHASSIS_NORMAL_MAX_VX * (RC_CH1_RUD_OFFSET / 660.0);
		vyExp = CHASSIS_NORMAL_MAX_VY * (RC_CH0_RLR_OFFSET / 660.0);
		vzExp = -CHASSIS_NORMAL_MAX_VZ * (RC_CH4_ROT_OFFSET / 660.0);
	}
	else if(chassisTaskStructure.chassis_STA == SERVO_CAM_NORMAL)
	{
		vxExp = -CHASSIS_NORMAL_MAX_VX * (RC_CH1_RUD_OFFSET / 660.0);
		vyExp = -CHASSIS_NORMAL_MAX_VY * (RC_CH0_RLR_OFFSET / 660.0);
		vzExp = -CHASSIS_NORMAL_MAX_VZ * (RC_CH4_ROT_OFFSET / 660.0);
	}
	
  //斜坡输出
  *vx = RAMP_float(vxExp, *vx, 1000 / 1000.0 * GRAB_TASK_MS); //务必使用斜坡
  *vy = RAMP_float(vyExp, *vy, 1000 / 1000.0 * GRAB_TASK_MS); //务必使用斜坡
  *vz = RAMP_float(vzExp, *vz, 1000 / 1000.0 * GRAB_TASK_MS);//-chassisTaskStructure.anglePidParameter.output;
}

