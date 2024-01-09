#include "grab_behaviour.h"
#include "grab_task.h"
#include "detect_task.h"
#include "user_lib.h"
#include "usart6.h"
#include "chassis_task.h"
#include "basic_task.h"
#include "mf9025.h"
#include "robot.h"



#define JUDGE_STUCK_SIGN 0x01
#define JUDGE_ECD_SIGN 0x00

//电机堵转检测
#define JUDGE_STUCK_ARMBASEY JUDGE_ECD_MOTOR_READY(JUDGE_ARMBASE_Y_MOTOR,grabTaskStructure.armbaseyMotor.last_totalEcd , JUDGE_STUCK_SIGN)
#define JUDGE_STUCK_ARMBASEP JUDGE_ECD_MOTOR_READY(JUDGE_ARMBASE_P_MOTOR1,grabTaskStructure.armbasepMotor1.last_totalEcd , JUDGE_STUCK_SIGN)
#define JUDGE_STUCK_ARMMIDP JUDGE_ECD_MOTOR_READY(JUDGE_ARMMID_P_MOTOR,grabTaskStructure.armmidpMotor.last_totalEcd , JUDGE_STUCK_SIGN)
#define JUDGE_STUCK_SINGLE_ROTATE JUDGE_ECD_MOTOR_READY(JUDGE_SINGLE_ROTATE_MOTOR,grabTaskStructure.singlerotateMotor.last_totalEcd , JUDGE_STUCK_SIGN)
#define JUDGE_STUCK_XPLAT JUDGE_ECD_MOTOR_READY(JUDGE_PLAT_X_MOTOR,grabTaskStructure.xplatMotor1.last_totalEcd , JUDGE_STUCK_SIGN)
#define JUDGE_STUCK_YPLAT1 JUDGE_ECD_MOTOR_READY(JUDGE_PLAT_Y_MOTOR1,grabTaskStructure.yplatMotor1.last_totalEcd , JUDGE_STUCK_SIGN)
#define JUDGE_STUCK_YPLAT2 JUDGE_ECD_MOTOR_READY(JUDGE_PLAT_Y_MOTOR2,grabTaskStructure.yplatMotor2.last_totalEcd , JUDGE_STUCK_SIGN)
#define JUDGE_STUCK_PARALLElLMOTOR1 JUDGE_ECD_MOTOR_READY(JUDGE_PARALLELMOTOR1,grabTaskStructure.parallelMotor1.last_totalEcd , JUDGE_STUCK_SIGN)//cjh
#define JUDGE_STUCK_PARALLElLMOTOR2 JUDGE_ECD_MOTOR_READY(JUDGE_PARALLELMOTOR2,grabTaskStructure.parallelMotor2.last_totalEcd , JUDGE_STUCK_SIGN)//cjh

#define JUDGE_STUCK_LIFT1 JUDGE_ECD_MOTOR_READY(JUDGE_LIFT_MOTOR1,grabTaskStructure.liftoreMotor1.last_totalEcd , JUDGE_STUCK_SIGN)
#define JUDGE_STUCK_LIFT2 JUDGE_ECD_MOTOR_READY(JUDGE_LIFT_MOTOR2,grabTaskStructure.liftoreMotor2.last_totalEcd , JUDGE_STUCK_SIGN)
//电机到特定位置检测
//#define JUDGE_ECD_ARMBASEY(ecdSet) JUDGE_ECD_MOTOR_READY(JUDGE_ARMBASE_Y_MOTOR , ecdSet , JUDGE_ECD_SIGN)
#define JUDGE_ECD_ARMBASEP(ecdSet) JUDGE_ECD_MOTOR_READY(JUDGE_ARMBASE_P_MOTOR1 , ecdSet , JUDGE_ECD_SIGN)
#define JUDGE_ECD_ARMMIDY(ecdSet) JUDGE_ECD_MOTOR_READY(JUDGE_ARMMID_Y_MOTOR , ecdSet , JUDGE_ECD_SIGN) //cjh p改y
#define JUDGE_ECD_SINGLE_ROTATE(ecdSet) JUDGE_ECD_MOTOR_READY(JUDGE_SINGLE_ROTATE_MOTOR , ecdSet , JUDGE_ECD_SIGN)
#define JUDGE_ECD_XPLAT(ecdSet) JUDGE_ECD_MOTOR_READY(JUDGE_PLAT_X_MOTOR , ecdSet , JUDGE_ECD_SIGN)
#define JUDGE_ECD_YPLAT1(ecdSet) JUDGE_ECD_MOTOR_READY(JUDGE_PLAT_Y_MOTOR1 , ecdSet , JUDGE_ECD_SIGN)
#define JUDGE_ECD_YPLAT2(ecdSet) JUDGE_ECD_MOTOR_READY(JUDGE_PLAT_Y_MOTOR2 , ecdSet , JUDGE_ECD_SIGN)//cjh
#define JUDGE_ECD_PARALLElLMOTOR2(ecdSet) JUDGE_ECD_MOTOR_READY(JUDGE_PARALLELMOTOR2 , ecdSet , JUDGE_ECD_SIGN)//cjh
#define JUDGE_ECD_PARALLElLMOTOR1(ecdSet) JUDGE_ECD_MOTOR_READY(JUDGE_PARALLELMOTOR1 , ecdSet , JUDGE_ECD_SIGN)//cjh

#define JUDGE_ECD_LIFT1(ecdSet) JUDGE_ECD_MOTOR_READY(JUDGE_LIFT_MOTOR1 , ecdSet , JUDGE_ECD_SIGN)
#define JUDGE_ECD_LIFT2(ecdSet) JUDGE_ECD_MOTOR_READY(JUDGE_LIFT_MOTOR2 , ecdSet , JUDGE_ECD_SIGN)


//舵机原始数据
extern float rawData_Servo[RAWDATA_LENGTH] , rawData_armmidp[RAWDATA_LENGTH] , rawData_armbasep[RAWDATA_LENGTH];

//增加值和当前值限幅
void singleNum_Limit(int32_t *limitMin , int32_t *nowNum , int32_t *addNum , int32_t *limitMax)
{
	//增加值限幅
	if(*nowNum + *addNum > *limitMax) *addNum = *limitMax - *nowNum;
	else if(*nowNum + *addNum < *limitMin) *addNum = *limitMin - *nowNum;
}

////仿真舵机数据处理
//int16_t dataTrans_Servo(float rawAngle)
//{
//	int16_t angle_Offset = grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_NORMAL] - 10;//舵机竖直时为初始值
//	return angle_Offset - rawAngle;
//}

////仿真armmidp数据处理
//int16_t dataTrans_Armmidp(float rawAngle)
//{
//	int16_t angle_Offset = grabTaskStructure.armmidpMotor.fixedEcd[ARM_MID_P_NORMAL];
//	return angle_Offset + (rawAngle * 2.5) / 360 * 8191;
//}

////仿真armbasep数据处理
//int16_t dataTrans_Armbasep(float rawAngle)
//{
//	int16_t angle_Offset = grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_STORE_START] + 291;
//	return angle_Offset + rawAngle/360 * 65535;
//}

//判断电机是否在某值附近
uint8_t JUDGE_ECD_MOTOR_READY(uint32_t ID, int32_t SET , uint8_t SIGN) { 
  int32_t REAL = 0 , RANGE = 0;
  switch(ID) {
    case JUDGE_ARMMID_Y_MOTOR:
      REAL = grabTaskStructure.armmidyMotor.totalEcd;//中间yaw轴电机
      if(SIGN == JUDGE_ECD_SIGN)  RANGE = 10;
		  else if(SIGN == JUDGE_STUCK_SIGN) RANGE = 50;
			break;
		
    case JUDGE_ARMBASE_P_MOTOR1:
      REAL = grabTaskStructure.armbasepMotor1.totalEcd;//底座pitch轴电机(MF9025)1  //m3508
      if(SIGN == JUDGE_ECD_SIGN)  RANGE = 300;
		  else if(SIGN == JUDGE_STUCK_SIGN) RANGE = 50;
			break;
		
    case JUDGE_ARMBASE_P_MOTOR2:
      REAL = grabTaskStructure.armbasepMotor2.totalEcd;//底座pitch轴电机(MF9025)2  //m3508
      if(SIGN == JUDGE_ECD_SIGN)  RANGE = 300;
		  else if(SIGN == JUDGE_STUCK_SIGN) RANGE = 50;
			break;
		
//    case JUDGE_ARMMID_P_MOTOR:
//      REAL = grabTaskStructure.armmidpMotor.totalEcd;//底座yaw轴电机
//      if(SIGN == JUDGE_ECD_SIGN)  RANGE = 100;
//		  else if(SIGN == JUDGE_STUCK_SIGN) RANGE = 50;
//      break;
		
//    case JUDGE_SINGLE_ROTATE_MOTOR:
//      REAL = grabTaskStructure.singlerotateMotor.totalEcd;//夹爪
//      if(SIGN == JUDGE_ECD_SIGN)  RANGE = 50;
//		  else if(SIGN == JUDGE_STUCK_SIGN) RANGE = 50;
//      break;

    case JUDGE_PLAT_X_MOTOR:
      REAL = grabTaskStructure.xplatMotor1.totalEcd;//
      if(SIGN == JUDGE_ECD_SIGN)  RANGE = 100;
		  else if(SIGN == JUDGE_STUCK_SIGN) RANGE = 50;
      break;
		
    case JUDGE_PLAT_Y_MOTOR1:
      REAL = grabTaskStructure.yplatMotor1.totalEcd;//
      if(SIGN == JUDGE_ECD_SIGN)  RANGE = 150;
		  else if(SIGN == JUDGE_STUCK_SIGN) RANGE = 3;
      break;
		case JUDGE_PLAT_Y_MOTOR2:
      REAL = grabTaskStructure.yplatMotor2.totalEcd;//cjh
      if(SIGN == JUDGE_ECD_SIGN)  RANGE = 150;
		  else if(SIGN == JUDGE_STUCK_SIGN) RANGE = 3;
      break;
    case JUDGE_LIFT_MOTOR1:
      REAL = grabTaskStructure.liftoreMotor1.totalEcd;
      if(SIGN == JUDGE_ECD_SIGN)  RANGE = 200;
		  else if(SIGN == JUDGE_STUCK_SIGN) RANGE = 50;
      break;

    case JUDGE_LIFT_MOTOR2:
      REAL = grabTaskStructure.liftoreMotor2.totalEcd;
      if(SIGN == JUDGE_ECD_SIGN)  RANGE = 200;
		  else if(SIGN == JUDGE_STUCK_SIGN) RANGE = 50;
      break;
		case JUDGE_PARALLELMOTOR1:
      REAL = grabTaskStructure.liftoreMotor2.totalEcd;
      if(SIGN == JUDGE_ECD_SIGN)  RANGE = 200;
		  else if(SIGN == JUDGE_STUCK_SIGN) RANGE = 50;
      break;
		case JUDGE_PARALLELMOTOR2:
      REAL = grabTaskStructure.liftoreMotor2.totalEcd;
      if(SIGN == JUDGE_ECD_SIGN)  RANGE = 200;
		  else if(SIGN == JUDGE_STUCK_SIGN) RANGE = 50;
      break;
		
  }
  return (s16_abs(REAL - SET) <= RANGE);
}

//#define GRAB_DEBUG_SPEED
//#define GRAB_DEBUG_POSITION
#define GRAB_DEBUG_POSITION_MAX 3500
#define GRAB_DEBUG_POSITION_MIN 0

float midTmp=0 , *debug_tmp;
//----------GRAB_DEBUG----------
void grabDebugBehaviourHandleFun(float *armbaseyExp, float *armbasepExp, float *armmidpExp, float *singlerotateExp, float *xplatExp, float *yplatExp, float *liftoreExp) 
	{
	 if(JUDGE_ALLEY_OPP == 0)
	 {
//		 buzzerOn(50, 1, 40);
	 }
	 
	 debug_tmp=armbaseyExp;
	 
	 #ifdef GRAB_DEBUG_POSITION
	 midTmp+=5.0*(RC_CH3_LUD_OFFSET / 660.0);
	 if(midTmp<GRAB_DEBUG_POSITION_MIN) midTmp=GRAB_DEBUG_POSITION_MIN;
	 else if(midTmp >GRAB_DEBUG_POSITION_MAX) midTmp=GRAB_DEBUG_POSITION_MAX;
	 *debug_tmp=midTmp;
	 #endif
	 
	 #ifdef GRAB_DEBUG_SPEED
	 *debug_tmp=100*(RC_CH3_LUD_OFFSET / 660.0);
	 #endif
	 //X   Y   Z   armbasey   armbasep   armmidp  singlerotate  oreServo
		int32_t behaviour_ecdSet[7];//, behaviour_angleSet;
		static int32_t behaviour_Tmp[8];
		
		
		if(grabTaskStructure.currentExchlevel == EXCH_LEVEL_0)
		{
			behaviour_ecdSet[4] = grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_EXCH_LEVEL_01];
			behaviour_ecdSet[5] = grabTaskStructure.armmidyMotor.fixedEcd[ARM_MID_Y_EXCH_LEVEL_01];
		}
		else if(grabTaskStructure.currentExchlevel == EXCH_LEVEL_1)
		{
			behaviour_ecdSet[4] = grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_EXCH_LEVEL_12];
			behaviour_ecdSet[5] = grabTaskStructure.armmidyMotor.fixedEcd[ARM_MID_Y_EXCH_LEVEL_12];
		}
		else if(grabTaskStructure.currentExchlevel == EXCH_LEVEL_2)
		{
			behaviour_ecdSet[4] = grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_EXCH_LEVEL_24];
			behaviour_ecdSet[5] = grabTaskStructure.armmidyMotor.fixedEcd[ARM_MID_Y_EXCH_LEVEL_24];
		}
		
		behaviour_ecdSet[0] = grabTaskStructure.xplatMotor1.fixedEcd[PLAT_X_NORMAL];
		behaviour_ecdSet[1] = grabTaskStructure.yplatMotor1.fixedEcd[PLAT_Y_NORMAL];
		behaviour_ecdSet[2] = grabTaskStructure.liftoreMotor1.fixedEcd[LIFT_LITTLE_UP];
		behaviour_ecdSet[3] = grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_NORMAL];
		behaviour_ecdSet[6] = grabTaskStructure.singlerotateMotor.fixedEcd[ROTATE_SINGLE_NORMAL];
		
		//if(RC_CH1_RUD_OFFSET / 660.0);
		behaviour_Tmp[0]=(RC_CH1_RUD_OFFSET / 660.0)*8190;
		behaviour_Tmp[1]=(RC_CH0_RLR_OFFSET / 660.0)*8190;

//		//按键控制八个轴
//		//                                                             Z
//		if(IF_KEY_PRESSED_SHIFT && IF_KEY_PRESSED_F) //平台左
//			behaviour_Tmp[0]-=15;
//		else if(IF_KEY_PRESSED_F)  //平台右
//			behaviour_Tmp[0]+=10;
//		//                                                             X
//		else if(IF_KEY_PRESSED_SHIFT && IF_KEY_PRESSED_G) //平台后
//			behaviour_Tmp[1]-=15;
//		else if(IF_KEY_PRESSED_G)  //平台前
//			behaviour_Tmp[1]+=15;
//		//                                                             C
//		else if(IF_KEY_PRESSED_SHIFT && IF_KEY_PRESSED_R) //平台降
//			behaviour_Tmp[2]-=10;
//		else if(IF_KEY_PRESSED_R)  //平台升
//			behaviour_Tmp[2]+=10;
//		//
//		else if(IF_KEY_PRESSED_SHIFT && IF_KEY_PRESSED_Z) //armbasey
//			behaviour_Tmp[3]-=3;
//		else if(IF_KEY_PRESSED_Z)  
//			behaviour_Tmp[3]+=3;
//		//
//		else if(IF_KEY_PRESSED_SHIFT && IF_KEY_PRESSED_X) //armbasep
//			behaviour_Tmp[4]-=20;
//		else if(IF_KEY_PRESSED_X)  
//			behaviour_Tmp[4]+=20;
//		//
//		else if(IF_KEY_PRESSED_SHIFT && IF_KEY_PRESSED_C) //armmidp
//			behaviour_Tmp[5]+=3;
//		else if(IF_KEY_PRESSED_C)  
//			behaviour_Tmp[5]-=3;
//		//
//		else if(IF_KEY_PRESSED_SHIFT && IF_KEY_PRESSED_V) //singlerotate
//			behaviour_Tmp[6]-=2;
//		else if(IF_KEY_PRESSED_V)  
//			behaviour_Tmp[6]+=2;
//		//
//		else if(IF_KEY_PRESSED_SHIFT && IF_KEY_PRESSED_B) //OreAngle
//			behaviour_Tmp[7]-=2;
//		else if(IF_KEY_PRESSED_B)  
//			behaviour_Tmp[7]+=2;		


//增加值限幅
		//xplat
		singleNum_Limit(&grabTaskStructure.xplatMotor1.fixedEcd[PLAT_X_MIN] , &behaviour_ecdSet[0] , &behaviour_Tmp[0] , &grabTaskStructure.xplatMotor1.fixedEcd[PLAT_X_RESET]);
		//yplat
		singleNum_Limit(&grabTaskStructure.yplatMotor1.fixedEcd[PLAT_Y_MIN] , &behaviour_ecdSet[1] , &behaviour_Tmp[1] , &grabTaskStructure.yplatMotor1.fixedEcd[PLAT_Y_FRONT_MAX]);
		//liftore
		singleNum_Limit(&grabTaskStructure.liftoreMotor1.fixedEcd[LIFT_RESET] , &behaviour_ecdSet[2] , &behaviour_Tmp[2] , &grabTaskStructure.liftoreMotor1.fixedEcd[LIFT_MAX]);
		//armbasey
		//singleNum_Limit(&grabTaskStructure.armbaseyMotor.fixedEcd[ARM_BASE_Y_RESET] , &behaviour_ecdSet[3] , &behaviour_Tmp[3] , &grabTaskStructure.armbaseyMotor.fixedEcd[ARM_BASE_Y_MAX]);
		//armbasep
		singleNum_Limit(&grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_MIN] , &behaviour_ecdSet[4] , &behaviour_Tmp[4] , &grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_RESET]);
		//armmidy
		singleNum_Limit(&grabTaskStructure.armmidyMotor.fixedEcd[ARM_MID_Y_RESET] , &behaviour_ecdSet[5] , &behaviour_Tmp[5] , &grabTaskStructure.armmidyMotor.fixedEcd[ARM_MID_Y_MAX]);
		//singlerotate
		singleNum_Limit(&grabTaskStructure.singlerotateMotor.fixedEcd[ROTATE_SINGLE_RESET] , &behaviour_ecdSet[6] , &behaviour_Tmp[6] , &grabTaskStructure.singlerotateMotor.fixedEcd[ROTATE_SINGLE_MAX]);
		
		//先限幅，再传递增加量
		for(int i=0 ; i<7 ; i++) behaviour_ecdSet[i]+=behaviour_Tmp[i];
		
		
		*xplatExp = RAMP_float(behaviour_ecdSet[0] , *xplatExp , 10000/1000.0*GRAB_TASK_MS);
		*yplatExp = RAMP_float(behaviour_ecdSet[1] , *yplatExp , 10000/1000.0*GRAB_TASK_MS);
		*liftoreExp = RAMP_float(behaviour_ecdSet[2] , *liftoreExp , 5000/1000.0*GRAB_TASK_MS);
		*armbaseyExp = RAMP_float(behaviour_ecdSet[3] , *armbaseyExp , 3000/1000.0*GRAB_TASK_MS);
		*armbasepExp = RAMP_float(behaviour_ecdSet[4] , *armbasepExp , 6000/1000.0*GRAB_TASK_MS);
		*armmidpExp = RAMP_float(behaviour_ecdSet[5] , *armmidpExp , 1000/1000.0*GRAB_TASK_MS);
		*singlerotateExp = RAMP_float(behaviour_ecdSet[6] , *singlerotateExp , 1500/1000.0*GRAB_TASK_MS);
	
		if(GRAB_FIRST_PRESS_W)//W切换默认高度
		{
			for(int i=4 ; i<6 ; i++) behaviour_Tmp[i]=0;
			
			if(grabTaskStructure.currentExchlevel == EXCH_LEVEL_0) grabTaskStructure.currentExchlevel = EXCH_LEVEL_1;
			else if(grabTaskStructure.currentExchlevel == EXCH_LEVEL_1) grabTaskStructure.currentExchlevel = EXCH_LEVEL_2;
			else if(grabTaskStructure.currentExchlevel == EXCH_LEVEL_2) grabTaskStructure.currentExchlevel = EXCH_LEVEL_0;
		}
//		//按E结束兑换,气泵关
//		else if(GRAB_FIRST_PRESS_E && JUDGE_ECD_ARMMIDY(behaviour_ecdSet[5]))
//		{
//			for(int i=0 ; i<8 ; i++) behaviour_Tmp[i]=0;
//			PUMP_ORE_OFF;//气泵关
//			grabTaskStructure.behaviourStep++;
//		}
}



bool_t grabDebugBehaviourEnterCondition() {
  #ifdef DEBUG_GRAB
  return TRUE;
  #endif

  #ifndef DEBUG_GRAB
//  if(IF_RC_SW1_UP && GRAB_FIRST_S2_UP) { //S1上 S2上 小资源岛
//    grabTaskStructure.currentTarget = TARGET_SMALL_ISLAND;
//    grabBehaviorChange(grabTaskStructure.behaviorList + GRAB_PREPARE);
//		basicTaskStructure.servoCam_Sign = SERVO_CAM_GRAB;

//  } 
//	else if(IF_RC_SW1_UP && GRAB_FIRST_S2_MID) { //S1上 S2中 大资源岛
//    grabTaskStructure.currentTarget = TARGET_BIG_ISLAND;
//    grabBehaviorChange(grabTaskStructure.behaviorList + GRAB_PREPARE);
//		basicTaskStructure.servoCam_Sign = SERVO_CAM_GRAB;
//  } 
//	else if(IF_RC_SW1_UP && GRAB_FIRST_S2_DOWN) { //S1上 S2下 空接
//    grabTaskStructure.currentTarget = TARGET_ALLEY_OPP;
//    grabBehaviorChange(grabTaskStructure.behaviorList + GRAB_PREPARE);
//		basicTaskStructure.servoCam_Sign = SERVO_CAM_GRAB;
//  } 
//	else if(IF_RC_SW1_DOWN && GRAB_FIRST_S2_UP) { //S1下 S2上 兑换
//    grabTaskStructure.currentTarget = TARGET_EXCH_STA;
//		if(grabTaskStructure.take2exch_Sign == 1)//直接到兑换
//			grabBehaviorChange(grabTaskStructure.behaviorList + GRAB_EXCHANGE);
//		else if(grabTaskStructure.take2exch_Sign == 0)//不是直接到兑换
//			grabBehaviorChange(grabTaskStructure.behaviorList + GRAB_TAKE);
//		basicTaskStructure.servoCam_Sign = SERVO_CAM_GRAB;
//	}
//	else if(IF_RC_SW1_DOWN && GRAB_FIRST_S2_MID) { //S1下 S2中 强制回到普通模式
//    grabBehaviorChange(grabTaskStructure.behaviorList + GRAB_NORMAL);
//	}
//	else if(IF_RC_SW1_DOWN && GRAB_FIRST_S2_DOWN) { //S1下 S2下 重新初始化
//		grabTaskStructure.inited = 0;//初始化状态清零
//    grabBehaviorChange(grabTaskStructure.behaviorList + GRAB_NORMAL);//先回到普通模式
//	}
  return FALSE;
  #endif
}

bool_t grabDebugBehaviourOutCondition() {
  #ifdef DEBUG_GRAB
  return FALSE;
  #endif

  #ifndef DEBUG_GRAB
  return TRUE;
  #endif
}

//----------GRAB_ZERO_FORCE----------
void grabZeroForceBehaviourHandleFun(float *armbaseyExp, float *armbasepExp, float *armmidpExp, float *singlerotateExp, float *xplatExp, float *yplatExp, float *liftoreExp) {
  *armbaseyExp = 0;
	*armbasepExp = 0;
  *armmidpExp = 0;
  *singlerotateExp = 0;
  *xplatExp = 0;
	*yplatExp = 0;
  *liftoreExp = 0;
}

bool_t grabZeroForceBehaviourEnterCondition() {

//	for(int i=1 ; i<errorListLength-4 ; i++) //除底盘4电机外所有电机离线检测
//	{
//		if(toe_is_error(i)) // 所有电机离线检测
//		{
//			buzzerOn(100,1, 60);
//			return TRUE;
//		}
//	}
	
  if(toe_is_error(DBUSTOE)) // 遥控器离线
    return TRUE;

  return FALSE;
}

bool_t grabZeroForceBehaviourOutCondition() {
  if(!grabZeroForceBehaviourEnterCondition())
    return TRUE;

  return FALSE;
}



//----------GRAB_INIT----------
void grabInitBehaviourHandleFun(float *armbaseyExp, float *armbasepExp, float *armmidpExp, float *singlerotateExp, float *xplatExp, float *yplatExp, float *liftoreExp) {
  
	grabTaskStructure.behaviourTime += GRAB_TASK_MS;//计时
	
	//防止重复初始化
  if(grabTaskStructure.inited) { 
    grabBehaviorChange(grabTaskStructure.behaviorList + GRAB_ZERO_FORCE);
    return;
  }
	
	/*初始化准备*/
	if(grabTaskStructure.behaviourStep == 0 && GRAB_FIRST_PRESS_B)
	{
		PUMP_ORE_OFF;  //关闭抓矿气泵
		
		//修改电机控制方式为：电流环
    //grabTaskStructure.singlerotateMotor.mode = GRAB_MOTOR_SPEED; //爪子旋转电机
		grabTaskStructure.armmidyMotor.mode = GRAB_MOTOR_RAW;
		grabTaskStructure.yplatMotor1.mode = GRAB_MOTOR_POSITION;
		//grabTaskStructure.yplatMotor1.mode = GRAB_MOTOR_POSITION;
		grabTaskStructure.xplatMotor1.mode = GRAB_MOTOR_POSITION;
		grabTaskStructure.armbasepMotor1.mode = GRAB_MOTOR_POSITION;
		grabTaskStructure.armbasepMotor1.totalEcdSet = 0;
		
		grabTaskStructure.behaviourStep=4; //跳到Step4
	}
	
	else if(grabTaskStructure.behaviourStep == 4)
	{
		int32_t behaviour_ecdSet=grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_RESET];
		
		*armbasepExp = RAMP_float(behaviour_ecdSet , *armbasepExp , 12000/1000.0*GRAB_TASK_MS);
		
		if(JUDGE_ECD_ARMBASEP(behaviour_ecdSet))
		{
			grabTaskStructure.behaviourStep ++;
		}
	}		
	/*平台Y方向前送一小段，防止X方向初始化时机械干涉*/
	else if(grabTaskStructure.behaviourStep == 5) 
	{
		int32_t behaviour_ecdSet=grabTaskStructure.yplatMotor1.fixedEcd[PLAT_Y_FRONT_LITTLE_INIT];
		
		grabTaskStructure.yplatMotor1.totalEcdSet=behaviour_ecdSet;
		
		if(JUDGE_ECD_YPLAT1(behaviour_ecdSet)) 
			grabTaskStructure.behaviourStep++;
	}
	/*平台X方向右送一小段，防止初始化前X方向上滑块碰到限位开关*/
	else if(grabTaskStructure.behaviourStep == 6)
	{
		int32_t behaviour_ecdSet=grabTaskStructure.xplatMotor1.fixedEcd[PLAT_X_RIGHT_LITTLE];
		
		*xplatExp = RAMP_float(behaviour_ecdSet , *xplatExp , 10000/1000.0*GRAB_TASK_MS);
		
		if(JUDGE_ECD_XPLAT(behaviour_ecdSet))
		{
		//	grabTaskStructure.armbaseyMotor.mode = GRAB_MOTOR_SPEED;
			grabTaskStructure.xplatMotor1.mode = GRAB_MOTOR_SPEED;
			
			grabTaskStructure.behaviourTime = 0;
			grabTaskStructure.behaviourStep++;
		}
	}
	/* 电流环初始化的电机开始控制*/
	else if(grabTaskStructure.behaviourStep == 7) 
	{
		//grabTaskStructure.armbaseyMotor.speedSet = -50; //逆时针 ok
		grabTaskStructure.singlerotateMotor.speedSet = -70;//左旋
		grabTaskStructure.xplatMotor1.speedSet = 70;//平台左方向移动、
	
		if(grabTaskStructure.limsw[LIMSW_ALL] == 0x0F)//限位开关全部到位
		{
			if(grabTaskStructure.behaviourTime < 200) return;
			//grabTaskStructure.armbaseyMotor.baseInf.relativeEcd=0;
			grabTaskStructure.armmidyMotor.baseInf.relativeEcd=0;
			grabTaskStructure.singlerotateMotor.baseInf.relativeEcd=0;
			grabTaskStructure.xplatMotor1.baseInf.relativeEcd=0;
			
			//grabTaskStructure.armbaseyMotor.mode = GRAB_MOTOR_POSITION;
			grabTaskStructure.armmidyMotor.mode = GRAB_MOTOR_POSITION;
			grabTaskStructure.singlerotateMotor.mode = GRAB_MOTOR_POSITION;
			grabTaskStructure.xplatMotor1.mode = GRAB_MOTOR_POSITION;
			
			grabTaskStructure.behaviourStep ++;
		}
		else grabTaskStructure.behaviourTime = 0;
	}
	/*机械臂回到普通状态，平台X方向往右伸出一段距离，使滑块置中*/
	else if(grabTaskStructure.behaviourStep == 8)
	{
		//        armbasey              armmidp             xplat             singlerotate
		int32_t behaviour_ecdSet1 , behaviour_ecdSet2 , behaviour_ecdSet3 , behaviour_ecdSet4;
		
		//behaviour_ecdSet1 = grabTaskStructure.armbaseyMotor.fixedEcd[ARM_BASE_Y_NORMAL];
		behaviour_ecdSet2 = grabTaskStructure.armmidyMotor.fixedEcd[ARM_MID_Y_NORMAL];
		behaviour_ecdSet3 = grabTaskStructure.xplatMotor1.fixedEcd[PLAT_X_INIT_RIGHT];
		behaviour_ecdSet4 = grabTaskStructure.singlerotateMotor.fixedEcd[ROTATE_SINGLE_NORMAL];
		
		*armbaseyExp = RAMP_float(behaviour_ecdSet1 , *armbaseyExp , 5000/1000.0 * GRAB_TASK_MS);
		*armmidpExp = RAMP_float(behaviour_ecdSet2 , *armmidpExp , 3000/1000.0 * GRAB_TASK_MS);
		*xplatExp = RAMP_float(behaviour_ecdSet3 , *xplatExp , 15000/1000.0 * GRAB_TASK_MS);
		*singlerotateExp = RAMP_float(behaviour_ecdSet4 , *singlerotateExp,5000/1000.0 * GRAB_TASK_MS);
		
		//判定是否到位
		if(JUDGE_ECD_ARMBASEP(behaviour_ecdSet1) && JUDGE_ECD_ARMMIDY(behaviour_ecdSet2) && 
			 JUDGE_ECD_XPLAT(behaviour_ecdSet3) && JUDGE_ECD_PARALLElLMOTOR1(behaviour_ecdSet4))
			{
				grabTaskStructure.yplatMotor1.mode = GRAB_MOTOR_SPEED;  //电流环
				grabTaskStructure.behaviourStep++;
			}
	}
	/*平台X方向回到普通位置*/
	else if(grabTaskStructure.behaviourStep == 9)
	{
		int32_t behaviour_ecdSet;
		behaviour_ecdSet=grabTaskStructure.xplatMotor1.fixedEcd[PLAT_X_NORMAL];
		
		*xplatExp=RAMP_float(behaviour_ecdSet , *xplatExp , 13000/1000.0*GRAB_TASK_MS);
		if(JUDGE_ECD_XPLAT(behaviour_ecdSet))
			grabTaskStructure.behaviourStep++;
	}
	else if(grabTaskStructure.behaviourStep == 10)//平台Y方向电机电流环初始化
	{
		grabTaskStructure.yplatMotor1.speedSet = -80; //Y方向往回收
		grabTaskStructure.behaviourTime = 0;//堵转电流清零
		grabTaskStructure.behaviourStep++;
	}
	else if(grabTaskStructure.behaviourStep == 11)//检测平台Y方向电机堵转
	{
		if(!JUDGE_STUCK_YPLAT1) grabTaskStructure.behaviourTime = 0; //未到位，时间清零
	
		if(grabTaskStructure.behaviourTime >= 200) //堵转时间到，判定为复位
		{
			grabTaskStructure.liftoreMotor1.baseInf.relativeEcd = 0; 
			grabTaskStructure.liftoreMotor2.baseInf.relativeEcd = 0; 
			
			grabTaskStructure.yplatMotor1.baseInf.relativeEcd = 0; 
			grabTaskStructure.yplatMotor1.mode = GRAB_MOTOR_POSITION;
			grabTaskStructure.behaviourStep++;
		}
	}
	else if(grabTaskStructure.behaviourStep == 12)
	{
		buzzerOn(50, 1, 40);
		grabTaskStructure.inited = 1; //防止重复初始化
	}
}

bool_t grabInitBehaviourEnterCondition() {
  //if(robotInf.modeStep == ROBOT_INIT_GRAB && robotInf.robotMode == ROBOT_INIT && !grabTaskStructure.inited)
  if(grabTaskStructure.inited == 0 && robotInf.robotMode == ROBOT_INIT)
    return TRUE;

  return FALSE;
}

bool_t grabInitBehaviourOutCondition() {
  if(grabInitBehaviourEnterCondition())
    return FALSE;

  return TRUE;
}

//----------GRAB_PREPARE----------准备模式
void grabPrepareBehaviourHandleFun(float *armbaseyExp, float *armbasepExp, float *armmidpExp, float *singlerotateExp, float *xplatExp, float *yplatExp, float *liftoreExp) {
  grabTaskStructure.behaviourTime += GRAB_TASK_MS;//计时
  
		/* 平台抬升*/
	if(grabTaskStructure.behaviourStep == 0)
	{
		int32_t behaviour_ecdSet;
		
		//判定不同对象所需的移动目标值
		if(grabTaskStructure.currentTarget == TARGET_SMALL_ISLAND)//小资源岛
			behaviour_ecdSet=grabTaskStructure.liftoreMotor1.fixedEcd[LIFT_SMALL_ISLAND];
		else if(grabTaskStructure.currentTarget == TARGET_BIG_ISLAND)//大资源岛
			behaviour_ecdSet=grabTaskStructure.liftoreMotor1.fixedEcd[LIFT_BIG_ISLAND];
		else if(grabTaskStructure.currentTarget == TARGET_ALLEY_OPP)//空接
			behaviour_ecdSet=grabTaskStructure.liftoreMotor1.fixedEcd[LIFT_ALLEY_OPP];
		
		*liftoreExp=RAMP_float(behaviour_ecdSet , *liftoreExp , 12000/1000.0*GRAB_TASK_MS);
		
		if(JUDGE_ECD_LIFT1(behaviour_ecdSet))
			grabTaskStructure.behaviourStep = 2;
	}
		/* 平台Y方向前推*/
	else if(grabTaskStructure.behaviourStep == 2)
	{
		int32_t behaviour_ecdSet;
		
		//判定不同对象所需的移动目标值
		if(grabTaskStructure.currentTarget == TARGET_SMALL_ISLAND)//小资源岛
			behaviour_ecdSet=grabTaskStructure.yplatMotor1.fixedEcd[PLAT_Y_SMALL_ISLAND];
		else if(grabTaskStructure.currentTarget == TARGET_BIG_ISLAND)//大资源岛
			behaviour_ecdSet=grabTaskStructure.yplatMotor1.fixedEcd[PLAT_Y_BIG_ISLAND];
		else if(grabTaskStructure.currentTarget == TARGET_ALLEY_OPP)//空接
			behaviour_ecdSet=grabTaskStructure.yplatMotor1.fixedEcd[PLAT_Y_ALLEY_OPP];
		
		*yplatExp=RAMP_float(behaviour_ecdSet , *yplatExp , 14000/1000.0*GRAB_TASK_MS);
		
		if(JUDGE_ECD_YPLAT1(behaviour_ecdSet))
			grabTaskStructure.behaviourStep++;
	}
		/* 机械臂准备*/
	else if(grabTaskStructure.behaviourStep == 3)
	{
		           //armbasep             armmidp      
		int32_t  behaviour_ecdSet1 , behaviour_ecdSet2;
		
		//判定不同对象所需的移动目标值
		if(grabTaskStructure.currentTarget == TARGET_SMALL_ISLAND)//小资源岛
		{
			//grabTaskStructure.oreServo.angleSet=grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_NORMAL];
			behaviour_ecdSet1=grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_SMALL_ISLAND];
			behaviour_ecdSet2=grabTaskStructure.armmidyMotor.fixedEcd[ARM_MID_Y_SMALL_ISLAND];
		}
		else if(grabTaskStructure.currentTarget == TARGET_BIG_ISLAND)//大资源岛
		{
			//grabTaskStructure.oreServo.angleSet=grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_NORMAL];
			behaviour_ecdSet1=grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_BIG_ISLAND];
			behaviour_ecdSet2=grabTaskStructure.armmidyMotor.fixedEcd[ARM_MID_Y_BIG_ISLAND];
		}
		else if(grabTaskStructure.currentTarget == TARGET_ALLEY_OPP)//空接
		{
			//grabTaskStructure.oreServo.angleSet=grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_ALLEY_OPP];
			behaviour_ecdSet1=grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_ALLEY_OPP];
			behaviour_ecdSet2=grabTaskStructure.armmidyMotor.fixedEcd[ARM_MID_Y_ALLEY_OPP];
		}
		
		*armbasepExp=RAMP_float(behaviour_ecdSet1 , *armbasepExp , 7000/1000.0*GRAB_TASK_MS);
		*armmidpExp=RAMP_float(behaviour_ecdSet2 , *armmidpExp , 5000/1000.0*GRAB_TASK_MS);
		
		if(JUDGE_ECD_ARMBASEP(behaviour_ecdSet1) && JUDGE_ECD_ARMMIDY(behaviour_ecdSet2))
			grabTaskStructure.behaviourStep++;
	}
		/*平台X方向和armbasey位置修正，按下E结束修正*/
	else if(grabTaskStructure.behaviourStep == 4)
	{
		int32_t behaviour_ecdSet1 , behaviour_ecdSet2;
		behaviour_ecdSet1 = grabTaskStructure.xplatMotor1.fixedEcd[PLAT_X_NORMAL];
		//behaviour_ecdSet2 = grabTaskStructure.armbaseyMotor.fixedEcd[ARM_BASE_Y_NORMAL];
		
		static int32_t behaviour_Tmp1 , behaviour_Tmp2;

		if(IF_KEY_PRESSED_F && IF_KEY_PRESSED_SHIFT)//Shift + F 右动修正 快
		{
			if(IF_KEY_PRESSED_CTRL)//慢
				behaviour_Tmp1-=5;
			else behaviour_Tmp1-=20;
		}		
		else if(IF_KEY_PRESSED_F)//F 左动修正 快
		{
			if(IF_KEY_PRESSED_CTRL)//慢
				behaviour_Tmp1+=5;
			else behaviour_Tmp1+=20;
		}
		else if(IF_KEY_PRESSED_Z)//F 左动修正 快
		{
			if(IF_KEY_PRESSED_SHIFT) behaviour_Tmp2-=2;
			else behaviour_Tmp2+=2;
		}
		
		
		behaviour_ecdSet1 += behaviour_Tmp1 ;
		behaviour_ecdSet2 += behaviour_Tmp2;
		
		*xplatExp=RAMP_float(behaviour_ecdSet1 , *xplatExp , 5000/1000.0*GRAB_TASK_MS);
		*armbaseyExp=RAMP_float(behaviour_ecdSet2 , *armbaseyExp , 2000/1000.0*GRAB_TASK_MS);
		//按下E结束修正
		if(JUDGE_ECD_XPLAT(behaviour_ecdSet1) && JUDGE_ECD_ARMBASEP(behaviour_ecdSet2) && GRAB_FIRST_PRESS_E)
		{
			PUMP_ORE_ON; //打开抓矿气泵
			
			behaviour_Tmp1=0; //static类型归零
			behaviour_Tmp2=0; //static类型归零
			
			grabTaskStructure.behaviourTime = 0;
			grabTaskStructure.behaviourStep++;
		}
	}
	/* 按下E，抬升，将矿石取出来*/
	else if(grabTaskStructure.behaviourStep == 5)
	{
		if(GRAB_FIRST_PRESS_E)
			grabTaskStructure.behaviourStep++;
	}
		/*抬升，并且舵机、平台Y方向微调*/
	else if(grabTaskStructure.behaviourStep == 6)
	{
		int32_t behaviour_ecdSet , behaviour_ecdSet2;
		behaviour_ecdSet = grabTaskStructure.liftoreMotor1.fixedEcd[LIFT_ALLEY_OPP];
		if(grabTaskStructure.currentTarget == TARGET_SMALL_ISLAND)//小资源岛，对平台Y方向微调
		{
			behaviour_ecdSet2=grabTaskStructure.yplatMotor1.fixedEcd[PLAT_Y_SMALL_ISLAND_LIFT];
			
			*yplatExp=RAMP_float(behaviour_ecdSet2 , *yplatExp , 6000/1000.0*GRAB_TASK_MS);
		}
		
		*liftoreExp=RAMP_float(behaviour_ecdSet , *liftoreExp , 5000/1000.0*GRAB_TASK_MS);
		
		//grabTaskStructure.oreServo.angleSet = grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_SMALL_ISLAND_COMP];
		
		if(JUDGE_ECD_LIFT1(behaviour_ecdSet))
		{
			grabTaskStructure.behaviourStep++;
		}
	}
	/*平台x方向和armbasey回到普通位置，准备放矿石*/
	else if(grabTaskStructure.behaviourStep == 7)
	{
		int32_t behaviour_ecdSet , behaviour_ecdSet2 , behaviour_ecdSet3;

		behaviour_ecdSet = grabTaskStructure.xplatMotor1.fixedEcd[PLAT_X_NORMAL];
		//behaviour_ecdSet2 = grabTaskStructure.armbaseyMotor.fixedEcd[ARM_BASE_Y_NORMAL];
		behaviour_ecdSet3 = grabTaskStructure.armmidyMotor.fixedEcd[ARM_MID_Y_PRE_ORE];
		
		*xplatExp=RAMP_float(behaviour_ecdSet , *xplatExp , 8000/1000.0*GRAB_TASK_MS);
		*armbaseyExp=RAMP_float(behaviour_ecdSet2 , *armbaseyExp , 4000/1000.0*GRAB_TASK_MS);
		*armmidpExp = RAMP_float(behaviour_ecdSet3 , *armmidpExp , 2000/1000.0*GRAB_TASK_MS);
			
		if(JUDGE_ECD_XPLAT(behaviour_ecdSet) && JUDGE_ECD_ARMBASEP(behaviour_ecdSet2) && JUDGE_ECD_ARMMIDY(behaviour_ecdSet3))
			grabBehaviorChange(grabTaskStructure.behaviorList + GRAB_TAKE);
	}

}


bool_t grabPrepareBehaviourEnterCondition() {
  return FALSE;//禁止自动进入
}

bool_t grabPrepareBehaviourOutCondition() {
  return FALSE;//禁止自动退出
}

//----------GRAB_TAKE----------抓取模式
void grabTakeBehaviourHandleFun(float *armbaseyExp, float *armbasepExp, float *armmidpExp, float *singlerotateExp, float *xplatExp, float *yplatExp, float *liftoreExp) {
  grabTaskStructure.behaviourTime += GRAB_TASK_MS;//计时
	
	/*按下E开始存储矿石*/
	if(grabTaskStructure.behaviourStep == 0)
	{
		if(grabTaskStructure.currentTarget == TARGET_EXCH_STA)//如果是兑换，则跳到第二步
			grabTaskStructure.behaviourStep = 2;
		else if(IF_KEY_PRESSED_SHIFT && GRAB_FIRST_PRESS_E) //按下shifit + E ，准备正常抓着跑
		{
			grabTaskStructure.take2exch_Sign = 1;//直接到兑换的标志
			grabTaskStructure.behaviourStep++;
		}
		else if(IF_KEY_PRESSED_CTRL && GRAB_FIRST_PRESS_E) //按下ctrl + E ，应对矿仓有矿石卡住，直接切换到兑换模式
		{
			grabBehaviorChange(grabTaskStructure.behaviorList + GRAB_EXCHANGE);
		}
		else if(GRAB_FIRST_PRESS_E) //按下E开始存储矿石
		{
			grabTaskStructure.behaviourTime = 0;
			grabTaskStructure.behaviourStep++;
		}
	}
		/*舵机和平台的升降和X方向到放矿前预备位置*/
	else if(grabTaskStructure.behaviourStep == 1)
	{
		int32_t behaviour_ecdSet1=grabTaskStructure.liftoreMotor1.fixedEcd[LIFT_NORMAL] , behaviour_ecdSet2=grabTaskStructure.xplatMotor1.fixedEcd[PLAT_X_NORMAL];
		
		*liftoreExp = RAMP_float(behaviour_ecdSet1 , *liftoreExp , 8000/1000.0*GRAB_TASK_MS);
		*xplatExp=RAMP_float(behaviour_ecdSet2 , *xplatExp , 12000/1000.0*GRAB_TASK_MS);
		//grabTaskStructure.oreServo.angleSet = grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_STORE_ORE];//抓矿舵机角度归一
		
		if(JUDGE_ECD_LIFT1(behaviour_ecdSet1) && JUDGE_ECD_XPLAT(behaviour_ecdSet2) && grabTaskStructure.behaviourTime > 1500)
		{
			if(grabTaskStructure.take2exch_Sign == 1)//直接到兑换的标志
				grabTaskStructure.behaviourStep = 3;
			else if(grabTaskStructure.take2exch_Sign == 0)
				grabTaskStructure.behaviourStep++;
		}	
	}
	  /*平台Y方向前伸，到准备存取矿仓矿石的位置*/
	else if(grabTaskStructure.behaviourStep == 2)
	{
		int32_t behaviour_ecdSet=grabTaskStructure.yplatMotor1.fixedEcd[PLAT_Y_STORE_ORE];
		
		*yplatExp=RAMP_float(behaviour_ecdSet , *yplatExp , 20000/1000.0*GRAB_TASK_MS);
		
		if(JUDGE_ECD_YPLAT1(behaviour_ecdSet))
			grabTaskStructure.behaviourStep++;
		
	}
		/*armbasey转至面向矿仓位置*/
	else if(grabTaskStructure.behaviourStep == 3)
	{
		int32_t behaviour_ecdSet=grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_STORE_ORE];
		
		*armbaseyExp = RAMP_float(behaviour_ecdSet , *armbaseyExp , 9000/1000.0*GRAB_TASK_MS);
		
		if(JUDGE_ECD_ARMBASEP(behaviour_ecdSet))
		{
			if(grabTaskStructure.take2exch_Sign != 1)//不是直接到兑换的标志 / 是的话直接等待拨杆
			{
				grabTaskStructure.behaviourTime = 0;//时间清零，为下一步做准备
				grabTaskStructure.behaviourStep++;
			}
		}
	}
//	/*舵机转到直对矿仓位置*/
//	else if(grabTaskStructure.behaviourStep == 4)
//	{
//		if(grabTaskStructure.currentTarget == TARGET_EXCH_STA)//取矿
//			//grabTaskStructure.oreServo.angleSet = grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_DIRECT_EXCH];
//		else //放矿
//			grabTaskStructure.oreServo.angleSet = grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_DIRECT_STORE];
//		if(grabTaskStructure.behaviourTime > 1000) //延时
//			grabTaskStructure.behaviourStep++;
//	}
		/*armmidp转至放矿/取矿位置*/
	else if(grabTaskStructure.behaviourStep == 5)
	{
		int32_t behaviour_ecdSet1 , behaviour_ecdSet2 = grabTaskStructure.armmidyMotor.fixedEcd[ARM_MID_Y_NORMAL];
		
		if(grabTaskStructure.currentTarget == TARGET_EXCH_STA)//取矿
			behaviour_ecdSet1 =grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_EXCH_ORE];
		else //放矿
			behaviour_ecdSet1 =grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_STORE_ORE];
		
		*armbasepExp = RAMP_float(behaviour_ecdSet1 , *armbasepExp , 8000/1000.0*GRAB_TASK_MS);
		*armmidpExp = RAMP_float(behaviour_ecdSet2 , *armmidpExp , 3000/1000.0*GRAB_TASK_MS);
		
		if(JUDGE_ECD_ARMBASEP(behaviour_ecdSet1) && JUDGE_ECD_ARMMIDY(behaviour_ecdSet2))
		{
			grabTaskStructure.behaviourTime = 0;
			grabTaskStructure.behaviourStep++;
		}
	}
	/*平台Y方向收回，到直对矿仓的位置*/
	else if(grabTaskStructure.behaviourStep == 6)
	{
		int32_t behaviour_ecdSet=grabTaskStructure.yplatMotor1.fixedEcd[PLAT_Y_EXCH_ORE];
		
		*yplatExp=RAMP_float(behaviour_ecdSet , *yplatExp , 20000/1000.0*GRAB_TASK_MS);
		
		if(JUDGE_STUCK_YPLAT1 && grabTaskStructure.behaviourTime>2000 && GRAB_FIRST_PRESS_E)//按E
		{
			if(grabTaskStructure.currentTarget == TARGET_EXCH_STA)//取矿
				PUMP_ORE_ON;
			else //放矿
				PUMP_ORE_OFF;
			grabTaskStructure.behaviourTime = 0;
			grabTaskStructure.behaviourStep++;
		}
	}
	/*平台Y方向前伸一段距离*/
	else if(grabTaskStructure.behaviourStep == 7)
	{
		int32_t behaviour_ecdSet=grabTaskStructure.yplatMotor1.fixedEcd[PLAT_Y_FRONT_LITTLE_EXCH];
		
		if(grabTaskStructure.behaviourTime > 1000)// 开/关泵延时1s
			*yplatExp=RAMP_float(behaviour_ecdSet , *yplatExp , 15000/1000.0*GRAB_TASK_MS);
		
		if(JUDGE_ECD_YPLAT1(behaviour_ecdSet))
		{
			if(grabTaskStructure.currentTarget == TARGET_EXCH_STA)//取矿
				grabBehaviorChange(grabTaskStructure.behaviorList + GRAB_EXCHANGE);
			else //放矿，继续
				grabTaskStructure.behaviourStep++;
		}
	}
	
		/*抓取行为结束*/
	else if(grabTaskStructure.behaviourStep == 8)
	{
		if(GRAB_FIRST_PRESS_E) //E 结束本轮取矿，回到普通模式
		{
			grabTaskStructure.collectedOre++;
			//basicTaskStructure.servoCam_Sign = SERVO_CAM_NORMAL;
			grabBehaviorChange(grabTaskStructure.behaviorList + GRAB_NORMAL);
		}
		else if(GRAB_FIRST_PRESS_R) //R 继续取矿
			grabBehaviorChange(grabTaskStructure.behaviorList + GRAB_PREPARE);
	}
}

bool_t grabTakeBehaviourEnterCondition() {

  return FALSE;//禁止自动进入
}

bool_t grabTakeBehaviourOutCondition() {
  return FALSE;//禁止自动退出
}


//int32_t see_private[8];

//----------GRAB_EXCHANGE----------兑换模式
void grabExchangeBehaviourHandleFun(float *armbaseyExp, float *armbasepExp, float *armmidpExp, float *singlerotateExp, float *xplatExp, float *yplatExp, float *liftoreExp) {

  grabTaskStructure.behaviourTime += GRAB_TASK_MS;//计时
	
	/*Step0预定*/
	if(grabTaskStructure.behaviourStep == 0)
	{
		//grabTaskStructure.oreServo.angleSet = grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_STORE_ORE];
		if(grabTaskStructure.behaviourTime > 1000)
		{
			grabTaskStructure.behaviourStep++;
		}
			
	}
	/*armbasep armmidp归位*/
	else if(grabTaskStructure.behaviourStep == 1)
	{
		int32_t behaviour_ecdSet1 =grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_NORMAL] - 1000 , behaviour_ecdSet2 = grabTaskStructure.armmidyMotor.fixedEcd[ARM_MID_Y_NORMAL];
		
		*armbasepExp = RAMP_float(behaviour_ecdSet1 , *armbasepExp , 7000/1000.0*GRAB_TASK_MS);
		*armmidpExp = RAMP_float(behaviour_ecdSet2 , *armmidpExp , 4000/1000.0*GRAB_TASK_MS);
		
		if(JUDGE_ECD_ARMBASEP(behaviour_ecdSet1) && JUDGE_ECD_ARMMIDY(behaviour_ecdSet2))
		{
			grabTaskStructure.behaviourTime = 0;
			grabTaskStructure.behaviourStep++;
		}
	}
	/*armbasey和升降转向前方*/
	else if(grabTaskStructure.behaviourStep == 2)
	{
		int32_t behaviour_ecdSet1 , behaviour_ecdSet2;
		behaviour_ecdSet1=grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_NORMAL];
		behaviour_ecdSet2 = grabTaskStructure.liftoreMotor1.fixedEcd[LIFT_NORMAL];
		
		*armbaseyExp = RAMP_float(behaviour_ecdSet1 , *armbaseyExp , 7000/1000.0*GRAB_TASK_MS);
		*liftoreExp = RAMP_float(behaviour_ecdSet2 , *liftoreExp , 10000/1000.0*GRAB_TASK_MS);
		
		if(JUDGE_ECD_ARMBASEP(behaviour_ecdSet1) && grabTaskStructure.behaviourTime > 2500 && 
			 JUDGE_ECD_LIFT1(behaviour_ecdSet2))
			grabTaskStructure.behaviourStep++;
	}
	/*机械臂armmidp到012级兑换预备位置*/
	else if(grabTaskStructure.behaviourStep == 3)
	{
		int32_t behaviour_ecdSet2 ;
		behaviour_ecdSet2 = grabTaskStructure.armmidyMotor.fixedEcd[ARM_MID_Y_EXCH_LEVEL_01];
		
		//grabTaskStructure.oreServo.angleSet = grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_NORMAL];
		*armmidpExp = RAMP_float(behaviour_ecdSet2 , *armmidpExp , 3000/1000.0*GRAB_TASK_MS);
		
		if(JUDGE_ECD_ARMMIDY(behaviour_ecdSet2))
				grabTaskStructure.behaviourStep++;
	}
	/*机械臂armbasep到012级兑换预备位置*/
	else if(grabTaskStructure.behaviourStep == 4)
	{
		int32_t behaviour_ecdSet1;
		
		behaviour_ecdSet1 = grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_EXCH_LEVEL_01];
		
		//grabTaskStructure.oreServo.angleSet = grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_NORMAL];
		*armbasepExp = RAMP_float(behaviour_ecdSet1 , *armbasepExp , 6000/1000.0*GRAB_TASK_MS);
		
		if(JUDGE_ECD_ARMBASEP(behaviour_ecdSet1))//选择兑换高度种类
		{
				grabTaskStructure.currentExchlevel = EXCH_LEVEL_0;
				grabTaskStructure.behaviourStep++;
		}
	}

	/*八轴手搓*/
	else if(grabTaskStructure.behaviourStep == 5)
	{
		//X   Y   Z   armbasey   armbasep   armmidp  singlerotate  oreServo
		int32_t behaviour_ecdSet[7];//, behaviour_angleSet;
		static int32_t behaviour_Tmp[8];
		
		
		if(grabTaskStructure.currentExchlevel == EXCH_LEVEL_0)
		{
			behaviour_ecdSet[4] = grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_EXCH_LEVEL_01];
			behaviour_ecdSet[5] = grabTaskStructure.armmidyMotor.fixedEcd[ARM_MID_Y_EXCH_LEVEL_01];
			//behaviour_angleSet  = grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_EXCH_LEVEL_01];
		}
		else if(grabTaskStructure.currentExchlevel == EXCH_LEVEL_1)
		{
			behaviour_ecdSet[4] = grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_EXCH_LEVEL_12];
			behaviour_ecdSet[5] = grabTaskStructure.armmidyMotor.fixedEcd[ARM_MID_Y_EXCH_LEVEL_12];
			//behaviour_angleSet  = grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_EXCH_LEVEL_12];
		}
		else if(grabTaskStructure.currentExchlevel == EXCH_LEVEL_2)
		{
			behaviour_ecdSet[4] = grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_EXCH_LEVEL_24];
			behaviour_ecdSet[5] = grabTaskStructure.armmidyMotor.fixedEcd[ARM_MID_Y_EXCH_LEVEL_24];
			//behaviour_angleSet  = grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_EXCH_LEVEL_24];
		}
		
		behaviour_ecdSet[0] = grabTaskStructure.xplatMotor1.fixedEcd[PLAT_X_NORMAL];
		behaviour_ecdSet[1] = grabTaskStructure.yplatMotor1.fixedEcd[PLAT_Y_NORMAL];
		behaviour_ecdSet[2] = grabTaskStructure.liftoreMotor1.fixedEcd[LIFT_LITTLE_UP];
		behaviour_ecdSet[3] = grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_NORMAL];
		behaviour_ecdSet[6] = grabTaskStructure.singlerotateMotor.fixedEcd[ROTATE_SINGLE_NORMAL];
		
		
		//按键控制八个轴
		//                                                             Z
		if(IF_KEY_PRESSED_SHIFT && IF_KEY_PRESSED_F) //平台左
			behaviour_Tmp[0]-=15;
		else if(IF_KEY_PRESSED_F)  //平台右
			behaviour_Tmp[0]+=10;
		//                                                             X
		else if(IF_KEY_PRESSED_SHIFT && IF_KEY_PRESSED_G) //平台后
			behaviour_Tmp[1]-=15;
		else if(IF_KEY_PRESSED_G)  //平台前
			behaviour_Tmp[1]+=15;
		//                                                             C
		else if(IF_KEY_PRESSED_SHIFT && IF_KEY_PRESSED_R) //平台降
			behaviour_Tmp[2]-=10;
		else if(IF_KEY_PRESSED_R)  //平台升
			behaviour_Tmp[2]+=10;
		//
		else if(IF_KEY_PRESSED_SHIFT && IF_KEY_PRESSED_Z) //armbasey
			behaviour_Tmp[3]-=3;
		else if(IF_KEY_PRESSED_Z)  
			behaviour_Tmp[3]+=3;
		//
		else if(IF_KEY_PRESSED_SHIFT && IF_KEY_PRESSED_X) //armbasep
			behaviour_Tmp[4]-=20;
		else if(IF_KEY_PRESSED_X)  
			behaviour_Tmp[4]+=20;
		//
		else if(IF_KEY_PRESSED_SHIFT && IF_KEY_PRESSED_C) //armmidp
			behaviour_Tmp[5]+=3;
		else if(IF_KEY_PRESSED_C)  
			behaviour_Tmp[5]-=3;
		//
		else if(IF_KEY_PRESSED_SHIFT && IF_KEY_PRESSED_V) //singlerotate
			behaviour_Tmp[6]-=2;
		else if(IF_KEY_PRESSED_V)  
			behaviour_Tmp[6]+=2;
		//
		else if(IF_KEY_PRESSED_SHIFT && IF_KEY_PRESSED_B) //OreAngle
			behaviour_Tmp[7]-=2;
		else if(IF_KEY_PRESSED_B)  
			behaviour_Tmp[7]+=2;		
		
//增加值限幅
		//xplat
		singleNum_Limit(&grabTaskStructure.xplatMotor1.fixedEcd[PLAT_X_MIN] , &behaviour_ecdSet[0] , &behaviour_Tmp[0] , &grabTaskStructure.xplatMotor1.fixedEcd[PLAT_X_RESET]);
		//yplat
		singleNum_Limit(&grabTaskStructure.yplatMotor1.fixedEcd[PLAT_Y_MIN] , &behaviour_ecdSet[1] , &behaviour_Tmp[1] , &grabTaskStructure.yplatMotor1.fixedEcd[PLAT_Y_FRONT_MAX]);
		//liftore
		singleNum_Limit(&grabTaskStructure.liftoreMotor1.fixedEcd[LIFT_RESET] , &behaviour_ecdSet[2] , &behaviour_Tmp[2] , &grabTaskStructure.liftoreMotor1.fixedEcd[LIFT_MAX]);
		//armbasey
		//singleNum_Limit(&grabTaskStructure.armbaseyMotor.fixedEcd[ARM_BASE_Y_RESET] , &behaviour_ecdSet[3] , &behaviour_Tmp[3] , &grabTaskStructure.armbaseyMotor.fixedEcd[ARM_BASE_Y_MAX]);
		//armbasep
		singleNum_Limit(&grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_MIN] , &behaviour_ecdSet[4] , &behaviour_Tmp[4] , &grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_RESET]);
		//armmidy
		singleNum_Limit(&grabTaskStructure.armmidyMotor.fixedEcd[ARM_MID_Y_RESET] , &behaviour_ecdSet[5] , &behaviour_Tmp[5] , &grabTaskStructure.armmidyMotor.fixedEcd[ARM_MID_Y_MAX]);
		//singlerotate
		singleNum_Limit(&grabTaskStructure.singlerotateMotor.fixedEcd[ROTATE_SINGLE_RESET] , &behaviour_ecdSet[6] , &behaviour_Tmp[6] , &grabTaskStructure.singlerotateMotor.fixedEcd[ROTATE_SINGLE_MAX]);
		
		//先限幅，再传递增加量
		for(int i=0 ; i<7 ; i++) behaviour_ecdSet[i]+=behaviour_Tmp[i];
		
//		//舵机限幅额外处理
//		if(behaviour_angleSet + behaviour_Tmp[7]*0.05 < grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_ALLEY_OPP])
//			behaviour_Tmp[7] = (grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_ALLEY_OPP] - behaviour_angleSet)/0.05;
//		else if(behaviour_angleSet + behaviour_Tmp[7]*0.05 > grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_DIRECT_GROUND])
//			behaviour_Tmp[7] = (grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_DIRECT_GROUND] - behaviour_angleSet)/0.05;
//		//传递舵机值
//		behaviour_angleSet += behaviour_Tmp[7]*0.05;
		
		
		*xplatExp = RAMP_float(behaviour_ecdSet[0] , *xplatExp , 10000/1000.0*GRAB_TASK_MS);
		*yplatExp = RAMP_float(behaviour_ecdSet[1] , *yplatExp , 10000/1000.0*GRAB_TASK_MS);
		*liftoreExp = RAMP_float(behaviour_ecdSet[2] , *liftoreExp , 5000/1000.0*GRAB_TASK_MS);
		*armbaseyExp = RAMP_float(behaviour_ecdSet[3] , *armbaseyExp , 3000/1000.0*GRAB_TASK_MS);
		*armbasepExp = RAMP_float(behaviour_ecdSet[4] , *armbasepExp , 6000/1000.0*GRAB_TASK_MS);
		*armmidpExp = RAMP_float(behaviour_ecdSet[5] , *armmidpExp , 1000/1000.0*GRAB_TASK_MS);
		*singlerotateExp = RAMP_float(behaviour_ecdSet[6] , *singlerotateExp , 1500/1000.0*GRAB_TASK_MS);
		//grabTaskStructure.oreServo.angleSet = behaviour_angleSet;
	
		if(GRAB_FIRST_PRESS_W)//W切换默认高度
		{
			for(int i=4 ; i<6 ; i++) behaviour_Tmp[i]=0;
			
			if(grabTaskStructure.currentExchlevel == EXCH_LEVEL_0) grabTaskStructure.currentExchlevel = EXCH_LEVEL_1;
			else if(grabTaskStructure.currentExchlevel == EXCH_LEVEL_1) grabTaskStructure.currentExchlevel = EXCH_LEVEL_2;
			else if(grabTaskStructure.currentExchlevel == EXCH_LEVEL_2) grabTaskStructure.currentExchlevel = EXCH_LEVEL_0;
		}
		//按E结束兑换,气泵关
		else if(GRAB_FIRST_PRESS_E && JUDGE_ECD_ARMMIDY(behaviour_ecdSet[5]))
		{
			for(int i=0 ; i<8 ; i++) behaviour_Tmp[i]=0;
			PUMP_ORE_OFF;//气泵关
			grabTaskStructure.behaviourStep++;
		}
	}
	/*按E结束，回到普通模式*/
	else if(grabTaskStructure.behaviourStep == 6)
	{
		if(GRAB_FIRST_PRESS_E)
		{
			if(grabTaskStructure.take2exch_Sign == 1)//直接到兑换的标志
				grabTaskStructure.take2exch_Sign = 0;
			
			grabTaskStructure.collectedOre--;
			
			//basicTaskStructure.servoCam_Sign = SERVO_CAM_NORMAL;//自动切换舵机视角
			grabBehaviorChange(grabTaskStructure.behaviorList + GRAB_NORMAL);
		}
	}
}

bool_t grabExchangeBehaviourEnterCondition() {
  return FALSE;//禁止自动进入
}

bool_t grabExchangeBehaviourOutCondition() {
  return FALSE;//禁止自动退出
}

//----------GRAB_NORMAL----------
bool_t grabNormalBehaviourEnterCondition() {
  return TRUE; //允许自动进入
}

bool_t grabNormalBehaviourOutCondition() {
  return FALSE; //不允许自动进入
}

void grabNormalBehaviourHandleFun(float *armbaseyExp, float *armbasepExp, float *armmidpExp, float *singlerotateExp, float *xplatExp, float *yplatExp, float *liftoreExp) 
{
	grabTaskStructure.behaviourTime += GRAB_TASK_MS;//计时
	
		/* Step0预留*/
	if(grabTaskStructure.behaviourStep == 0)
	{
		PUMP_ORE_OFF;
		//grabTaskStructure.oreServo.angleSet=grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_NORMAL];
		
		if(grabTaskStructure.behaviourTime > 500)//预留舵机旋转时间
			grabTaskStructure.behaviourStep++;
	}
		/* 爪子旋转电机到目标位置*/
	else if(grabTaskStructure.behaviourStep == 1)
	{
		int32_t behaviour_ecdSet=grabTaskStructure.singlerotateMotor.fixedEcd[ROTATE_SINGLE_NORMAL];
		
		*singlerotateExp=RAMP_float(behaviour_ecdSet , *singlerotateExp , 5000/1000.0*GRAB_TASK_MS);
		
		if(JUDGE_ECD_PARALLElLMOTOR1(behaviour_ecdSet))
			grabTaskStructure.behaviourStep++;
	}
		/* armmidp电机到目标位置*/
	else if(grabTaskStructure.behaviourStep == 2)
	{
		int32_t behaviour_ecdSet1=grabTaskStructure.armmidyMotor.fixedEcd[ARM_MID_Y_NORMAL];
		
		*armmidpExp=RAMP_float(behaviour_ecdSet1 , *armmidpExp , 3000/1000.0*GRAB_TASK_MS);
		
		if(JUDGE_ECD_ARMMIDY(behaviour_ecdSet1))
			grabTaskStructure.behaviourStep++;
	}
		/* armbasep电机（MF9025）到目标位置  armmidp电机到目标位置*/
	else if(grabTaskStructure.behaviourStep == 3)
	{
		int32_t behaviour_ecdSet1=grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_NORMAL];
		
		
		*armbasepExp=RAMP_float(behaviour_ecdSet1 , *armbasepExp , 8000/1000.0*GRAB_TASK_MS);
		
		if(JUDGE_ECD_ARMBASEP(behaviour_ecdSet1))
			grabTaskStructure.behaviourStep++;
	}
		/* armbasey电机到目标位置*/
//	else if(grabTaskStructure.behaviourStep == 4)
//	{
//		int32_t behaviour_ecdSet=grabTaskStructure.armbaseyMotor.fixedEcd[ARM_BASE_Y_NORMAL];
//		
//		*armbaseyExp=RAMP_float(behaviour_ecdSet , *armbaseyExp , 9000/1000.0*GRAB_TASK_MS);
//		
//		if(JUDGE_ECD_ARMBASEY(behaviour_ecdSet))
//			grabTaskStructure.behaviourStep++;
//	}
		/* 平台X方向电机到目标位置*/
	else if(grabTaskStructure.behaviourStep == 5)
	{
		int32_t behaviour_ecdSet=grabTaskStructure.xplatMotor1.fixedEcd[PLAT_X_NORMAL];
		
		*xplatExp=RAMP_float(behaviour_ecdSet , *xplatExp , 15000/1000.0*GRAB_TASK_MS);
		
		if(JUDGE_ECD_XPLAT(behaviour_ecdSet))
			grabTaskStructure.behaviourStep++;
	}
		/* 平台Y方向电机到目标位置*/		
	else if(grabTaskStructure.behaviourStep == 6)
	{
		int32_t behaviour_ecdSet=grabTaskStructure.yplatMotor1.fixedEcd[PLAT_Y_NORMAL];
		
		*yplatExp=RAMP_float(behaviour_ecdSet , *yplatExp , 20000/1000.0*GRAB_TASK_MS);
		
		if(JUDGE_ECD_YPLAT1(behaviour_ecdSet))
			grabTaskStructure.behaviourStep++;
	}
		/* 平台升降电机到目标位置*/
	else if(grabTaskStructure.behaviourStep == 7)
	{
		int32_t behaviour_ecdSet=grabTaskStructure.liftoreMotor1.fixedEcd[LIFT_NORMAL];
		
		*liftoreExp=RAMP_float(behaviour_ecdSet , *liftoreExp , 7000/1000.0*GRAB_TASK_MS);
		
		if(JUDGE_ECD_LIFT1(behaviour_ecdSet))
			grabTaskStructure.behaviourStep++;
	}

	if(grabTaskStructure.inited == 0)//重新初始化
		grabBehaviorChange(grabTaskStructure.behaviorList + GRAB_INIT);
	
}
