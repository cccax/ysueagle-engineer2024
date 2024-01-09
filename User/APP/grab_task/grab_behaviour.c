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

//�����ת���
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
//������ض�λ�ü��
//#define JUDGE_ECD_ARMBASEY(ecdSet) JUDGE_ECD_MOTOR_READY(JUDGE_ARMBASE_Y_MOTOR , ecdSet , JUDGE_ECD_SIGN)
#define JUDGE_ECD_ARMBASEP(ecdSet) JUDGE_ECD_MOTOR_READY(JUDGE_ARMBASE_P_MOTOR1 , ecdSet , JUDGE_ECD_SIGN)
#define JUDGE_ECD_ARMMIDY(ecdSet) JUDGE_ECD_MOTOR_READY(JUDGE_ARMMID_Y_MOTOR , ecdSet , JUDGE_ECD_SIGN) //cjh p��y
#define JUDGE_ECD_SINGLE_ROTATE(ecdSet) JUDGE_ECD_MOTOR_READY(JUDGE_SINGLE_ROTATE_MOTOR , ecdSet , JUDGE_ECD_SIGN)
#define JUDGE_ECD_XPLAT(ecdSet) JUDGE_ECD_MOTOR_READY(JUDGE_PLAT_X_MOTOR , ecdSet , JUDGE_ECD_SIGN)
#define JUDGE_ECD_YPLAT1(ecdSet) JUDGE_ECD_MOTOR_READY(JUDGE_PLAT_Y_MOTOR1 , ecdSet , JUDGE_ECD_SIGN)
#define JUDGE_ECD_YPLAT2(ecdSet) JUDGE_ECD_MOTOR_READY(JUDGE_PLAT_Y_MOTOR2 , ecdSet , JUDGE_ECD_SIGN)//cjh
#define JUDGE_ECD_PARALLElLMOTOR2(ecdSet) JUDGE_ECD_MOTOR_READY(JUDGE_PARALLELMOTOR2 , ecdSet , JUDGE_ECD_SIGN)//cjh
#define JUDGE_ECD_PARALLElLMOTOR1(ecdSet) JUDGE_ECD_MOTOR_READY(JUDGE_PARALLELMOTOR1 , ecdSet , JUDGE_ECD_SIGN)//cjh

#define JUDGE_ECD_LIFT1(ecdSet) JUDGE_ECD_MOTOR_READY(JUDGE_LIFT_MOTOR1 , ecdSet , JUDGE_ECD_SIGN)
#define JUDGE_ECD_LIFT2(ecdSet) JUDGE_ECD_MOTOR_READY(JUDGE_LIFT_MOTOR2 , ecdSet , JUDGE_ECD_SIGN)


//���ԭʼ����
extern float rawData_Servo[RAWDATA_LENGTH] , rawData_armmidp[RAWDATA_LENGTH] , rawData_armbasep[RAWDATA_LENGTH];

//����ֵ�͵�ǰֵ�޷�
void singleNum_Limit(int32_t *limitMin , int32_t *nowNum , int32_t *addNum , int32_t *limitMax)
{
	//����ֵ�޷�
	if(*nowNum + *addNum > *limitMax) *addNum = *limitMax - *nowNum;
	else if(*nowNum + *addNum < *limitMin) *addNum = *limitMin - *nowNum;
}

////���������ݴ���
//int16_t dataTrans_Servo(float rawAngle)
//{
//	int16_t angle_Offset = grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_NORMAL] - 10;//�����ֱʱΪ��ʼֵ
//	return angle_Offset - rawAngle;
//}

////����armmidp���ݴ���
//int16_t dataTrans_Armmidp(float rawAngle)
//{
//	int16_t angle_Offset = grabTaskStructure.armmidpMotor.fixedEcd[ARM_MID_P_NORMAL];
//	return angle_Offset + (rawAngle * 2.5) / 360 * 8191;
//}

////����armbasep���ݴ���
//int16_t dataTrans_Armbasep(float rawAngle)
//{
//	int16_t angle_Offset = grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_STORE_START] + 291;
//	return angle_Offset + rawAngle/360 * 65535;
//}

//�жϵ���Ƿ���ĳֵ����
uint8_t JUDGE_ECD_MOTOR_READY(uint32_t ID, int32_t SET , uint8_t SIGN) { 
  int32_t REAL = 0 , RANGE = 0;
  switch(ID) {
    case JUDGE_ARMMID_Y_MOTOR:
      REAL = grabTaskStructure.armmidyMotor.totalEcd;//�м�yaw����
      if(SIGN == JUDGE_ECD_SIGN)  RANGE = 10;
		  else if(SIGN == JUDGE_STUCK_SIGN) RANGE = 50;
			break;
		
    case JUDGE_ARMBASE_P_MOTOR1:
      REAL = grabTaskStructure.armbasepMotor1.totalEcd;//����pitch����(MF9025)1  //m3508
      if(SIGN == JUDGE_ECD_SIGN)  RANGE = 300;
		  else if(SIGN == JUDGE_STUCK_SIGN) RANGE = 50;
			break;
		
    case JUDGE_ARMBASE_P_MOTOR2:
      REAL = grabTaskStructure.armbasepMotor2.totalEcd;//����pitch����(MF9025)2  //m3508
      if(SIGN == JUDGE_ECD_SIGN)  RANGE = 300;
		  else if(SIGN == JUDGE_STUCK_SIGN) RANGE = 50;
			break;
		
//    case JUDGE_ARMMID_P_MOTOR:
//      REAL = grabTaskStructure.armmidpMotor.totalEcd;//����yaw����
//      if(SIGN == JUDGE_ECD_SIGN)  RANGE = 100;
//		  else if(SIGN == JUDGE_STUCK_SIGN) RANGE = 50;
//      break;
		
//    case JUDGE_SINGLE_ROTATE_MOTOR:
//      REAL = grabTaskStructure.singlerotateMotor.totalEcd;//��צ
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

//		//�������ư˸���
//		//                                                             Z
//		if(IF_KEY_PRESSED_SHIFT && IF_KEY_PRESSED_F) //ƽ̨��
//			behaviour_Tmp[0]-=15;
//		else if(IF_KEY_PRESSED_F)  //ƽ̨��
//			behaviour_Tmp[0]+=10;
//		//                                                             X
//		else if(IF_KEY_PRESSED_SHIFT && IF_KEY_PRESSED_G) //ƽ̨��
//			behaviour_Tmp[1]-=15;
//		else if(IF_KEY_PRESSED_G)  //ƽ̨ǰ
//			behaviour_Tmp[1]+=15;
//		//                                                             C
//		else if(IF_KEY_PRESSED_SHIFT && IF_KEY_PRESSED_R) //ƽ̨��
//			behaviour_Tmp[2]-=10;
//		else if(IF_KEY_PRESSED_R)  //ƽ̨��
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


//����ֵ�޷�
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
		
		//���޷����ٴ���������
		for(int i=0 ; i<7 ; i++) behaviour_ecdSet[i]+=behaviour_Tmp[i];
		
		
		*xplatExp = RAMP_float(behaviour_ecdSet[0] , *xplatExp , 10000/1000.0*GRAB_TASK_MS);
		*yplatExp = RAMP_float(behaviour_ecdSet[1] , *yplatExp , 10000/1000.0*GRAB_TASK_MS);
		*liftoreExp = RAMP_float(behaviour_ecdSet[2] , *liftoreExp , 5000/1000.0*GRAB_TASK_MS);
		*armbaseyExp = RAMP_float(behaviour_ecdSet[3] , *armbaseyExp , 3000/1000.0*GRAB_TASK_MS);
		*armbasepExp = RAMP_float(behaviour_ecdSet[4] , *armbasepExp , 6000/1000.0*GRAB_TASK_MS);
		*armmidpExp = RAMP_float(behaviour_ecdSet[5] , *armmidpExp , 1000/1000.0*GRAB_TASK_MS);
		*singlerotateExp = RAMP_float(behaviour_ecdSet[6] , *singlerotateExp , 1500/1000.0*GRAB_TASK_MS);
	
		if(GRAB_FIRST_PRESS_W)//W�л�Ĭ�ϸ߶�
		{
			for(int i=4 ; i<6 ; i++) behaviour_Tmp[i]=0;
			
			if(grabTaskStructure.currentExchlevel == EXCH_LEVEL_0) grabTaskStructure.currentExchlevel = EXCH_LEVEL_1;
			else if(grabTaskStructure.currentExchlevel == EXCH_LEVEL_1) grabTaskStructure.currentExchlevel = EXCH_LEVEL_2;
			else if(grabTaskStructure.currentExchlevel == EXCH_LEVEL_2) grabTaskStructure.currentExchlevel = EXCH_LEVEL_0;
		}
//		//��E�����һ�,���ù�
//		else if(GRAB_FIRST_PRESS_E && JUDGE_ECD_ARMMIDY(behaviour_ecdSet[5]))
//		{
//			for(int i=0 ; i<8 ; i++) behaviour_Tmp[i]=0;
//			PUMP_ORE_OFF;//���ù�
//			grabTaskStructure.behaviourStep++;
//		}
}



bool_t grabDebugBehaviourEnterCondition() {
  #ifdef DEBUG_GRAB
  return TRUE;
  #endif

  #ifndef DEBUG_GRAB
//  if(IF_RC_SW1_UP && GRAB_FIRST_S2_UP) { //S1�� S2�� С��Դ��
//    grabTaskStructure.currentTarget = TARGET_SMALL_ISLAND;
//    grabBehaviorChange(grabTaskStructure.behaviorList + GRAB_PREPARE);
//		basicTaskStructure.servoCam_Sign = SERVO_CAM_GRAB;

//  } 
//	else if(IF_RC_SW1_UP && GRAB_FIRST_S2_MID) { //S1�� S2�� ����Դ��
//    grabTaskStructure.currentTarget = TARGET_BIG_ISLAND;
//    grabBehaviorChange(grabTaskStructure.behaviorList + GRAB_PREPARE);
//		basicTaskStructure.servoCam_Sign = SERVO_CAM_GRAB;
//  } 
//	else if(IF_RC_SW1_UP && GRAB_FIRST_S2_DOWN) { //S1�� S2�� �ս�
//    grabTaskStructure.currentTarget = TARGET_ALLEY_OPP;
//    grabBehaviorChange(grabTaskStructure.behaviorList + GRAB_PREPARE);
//		basicTaskStructure.servoCam_Sign = SERVO_CAM_GRAB;
//  } 
//	else if(IF_RC_SW1_DOWN && GRAB_FIRST_S2_UP) { //S1�� S2�� �һ�
//    grabTaskStructure.currentTarget = TARGET_EXCH_STA;
//		if(grabTaskStructure.take2exch_Sign == 1)//ֱ�ӵ��һ�
//			grabBehaviorChange(grabTaskStructure.behaviorList + GRAB_EXCHANGE);
//		else if(grabTaskStructure.take2exch_Sign == 0)//����ֱ�ӵ��һ�
//			grabBehaviorChange(grabTaskStructure.behaviorList + GRAB_TAKE);
//		basicTaskStructure.servoCam_Sign = SERVO_CAM_GRAB;
//	}
//	else if(IF_RC_SW1_DOWN && GRAB_FIRST_S2_MID) { //S1�� S2�� ǿ�ƻص���ͨģʽ
//    grabBehaviorChange(grabTaskStructure.behaviorList + GRAB_NORMAL);
//	}
//	else if(IF_RC_SW1_DOWN && GRAB_FIRST_S2_DOWN) { //S1�� S2�� ���³�ʼ��
//		grabTaskStructure.inited = 0;//��ʼ��״̬����
//    grabBehaviorChange(grabTaskStructure.behaviorList + GRAB_NORMAL);//�Ȼص���ͨģʽ
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

//	for(int i=1 ; i<errorListLength-4 ; i++) //������4��������е�����߼��
//	{
//		if(toe_is_error(i)) // ���е�����߼��
//		{
//			buzzerOn(100,1, 60);
//			return TRUE;
//		}
//	}
	
  if(toe_is_error(DBUSTOE)) // ң��������
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
  
	grabTaskStructure.behaviourTime += GRAB_TASK_MS;//��ʱ
	
	//��ֹ�ظ���ʼ��
  if(grabTaskStructure.inited) { 
    grabBehaviorChange(grabTaskStructure.behaviorList + GRAB_ZERO_FORCE);
    return;
  }
	
	/*��ʼ��׼��*/
	if(grabTaskStructure.behaviourStep == 0 && GRAB_FIRST_PRESS_B)
	{
		PUMP_ORE_OFF;  //�ر�ץ������
		
		//�޸ĵ�����Ʒ�ʽΪ��������
    //grabTaskStructure.singlerotateMotor.mode = GRAB_MOTOR_SPEED; //צ����ת���
		grabTaskStructure.armmidyMotor.mode = GRAB_MOTOR_RAW;
		grabTaskStructure.yplatMotor1.mode = GRAB_MOTOR_POSITION;
		//grabTaskStructure.yplatMotor1.mode = GRAB_MOTOR_POSITION;
		grabTaskStructure.xplatMotor1.mode = GRAB_MOTOR_POSITION;
		grabTaskStructure.armbasepMotor1.mode = GRAB_MOTOR_POSITION;
		grabTaskStructure.armbasepMotor1.totalEcdSet = 0;
		
		grabTaskStructure.behaviourStep=4; //����Step4
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
	/*ƽ̨Y����ǰ��һС�Σ���ֹX�����ʼ��ʱ��е����*/
	else if(grabTaskStructure.behaviourStep == 5) 
	{
		int32_t behaviour_ecdSet=grabTaskStructure.yplatMotor1.fixedEcd[PLAT_Y_FRONT_LITTLE_INIT];
		
		grabTaskStructure.yplatMotor1.totalEcdSet=behaviour_ecdSet;
		
		if(JUDGE_ECD_YPLAT1(behaviour_ecdSet)) 
			grabTaskStructure.behaviourStep++;
	}
	/*ƽ̨X��������һС�Σ���ֹ��ʼ��ǰX�����ϻ���������λ����*/
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
	/* ��������ʼ���ĵ����ʼ����*/
	else if(grabTaskStructure.behaviourStep == 7) 
	{
		//grabTaskStructure.armbaseyMotor.speedSet = -50; //��ʱ�� ok
		grabTaskStructure.singlerotateMotor.speedSet = -70;//����
		grabTaskStructure.xplatMotor1.speedSet = 70;//ƽ̨�����ƶ���
	
		if(grabTaskStructure.limsw[LIMSW_ALL] == 0x0F)//��λ����ȫ����λ
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
	/*��е�ۻص���ͨ״̬��ƽ̨X�����������һ�ξ��룬ʹ��������*/
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
		
		//�ж��Ƿ�λ
		if(JUDGE_ECD_ARMBASEP(behaviour_ecdSet1) && JUDGE_ECD_ARMMIDY(behaviour_ecdSet2) && 
			 JUDGE_ECD_XPLAT(behaviour_ecdSet3) && JUDGE_ECD_PARALLElLMOTOR1(behaviour_ecdSet4))
			{
				grabTaskStructure.yplatMotor1.mode = GRAB_MOTOR_SPEED;  //������
				grabTaskStructure.behaviourStep++;
			}
	}
	/*ƽ̨X����ص���ͨλ��*/
	else if(grabTaskStructure.behaviourStep == 9)
	{
		int32_t behaviour_ecdSet;
		behaviour_ecdSet=grabTaskStructure.xplatMotor1.fixedEcd[PLAT_X_NORMAL];
		
		*xplatExp=RAMP_float(behaviour_ecdSet , *xplatExp , 13000/1000.0*GRAB_TASK_MS);
		if(JUDGE_ECD_XPLAT(behaviour_ecdSet))
			grabTaskStructure.behaviourStep++;
	}
	else if(grabTaskStructure.behaviourStep == 10)//ƽ̨Y��������������ʼ��
	{
		grabTaskStructure.yplatMotor1.speedSet = -80; //Y����������
		grabTaskStructure.behaviourTime = 0;//��ת��������
		grabTaskStructure.behaviourStep++;
	}
	else if(grabTaskStructure.behaviourStep == 11)//���ƽ̨Y��������ת
	{
		if(!JUDGE_STUCK_YPLAT1) grabTaskStructure.behaviourTime = 0; //δ��λ��ʱ������
	
		if(grabTaskStructure.behaviourTime >= 200) //��תʱ�䵽���ж�Ϊ��λ
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
		grabTaskStructure.inited = 1; //��ֹ�ظ���ʼ��
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

//----------GRAB_PREPARE----------׼��ģʽ
void grabPrepareBehaviourHandleFun(float *armbaseyExp, float *armbasepExp, float *armmidpExp, float *singlerotateExp, float *xplatExp, float *yplatExp, float *liftoreExp) {
  grabTaskStructure.behaviourTime += GRAB_TASK_MS;//��ʱ
  
		/* ƽ̨̧��*/
	if(grabTaskStructure.behaviourStep == 0)
	{
		int32_t behaviour_ecdSet;
		
		//�ж���ͬ����������ƶ�Ŀ��ֵ
		if(grabTaskStructure.currentTarget == TARGET_SMALL_ISLAND)//С��Դ��
			behaviour_ecdSet=grabTaskStructure.liftoreMotor1.fixedEcd[LIFT_SMALL_ISLAND];
		else if(grabTaskStructure.currentTarget == TARGET_BIG_ISLAND)//����Դ��
			behaviour_ecdSet=grabTaskStructure.liftoreMotor1.fixedEcd[LIFT_BIG_ISLAND];
		else if(grabTaskStructure.currentTarget == TARGET_ALLEY_OPP)//�ս�
			behaviour_ecdSet=grabTaskStructure.liftoreMotor1.fixedEcd[LIFT_ALLEY_OPP];
		
		*liftoreExp=RAMP_float(behaviour_ecdSet , *liftoreExp , 12000/1000.0*GRAB_TASK_MS);
		
		if(JUDGE_ECD_LIFT1(behaviour_ecdSet))
			grabTaskStructure.behaviourStep = 2;
	}
		/* ƽ̨Y����ǰ��*/
	else if(grabTaskStructure.behaviourStep == 2)
	{
		int32_t behaviour_ecdSet;
		
		//�ж���ͬ����������ƶ�Ŀ��ֵ
		if(grabTaskStructure.currentTarget == TARGET_SMALL_ISLAND)//С��Դ��
			behaviour_ecdSet=grabTaskStructure.yplatMotor1.fixedEcd[PLAT_Y_SMALL_ISLAND];
		else if(grabTaskStructure.currentTarget == TARGET_BIG_ISLAND)//����Դ��
			behaviour_ecdSet=grabTaskStructure.yplatMotor1.fixedEcd[PLAT_Y_BIG_ISLAND];
		else if(grabTaskStructure.currentTarget == TARGET_ALLEY_OPP)//�ս�
			behaviour_ecdSet=grabTaskStructure.yplatMotor1.fixedEcd[PLAT_Y_ALLEY_OPP];
		
		*yplatExp=RAMP_float(behaviour_ecdSet , *yplatExp , 14000/1000.0*GRAB_TASK_MS);
		
		if(JUDGE_ECD_YPLAT1(behaviour_ecdSet))
			grabTaskStructure.behaviourStep++;
	}
		/* ��е��׼��*/
	else if(grabTaskStructure.behaviourStep == 3)
	{
		           //armbasep             armmidp      
		int32_t  behaviour_ecdSet1 , behaviour_ecdSet2;
		
		//�ж���ͬ����������ƶ�Ŀ��ֵ
		if(grabTaskStructure.currentTarget == TARGET_SMALL_ISLAND)//С��Դ��
		{
			//grabTaskStructure.oreServo.angleSet=grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_NORMAL];
			behaviour_ecdSet1=grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_SMALL_ISLAND];
			behaviour_ecdSet2=grabTaskStructure.armmidyMotor.fixedEcd[ARM_MID_Y_SMALL_ISLAND];
		}
		else if(grabTaskStructure.currentTarget == TARGET_BIG_ISLAND)//����Դ��
		{
			//grabTaskStructure.oreServo.angleSet=grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_NORMAL];
			behaviour_ecdSet1=grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_BIG_ISLAND];
			behaviour_ecdSet2=grabTaskStructure.armmidyMotor.fixedEcd[ARM_MID_Y_BIG_ISLAND];
		}
		else if(grabTaskStructure.currentTarget == TARGET_ALLEY_OPP)//�ս�
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
		/*ƽ̨X�����armbaseyλ������������E��������*/
	else if(grabTaskStructure.behaviourStep == 4)
	{
		int32_t behaviour_ecdSet1 , behaviour_ecdSet2;
		behaviour_ecdSet1 = grabTaskStructure.xplatMotor1.fixedEcd[PLAT_X_NORMAL];
		//behaviour_ecdSet2 = grabTaskStructure.armbaseyMotor.fixedEcd[ARM_BASE_Y_NORMAL];
		
		static int32_t behaviour_Tmp1 , behaviour_Tmp2;

		if(IF_KEY_PRESSED_F && IF_KEY_PRESSED_SHIFT)//Shift + F �Ҷ����� ��
		{
			if(IF_KEY_PRESSED_CTRL)//��
				behaviour_Tmp1-=5;
			else behaviour_Tmp1-=20;
		}		
		else if(IF_KEY_PRESSED_F)//F ������ ��
		{
			if(IF_KEY_PRESSED_CTRL)//��
				behaviour_Tmp1+=5;
			else behaviour_Tmp1+=20;
		}
		else if(IF_KEY_PRESSED_Z)//F ������ ��
		{
			if(IF_KEY_PRESSED_SHIFT) behaviour_Tmp2-=2;
			else behaviour_Tmp2+=2;
		}
		
		
		behaviour_ecdSet1 += behaviour_Tmp1 ;
		behaviour_ecdSet2 += behaviour_Tmp2;
		
		*xplatExp=RAMP_float(behaviour_ecdSet1 , *xplatExp , 5000/1000.0*GRAB_TASK_MS);
		*armbaseyExp=RAMP_float(behaviour_ecdSet2 , *armbaseyExp , 2000/1000.0*GRAB_TASK_MS);
		//����E��������
		if(JUDGE_ECD_XPLAT(behaviour_ecdSet1) && JUDGE_ECD_ARMBASEP(behaviour_ecdSet2) && GRAB_FIRST_PRESS_E)
		{
			PUMP_ORE_ON; //��ץ������
			
			behaviour_Tmp1=0; //static���͹���
			behaviour_Tmp2=0; //static���͹���
			
			grabTaskStructure.behaviourTime = 0;
			grabTaskStructure.behaviourStep++;
		}
	}
	/* ����E��̧��������ʯȡ����*/
	else if(grabTaskStructure.behaviourStep == 5)
	{
		if(GRAB_FIRST_PRESS_E)
			grabTaskStructure.behaviourStep++;
	}
		/*̧�������Ҷ����ƽ̨Y����΢��*/
	else if(grabTaskStructure.behaviourStep == 6)
	{
		int32_t behaviour_ecdSet , behaviour_ecdSet2;
		behaviour_ecdSet = grabTaskStructure.liftoreMotor1.fixedEcd[LIFT_ALLEY_OPP];
		if(grabTaskStructure.currentTarget == TARGET_SMALL_ISLAND)//С��Դ������ƽ̨Y����΢��
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
	/*ƽ̨x�����armbasey�ص���ͨλ�ã�׼���ſ�ʯ*/
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
  return FALSE;//��ֹ�Զ�����
}

bool_t grabPrepareBehaviourOutCondition() {
  return FALSE;//��ֹ�Զ��˳�
}

//----------GRAB_TAKE----------ץȡģʽ
void grabTakeBehaviourHandleFun(float *armbaseyExp, float *armbasepExp, float *armmidpExp, float *singlerotateExp, float *xplatExp, float *yplatExp, float *liftoreExp) {
  grabTaskStructure.behaviourTime += GRAB_TASK_MS;//��ʱ
	
	/*����E��ʼ�洢��ʯ*/
	if(grabTaskStructure.behaviourStep == 0)
	{
		if(grabTaskStructure.currentTarget == TARGET_EXCH_STA)//����Ƕһ����������ڶ���
			grabTaskStructure.behaviourStep = 2;
		else if(IF_KEY_PRESSED_SHIFT && GRAB_FIRST_PRESS_E) //����shifit + E ��׼������ץ����
		{
			grabTaskStructure.take2exch_Sign = 1;//ֱ�ӵ��һ��ı�־
			grabTaskStructure.behaviourStep++;
		}
		else if(IF_KEY_PRESSED_CTRL && GRAB_FIRST_PRESS_E) //����ctrl + E ��Ӧ�Կ���п�ʯ��ס��ֱ���л����һ�ģʽ
		{
			grabBehaviorChange(grabTaskStructure.behaviorList + GRAB_EXCHANGE);
		}
		else if(GRAB_FIRST_PRESS_E) //����E��ʼ�洢��ʯ
		{
			grabTaskStructure.behaviourTime = 0;
			grabTaskStructure.behaviourStep++;
		}
	}
		/*�����ƽ̨��������X���򵽷ſ�ǰԤ��λ��*/
	else if(grabTaskStructure.behaviourStep == 1)
	{
		int32_t behaviour_ecdSet1=grabTaskStructure.liftoreMotor1.fixedEcd[LIFT_NORMAL] , behaviour_ecdSet2=grabTaskStructure.xplatMotor1.fixedEcd[PLAT_X_NORMAL];
		
		*liftoreExp = RAMP_float(behaviour_ecdSet1 , *liftoreExp , 8000/1000.0*GRAB_TASK_MS);
		*xplatExp=RAMP_float(behaviour_ecdSet2 , *xplatExp , 12000/1000.0*GRAB_TASK_MS);
		//grabTaskStructure.oreServo.angleSet = grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_STORE_ORE];//ץ�����Ƕȹ�һ
		
		if(JUDGE_ECD_LIFT1(behaviour_ecdSet1) && JUDGE_ECD_XPLAT(behaviour_ecdSet2) && grabTaskStructure.behaviourTime > 1500)
		{
			if(grabTaskStructure.take2exch_Sign == 1)//ֱ�ӵ��һ��ı�־
				grabTaskStructure.behaviourStep = 3;
			else if(grabTaskStructure.take2exch_Sign == 0)
				grabTaskStructure.behaviourStep++;
		}	
	}
	  /*ƽ̨Y����ǰ�죬��׼����ȡ��ֿ�ʯ��λ��*/
	else if(grabTaskStructure.behaviourStep == 2)
	{
		int32_t behaviour_ecdSet=grabTaskStructure.yplatMotor1.fixedEcd[PLAT_Y_STORE_ORE];
		
		*yplatExp=RAMP_float(behaviour_ecdSet , *yplatExp , 20000/1000.0*GRAB_TASK_MS);
		
		if(JUDGE_ECD_YPLAT1(behaviour_ecdSet))
			grabTaskStructure.behaviourStep++;
		
	}
		/*armbaseyת��������λ��*/
	else if(grabTaskStructure.behaviourStep == 3)
	{
		int32_t behaviour_ecdSet=grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_STORE_ORE];
		
		*armbaseyExp = RAMP_float(behaviour_ecdSet , *armbaseyExp , 9000/1000.0*GRAB_TASK_MS);
		
		if(JUDGE_ECD_ARMBASEP(behaviour_ecdSet))
		{
			if(grabTaskStructure.take2exch_Sign != 1)//����ֱ�ӵ��һ��ı�־ / �ǵĻ�ֱ�ӵȴ�����
			{
				grabTaskStructure.behaviourTime = 0;//ʱ�����㣬Ϊ��һ����׼��
				grabTaskStructure.behaviourStep++;
			}
		}
	}
//	/*���ת��ֱ�Կ��λ��*/
//	else if(grabTaskStructure.behaviourStep == 4)
//	{
//		if(grabTaskStructure.currentTarget == TARGET_EXCH_STA)//ȡ��
//			//grabTaskStructure.oreServo.angleSet = grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_DIRECT_EXCH];
//		else //�ſ�
//			grabTaskStructure.oreServo.angleSet = grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_DIRECT_STORE];
//		if(grabTaskStructure.behaviourTime > 1000) //��ʱ
//			grabTaskStructure.behaviourStep++;
//	}
		/*armmidpת���ſ�/ȡ��λ��*/
	else if(grabTaskStructure.behaviourStep == 5)
	{
		int32_t behaviour_ecdSet1 , behaviour_ecdSet2 = grabTaskStructure.armmidyMotor.fixedEcd[ARM_MID_Y_NORMAL];
		
		if(grabTaskStructure.currentTarget == TARGET_EXCH_STA)//ȡ��
			behaviour_ecdSet1 =grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_EXCH_ORE];
		else //�ſ�
			behaviour_ecdSet1 =grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_STORE_ORE];
		
		*armbasepExp = RAMP_float(behaviour_ecdSet1 , *armbasepExp , 8000/1000.0*GRAB_TASK_MS);
		*armmidpExp = RAMP_float(behaviour_ecdSet2 , *armmidpExp , 3000/1000.0*GRAB_TASK_MS);
		
		if(JUDGE_ECD_ARMBASEP(behaviour_ecdSet1) && JUDGE_ECD_ARMMIDY(behaviour_ecdSet2))
		{
			grabTaskStructure.behaviourTime = 0;
			grabTaskStructure.behaviourStep++;
		}
	}
	/*ƽ̨Y�����ջأ���ֱ�Կ�ֵ�λ��*/
	else if(grabTaskStructure.behaviourStep == 6)
	{
		int32_t behaviour_ecdSet=grabTaskStructure.yplatMotor1.fixedEcd[PLAT_Y_EXCH_ORE];
		
		*yplatExp=RAMP_float(behaviour_ecdSet , *yplatExp , 20000/1000.0*GRAB_TASK_MS);
		
		if(JUDGE_STUCK_YPLAT1 && grabTaskStructure.behaviourTime>2000 && GRAB_FIRST_PRESS_E)//��E
		{
			if(grabTaskStructure.currentTarget == TARGET_EXCH_STA)//ȡ��
				PUMP_ORE_ON;
			else //�ſ�
				PUMP_ORE_OFF;
			grabTaskStructure.behaviourTime = 0;
			grabTaskStructure.behaviourStep++;
		}
	}
	/*ƽ̨Y����ǰ��һ�ξ���*/
	else if(grabTaskStructure.behaviourStep == 7)
	{
		int32_t behaviour_ecdSet=grabTaskStructure.yplatMotor1.fixedEcd[PLAT_Y_FRONT_LITTLE_EXCH];
		
		if(grabTaskStructure.behaviourTime > 1000)// ��/�ر���ʱ1s
			*yplatExp=RAMP_float(behaviour_ecdSet , *yplatExp , 15000/1000.0*GRAB_TASK_MS);
		
		if(JUDGE_ECD_YPLAT1(behaviour_ecdSet))
		{
			if(grabTaskStructure.currentTarget == TARGET_EXCH_STA)//ȡ��
				grabBehaviorChange(grabTaskStructure.behaviorList + GRAB_EXCHANGE);
			else //�ſ󣬼���
				grabTaskStructure.behaviourStep++;
		}
	}
	
		/*ץȡ��Ϊ����*/
	else if(grabTaskStructure.behaviourStep == 8)
	{
		if(GRAB_FIRST_PRESS_E) //E ��������ȡ�󣬻ص���ͨģʽ
		{
			grabTaskStructure.collectedOre++;
			//basicTaskStructure.servoCam_Sign = SERVO_CAM_NORMAL;
			grabBehaviorChange(grabTaskStructure.behaviorList + GRAB_NORMAL);
		}
		else if(GRAB_FIRST_PRESS_R) //R ����ȡ��
			grabBehaviorChange(grabTaskStructure.behaviorList + GRAB_PREPARE);
	}
}

bool_t grabTakeBehaviourEnterCondition() {

  return FALSE;//��ֹ�Զ�����
}

bool_t grabTakeBehaviourOutCondition() {
  return FALSE;//��ֹ�Զ��˳�
}


//int32_t see_private[8];

//----------GRAB_EXCHANGE----------�һ�ģʽ
void grabExchangeBehaviourHandleFun(float *armbaseyExp, float *armbasepExp, float *armmidpExp, float *singlerotateExp, float *xplatExp, float *yplatExp, float *liftoreExp) {

  grabTaskStructure.behaviourTime += GRAB_TASK_MS;//��ʱ
	
	/*Step0Ԥ��*/
	if(grabTaskStructure.behaviourStep == 0)
	{
		//grabTaskStructure.oreServo.angleSet = grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_STORE_ORE];
		if(grabTaskStructure.behaviourTime > 1000)
		{
			grabTaskStructure.behaviourStep++;
		}
			
	}
	/*armbasep armmidp��λ*/
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
	/*armbasey������ת��ǰ��*/
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
	/*��е��armmidp��012���һ�Ԥ��λ��*/
	else if(grabTaskStructure.behaviourStep == 3)
	{
		int32_t behaviour_ecdSet2 ;
		behaviour_ecdSet2 = grabTaskStructure.armmidyMotor.fixedEcd[ARM_MID_Y_EXCH_LEVEL_01];
		
		//grabTaskStructure.oreServo.angleSet = grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_NORMAL];
		*armmidpExp = RAMP_float(behaviour_ecdSet2 , *armmidpExp , 3000/1000.0*GRAB_TASK_MS);
		
		if(JUDGE_ECD_ARMMIDY(behaviour_ecdSet2))
				grabTaskStructure.behaviourStep++;
	}
	/*��е��armbasep��012���һ�Ԥ��λ��*/
	else if(grabTaskStructure.behaviourStep == 4)
	{
		int32_t behaviour_ecdSet1;
		
		behaviour_ecdSet1 = grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_EXCH_LEVEL_01];
		
		//grabTaskStructure.oreServo.angleSet = grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_NORMAL];
		*armbasepExp = RAMP_float(behaviour_ecdSet1 , *armbasepExp , 6000/1000.0*GRAB_TASK_MS);
		
		if(JUDGE_ECD_ARMBASEP(behaviour_ecdSet1))//ѡ��һ��߶�����
		{
				grabTaskStructure.currentExchlevel = EXCH_LEVEL_0;
				grabTaskStructure.behaviourStep++;
		}
	}

	/*�����ִ�*/
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
		
		
		//�������ư˸���
		//                                                             Z
		if(IF_KEY_PRESSED_SHIFT && IF_KEY_PRESSED_F) //ƽ̨��
			behaviour_Tmp[0]-=15;
		else if(IF_KEY_PRESSED_F)  //ƽ̨��
			behaviour_Tmp[0]+=10;
		//                                                             X
		else if(IF_KEY_PRESSED_SHIFT && IF_KEY_PRESSED_G) //ƽ̨��
			behaviour_Tmp[1]-=15;
		else if(IF_KEY_PRESSED_G)  //ƽ̨ǰ
			behaviour_Tmp[1]+=15;
		//                                                             C
		else if(IF_KEY_PRESSED_SHIFT && IF_KEY_PRESSED_R) //ƽ̨��
			behaviour_Tmp[2]-=10;
		else if(IF_KEY_PRESSED_R)  //ƽ̨��
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
		
//����ֵ�޷�
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
		
		//���޷����ٴ���������
		for(int i=0 ; i<7 ; i++) behaviour_ecdSet[i]+=behaviour_Tmp[i];
		
//		//����޷����⴦��
//		if(behaviour_angleSet + behaviour_Tmp[7]*0.05 < grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_ALLEY_OPP])
//			behaviour_Tmp[7] = (grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_ALLEY_OPP] - behaviour_angleSet)/0.05;
//		else if(behaviour_angleSet + behaviour_Tmp[7]*0.05 > grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_DIRECT_GROUND])
//			behaviour_Tmp[7] = (grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_DIRECT_GROUND] - behaviour_angleSet)/0.05;
//		//���ݶ��ֵ
//		behaviour_angleSet += behaviour_Tmp[7]*0.05;
		
		
		*xplatExp = RAMP_float(behaviour_ecdSet[0] , *xplatExp , 10000/1000.0*GRAB_TASK_MS);
		*yplatExp = RAMP_float(behaviour_ecdSet[1] , *yplatExp , 10000/1000.0*GRAB_TASK_MS);
		*liftoreExp = RAMP_float(behaviour_ecdSet[2] , *liftoreExp , 5000/1000.0*GRAB_TASK_MS);
		*armbaseyExp = RAMP_float(behaviour_ecdSet[3] , *armbaseyExp , 3000/1000.0*GRAB_TASK_MS);
		*armbasepExp = RAMP_float(behaviour_ecdSet[4] , *armbasepExp , 6000/1000.0*GRAB_TASK_MS);
		*armmidpExp = RAMP_float(behaviour_ecdSet[5] , *armmidpExp , 1000/1000.0*GRAB_TASK_MS);
		*singlerotateExp = RAMP_float(behaviour_ecdSet[6] , *singlerotateExp , 1500/1000.0*GRAB_TASK_MS);
		//grabTaskStructure.oreServo.angleSet = behaviour_angleSet;
	
		if(GRAB_FIRST_PRESS_W)//W�л�Ĭ�ϸ߶�
		{
			for(int i=4 ; i<6 ; i++) behaviour_Tmp[i]=0;
			
			if(grabTaskStructure.currentExchlevel == EXCH_LEVEL_0) grabTaskStructure.currentExchlevel = EXCH_LEVEL_1;
			else if(grabTaskStructure.currentExchlevel == EXCH_LEVEL_1) grabTaskStructure.currentExchlevel = EXCH_LEVEL_2;
			else if(grabTaskStructure.currentExchlevel == EXCH_LEVEL_2) grabTaskStructure.currentExchlevel = EXCH_LEVEL_0;
		}
		//��E�����һ�,���ù�
		else if(GRAB_FIRST_PRESS_E && JUDGE_ECD_ARMMIDY(behaviour_ecdSet[5]))
		{
			for(int i=0 ; i<8 ; i++) behaviour_Tmp[i]=0;
			PUMP_ORE_OFF;//���ù�
			grabTaskStructure.behaviourStep++;
		}
	}
	/*��E�������ص���ͨģʽ*/
	else if(grabTaskStructure.behaviourStep == 6)
	{
		if(GRAB_FIRST_PRESS_E)
		{
			if(grabTaskStructure.take2exch_Sign == 1)//ֱ�ӵ��һ��ı�־
				grabTaskStructure.take2exch_Sign = 0;
			
			grabTaskStructure.collectedOre--;
			
			//basicTaskStructure.servoCam_Sign = SERVO_CAM_NORMAL;//�Զ��л�����ӽ�
			grabBehaviorChange(grabTaskStructure.behaviorList + GRAB_NORMAL);
		}
	}
}

bool_t grabExchangeBehaviourEnterCondition() {
  return FALSE;//��ֹ�Զ�����
}

bool_t grabExchangeBehaviourOutCondition() {
  return FALSE;//��ֹ�Զ��˳�
}

//----------GRAB_NORMAL----------
bool_t grabNormalBehaviourEnterCondition() {
  return TRUE; //�����Զ�����
}

bool_t grabNormalBehaviourOutCondition() {
  return FALSE; //�������Զ�����
}

void grabNormalBehaviourHandleFun(float *armbaseyExp, float *armbasepExp, float *armmidpExp, float *singlerotateExp, float *xplatExp, float *yplatExp, float *liftoreExp) 
{
	grabTaskStructure.behaviourTime += GRAB_TASK_MS;//��ʱ
	
		/* Step0Ԥ��*/
	if(grabTaskStructure.behaviourStep == 0)
	{
		PUMP_ORE_OFF;
		//grabTaskStructure.oreServo.angleSet=grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_NORMAL];
		
		if(grabTaskStructure.behaviourTime > 500)//Ԥ�������תʱ��
			grabTaskStructure.behaviourStep++;
	}
		/* צ����ת�����Ŀ��λ��*/
	else if(grabTaskStructure.behaviourStep == 1)
	{
		int32_t behaviour_ecdSet=grabTaskStructure.singlerotateMotor.fixedEcd[ROTATE_SINGLE_NORMAL];
		
		*singlerotateExp=RAMP_float(behaviour_ecdSet , *singlerotateExp , 5000/1000.0*GRAB_TASK_MS);
		
		if(JUDGE_ECD_PARALLElLMOTOR1(behaviour_ecdSet))
			grabTaskStructure.behaviourStep++;
	}
		/* armmidp�����Ŀ��λ��*/
	else if(grabTaskStructure.behaviourStep == 2)
	{
		int32_t behaviour_ecdSet1=grabTaskStructure.armmidyMotor.fixedEcd[ARM_MID_Y_NORMAL];
		
		*armmidpExp=RAMP_float(behaviour_ecdSet1 , *armmidpExp , 3000/1000.0*GRAB_TASK_MS);
		
		if(JUDGE_ECD_ARMMIDY(behaviour_ecdSet1))
			grabTaskStructure.behaviourStep++;
	}
		/* armbasep�����MF9025����Ŀ��λ��  armmidp�����Ŀ��λ��*/
	else if(grabTaskStructure.behaviourStep == 3)
	{
		int32_t behaviour_ecdSet1=grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_NORMAL];
		
		
		*armbasepExp=RAMP_float(behaviour_ecdSet1 , *armbasepExp , 8000/1000.0*GRAB_TASK_MS);
		
		if(JUDGE_ECD_ARMBASEP(behaviour_ecdSet1))
			grabTaskStructure.behaviourStep++;
	}
		/* armbasey�����Ŀ��λ��*/
//	else if(grabTaskStructure.behaviourStep == 4)
//	{
//		int32_t behaviour_ecdSet=grabTaskStructure.armbaseyMotor.fixedEcd[ARM_BASE_Y_NORMAL];
//		
//		*armbaseyExp=RAMP_float(behaviour_ecdSet , *armbaseyExp , 9000/1000.0*GRAB_TASK_MS);
//		
//		if(JUDGE_ECD_ARMBASEY(behaviour_ecdSet))
//			grabTaskStructure.behaviourStep++;
//	}
		/* ƽ̨X��������Ŀ��λ��*/
	else if(grabTaskStructure.behaviourStep == 5)
	{
		int32_t behaviour_ecdSet=grabTaskStructure.xplatMotor1.fixedEcd[PLAT_X_NORMAL];
		
		*xplatExp=RAMP_float(behaviour_ecdSet , *xplatExp , 15000/1000.0*GRAB_TASK_MS);
		
		if(JUDGE_ECD_XPLAT(behaviour_ecdSet))
			grabTaskStructure.behaviourStep++;
	}
		/* ƽ̨Y��������Ŀ��λ��*/		
	else if(grabTaskStructure.behaviourStep == 6)
	{
		int32_t behaviour_ecdSet=grabTaskStructure.yplatMotor1.fixedEcd[PLAT_Y_NORMAL];
		
		*yplatExp=RAMP_float(behaviour_ecdSet , *yplatExp , 20000/1000.0*GRAB_TASK_MS);
		
		if(JUDGE_ECD_YPLAT1(behaviour_ecdSet))
			grabTaskStructure.behaviourStep++;
	}
		/* ƽ̨���������Ŀ��λ��*/
	else if(grabTaskStructure.behaviourStep == 7)
	{
		int32_t behaviour_ecdSet=grabTaskStructure.liftoreMotor1.fixedEcd[LIFT_NORMAL];
		
		*liftoreExp=RAMP_float(behaviour_ecdSet , *liftoreExp , 7000/1000.0*GRAB_TASK_MS);
		
		if(JUDGE_ECD_LIFT1(behaviour_ecdSet))
			grabTaskStructure.behaviourStep++;
	}

	if(grabTaskStructure.inited == 0)//���³�ʼ��
		grabBehaviorChange(grabTaskStructure.behaviorList + GRAB_INIT);
	
}
