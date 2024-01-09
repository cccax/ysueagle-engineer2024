#include "basic_task.h"
#include "grab_task.h"
#include "chassis_task.h"
#include "messageBoard_B.h"

BasicCtrl_t basicTaskStructure;
extern grabCtrl_t grabTaskStructure;
void basicTaskInit(void);
void pumpTask(void);
void servoOreTask(void);
void basicFeedbackUpdate(void);
void viewTask(float *camtopExp);
void basicInit(void);
void basicPidCalc(void);
void basicCtrlChangeHandle(void);
void camTopMotor_Behaviour(float *camtopExp);

void basic_task(void *pvParameters) {
  portTickType currentTime;
	
	basicInit();
	
  basicTaskInit();
  vTaskDelay(5);
  currentTime = xTaskGetTickCount();

  while(1) {

    //����ʵ��ֵ���� Ҫ���� ��ֹ��һ�����������ֵ
    basicFeedbackUpdate();
		
    //���ģʽת������
    basicCtrlChangeHandle();
		
    //����ֵ�趨
		float *camtopExp;//ͼ�����
		
    //ͼ�������ת����
    if(basicTaskStructure.camtopMotor.mode == BASIC_MOTOR_POSITION)//λ��˫������
      camtopExp = &(basicTaskStructure.camtopMotor.totalEcdSet);
    else if(basicTaskStructure.camtopMotor.mode == BASIC_MOTOR_SPEED)//�ٶȵ�������
      camtopExp = &(basicTaskStructure.camtopMotor.speedSet);
    else if(basicTaskStructure.camtopMotor.mode == BASIC_MOTOR_RAW)//ԭʼ��������
      camtopExp = &(basicTaskStructure.camtopMotor.rawCmdCurrent);
		
		//ͼ������趨���ݴ���
		camTopMotor_Behaviour(camtopExp);
		
    //PID����
    basicPidCalc();
		
		
    //�ӽ��л�����
    viewTask(camtopExp);
		//��ʯ���	���ͺ���
		//servoOreTask();
		//������
		pumpTask();
    //����������
    buzzerCtrl();
    //����ң��������
    rcDataCopy(&(basicTaskStructure.rc.last));

    vTaskDelayUntil(&currentTime, BASIC_TASK_MS);
  }
}

void basicInit(void) 
{
  //��ȡң��������
  basicTaskStructure.rc.now = get_remote_control_point();

  //���������ʼ��
  motorInit(&(basicTaskStructure.camtopMotor.baseInf), 1);									//��е�۵ײ�yaw����

  basicTaskStructure.camtopMotor.lastMode = BASIC_MOTOR_RAW;
	
  //��׼λ��ֵ��ʼ�� XXX_RESET��Ϊ0
	//-----------------���--------------------------
  basicTaskStructure.camtopMotor.fixedEcd[CAM_TOP_MIN]  = 1100;//ok
	basicTaskStructure.camtopMotor.fixedEcd[CAM_TOP_NORMAL]  = 7500;//ok
	basicTaskStructure.camtopMotor.fixedEcd[CAM_TOP_GRAB]  = 3400;//ok
	basicTaskStructure.camtopMotor.fixedEcd[CAM_TOP_SIDE1]  = 7200;//ok
	basicTaskStructure.camtopMotor.fixedEcd[CAM_TOP_SIDE2]  = 3200;//ok
	basicTaskStructure.camtopMotor.fixedEcd[CAM_TOP_MAX]  = 7250;//ok
  //void PIDInit(PidTypeDef *pid,float kp,float ki,float kd,float ka,float max_out,float dead_band,float i_band,float max_input, float i_maxout, pid_mode_e model)
  //																																	   kp	,		ki	,	 kd	, ka, max_out	,dead_band,i_band	,max_input, i_maxout, pid_mode_e model
  PIDInit(&(basicTaskStructure.camtopMotor.pidParameter[INNER])			, 25	, 0	, 18, 0	, 13000		,		0	  	,	0	,			-1	,		3000	, SPEED);		//OK
  PIDInit(&(basicTaskStructure.camtopMotor.pidParameter[OUTER])			, 1	,0.05	, 2.1	, 0	, 10000		,		5			, 200	, 		-1	, 	4000			, SPEED);

  //����������ٶ��˲�
  KalmanCreate(&(basicTaskStructure.camtopMotor.klmFiller) , 1, 40);//
	
}

void basicFeedbackUpdate() {
  //�ٶȸ���
  basicTaskStructure.camtopMotor.filterSpeed = KalmanFilter(&(basicTaskStructure.camtopMotor.klmFiller), basicTaskStructure.camtopMotor.baseInf.real_speed_rpm);			//

	//��һ��λ�õĸ���
  basicTaskStructure.camtopMotor.last_totalEcd = basicTaskStructure.camtopMotor.totalEcd;		

  //λ�ø���
  basicTaskStructure.camtopMotor.totalEcd = basicTaskStructure.camtopMotor.baseInf.rotor_ecd;		

}

//���α�������
int16_t setspeed_basic, realspeed_basic,setEcd_basic,realEcd_basic;

void basicPidCalc() { //PID���㺯��
	
  //��е�۵���yaw����
  if(basicTaskStructure.camtopMotor.mode == BASIC_MOTOR_RAW) //ԭʼ��������
    basicTaskStructure.camtopMotor.currentSet = basicTaskStructure.camtopMotor.rawCmdCurrent;
  else if(basicTaskStructure.camtopMotor.mode == BASIC_MOTOR_SPEED) { //�ٶȵ�������
    PID_Calc(&(basicTaskStructure.camtopMotor.pidParameter[INNER]), basicTaskStructure.camtopMotor.filterSpeed, basicTaskStructure.camtopMotor.speedSet);
    basicTaskStructure.camtopMotor.currentSet = basicTaskStructure.camtopMotor.pidParameter[INNER].output;
  } else if(basicTaskStructure.camtopMotor.mode == BASIC_MOTOR_POSITION) { //λ��˫������
    PID_Calc(&(basicTaskStructure.camtopMotor.pidParameter[OUTER]), basicTaskStructure.camtopMotor.totalEcd, basicTaskStructure.camtopMotor.totalEcdSet);
	    PID_Calc(&(basicTaskStructure.camtopMotor.pidParameter[INNER]), basicTaskStructure.camtopMotor.filterSpeed, basicTaskStructure.camtopMotor.pidParameter[OUTER].output);
    basicTaskStructure.camtopMotor.currentSet = basicTaskStructure.camtopMotor.pidParameter[INNER].output;
  } else
    basicTaskStructure.camtopMotor.currentSet = 0;
	
  basicTaskStructure.camtopMotor.baseInf.given_current = (s16)(basicTaskStructure.camtopMotor.currentSet);

	//���� 
	#define debug_basicName camtopMotor
  setspeed_basic = basicTaskStructure.debug_basicName.speedSet;
  realspeed_basic = basicTaskStructure.debug_basicName.filterSpeed;
	setEcd_basic = basicTaskStructure.debug_basicName.totalEcdSet;
	realEcd_basic = basicTaskStructure.debug_basicName.totalEcd;
	
}

void basicCtrlChangeHandle(void){
	 //��е�۵���yaw��
  if(basicTaskStructure.camtopMotor.lastMode != basicTaskStructure.camtopMotor.mode) {
    if(basicTaskStructure.camtopMotor.mode == BASIC_MOTOR_POSITION)//λ��˫������
      basicTaskStructure.camtopMotor.totalEcdSet = basicTaskStructure.camtopMotor.totalEcd;
    else if(basicTaskStructure.camtopMotor.mode == BASIC_MOTOR_SPEED)//�ٶȵ�������
      basicTaskStructure.camtopMotor.speedSet = 0;
    else if(basicTaskStructure.camtopMotor.mode == BASIC_MOTOR_RAW)//ԭʼ��������
      basicTaskStructure.camtopMotor.rawCmdCurrent = 0;

    basicTaskStructure.camtopMotor.lastMode = basicTaskStructure.camtopMotor.mode;
  }
}

float midTmp_basic;
void camTopMotor_Behaviour(float *camtopExp)
{
	#ifdef DEBUG_BASIC
	basicTaskStructure.camtopMotor.mode = BASIC_MOTOR_POSITION; //ͼ�����ģʽadd
	midTmp_basic+=300.0*(RC_CH3_LUD_OFFSET / 660.0);
	*camtopExp = midTmp_basic;
	//*camtopExp = RAMP_float(basicTaskStructure.camtopMotor.fixedEcd[CAM_TOP_NORMAL] , *camtopExp , 3000/1000.0*BASIC_TASK_MS);	
	#endif
}

void basicTaskInit(void) {
  //��ȡң��������
  basicTaskStructure.rc.now = get_remote_control_point();

}

void viewTask(float *camtopExp) {
  if(basicTaskStructure.servoCam_Sign == SERVO_CAM_NORMAL) {
    
		setServoAngle(CAM_SERVO, CAM_ANGLE_NORMAL);
		*camtopExp  = basicTaskStructure.camtopMotor.fixedEcd[CAM_TOP_NORMAL];
		
		chassisTaskStructure.chassis_STA = SERVO_CAM_NORMAL; //���̷���ģʽ����
		
		basicTaskStructure.servoCam_Sign = SERVO_CAM_NONE; //����
  }
	else if(basicTaskStructure.servoCam_Sign == SERVO_CAM_GRAB) {
    setServoAngle(CAM_SERVO, CAM_ANGLE_GRAB);
		
		chassisTaskStructure.chassis_STA = SERVO_CAM_GRAB;//���̷���ģʽ����
		*camtopExp  = basicTaskStructure.camtopMotor.fixedEcd[CAM_TOP_GRAB];
		
		basicTaskStructure.servoCam_Sign = SERVO_CAM_NONE;//����
	}
	if(IF_KEY_PRESSED_SHIFT && BASIC_FIRST_PRESS_Q) //Ctrl + shift + Q ǿ���л�ͼ������͵��̷���
	{
		buzzerOn(50, 1, 40);
		basicTaskStructure.camSwitch = (basicTaskStructure.camSwitch+1) % 2; //camSwitch 0 1�л�
		
		chassisTaskStructure.chassis_STA = basicTaskStructure.camSwitch?SERVO_CAM_NORMAL:SERVO_CAM_GRAB;
		setServoAngle(CAM_SERVO,basicTaskStructure.camSwitch?CAM_ANGLE_NORMAL:CAM_ANGLE_GRAB); //ѡ����ʽ
	}
		*camtopExp = basicTaskStructure.camSwitch?basicTaskStructure.camtopMotor.fixedEcd[CAM_TOP_NORMAL]:basicTaskStructure.camtopMotor.fixedEcd[CAM_TOP_GRAB];

}

void pumpTask(void)
{
	if(IF_KEY_PRESSED_SHIFT && BASIC_FIRST_PRESS_A)
		PUMP_ORE_OFF;
	else if(BASIC_FIRST_PRESS_A)
		PUMP_ORE_ON;
}

void servoOreTask(void)
{
	//B��ȡ�������Ʒ���
	//Board_B_ServoAngleSendQueue(CAN2,grabTaskStructure.oreServo.angleSet);
}
