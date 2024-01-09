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

    //更新实际值数据 要先做 防止下一步传入错误数值
    basicFeedbackUpdate();
		
    //电机模式转换处理
    basicCtrlChangeHandle();
		
    //期望值设定
		float *camtopExp;//图传电机
		
    //图传电机旋转期望
    if(basicTaskStructure.camtopMotor.mode == BASIC_MOTOR_POSITION)//位置双环控制
      camtopExp = &(basicTaskStructure.camtopMotor.totalEcdSet);
    else if(basicTaskStructure.camtopMotor.mode == BASIC_MOTOR_SPEED)//速度单环控制
      camtopExp = &(basicTaskStructure.camtopMotor.speedSet);
    else if(basicTaskStructure.camtopMotor.mode == BASIC_MOTOR_RAW)//原始电流控制
      camtopExp = &(basicTaskStructure.camtopMotor.rawCmdCurrent);
		
		//图传电机设定数据处理
		camTopMotor_Behaviour(camtopExp);
		
    //PID计算
    basicPidCalc();
		
		
    //视角切换任务
    viewTask(camtopExp);
		//矿石舵机	发送函数
		//servoOreTask();
		//泵任务
		pumpTask();
    //蜂鸣器控制
    buzzerCtrl();
    //更新遥控器参数
    rcDataCopy(&(basicTaskStructure.rc.last));

    vTaskDelayUntil(&currentTime, BASIC_TASK_MS);
  }
}

void basicInit(void) 
{
  //获取遥控器数据
  basicTaskStructure.rc.now = get_remote_control_point();

  //电机参数初始化
  motorInit(&(basicTaskStructure.camtopMotor.baseInf), 1);									//机械臂底部yaw轴电机

  basicTaskStructure.camtopMotor.lastMode = BASIC_MOTOR_RAW;
	
  //标准位置值初始化 XXX_RESET均为0
	//-----------------电机--------------------------
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

  //卡尔曼电机速度滤波
  KalmanCreate(&(basicTaskStructure.camtopMotor.klmFiller) , 1, 40);//
	
}

void basicFeedbackUpdate() {
  //速度更新
  basicTaskStructure.camtopMotor.filterSpeed = KalmanFilter(&(basicTaskStructure.camtopMotor.klmFiller), basicTaskStructure.camtopMotor.baseInf.real_speed_rpm);			//

	//上一次位置的更新
  basicTaskStructure.camtopMotor.last_totalEcd = basicTaskStructure.camtopMotor.totalEcd;		

  //位置更新
  basicTaskStructure.camtopMotor.totalEcd = basicTaskStructure.camtopMotor.baseInf.rotor_ecd;		

}

//调参变量定义
int16_t setspeed_basic, realspeed_basic,setEcd_basic,realEcd_basic;

void basicPidCalc() { //PID计算函数
	
  //机械臂底座yaw轴电机
  if(basicTaskStructure.camtopMotor.mode == BASIC_MOTOR_RAW) //原始电流控制
    basicTaskStructure.camtopMotor.currentSet = basicTaskStructure.camtopMotor.rawCmdCurrent;
  else if(basicTaskStructure.camtopMotor.mode == BASIC_MOTOR_SPEED) { //速度单环控制
    PID_Calc(&(basicTaskStructure.camtopMotor.pidParameter[INNER]), basicTaskStructure.camtopMotor.filterSpeed, basicTaskStructure.camtopMotor.speedSet);
    basicTaskStructure.camtopMotor.currentSet = basicTaskStructure.camtopMotor.pidParameter[INNER].output;
  } else if(basicTaskStructure.camtopMotor.mode == BASIC_MOTOR_POSITION) { //位置双环控制
    PID_Calc(&(basicTaskStructure.camtopMotor.pidParameter[OUTER]), basicTaskStructure.camtopMotor.totalEcd, basicTaskStructure.camtopMotor.totalEcdSet);
	    PID_Calc(&(basicTaskStructure.camtopMotor.pidParameter[INNER]), basicTaskStructure.camtopMotor.filterSpeed, basicTaskStructure.camtopMotor.pidParameter[OUTER].output);
    basicTaskStructure.camtopMotor.currentSet = basicTaskStructure.camtopMotor.pidParameter[INNER].output;
  } else
    basicTaskStructure.camtopMotor.currentSet = 0;
	
  basicTaskStructure.camtopMotor.baseInf.given_current = (s16)(basicTaskStructure.camtopMotor.currentSet);

	//调参 
	#define debug_basicName camtopMotor
  setspeed_basic = basicTaskStructure.debug_basicName.speedSet;
  realspeed_basic = basicTaskStructure.debug_basicName.filterSpeed;
	setEcd_basic = basicTaskStructure.debug_basicName.totalEcdSet;
	realEcd_basic = basicTaskStructure.debug_basicName.totalEcd;
	
}

void basicCtrlChangeHandle(void){
	 //机械臂底座yaw轴
  if(basicTaskStructure.camtopMotor.lastMode != basicTaskStructure.camtopMotor.mode) {
    if(basicTaskStructure.camtopMotor.mode == BASIC_MOTOR_POSITION)//位置双环控制
      basicTaskStructure.camtopMotor.totalEcdSet = basicTaskStructure.camtopMotor.totalEcd;
    else if(basicTaskStructure.camtopMotor.mode == BASIC_MOTOR_SPEED)//速度单环控制
      basicTaskStructure.camtopMotor.speedSet = 0;
    else if(basicTaskStructure.camtopMotor.mode == BASIC_MOTOR_RAW)//原始电流控制
      basicTaskStructure.camtopMotor.rawCmdCurrent = 0;

    basicTaskStructure.camtopMotor.lastMode = basicTaskStructure.camtopMotor.mode;
  }
}

float midTmp_basic;
void camTopMotor_Behaviour(float *camtopExp)
{
	#ifdef DEBUG_BASIC
	basicTaskStructure.camtopMotor.mode = BASIC_MOTOR_POSITION; //图传电机模式add
	midTmp_basic+=300.0*(RC_CH3_LUD_OFFSET / 660.0);
	*camtopExp = midTmp_basic;
	//*camtopExp = RAMP_float(basicTaskStructure.camtopMotor.fixedEcd[CAM_TOP_NORMAL] , *camtopExp , 3000/1000.0*BASIC_TASK_MS);	
	#endif
}

void basicTaskInit(void) {
  //获取遥控器数据
  basicTaskStructure.rc.now = get_remote_control_point();

}

void viewTask(float *camtopExp) {
  if(basicTaskStructure.servoCam_Sign == SERVO_CAM_NORMAL) {
    
		setServoAngle(CAM_SERVO, CAM_ANGLE_NORMAL);
		*camtopExp  = basicTaskStructure.camtopMotor.fixedEcd[CAM_TOP_NORMAL];
		
		chassisTaskStructure.chassis_STA = SERVO_CAM_NORMAL; //底盘方向模式更改
		
		basicTaskStructure.servoCam_Sign = SERVO_CAM_NONE; //归零
  }
	else if(basicTaskStructure.servoCam_Sign == SERVO_CAM_GRAB) {
    setServoAngle(CAM_SERVO, CAM_ANGLE_GRAB);
		
		chassisTaskStructure.chassis_STA = SERVO_CAM_GRAB;//底盘方向模式更改
		*camtopExp  = basicTaskStructure.camtopMotor.fixedEcd[CAM_TOP_GRAB];
		
		basicTaskStructure.servoCam_Sign = SERVO_CAM_NONE;//归零
	}
	if(IF_KEY_PRESSED_SHIFT && BASIC_FIRST_PRESS_Q) //Ctrl + shift + Q 强制切换图传方向和底盘方向
	{
		buzzerOn(50, 1, 40);
		basicTaskStructure.camSwitch = (basicTaskStructure.camSwitch+1) % 2; //camSwitch 0 1切换
		
		chassisTaskStructure.chassis_STA = basicTaskStructure.camSwitch?SERVO_CAM_NORMAL:SERVO_CAM_GRAB;
		setServoAngle(CAM_SERVO,basicTaskStructure.camSwitch?CAM_ANGLE_NORMAL:CAM_ANGLE_GRAB); //选择表达式
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
	//B板取矿舵机控制发送
	//Board_B_ServoAngleSendQueue(CAN2,grabTaskStructure.oreServo.angleSet);
}
