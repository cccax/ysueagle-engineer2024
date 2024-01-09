#ifndef CHASSIS_TASK
#define CHASSIS_TASK
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "chassis_behaviour.h"
#include "robot.h"
#include "Detect_Task.h"
#include "remote_control.h"
#include "math.h"
#include "pid.h"
#include "motor.h"
#include "kalman.h"
#include "user_lib.h"
#include "can.h"
#include "INS_task.h"
#include "basic_task.h"
#include "stm32f4xx.h"

#define CHASSIS_CANBUS_SEND_HEADER14 0x200
#define CHASSIS_CANBUS_SEND_HEADER58 0x1FF

#define CHASSIS_FIRST_PRESS_W		!IF_LAST_KEY_PRESSED_W(chassisTaskStructure.rc.last) && IF_KEY_PRESSED_W
#define CHASSIS_FIRST_PRESS_S		!IF_LAST_KEY_PRESSED_S(chassisTaskStructure.rc.last) && IF_KEY_PRESSED_S
#define CHASSIS_FIRST_PRESS_A		!IF_LAST_KEY_PRESSED_A(chassisTaskStructure.rc.last) && IF_KEY_PRESSED_A
#define CHASSIS_FIRST_PRESS_D		!IF_LAST_KEY_PRESSED_D(chassisTaskStructure.rc.last) && IF_KEY_PRESSED_D
#define CHASSIS_FIRST_PRESS_Q		!IF_LAST_KEY_PRESSED_Q(chassisTaskStructure.rc.last) && IF_KEY_PRESSED_Q
#define CHASSIS_FIRST_PRESS_E		!IF_LAST_KEY_PRESSED_E(chassisTaskStructure.rc.last) && IF_KEY_PRESSED_E
#define CHASSIS_FIRST_PRESS_G		!IF_LAST_KEY_PRESSED_G(chassisTaskStructure.rc.last) && IF_KEY_PRESSED_G
#define CHASSIS_FIRST_PRESS_X		!IF_LAST_KEY_PRESSED_X(chassisTaskStructure.rc.last) && IF_KEY_PRESSED_X
#define CHASSIS_FIRST_PRESS_Z		!IF_LAST_KEY_PRESSED_Z(chassisTaskStructure.rc.last) && IF_KEY_PRESSED_Z
#define CHASSIS_FIRST_PRESS_C		!IF_LAST_KEY_PRESSED_C(chassisTaskStructure.rc.last) && IF_KEY_PRESSED_C
#define CHASSIS_FIRST_PRESS_B		!IF_LAST_KEY_PRESSED_B(chassisTaskStructure.rc.last) && IF_KEY_PRESSED_B
#define CHASSIS_FIRST_PRESS_V		!IF_LAST_KEY_PRESSED_V(chassisTaskStructure.rc.last) && IF_KEY_PRESSED_V
#define CHASSIS_FIRST_PRESS_F		!IF_LAST_KEY_PRESSED_F(chassisTaskStructure.rc.last) && IF_KEY_PRESSED_F
#define CHASSIS_FIRST_PRESS_R		!IF_LAST_KEY_PRESSED_R(chassisTaskStructure.rc.last) && IF_KEY_PRESSED_R
#define CHASSIS_FIRST_PRESS_CTRL	!IF_LAST_KEY_PRESSED_CTRL(chassisTaskStructure.rc.last) && IF_KEY_PRESSED_CTRL
#define CHASSIS_FIRST_PRESS_SHIFT	!IF_LAST_KEY_PRESSED_SHIFT(chassisTaskStructure.rc.last) && IF_KEY_PRESSED_SHIFT

#define CHASSIS_FIRST_S1_UP			IF_RC_SW1_UP && (!(chassisTaskStructure.rc.last.rc.s[0] == 1))
#define CHASSIS_FIRST_S1_NOT_UP		(!rc_ctrl.rc.s[0] == RC_SW_UP) && ((chassisTaskStructure.rc.last.rc.s[0] == 1))
#define CHASSIS_FIRST_S1_DOWN		IF_RC_SW1_DOWN && (!(chassisTaskStructure.rc.last.rc.s[0] == 2))
#define CHASSIS_FIRST_S1_MID		IF_RC_SW1_MID && (!(chassisTaskStructure.rc.last.rc.s[0] == 3))
#define CHASSIS_FIRST_S2_UP			IF_RC_SW2_UP && (!(chassisTaskStructure.rc.last.rc.s[1] == 1))
#define CHASSIS_FIRST_S2_DOWN		IF_RC_SW2_DOWN && (!(chassisTaskStructure.rc.last.rc.s[1] == 2))
#define CHASSIS_FIRST_S2_MID		IF_RC_SW2_MID && (!(chassisTaskStructure.rc.last.rc.s[1] == 3))

#define CHASSIS_FIRST_MOUSE_X_STOP	MOUSE_X_MOVE_SPEED == 0 && chassisTaskStructure.rc.last.mouse.x != 0
#define CHASSIS_FIRST_MOUSE_Y_STOP	MOUSE_Y_MOVE_SPEED == 0 && chassisTaskStructure.rc.last.mouse.y != 0
#define CHASSIS_FIRST_CH0_MID		RC_CH0_RLR_OFFSET == 0 && chassisTaskStructure.rc.last.rc.ch[0] != 0
#define CHASSIS_FIRST_CH1_MID		RC_CH1_RUD_OFFSET == 0 && chassisTaskStructure.rc.last.rc.ch[1] != 0
#define CHASSIS_FIRST_CH2_MID		RC_CH2_LLR_OFFSET == 0 && chassisTaskStructure.rc.last.rc.ch[2] != 0
#define CHASSIS_FIRST_CH3_MID		RC_CH3_LUD_OFFSET == 0 && chassisTaskStructure.rc.last.rc.ch[3] != 0
#define CHASSIS_FIRST_CH4_MID		RC_CH4_ROT_OFFSET == 0 && chassisTaskStructure.rc.last.rc.ch[4] != 0

#define CHASSIS_MOTOR_REDUCTION_RATIO 19

#define CHASSIS_CM1	0
#define CHASSIS_CM2	1
#define CHASSIS_CM3	2
#define CHASSIS_CM4	3
//#define ORE_ROTATE_RM1 4
//#define ORE_ROTATE_RM2 5
//#define ORE_LIFT_LM 6

#ifndef INNER     //速度环PID
#define INNER 0
#endif

#ifndef OUTER      //位置环PID
#define OUTER 1
#endif

#define CHASSIS_NORMAL_MAX_VX 440 //MAX 450
#define CHASSIS_NORMAL_MAX_VY 440
#define CHASSIS_NORMAL_MAX_VZ 200

#define CHASSIS_CONTROL_TIME 0.04 //底盘任务控制间隔

typedef enum {
  VX = 0,
  VY,
  VZ
} chassis_speed_e;

typedef enum {
  CHASSIS_ANGLE = 0,	//角度控制（和速度无实际区别）
  CHASSIS_SPEED,		//速度控制
  CHASSIS_RAW,		//电流控制
  //ORE_POSITION,  //矿仓位置控制
} chassis_mode_e;

typedef struct {
  PidTypeDef pidParameter;
  fp32 rawCmdCurrent;
  fp32 currentSet;
  motor_measure_t baseInf;
  fp32 filterSpeed;
  extKalman_t klmFiller;
  float speedSet;
} chassis_motor_t;

typedef enum {
  CHASSIS_DEBUG = 0,
  CHASSIS_ZERO_FORCE,//无力
  CHASSIS_INIT, 	//初始化模式,
  CHASSIS_SLEEP,	//静止底盘四个电机速度期望为0.
  CHASSIS_ALONE,	//独立模式,平移旋转都直接受遥控器控制
  CHASSIS_BEHAVOUR_LENGTH,
} chassis_behaviour_e;

typedef struct {
  chassis_behaviour_e num;
  chassis_mode_e mode;
//	  chassis_mode_e ore_rotate_Mode;
//		chassis_mode_e ore_lift_Mode;

  bool_t (*enterBehaviorCondition)(void);
  bool_t (*outBehaviorCondition)(void);
  void (*enterBehaviorFun)(void);
  void (*outBehaviorFun)(void);
  void (*behaviorHandleFun)(float* vx, float* vy, float* vz, float* raw);
} chassis_behaviour_t;

typedef struct {
  fp32 relativeSpeedSet[3];
  fp32 chassisSpeedSet[3];
  fp32 relativeSpeedMax[3];

  struct {
    const RC_ctrl_t *now; //遥控器当前值
    RC_ctrl_t last; //遥控器上一次的值
  } rc;
  chassis_mode_e mode;
  chassis_mode_e lastMode;
  chassis_motor_t motor[4];
//		x_platmotor_t xplatMotor;
//		x_platmotor_t orerotateMotor2;
  //lift_oremotor_t oreliftMotor;
  fp32* relativeAngle; //对地角度
  fp32 relativeAngleSet; //对地角度期望
  fp32 maxRelativeAngle; //最大对地角度
  fp32 minRelativeAngle; //最小对地角度
  PidTypeDef anglePidParameter; //对地角度pid

  chassis_behaviour_t behaviorList[CHASSIS_BEHAVOUR_LENGTH];
  chassis_behaviour_t* behaviorNow;
  u8 behaviourStep;
	
	cam_servo_t chassis_STA; //底盘方向根据前后当头更改
} ChassisCtrl_t;

void chassisTaskInit(void);
void chassisTask(void *pvParameters);

void chassisBehaviourInit(chassis_behaviour_t *initBehavior, chassis_behaviour_e num, chassis_mode_e mode,
                          bool_t (*enterBehaviorCondition)(void),
                          bool_t (*outBehaviorCondition)(void),
                          void (*enterBehaviorFun)(void),
                          void (*outBehaviorFun)(void),
                          void (*behaviorHandleFun)(float* vx, float* vy, float* vz, float* raw ));
void chassisBehaviorSelect(void);
void chassisBehaviorChange(chassis_behaviour_t *next);
void chassisModeChangeHandle(void);
void chassisFeedBackUpdate(void);
void chassisSpeedLimit(void);
void chassisPidCalc(void);
void chassisCanbuscCtrlMotors(s16 cm1Current, s16 cm2Current, s16 cm3Current, s16 cm4Current);

extern ChassisCtrl_t chassisTaskStructure;

#endif
