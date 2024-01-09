#ifndef GRAB_TASK_H
#define GRAB_TASK_H
#include "stm32f4xx.h"
#include "remote_control.h"
#include "motor.h"
#include "PID.h"
#include "kalman.h"
#include "user_lib.h"
#include "main.h"
#include "grab_behaviour.h"
#include "robot.h"
#include "detect_task.h"
#include "can.h"
#include "chassis_task.h"

#define GRAB_CANBUS_SEND_HEADER14 0x200
#define GRAB_CANBUS_SEND_HEADER58 0x1FF
#define GRAB_CANBUS_SEND_MF9025 0x140

#define RAWDATA_LENGTH 465  //微机的舵机原始数据数组长度

#ifndef INNER     //速度环PID
#define INNER 0
#endif

#ifndef OUTER      //位置环PID
#define OUTER 1
#endif

#define GRAB_FIRST_PRESS_W		(!IF_LAST_KEY_PRESSED_W(grabTaskStructure.rc.last) && IF_KEY_PRESSED_W)
#define GRAB_FIRST_PRESS_S		(!IF_LAST_KEY_PRESSED_S(grabTaskStructure.rc.last) && IF_KEY_PRESSED_S)
#define GRAB_FIRST_PRESS_A		(!IF_LAST_KEY_PRESSED_A(grabTaskStructure.rc.last) && IF_KEY_PRESSED_A)
#define GRAB_FIRST_PRESS_D		(!IF_LAST_KEY_PRESSED_D(grabTaskStructure.rc.last) && IF_KEY_PRESSED_D)
#define GRAB_FIRST_PRESS_Q		(!IF_LAST_KEY_PRESSED_Q(grabTaskStructure.rc.last) && IF_KEY_PRESSED_Q)
#define GRAB_FIRST_PRESS_E		(!IF_LAST_KEY_PRESSED_E(grabTaskStructure.rc.last) && IF_KEY_PRESSED_E)
#define GRAB_FIRST_PRESS_G		(!IF_LAST_KEY_PRESSED_G(grabTaskStructure.rc.last) && IF_KEY_PRESSED_G)
#define GRAB_FIRST_PRESS_X		(!IF_LAST_KEY_PRESSED_X(grabTaskStructure.rc.last) && IF_KEY_PRESSED_X)
#define GRAB_FIRST_PRESS_Z		(!IF_LAST_KEY_PRESSED_Z(grabTaskStructure.rc.last) && IF_KEY_PRESSED_Z)
#define GRAB_FIRST_PRESS_C		(!IF_LAST_KEY_PRESSED_C(grabTaskStructure.rc.last) && IF_KEY_PRESSED_C)
#define GRAB_FIRST_PRESS_B		(!IF_LAST_KEY_PRESSED_B(grabTaskStructure.rc.last) && IF_KEY_PRESSED_B)
#define GRAB_FIRST_PRESS_V		(!IF_LAST_KEY_PRESSED_V(grabTaskStructure.rc.last) && IF_KEY_PRESSED_V)
#define GRAB_FIRST_PRESS_F		(!IF_LAST_KEY_PRESSED_F(grabTaskStructure.rc.last) && IF_KEY_PRESSED_F)
#define GRAB_FIRST_PRESS_R		(!IF_LAST_KEY_PRESSED_R(grabTaskStructure.rc.last) && IF_KEY_PRESSED_R)
#define GRAB_FIRST_PRESS_CTRL	(!IF_LAST_KEY_PRESSED_CTRL(grabTaskStructure.rc.last) && IF_KEY_PRESSED_CTRL)
#define GRAB_FIRST_PRESS_SHIFT	(!IF_LAST_KEY_PRESSED_SHIFT(grabTaskStructure.rc.last) && IF_KEY_PRESSED_SHIFT)

#define GRAB_FIRST_S1_UP			(IF_RC_SW1_UP && (grabTaskStructure.rc.last.rc.s[0] != 1))
#define GRAB_FIRST_S1_DOWN			(IF_RC_SW1_DOWN && (grabTaskStructure.rc.last.rc.s[0] != 2))
#define GRAB_FIRST_S1_MID			(IF_RC_SW1_MID && (grabTaskStructure.rc.last.rc.s[0] != 3))

#define GRAB_FIRST_S2_UP			(IF_RC_SW2_UP && (grabTaskStructure.rc.last.rc.s[1] != 1))
#define GRAB_FIRST_S2_DOWN			(IF_RC_SW2_DOWN && (grabTaskStructure.rc.last.rc.s[1] != 2))
#define GRAB_FIRST_S2_MID			(IF_RC_SW2_MID && (grabTaskStructure.rc.last.rc.s[1] != 3))

#define GRAB_FIRST_MOUSE_X_STOP	MOUSE_X_MOVE_SPEED == 0 && grabTaskStructure.rc.last.mouse.x != 0
#define GRAB_FIRST_MOUSE_Y_STOP	MOUSE_Y_MOVE_SPEED == 0 && grabTaskStructure.rc.last.mouse.y != 0
#define GRAB_FIRST_CH0_MID		RC_CH0_RLR_OFFSET == 0 && grabTaskStructure.rc.last.rc.ch[0] != 0
#define GRAB_FIRST_CH1_MID		RC_CH1_RUD_OFFSET == 0 && grabTaskStructure.rc.last.rc.ch[1] != 0
#define GRAB_FIRST_CH2_MID		RC_CH2_LLR_OFFSET == 0 && grabTaskStructure.rc.last.rc.ch[2] != 0
#define GRAB_FIRST_CH3_MID		RC_CH3_LUD_OFFSET == 0 && grabTaskStructure.rc.last.rc.ch[3] != 0

#define GRAB_FIRST_PRESS_LMB IF_MOUSE_PRESSED_LEFT && !grabTaskStructure.rc.last.mouse.press_l
#define GRAB_FIRST_PRESS_RMB IF_MOUSE_PRESSED_RIGH && !grabTaskStructure.rc.last.mouse.press_r

typedef enum {
  GRAB_MOTOR_POSITION = 0,
  GRAB_MOTOR_SPEED,
  GRAB_MOTOR_RAW,
} grab_motor_mode_e;

//-----------------------------Start:电机结构体声明区---------------------------------------//
////----------机械臂基座yaw轴旋转电机----------  Modified 

//typedef enum {           //enum存储固定位置变量下标的枚举型
//  ARM_BASE_Y_RESET = 0,			//
//	ARM_BASE_Y_NORMAL,            //常态，直对前方
//  ARM_BASE_Y_SMALL_ISLAND,			//小资源岛
//  ARM_BASE_Y_BIG_ISLAND,				//大资源岛
//	ARM_BASE_Y_ALLEY_OPP,				//空接
//	ARM_BASE_Y_STORE_ORE,        //转到直对矿仓
//  ARM_BASE_Y_EXCH_LEVEL_012,				//兑换区等级0-2 
//	ARM_BASE_Y_EXCH_LEVEL_THREE,    
//	ARM_BASE_Y_EXCH_LEVEL_FOUR,    
//	ARM_BASE_Y_MAX,
//  ARM_BASE_Y_STICKMODE_LENGTH,
//} arm_base_ymotor_mode_e; 

//typedef struct {
//  motor_measure_t baseInf;
//  PidTypeDef pidParameter[2];
//  float rawCmdCurrent;
//  float currentSet;
//  extKalman_t klmFiller;
//  float filterSpeed;
//  float speedSet;
//  float totalEcd;//相对上电时的编码器步数
//	float last_totalEcd;
//  float totalEcdSet;
//	int32_t Stucktime;   //电机堵转时间
//  int32_t fixedEcd[ARM_BASE_Y_STICKMODE_LENGTH];

//  grab_motor_mode_e mode;
//  grab_motor_mode_e lastMode;
//} arm_base_ymotor_t;



//----------新增：机械臂基座pitch轴旋转电机(MF9025)----------
//----修改：M3508
typedef enum {           //enum存储固定位置变量下标的枚举型
  ARM_BASE_P_RESET = 0,			//最低点，与底座紧密接触
	ARM_BASE_P_MIN,            
	ARM_BASE_P_NORMAL,        //常态，底部大臂水平放置
  ARM_BASE_P_SMALL_ISLAND,			//小资源岛
  ARM_BASE_P_BIG_ISLAND,				//大资源岛
	ARM_BASE_P_ALLEY_OPP,				//空接
	ARM_BASE_P_STORE_ORE,        //放矿
	ARM_BASE_P_STORE_START,
	ARM_BASE_P_EXCH_ORE,             //矿仓取矿
	ARM_BASE_P_EXCH_LEVEL_01,
	ARM_BASE_P_EXCH_LEVEL_12,
  ARM_BASE_P_EXCH_LEVEL_24,				//兑换区等级0-2
  ARM_BASE_P_STICKMODE_LENGTH,
} arm_base_pmotor_mode_e; 

typedef struct {
	uint16_t ecd;
	int32_t relativeEcd;
	int16_t speed;
	int16_t real_current;
	int16_t given_current;
	uint8_t temperate;
	uint16_t last_ecd;
} mf9025_measure_t;

typedef struct {
  motor_measure_t baseInf;
  PidTypeDef pidParameter[2];
  float rawCmdCurrent;
  float currentSet;
  extKalman_t klmFiller;
  float filterSpeed;
  float speedSet;
  float totalEcd;//相对上电时的编码器步数
	float last_totalEcd;
  float totalEcdSet;
	int32_t Stucktime;   //电机堵转时间
  int32_t fixedEcd[ARM_BASE_P_STICKMODE_LENGTH];

  grab_motor_mode_e mode;
  grab_motor_mode_e lastMode;
	
} arm_base_pmotor_t;

////----------机械臂中部pitch轴旋转电机---------- Modified
//typedef enum {
//  ARM_MID_P_RESET = 0,			//ok
//	ARM_MID_P_NORMAL,           //常态
//  ARM_MID_P_SMALL_ISLAND,			//小资源岛
//  ARM_MID_P_BIG_ISLAND,				//大资源岛
//	ARM_MID_P_ALLEY_OPP,				//空接
//	ARM_MID_P_PRE_ORE,          
//	ARM_MID_P_STORE_ORE,        //
//	ARM_MID_P_EXCH_ORE,         //从矿仓取矿
//	ARM_MID_P_EXCH_LEVEL_01,
//	ARM_MID_P_EXCH_LEVEL_12,
//  ARM_MID_P_EXCH_LEVEL_24,				//兑换区等级0-2  
//	ARM_MID_P_MAX,
//	ROTATE_STICKMODE_LENGTH_Y,
//} arm_mid_pmotor_mode_e;

//typedef struct {
//  motor_measure_t baseInf;
//  PidTypeDef pidParameter[2];
//  float rawCmdCurrent;
//  float currentSet;
//  extKalman_t klmFiller;
//  float filterSpeed;
//  float speedSet;
//  float totalEcd;//相对上电时的编码器步数
//	float last_totalEcd;
//  float totalEcdSet;
//	int32_t Stucktime;   //电机堵转时间
//  int32_t fixedEcd[ROTATE_STICKMODE_LENGTH_Y];

//  grab_motor_mode_e mode;
//  grab_motor_mode_e lastMode;
//} arm_mid_pmotor_t;

//Add:----------机械臂中部yaw轴旋转电机---------- 

typedef enum {
  ARM_MID_Y_RESET = 0,			//ok
	ARM_MID_Y_NORMAL,           //常态
  ARM_MID_Y_SMALL_ISLAND,			//小资源岛
  ARM_MID_Y_BIG_ISLAND,				//大资源岛
	ARM_MID_Y_ALLEY_OPP,				//空接
	ARM_MID_Y_PRE_ORE,          
	ARM_MID_Y_STORE_ORE,        //
	ARM_MID_Y_EXCH_ORE,         //从矿仓取矿
	ARM_MID_Y_EXCH_LEVEL_01,
	ARM_MID_Y_EXCH_LEVEL_12,
  ARM_MID_Y_EXCH_LEVEL_24,				//兑换区等级0-2  
	ARM_MID_Y_MAX,
	ROTATE_STICKMODE_LENGTH_Y,
} arm_mid_pmotor_mode_e;

typedef struct {
  motor_measure_t baseInf;
  PidTypeDef pidParameter[2];
  float rawCmdCurrent;
  float currentSet;
  extKalman_t klmFiller;
  float filterSpeed;
  float speedSet;
  float totalEcd;//相对上电时的编码器步数
	float last_totalEcd;
  float totalEcdSet;
	int32_t Stucktime;   //电机堵转时间
  int32_t fixedEcd[ROTATE_STICKMODE_LENGTH_Y];

  grab_motor_mode_e mode;
  grab_motor_mode_e lastMode;
} arm_mid_ymotor_t;


//----------夹子旋转电机----------    Modified  //改成并联机构电机cjh
typedef enum {
  ROTATE_SINGLE_RESET = 0,			//
	ROTATE_SINGLE_NORMAL,      //常态时水平
	ROTATE_SINGLE_MAX,
	ROTATE_MODE_LENGTH_J,
} rotate_singlemotor_mode_e;

typedef struct {
  motor_measure_t baseInf;
  PidTypeDef pidParameter[2];
  float rawCmdCurrent;
  float currentSet;
  extKalman_t klmFiller;
  float filterSpeed;
  float speedSet;
  float totalEcd;//相对上电时的编码器步数
	float last_totalEcd;
  float totalEcdSet;
	int32_t Stucktime;   //电机堵转时间
  int32_t fixedEcd[ROTATE_MODE_LENGTH_J];

  grab_motor_mode_e mode;
  grab_motor_mode_e lastMode;
} rotate_singlemotor_t;

//----------平台X方向电机----------  Modified
typedef enum {
  PLAT_X_RESET = 0,			//原地
	PLAT_X_NORMAL,
	PLAT_X_INIT_RIGHT,      //初始化后使滑块置中
	PLAT_X_RIGHT_LITTLE,  //初始化时用，防止初始化前X方向上滑块碰到限位开关
	PLAT_X_MIN,		//右移最大
  PLAT_X_OREMODE_LENGTH,
} x_platmotor_mode_e ;

typedef struct {
  motor_measure_t baseInf;
  PidTypeDef pidParameter[2];
  float rawCmdCurrent;
  float currentSet;
  extKalman_t klmFiller;
  float filterSpeed;
  float speedSet;
  float totalEcd;//相对上电时的编码器步数
	float last_totalEcd;
  float totalEcdSet;
	int32_t Stucktime;   //电机堵转时间
  int32_t fixedEcd[PLAT_X_OREMODE_LENGTH];

  grab_motor_mode_e mode;
  grab_motor_mode_e lastMode;
} x_platmotor_t;

//----------新增：平台Y方向电机---------- 
typedef enum {
  PLAT_Y_MIN = 0,			//原地
	PLAT_Y_FRONT_LITTLE_INIT,  //初始化时前伸一小段距离
	PLAT_Y_NORMAL,
	PLAT_Y_SMALL_ISLAND,   
	PLAT_Y_SMALL_ISLAND_LIFT, 
	PLAT_Y_BIG_ISLAND,
	PLAT_Y_ALLEY_OPP,	
	PLAT_Y_STORE_ORE,      //从矿仓中取矿石
	PLAT_Y_FRONT_LITTLE_EXCH,  //从矿仓取矿时前伸一小段距离
  PLAT_Y_FRONT_MAX,			//
	PLAT_Y_EXCH_ORE,      //
  PLAT_Y_OREMODE_LENGTH,
} y_platmotor_mode_e ;

typedef struct {
  motor_measure_t baseInf;
  PidTypeDef pidParameter[2];
  float rawCmdCurrent;
  float currentSet;
  extKalman_t klmFiller;
  float filterSpeed;
  float speedSet;
  float totalEcd;//相对上电时的编码器步数
	float last_totalEcd;
  float totalEcdSet;
	int32_t Stucktime;   //电机堵转时间
  int32_t fixedEcd[PLAT_Y_OREMODE_LENGTH];

  grab_motor_mode_e mode;
  grab_motor_mode_e lastMode;
} y_platmotor_t;

//----------提升电机----------  
typedef enum {
  LIFT_RESET = 0,			//
	LIFT_NORMAL,
	LIFT_SMALL_ISLAND,   
	LIFT_BIG_ISLAND,
	LIFT_ALLEY_OPP,  
	LIFT_ORE,         //兑换
	LIFT_LITTLE_UP,   //小段抬升
  LIFT_MAX,
  LIFT_OREMODE_LENGTH,
} lift_oremotor_mode_e;

typedef struct {
  motor_measure_t baseInf;
  PidTypeDef pidParameter[2];
  float rawCmdCurrent;
  float currentSet;
  extKalman_t klmFiller;
  float filterSpeed;
  float speedSet;
  float totalEcd;//相对上电时的编码器步数
	float last_totalEcd;
  float totalEcdSet;
	int32_t Stucktime;   //电机堵转时间
  int32_t fixedEcd[LIFT_OREMODE_LENGTH];

  grab_motor_mode_e mode;
  grab_motor_mode_e lastMode;
} lift_oremotor_t;
//----------------------------------End:电机结构体声明区---------------------------------------//

//----------判断电机是否转至某位置附近时的电机名----------------
typedef enum {
  //JUDGE_ARMBASE_Y_MOTOR = 0,		
  JUDGE_ARMBASE_P_MOTOR1=0,
	JUDGE_ARMBASE_P_MOTOR2,	
  JUDGE_ARMMID_Y_MOTOR,	
  //JUDGE_SINGLE_ROTATE_MOTOR,
  JUDGE_PARALLELMOTOR1,	
	JUDGE_PARALLELMOTOR2,
  JUDGE_PLAT_X_MOTOR,
	JUDGE_PLAT_Y_MOTOR1,
	JUDGE_PLAT_Y_MOTOR2,
	JUDGE_LIFT_MOTOR1,
	JUDGE_LIFT_MOTOR2,
} judge_ecd_motor_e ;
//----------抓取行为----------
typedef enum {
  GRAB_DEBUG = 0,			//调试
  GRAB_ZERO_FORCE,		//无力
  GRAB_INIT,				//初始化【只运行一次】
  GRAB_PREPARE,			//准备空接模式
  GRAB_TAKE,				//抓取模式
  GRAB_EXCHANGE,			//兑换模式
  GRAB_NORMAL,			//正常模式
  GRAB_BEHAVIOUR_LENGTH,
} grab_behaviour_e;

typedef enum {
  TARGET_NONE = 0,
  TARGET_SMALL_ISLAND, //小资源岛
  TARGET_BIG_ISLAND,  //大资源岛
  TARGET_ALLEY_OPP,  //空接
  TARGET_EXCH_STA,   //兑换
} grab_target_e;  //机械臂动作对象模式

typedef struct {
  grab_motor_mode_e armbaseyMode;
  grab_motor_mode_e armmidyMode;
  grab_motor_mode_e singlerotateMode;
	grab_motor_mode_e parallelMode;
  grab_motor_mode_e xplatMode;
	grab_motor_mode_e yplatMode;//Addtional
  grab_motor_mode_e liftoreMode;
	grab_motor_mode_e armbasepMode;//Addtional MF9025
	
  grab_behaviour_e num;
	
  void (*behaviorHandleFun)(float *armbaseyExp, float *armbasepExp, float *armmidpExp, float *singlerotateExp, float *xplatExp, float *yplatExp, float *liftoreExp); //此模式的主要处理函数，最终要修改pid计算的期望值
  bool_t (*enterBehaviorCondition)(void); //进入该模式的条件
  bool_t (*outBehaviorCondition)(void);	//退出该模式的条件
  void (*enterBehaviorFun)(void); //进入该模式的处理
  void (*outBehaviorFun)(void);	//退出该模式的处理
	
} grab_behaviour_t;
////------------------舵机控制结构体---------------------------
//typedef enum {
//	SERVO_ORE_NORMAL_COMP = 0, //兑换时的补偿
//	SERVO_ORE_SMALL_ISLAND_COMP, //小资源岛取出来时的补偿
//  SERVO_ORE_NORMAL,    //共线
//	SERVO_ORE_DIRECT_GROUND,  //吸盘直朝地面
//  SERVO_ORE_ALLEY_OPP,  //空接
//	SERVO_ORE_STORE_ORE, //取矿后放矿时机械臂底座旋转朝后前舵机准备好角度
//	SERVO_ORE_DIRECT_STORE,
//	SERVO_ORE_DIRECT_EXCH,
//	SERVO_ORE_EXCH_LEVEL_01,
//	SERVO_ORE_EXCH_LEVEL_12,
//	SERVO_ORE_EXCH_LEVEL_24,
//	SERVO_ORE_EXCH_ORE,      //矿仓取矿时舵机角度
//  SERVO_ORE_EXCH_STA,   //兑换
//	SERVO_ORE_LENGTH,     
//} servo_ore_e;  

//typedef struct {
//	uint16_t fixedAngle[SERVO_ORE_LENGTH];   //存储的舵机角度标准值
//	uint16_t angleSet;    //舵机角度设定值
//	uint32_t behaviourTime; //遍历时间
//} servo_ore_t ;

typedef enum {
	EXCH_LEVEL_NONE = 0,
	EXCH_LEVEL_0,
	EXCH_LEVEL_1,
	EXCH_LEVEL_2,
	EXCH_LEVEL_3,
	EXCH_LEVEL_4,
	EXCH_LEVEL_LENGTH,
} exch_level_e; //兑换等级枚举量

//------------------限位开关信息结构体---------------------------
typedef enum {
  LIMSW4 = 0,
  LIMSW5,
  LIMSW6,  
  LIMSW9,  
  LIMSW_ALL,   
	LIMSW_LENGTH,  
} limsw_e;  

//------------------抓取完整结构体---------------------------
typedef struct
{
 // arm_base_ymotor_t armbaseyMotor;  //Modified //底部yaw轴期望
	arm_base_pmotor_t armbasepMotor1; //Additional：MF9025 //底部pitch轴期望
	arm_base_pmotor_t armbasepMotor2; //Additional：MF9025
  arm_mid_ymotor_t armmidyMotor;  //Modified //中部pitch轴期望//cjh 中部yaw轴期望
	rotate_singlemotor_t singlerotateMotor;  //旋转方向期望
	rotate_singlemotor_t parallelMotor1;   //并联机构期望cjh
	rotate_singlemotor_t parallelMotor2;   //cjh
  x_platmotor_t xplatMotor1;  //Modified//x方向平台
	x_platmotor_t xplatMotor2;  //cjh x电机2
	y_platmotor_t yplatMotor1;  //Additional//y方向平台
	y_platmotor_t yplatMotor2;  //cjh y电机2
  lift_oremotor_t liftoreMotor1;//平台提升期望  
  lift_oremotor_t liftoreMotor2;  
	
  struct {
    const RC_ctrl_t *now; 		//遥控器当前值
    RC_ctrl_t last; 			//遥控器上一次的值
  } rc;
  grab_behaviour_t behaviorList[GRAB_BEHAVIOUR_LENGTH];
  grab_behaviour_t *nowBehaviour;
  grab_behaviour_e nowBehaviorName;
  uint8_t behaviourStep;			//行为的步骤
  uint32_t behaviourTime;			//行为的时间
	uint8_t take2exch_Sign;    //从取矿石直接跳到兑换
	uint8_t liftHigher;        //普通模式下升降高度变换
  uint8_t collectedOre;			//已经取得矿石数量
  grab_target_e currentTarget;	//当前目标为抓取/兑换
	exch_level_e currentExchlevel; //当前兑换难度
  uint8_t inited;					//已经执行过初始化工作
	
	uint8_t limsw[LIMSW_LENGTH];       //接收到的限位开关信号
	
//	servo_ore_t oreServo;         //抓矿舵机结构体变量
} grabCtrl_t;

void grabInit(void);
void grabTask(void *pvParameters);
void grabBehaviorChange(grab_behaviour_t *next);

extern grabCtrl_t grabTaskStructure;

#endif
