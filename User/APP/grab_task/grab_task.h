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

#define RAWDATA_LENGTH 465  //΢���Ķ��ԭʼ�������鳤��

#ifndef INNER     //�ٶȻ�PID
#define INNER 0
#endif

#ifndef OUTER      //λ�û�PID
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

//-----------------------------Start:����ṹ��������---------------------------------------//
////----------��е�ۻ���yaw����ת���----------  Modified 

//typedef enum {           //enum�洢�̶�λ�ñ����±��ö����
//  ARM_BASE_Y_RESET = 0,			//
//	ARM_BASE_Y_NORMAL,            //��̬��ֱ��ǰ��
//  ARM_BASE_Y_SMALL_ISLAND,			//С��Դ��
//  ARM_BASE_Y_BIG_ISLAND,				//����Դ��
//	ARM_BASE_Y_ALLEY_OPP,				//�ս�
//	ARM_BASE_Y_STORE_ORE,        //ת��ֱ�Կ��
//  ARM_BASE_Y_EXCH_LEVEL_012,				//�һ����ȼ�0-2 
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
//  float totalEcd;//����ϵ�ʱ�ı���������
//	float last_totalEcd;
//  float totalEcdSet;
//	int32_t Stucktime;   //�����תʱ��
//  int32_t fixedEcd[ARM_BASE_Y_STICKMODE_LENGTH];

//  grab_motor_mode_e mode;
//  grab_motor_mode_e lastMode;
//} arm_base_ymotor_t;



//----------��������е�ۻ���pitch����ת���(MF9025)----------
//----�޸ģ�M3508
typedef enum {           //enum�洢�̶�λ�ñ����±��ö����
  ARM_BASE_P_RESET = 0,			//��͵㣬��������ܽӴ�
	ARM_BASE_P_MIN,            
	ARM_BASE_P_NORMAL,        //��̬���ײ����ˮƽ����
  ARM_BASE_P_SMALL_ISLAND,			//С��Դ��
  ARM_BASE_P_BIG_ISLAND,				//����Դ��
	ARM_BASE_P_ALLEY_OPP,				//�ս�
	ARM_BASE_P_STORE_ORE,        //�ſ�
	ARM_BASE_P_STORE_START,
	ARM_BASE_P_EXCH_ORE,             //���ȡ��
	ARM_BASE_P_EXCH_LEVEL_01,
	ARM_BASE_P_EXCH_LEVEL_12,
  ARM_BASE_P_EXCH_LEVEL_24,				//�һ����ȼ�0-2
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
  float totalEcd;//����ϵ�ʱ�ı���������
	float last_totalEcd;
  float totalEcdSet;
	int32_t Stucktime;   //�����תʱ��
  int32_t fixedEcd[ARM_BASE_P_STICKMODE_LENGTH];

  grab_motor_mode_e mode;
  grab_motor_mode_e lastMode;
	
} arm_base_pmotor_t;

////----------��е���в�pitch����ת���---------- Modified
//typedef enum {
//  ARM_MID_P_RESET = 0,			//ok
//	ARM_MID_P_NORMAL,           //��̬
//  ARM_MID_P_SMALL_ISLAND,			//С��Դ��
//  ARM_MID_P_BIG_ISLAND,				//����Դ��
//	ARM_MID_P_ALLEY_OPP,				//�ս�
//	ARM_MID_P_PRE_ORE,          
//	ARM_MID_P_STORE_ORE,        //
//	ARM_MID_P_EXCH_ORE,         //�ӿ��ȡ��
//	ARM_MID_P_EXCH_LEVEL_01,
//	ARM_MID_P_EXCH_LEVEL_12,
//  ARM_MID_P_EXCH_LEVEL_24,				//�һ����ȼ�0-2  
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
//  float totalEcd;//����ϵ�ʱ�ı���������
//	float last_totalEcd;
//  float totalEcdSet;
//	int32_t Stucktime;   //�����תʱ��
//  int32_t fixedEcd[ROTATE_STICKMODE_LENGTH_Y];

//  grab_motor_mode_e mode;
//  grab_motor_mode_e lastMode;
//} arm_mid_pmotor_t;

//Add:----------��е���в�yaw����ת���---------- 

typedef enum {
  ARM_MID_Y_RESET = 0,			//ok
	ARM_MID_Y_NORMAL,           //��̬
  ARM_MID_Y_SMALL_ISLAND,			//С��Դ��
  ARM_MID_Y_BIG_ISLAND,				//����Դ��
	ARM_MID_Y_ALLEY_OPP,				//�ս�
	ARM_MID_Y_PRE_ORE,          
	ARM_MID_Y_STORE_ORE,        //
	ARM_MID_Y_EXCH_ORE,         //�ӿ��ȡ��
	ARM_MID_Y_EXCH_LEVEL_01,
	ARM_MID_Y_EXCH_LEVEL_12,
  ARM_MID_Y_EXCH_LEVEL_24,				//�һ����ȼ�0-2  
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
  float totalEcd;//����ϵ�ʱ�ı���������
	float last_totalEcd;
  float totalEcdSet;
	int32_t Stucktime;   //�����תʱ��
  int32_t fixedEcd[ROTATE_STICKMODE_LENGTH_Y];

  grab_motor_mode_e mode;
  grab_motor_mode_e lastMode;
} arm_mid_ymotor_t;


//----------������ת���----------    Modified  //�ĳɲ����������cjh
typedef enum {
  ROTATE_SINGLE_RESET = 0,			//
	ROTATE_SINGLE_NORMAL,      //��̬ʱˮƽ
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
  float totalEcd;//����ϵ�ʱ�ı���������
	float last_totalEcd;
  float totalEcdSet;
	int32_t Stucktime;   //�����תʱ��
  int32_t fixedEcd[ROTATE_MODE_LENGTH_J];

  grab_motor_mode_e mode;
  grab_motor_mode_e lastMode;
} rotate_singlemotor_t;

//----------ƽ̨X������----------  Modified
typedef enum {
  PLAT_X_RESET = 0,			//ԭ��
	PLAT_X_NORMAL,
	PLAT_X_INIT_RIGHT,      //��ʼ����ʹ��������
	PLAT_X_RIGHT_LITTLE,  //��ʼ��ʱ�ã���ֹ��ʼ��ǰX�����ϻ���������λ����
	PLAT_X_MIN,		//�������
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
  float totalEcd;//����ϵ�ʱ�ı���������
	float last_totalEcd;
  float totalEcdSet;
	int32_t Stucktime;   //�����תʱ��
  int32_t fixedEcd[PLAT_X_OREMODE_LENGTH];

  grab_motor_mode_e mode;
  grab_motor_mode_e lastMode;
} x_platmotor_t;

//----------������ƽ̨Y������---------- 
typedef enum {
  PLAT_Y_MIN = 0,			//ԭ��
	PLAT_Y_FRONT_LITTLE_INIT,  //��ʼ��ʱǰ��һС�ξ���
	PLAT_Y_NORMAL,
	PLAT_Y_SMALL_ISLAND,   
	PLAT_Y_SMALL_ISLAND_LIFT, 
	PLAT_Y_BIG_ISLAND,
	PLAT_Y_ALLEY_OPP,	
	PLAT_Y_STORE_ORE,      //�ӿ����ȡ��ʯ
	PLAT_Y_FRONT_LITTLE_EXCH,  //�ӿ��ȡ��ʱǰ��һС�ξ���
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
  float totalEcd;//����ϵ�ʱ�ı���������
	float last_totalEcd;
  float totalEcdSet;
	int32_t Stucktime;   //�����תʱ��
  int32_t fixedEcd[PLAT_Y_OREMODE_LENGTH];

  grab_motor_mode_e mode;
  grab_motor_mode_e lastMode;
} y_platmotor_t;

//----------�������----------  
typedef enum {
  LIFT_RESET = 0,			//
	LIFT_NORMAL,
	LIFT_SMALL_ISLAND,   
	LIFT_BIG_ISLAND,
	LIFT_ALLEY_OPP,  
	LIFT_ORE,         //�һ�
	LIFT_LITTLE_UP,   //С��̧��
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
  float totalEcd;//����ϵ�ʱ�ı���������
	float last_totalEcd;
  float totalEcdSet;
	int32_t Stucktime;   //�����תʱ��
  int32_t fixedEcd[LIFT_OREMODE_LENGTH];

  grab_motor_mode_e mode;
  grab_motor_mode_e lastMode;
} lift_oremotor_t;
//----------------------------------End:����ṹ��������---------------------------------------//

//----------�жϵ���Ƿ�ת��ĳλ�ø���ʱ�ĵ����----------------
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
//----------ץȡ��Ϊ----------
typedef enum {
  GRAB_DEBUG = 0,			//����
  GRAB_ZERO_FORCE,		//����
  GRAB_INIT,				//��ʼ����ֻ����һ�Ρ�
  GRAB_PREPARE,			//׼���ս�ģʽ
  GRAB_TAKE,				//ץȡģʽ
  GRAB_EXCHANGE,			//�һ�ģʽ
  GRAB_NORMAL,			//����ģʽ
  GRAB_BEHAVIOUR_LENGTH,
} grab_behaviour_e;

typedef enum {
  TARGET_NONE = 0,
  TARGET_SMALL_ISLAND, //С��Դ��
  TARGET_BIG_ISLAND,  //����Դ��
  TARGET_ALLEY_OPP,  //�ս�
  TARGET_EXCH_STA,   //�һ�
} grab_target_e;  //��е�۶�������ģʽ

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
	
  void (*behaviorHandleFun)(float *armbaseyExp, float *armbasepExp, float *armmidpExp, float *singlerotateExp, float *xplatExp, float *yplatExp, float *liftoreExp); //��ģʽ����Ҫ������������Ҫ�޸�pid���������ֵ
  bool_t (*enterBehaviorCondition)(void); //�����ģʽ������
  bool_t (*outBehaviorCondition)(void);	//�˳���ģʽ������
  void (*enterBehaviorFun)(void); //�����ģʽ�Ĵ���
  void (*outBehaviorFun)(void);	//�˳���ģʽ�Ĵ���
	
} grab_behaviour_t;
////------------------������ƽṹ��---------------------------
//typedef enum {
//	SERVO_ORE_NORMAL_COMP = 0, //�һ�ʱ�Ĳ���
//	SERVO_ORE_SMALL_ISLAND_COMP, //С��Դ��ȡ����ʱ�Ĳ���
//  SERVO_ORE_NORMAL,    //����
//	SERVO_ORE_DIRECT_GROUND,  //����ֱ������
//  SERVO_ORE_ALLEY_OPP,  //�ս�
//	SERVO_ORE_STORE_ORE, //ȡ���ſ�ʱ��е�۵�����ת����ǰ���׼���ýǶ�
//	SERVO_ORE_DIRECT_STORE,
//	SERVO_ORE_DIRECT_EXCH,
//	SERVO_ORE_EXCH_LEVEL_01,
//	SERVO_ORE_EXCH_LEVEL_12,
//	SERVO_ORE_EXCH_LEVEL_24,
//	SERVO_ORE_EXCH_ORE,      //���ȡ��ʱ����Ƕ�
//  SERVO_ORE_EXCH_STA,   //�һ�
//	SERVO_ORE_LENGTH,     
//} servo_ore_e;  

//typedef struct {
//	uint16_t fixedAngle[SERVO_ORE_LENGTH];   //�洢�Ķ���Ƕȱ�׼ֵ
//	uint16_t angleSet;    //����Ƕ��趨ֵ
//	uint32_t behaviourTime; //����ʱ��
//} servo_ore_t ;

typedef enum {
	EXCH_LEVEL_NONE = 0,
	EXCH_LEVEL_0,
	EXCH_LEVEL_1,
	EXCH_LEVEL_2,
	EXCH_LEVEL_3,
	EXCH_LEVEL_4,
	EXCH_LEVEL_LENGTH,
} exch_level_e; //�һ��ȼ�ö����

//------------------��λ������Ϣ�ṹ��---------------------------
typedef enum {
  LIMSW4 = 0,
  LIMSW5,
  LIMSW6,  
  LIMSW9,  
  LIMSW_ALL,   
	LIMSW_LENGTH,  
} limsw_e;  

//------------------ץȡ�����ṹ��---------------------------
typedef struct
{
 // arm_base_ymotor_t armbaseyMotor;  //Modified //�ײ�yaw������
	arm_base_pmotor_t armbasepMotor1; //Additional��MF9025 //�ײ�pitch������
	arm_base_pmotor_t armbasepMotor2; //Additional��MF9025
  arm_mid_ymotor_t armmidyMotor;  //Modified //�в�pitch������//cjh �в�yaw������
	rotate_singlemotor_t singlerotateMotor;  //��ת��������
	rotate_singlemotor_t parallelMotor1;   //������������cjh
	rotate_singlemotor_t parallelMotor2;   //cjh
  x_platmotor_t xplatMotor1;  //Modified//x����ƽ̨
	x_platmotor_t xplatMotor2;  //cjh x���2
	y_platmotor_t yplatMotor1;  //Additional//y����ƽ̨
	y_platmotor_t yplatMotor2;  //cjh y���2
  lift_oremotor_t liftoreMotor1;//ƽ̨��������  
  lift_oremotor_t liftoreMotor2;  
	
  struct {
    const RC_ctrl_t *now; 		//ң������ǰֵ
    RC_ctrl_t last; 			//ң������һ�ε�ֵ
  } rc;
  grab_behaviour_t behaviorList[GRAB_BEHAVIOUR_LENGTH];
  grab_behaviour_t *nowBehaviour;
  grab_behaviour_e nowBehaviorName;
  uint8_t behaviourStep;			//��Ϊ�Ĳ���
  uint32_t behaviourTime;			//��Ϊ��ʱ��
	uint8_t take2exch_Sign;    //��ȡ��ʯֱ�������һ�
	uint8_t liftHigher;        //��ͨģʽ�������߶ȱ任
  uint8_t collectedOre;			//�Ѿ�ȡ�ÿ�ʯ����
  grab_target_e currentTarget;	//��ǰĿ��Ϊץȡ/�һ�
	exch_level_e currentExchlevel; //��ǰ�һ��Ѷ�
  uint8_t inited;					//�Ѿ�ִ�й���ʼ������
	
	uint8_t limsw[LIMSW_LENGTH];       //���յ�����λ�����ź�
	
//	servo_ore_t oreServo;         //ץ�����ṹ�����
} grabCtrl_t;

void grabInit(void);
void grabTask(void *pvParameters);
void grabBehaviorChange(grab_behaviour_t *next);

extern grabCtrl_t grabTaskStructure;

#endif
