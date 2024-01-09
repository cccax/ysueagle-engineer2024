#ifndef BASIC_TASK
#define BASIC_TASK

#include "stm32f4xx.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "buzzer.h"
#include "valve.h"
#include "robot.h"
#include "remote_control.h"
#include "valve.h"
#include "pid.h"
#include "kalman.h"
#include "motor.h"
#include "servo.h"
#include "detect_task.h"
#include "can.h"

#define BASIC_CANBUS_SEND_HEADER14 0x200
#define BASIC_CANBUS_SEND_HEADER58 0x1FF

#ifndef INNER     //速度环PID
#define INNER 0
#endif

#ifndef OUTER      //位置环PID
#define OUTER 1
#endif

#define BASIC_FIRST_PRESS_W		!IF_LAST_KEY_PRESSED_W(basicTaskStructure.rc.last) && IF_KEY_PRESSED_W
#define BASIC_FIRST_PRESS_S		!IF_LAST_KEY_PRESSED_S(basicTaskStructure.rc.last) && IF_KEY_PRESSED_S
#define BASIC_FIRST_PRESS_A		!IF_LAST_KEY_PRESSED_A(basicTaskStructure.rc.last) && IF_KEY_PRESSED_A
#define BASIC_FIRST_PRESS_D		!IF_LAST_KEY_PRESSED_D(basicTaskStructure.rc.last) && IF_KEY_PRESSED_D
#define BASIC_FIRST_PRESS_Q		!IF_LAST_KEY_PRESSED_Q(basicTaskStructure.rc.last) && IF_KEY_PRESSED_Q
#define BASIC_FIRST_PRESS_E		!IF_LAST_KEY_PRESSED_E(basicTaskStructure.rc.last) && IF_KEY_PRESSED_E
#define BASIC_FIRST_PRESS_R		!IF_LAST_KEY_PRESSED_R(basicTaskStructure.rc.last) && IF_KEY_PRESSED_R

#define BASIC_FIRST_PRESS_F		!IF_LAST_KEY_PRESSED_F(basicTaskStructure.rc.last) && IF_KEY_PRESSED_F
#define BASIC_FIRST_PRESS_G		!IF_LAST_KEY_PRESSED_G(basicTaskStructure.rc.last) && IF_KEY_PRESSED_G

#define BASIC_FIRST_PRESS_X		!IF_LAST_KEY_PRESSED_X(basicTaskStructure.rc.last) && IF_KEY_PRESSED_X
#define BASIC_FIRST_PRESS_Z		!IF_LAST_KEY_PRESSED_Z(basicTaskStructure.rc.last) && IF_KEY_PRESSED_Z
#define BASIC_FIRST_PRESS_C		!IF_LAST_KEY_PRESSED_C(basicTaskStructure.rc.last) && IF_KEY_PRESSED_C
#define BASIC_FIRST_PRESS_V		!IF_LAST_KEY_PRESSED_V(basicTaskStructure.rc.last) && IF_KEY_PRESSED_V
#define BASIC_FIRST_PRESS_B		!IF_LAST_KEY_PRESSED_B(basicTaskStructure.rc.last) && IF_KEY_PRESSED_B

#define BASIC_FIRST_PRESS_CTRL	!IF_LAST_KEY_PRESSED_CTRL(basicTaskStructure.rc.last) && IF_KEY_PRESSED_CTRL
#define BASIC_FIRST_PRESS_SHIFT	!IF_LAST_KEY_PRESSED_SHIFT(basicTaskStructure.rc.last) && IF_KEY_PRESSED_SHIFT

#define BASIC_FIRST_S1_UP		IF_RC_SW1_UP && (!(basicTaskStructure.rc.last.rc.s[0] == 1))
#define BASIC_FIRST_S1_DOWN		IF_RC_SW1_DOWN && (!(basicTaskStructure.rc.last.rc.s[0] == 2))
#define BASIC_FIRST_S1_MID		IF_RC_SW1_MID && (!(basicTaskStructure.rc.last.rc.s[0] == 3))
#define BASIC_FIRST_S2_UP		IF_RC_SW2_UP && (!(basicTaskStructure.rc.last.rc.s[1] == 1))
#define BASIC_FIRST_S2_DOWN		IF_RC_SW2_DOWN && (!(basicTaskStructure.rc.last.rc.s[1] == 2))
#define BASIC_FIRST_S2_MID		IF_RC_SW2_MID && (!(basicTaskStructure.rc.last.rc.s[1] == 3))

#define BASIC_FIRST_MOUSE_X_STOP	MOUSE_X_MOVE_SPEED == 0 && basicTaskStructure.rc.last.mouse.x != 0
#define BASIC_FIRST_MOUSE_Y_STOP	MOUSE_Y_MOVE_SPEED == 0 && basicTaskStructure.rc.last.mouse.y != 0
#define BASIC_FIRST_CH0_MID		RC_CH0_RLR_OFFSET == 0 && basicTaskStructure.rc.last.rc.ch[0] != 0
#define BASIC_FIRST_CH1_MID		RC_CH1_RUD_OFFSET == 0 && basicTaskStructure.rc.last.rc.ch[1] != 0
#define BASIC_FIRST_CH2_MID		RC_CH2_LLR_OFFSET == 0 && basicTaskStructure.rc.last.rc.ch[2] != 0
#define BASIC_FIRST_CH3_MID		RC_CH3_LUD_OFFSET == 0 && basicTaskStructure.rc.last.rc.ch[3] != 0
#define BASIC_FIRST_CH4_MID		RC_CH4_ROT_OFFSET == 0 && basicTaskStructure.rc.last.rc.ch[4] != 0

#define BASIC_FIRST_PRESS_LMB IF_MOUSE_PRESSED_LEFT && !basicTaskStructure.rc.last.mouse.press_l
#define BASIC_FIRST_PRESS_RMB IF_MOUSE_PRESSED_RIGHT && !basicTaskStructure.rc.last.mouse.press_r

#define BASIC_CAMTOP_MOTOR_TYPE BASIC_MOTOR_POSITION


typedef enum {
	SERVO_CAM_NORMAL = 0,
	SERVO_CAM_GRAB,
	SERVO_CAM_NONE,
} cam_servo_t;

typedef enum {           //enum存储固定位置变量下标的枚举型
	CAM_TOP_NORMAL = 0,
	CAM_TOP_GRAB,
	CAM_TOP_MIN,
	CAM_TOP_SIDE1,
	CAM_TOP_SIDE2,
	CAM_TOP_MAX,
  CAM_TOP_STICKMODE_LENGTH,
} cam_top_motor_mode_e; 

typedef enum {
  BASIC_MOTOR_POSITION = 0,
  BASIC_MOTOR_SPEED,
  BASIC_MOTOR_RAW,
} basic_motor_mode_e;

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
  int32_t fixedEcd[CAM_TOP_STICKMODE_LENGTH];

  basic_motor_mode_e mode;
  basic_motor_mode_e lastMode;
} cam_top_motor_t;


typedef struct {
  struct {
    const RC_ctrl_t *now; //遥控器当前值
    RC_ctrl_t last; //遥控器上一次的值
  } rc;
	
	cam_top_motor_t camtopMotor; //图传电机结构体
	
  cam_servo_t servoCam_Sign; //图传舵机模式
	
	uint8_t camSwitch; //图传舵机切换标志
} BasicCtrl_t;

extern BasicCtrl_t basicTaskStructure;

void basic_task(void *pvParameters);

#endif
