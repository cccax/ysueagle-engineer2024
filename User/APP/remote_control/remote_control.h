/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       remote_control.c/h
  * @brief      遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H
#include "main.h"
#include "rc.h"

#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 18u

#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s) (s == RC_SW_MID)
#define switch_is_up(s) (s == RC_SW_UP)
/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_PRESSED_OFFSET_W ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B ((uint16_t)1 << 15)

/* 获取遥控器摇杆偏移值
   RLR：右摇杆左右移动  LUD：左摇杆上下移动	*/
#define    RC_CH0_RLR_OFFSET    (rc_ctrl.rc.ch[0])
#define    RC_CH1_RUD_OFFSET  	(rc_ctrl.rc.ch[1])
#define    RC_CH2_LLR_OFFSET  	(rc_ctrl.rc.ch[2])
#define    RC_CH3_LUD_OFFSET  	(rc_ctrl.rc.ch[3])
#define    RC_CH4_ROT_OFFSET  	(rc_ctrl.rc.ch[4])
#define	RC_CHX_MAX	660
#define RC_CH0_OFFSET RC_CH0_RLR_OFFSET
#define RC_CH1_OFFSET RC_CH1_RUD_OFFSET
#define RC_CH2_OFFSET RC_CH2_LLR_OFFSET
#define RC_CH3_OFFSET RC_CH3_LUD_OFFSET
#define RC_CH4_OFFSET RC_CH4_ROT_OFFSET
/* 检测遥控器开关状态 */
#define    IF_RC_SW1_UP      (rc_ctrl.rc.s[0] == RC_SW_UP)
#define    IF_RC_SW1_MID     (rc_ctrl.rc.s[0] == RC_SW_MID)
#define    IF_RC_SW1_DOWN    (rc_ctrl.rc.s[0] == RC_SW_DOWN)
#define    IF_RC_SW2_UP      (rc_ctrl.rc.s[1] == RC_SW_UP)
#define    IF_RC_SW2_MID     (rc_ctrl.rc.s[1] == RC_SW_MID)
#define    IF_RC_SW2_DOWN    (rc_ctrl.rc.s[1] == RC_SW_DOWN)

/* 获取鼠标三轴的移动速度 */
#define    MOUSE_X_MOVE_SPEED    (rc_ctrl.mouse.x)
#define    MOUSE_Y_MOVE_SPEED    (rc_ctrl.mouse.y)
#define    MOUSE_Z_MOVE_SPEED    (rc_ctrl.mouse.z)

/* 检测鼠标按键状态
   按下为1，没按下为0*/
#define    IF_MOUSE_PRESSED_LEFT    (rc_ctrl.mouse.press_l == 1)
#define    IF_MOUSE_PRESSED_RIGH    (rc_ctrl.mouse.press_r == 1)
/* 检测键盘按键状态
   若对应按键被按下，则逻辑表达式的值为1，否则为0 */
#define    IF_KEY_PRESSED         (  rc_ctrl.key.v  )
#define    IF_KEY_PRESSED_W       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_W)    != 0 )
#define    IF_KEY_PRESSED_S       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_S)    != 0 )
#define    IF_KEY_PRESSED_A       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_A)    != 0 )
#define    IF_KEY_PRESSED_D       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_D)    != 0 )
#define    IF_KEY_PRESSED_Q       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q)    != 0 )
#define    IF_KEY_PRESSED_E       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_E)    != 0 )
#define    IF_KEY_PRESSED_G       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_G)    != 0 )
#define    IF_KEY_PRESSED_X       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_X)    != 0 )
#define    IF_KEY_PRESSED_Z       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_Z)    != 0 )
#define    IF_KEY_PRESSED_C       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_C)    != 0 )
#define    IF_KEY_PRESSED_B       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_B)    != 0 )
#define    IF_KEY_PRESSED_V       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_V)    != 0 )
#define    IF_KEY_PRESSED_F       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_F)    != 0 )
#define    IF_KEY_PRESSED_R       ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_R)    != 0 )
#define    IF_KEY_PRESSED_CTRL    ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_CTRL) != 0 )
#define    IF_KEY_PRESSED_SHIFT   ( (rc_ctrl.key.v & KEY_PRESSED_OFFSET_SHIFT) != 0 )
/* ----------------------- Data Struct ------------------------------------- */
typedef __packed struct {
  __packed struct {
    int16_t ch[5];
    char s[2];
  } rc;
  __packed struct {
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t press_l;
    uint8_t press_r;
  } mouse;
  __packed struct {
    uint16_t v;
  } key;

} RC_ctrl_t;
extern RC_ctrl_t rc_ctrl;
/* ----------------------- Internal Data ----------------------------------- */

extern void remote_control_init(void);
extern const RC_ctrl_t *get_remote_control_point(void);
extern uint8_t RC_data_is_error(void);
extern void slove_RC_lost(void);
extern void slove_data_error(void);
void rcDataCopy(RC_ctrl_t *rc);
#define    IF_LAST_KEY_PRESSED(rc)         (  (rc).key.v  )
#define    IF_LAST_KEY_PRESSED_W(rc)       ( ((rc).key.v & KEY_PRESSED_OFFSET_W)    != 0 )
#define    IF_LAST_KEY_PRESSED_S(rc)       ( ((rc).key.v & KEY_PRESSED_OFFSET_S)    != 0 )
#define    IF_LAST_KEY_PRESSED_A(rc)       ( ((rc).key.v & KEY_PRESSED_OFFSET_A)    != 0 )
#define    IF_LAST_KEY_PRESSED_D(rc)       ( ((rc).key.v & KEY_PRESSED_OFFSET_D)    != 0 )
#define    IF_LAST_KEY_PRESSED_Q(rc)       ( ((rc).key.v & KEY_PRESSED_OFFSET_Q)    != 0 )
#define    IF_LAST_KEY_PRESSED_E(rc)       ( ((rc).key.v & KEY_PRESSED_OFFSET_E)    != 0 )
#define    IF_LAST_KEY_PRESSED_G(rc)       ( ((rc).key.v & KEY_PRESSED_OFFSET_G)    != 0 )
#define    IF_LAST_KEY_PRESSED_X(rc)       ( ((rc).key.v & KEY_PRESSED_OFFSET_X)    != 0 )
#define    IF_LAST_KEY_PRESSED_Z(rc)       ( ((rc).key.v & KEY_PRESSED_OFFSET_Z)    != 0 )
#define    IF_LAST_KEY_PRESSED_C(rc)       ( ((rc).key.v & KEY_PRESSED_OFFSET_C)    != 0 )
#define    IF_LAST_KEY_PRESSED_B(rc)       ( ((rc).key.v & KEY_PRESSED_OFFSET_B)    != 0 )
#define    IF_LAST_KEY_PRESSED_V(rc)       ( ((rc).key.v & KEY_PRESSED_OFFSET_V)    != 0 )
#define    IF_LAST_KEY_PRESSED_F(rc)       ( ((rc).key.v & KEY_PRESSED_OFFSET_F)    != 0 )
#define    IF_LAST_KEY_PRESSED_R(rc)       ( ((rc).key.v & KEY_PRESSED_OFFSET_R)    != 0 )
#define    IF_LAST_KEY_PRESSED_CTRL(rc)    ( ((rc).key.v & KEY_PRESSED_OFFSET_CTRL) != 0 )
#define    IF_LAST_KEY_PRESSED_SHIFT(rc)   ( ((rc).key.v & KEY_PRESSED_OFFSET_SHIFT) != 0 )

#endif
