/*


                                                         @@@@@@@@@@
                                             @@@@@@@@@@@@@@@@@@@@@@@@\         @@@@@@@////
                                     ]/@@@/[[[`  =@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@```````
                                ]/[   ,]]@@@@@@@` @@@@@@@@@@@@@@@@@@@@@@`
                              ,]/@@@@@@@@@/[[[[[`              [\@@@@@@@
                         ]/@@/[[         ,]]]]]O@@@@@@@@@@@@@@@@@@@@@@@@@\]]`       [[\@@@@]]]
                    ]//[`   ,]]/@@@@@@@@@/[[[[`                         @@@@@@@@@@@@@@@\]]   ,@@@@@@@@@
                    ]]@@@@@@[[[`                      燕      鹰                       [[\@@@@@@@@@@@@@@@@@@@]`
              ]/@@@/[`                                                                       [[@@@@@@]`,\@@@@@@\`
         ,/@@/[       @@@@@@@@@   @@@@@@@@^      @@@@@@@@@@@@@@@        @@@@@@@        @@@@@@      ,\@@@@@/@@@@@@@`
      ]@@[          ,@@@@@@@@    @@@@@@@@`    @@@@@@@      @@@@@       @@@@@@@         @@@@@@          ,@@@@@@@@@@@@
   //`             ,@@@@@@@@    @@@@@@@@`    @@@@@@                   @@@@@@           @@@@@@            ,@@@@@@@@@@
 ,`               ,@@@@@@@@   @@@@@@@@@        @@@@@@@@@@@@          @@@@@@            @@@@@              ,@@@@@@@@@
                  @@@@@@@@@@@@@@@@@@@/               @@@@@@@@@@     @@@@@@@           @@@@@@              ,@@@@@@@@
                   ,@@@@@@@@@@@@@@@@`    /@@@             @@@@@@/  \@@@@@@@         @@@@@@/            ,@@@@@@@@
                 /@@@@@@@@@@@@@@@@/      \@@@@@@       @@@@@@@@/    \@@@@@@@@    @@@@@@@@/          ]/@@@@@@
                /@@@@@@@@@@@@@@/          \@@@@@@@@@@@@@@@@@@/       \@@@@@@@@@@@@@@@@/



                            _ooOoo_
                           o8888888o
                           88" . "88
                           (| -_- |)
                            O\ = /O
                        ____/`---'\____
                      .   ' \\| |// `.
                       / \\||| : |||// \
                     / _||||| -:- |||||- \
                       | | \\\ - /// | |
                     | \_| ''\---/'' | |
                      \ .-\__ `-` ___/-. /
                   ___`. .' /--.--\ `. . __
                ."" '< `.___\_<|>_/___.' >'"".
               | | : `- \`.;`\ _ /`;.`/ - ` : | |
                 \ \ `-. \_ __\ /__ _/ .-` / /
         ======`-.____`-.___\_____/___.-`____.-'======
                            `=---='

         .............................................
                  佛祖保佑             永无BUG
          佛曰:
                  写字楼里写字间，写字间里程序员；
                  程序人员写程序，又拿程序换酒钱。
                  酒醒只在网上坐，酒醉还来网下眠；
                  酒醉酒醒日复日，网上网下年复年。
                  但愿老死电脑间，不愿鞠躬老板前；
                  奔驰宝马贵者趣，公交自行程序员。
                  别人笑我忒疯癫，我笑自己命太贱；
                  不见满街漂亮妹，哪个归得程序员？
*/

#include "main.h"
//FreeRTOS
#include "stm32f4xx.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "start_task.h"
#include "timer_send_task.h"
//BSP
#include "led.h"
#include "delay.h"
#include "power.h"
#include "can.h"
#include "usart6.h"
#include "uart7.h"
//USER
#include "robot.h"
#include "remote_control.h"
#include "buzzer.h"
#include "laser.h"
#include "sensor.h"
#include "servo.h"
#include "valve.h"
#include "mf9025.h"
#include "messageBoard_B.h"

void BSPInit() {
  //中断组 4 仅抢占优先级
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

  //can1初始化
  CAN1_Init();

  //can2初始化
  CAN2_Init();

  //滴答时钟初始化
  delay_init();

  //LED初始化
  led_configuration();

  //电源控制初始化
  power_ctrl_configuration();

  //蜂鸣器初始化
  buzzer_configuration();

  //激光IO初始化
  laser_configuration();

  //光电IO初始化
  sensor_configuration();

  //舵机IO初始化
  servo_configuration();

  //遥控器初始化
  remote_control_init();

  //串口6初始化
  USART6_Init();
	uart7_init();

  //陀螺仪初始化
  Spi5_Init();
  MPU6500_Init();
  IST8310_Init();
	
}

int main() {
  BSPInit();
  delay_ms(10);
  robotInit();
  timerSendCreate();
  startTask();   //创建任务
  vTaskStartScheduler();  //运行任务
	
  while(1) {

  }
}
