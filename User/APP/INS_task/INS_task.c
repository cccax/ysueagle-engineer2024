/**************************************外置陀螺仪********************************/
/*注：由于可能是通信延迟原因，暂时仅用外置陀螺仪的yaw轴角度*/
#include "INS_task.h"
#include  "IMU_ext.h"
#include "math.h"
#include "user_lib.h"

//#define USING_DJIAHRS_ALG //DJI-AHRS 9轴融合，存在与实际转动角度有误差问题（推荐使用）
//#define USING_AHRS_ALGO //AHRS 9轴融合，存在与实际转动角度不成正比问题（不推荐使用）
#define USING_AHRS_ALGO_IMU_ONLY //AHRS 6轴融合，存在运动后零漂问题（兼容性好）

//加速度计低通滤波参数
uint8_t decode_succ;//,last_decode_succ;
IMUPreDataTypedef imu_data; //原始数据和偏移量
IMUTypedef imu_; //输出数据
IMUTypedef imuu_;
extern raw_t rawa;
//static fp32 quat[4]; //四元数

void INS_task(void *pvParameters) {
  portTickType currentTime;
  currentTime = xTaskGetTickCount();
  vTaskDelay(100);

  while(1) {
    if(decode_succ) {
      datatrans(&rawa, &imu_);
      decode_succ = 0;
    }
    /**********准备任务切换**********/
    if(robotInf.modeStep == (robot_init_step)(ROBOT_INIT_IMU)) {
      static uint16_t initStableTime, targetInitStableTime = 0;

      if(1) initStableTime += IMU_TASK_MS; //等待数据稳定

      #ifdef USING_DJIAHRS_ALG
      targetInitStableTime = 0;
      #elif defined(USING_AHRS_ALGO)
      targetInitStableTime = 3000;
      #elif defined(USING_AHRS_ALGO_IMU_ONLY)
      targetInitStableTime = 1000;
      #endif

      if(initStableTime >= targetInitStableTime) robotInf.modeStep = (robot_init_step)(ROBOT_INIT_IMU + 1); //IMU初始化完成 继续机器人的其它功能初始化
    }

    vTaskDelayUntil(&currentTime, IMU_TASK_MS);
  }
}

