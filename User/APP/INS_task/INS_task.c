/**************************************����������********************************/
/*ע�����ڿ�����ͨ���ӳ�ԭ����ʱ�������������ǵ�yaw��Ƕ�*/
#include "INS_task.h"
#include  "IMU_ext.h"
#include "math.h"
#include "user_lib.h"

//#define USING_DJIAHRS_ALG //DJI-AHRS 9���ںϣ�������ʵ��ת���Ƕ���������⣨�Ƽ�ʹ�ã�
//#define USING_AHRS_ALGO //AHRS 9���ںϣ�������ʵ��ת���ǶȲ����������⣨���Ƽ�ʹ�ã�
#define USING_AHRS_ALGO_IMU_ONLY //AHRS 6���ںϣ������˶�����Ư���⣨�����Ժã�

//���ٶȼƵ�ͨ�˲�����
uint8_t decode_succ;//,last_decode_succ;
IMUPreDataTypedef imu_data; //ԭʼ���ݺ�ƫ����
IMUTypedef imu_; //�������
IMUTypedef imuu_;
extern raw_t rawa;
//static fp32 quat[4]; //��Ԫ��

void INS_task(void *pvParameters) {
  portTickType currentTime;
  currentTime = xTaskGetTickCount();
  vTaskDelay(100);

  while(1) {
    if(decode_succ) {
      datatrans(&rawa, &imu_);
      decode_succ = 0;
    }
    /**********׼�������л�**********/
    if(robotInf.modeStep == (robot_init_step)(ROBOT_INIT_IMU)) {
      static uint16_t initStableTime, targetInitStableTime = 0;

      if(1) initStableTime += IMU_TASK_MS; //�ȴ������ȶ�

      #ifdef USING_DJIAHRS_ALG
      targetInitStableTime = 0;
      #elif defined(USING_AHRS_ALGO)
      targetInitStableTime = 3000;
      #elif defined(USING_AHRS_ALGO_IMU_ONLY)
      targetInitStableTime = 1000;
      #endif

      if(initStableTime >= targetInitStableTime) robotInf.modeStep = (robot_init_step)(ROBOT_INIT_IMU + 1); //IMU��ʼ����� ���������˵��������ܳ�ʼ��
    }

    vTaskDelayUntil(&currentTime, IMU_TASK_MS);
  }
}

