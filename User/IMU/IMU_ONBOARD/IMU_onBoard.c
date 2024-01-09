#include "IMU_onBoard.h"

/**********����IMU IST�Ĵ���**********/
const uint8_t MPU6500_CONFIG_NUM = 14; //14������
const uint8_t MPU6500_CONFIG_DATA[MPU6500_CONFIG_NUM][3] = { //�Ĵ��� �趨ֵ ������
  {MPU6500_PWR_MGMT_1, 0x00, 1}, //����ʱ��ʹ��
  {MPU6500_PWR_MGMT_2, 0x00, 2}, //ʹ�����ᴫ����
  {MPU6500_SMPLRT_DIV, 0x00, 3}, //������1��Ƶ�������䣩
  {MPU6500_CONFIG, 0x02, 4}, //���ֵ�ͨ�˲�ģʽ3
  {MPU6500_GYRO_CONFIG, MPU_GYRO_RANGLE, 5}, //���������� ���б�Ҫ���޸�IMU_onBoard.h�궨��
  {MPU6500_ACCEL_CONFIG, MPU_ACCEL_RANGLE, 6}, //���ٶȼ����� ���б�Ҫ���޸�IMU_onBoard.h�궨��
  {MPU6500_ACCEL_CONFIG_2, 0x00, 7}, //���ٶȼ��˲�ģʽ2
  {MPU6500_MOT_THR, 0x08, 8}, //����������ֵ40��DEC)
  {MPU6500_I2C_MST_CTRL, 0x4D, 9}, //�趨������IICƵ��400kHz
  {MPU6500_INT_PIN_CFG, 0x02, 10}, //δʹ��IICʱ��·����
  {MPU6500_INT_ENABLE, 0x01, 11}, //����׼�����ж�
  {MPU6500_I2C_MST_DELAY_CTRL, 0x01, 12}, //ͨ��0��ʱʹ��
  {MPU6500_MOT_DETECT_CTRL, 0xC0, 13}, //��������ж�ʹ��
  {MPU6500_USER_CTRL, 0x30, 14}, //ʹ����IIC����λ
};
const uint8_t IST8310_CONFIG_NUM = 4; //4������
const uint8_t IST8310_CONFIG_DATA[IST8310_CONFIG_NUM][3] = { //�Ĵ��� �趨ֵ ������
  {0x0B, 0x08, 1},
  {0x41, 0x09, 2},
  {0x42, 0xC0, 3},
  {0x0A, 0x0B, 4}
};

/**********������任����**********/
static fp32 Gyro_Scale_Factor[3][3] = {IMU_BOARD_INSTALL_SPIN_MATRIX}; //������У׼���Զ�
static fp32 Accel_Scale_Factor[3][3] = {IMU_BOARD_INSTALL_SPIN_MATRIX}; //���ٶ�У׼���Զ�
static fp32 Mag_Scale_Factor[3][3] = {IST_BOARD_INSTALL_SPIN_MATRIX};  //������У׼���Զ�

uint8_t MPU6500_Init(void) { //��ʼ��IMU
  MPU6500_Write_Reg(MPU6500_PWR_MGMT_1, 0x80); //��λоƬ
  smartDelayMs(300); //�ȴ�оƬ��ȫ��λ

  if(MPU6500_Read_Reg(MPU6500_WHO_AM_I) != MPU6500_ID)
    return 0xFF; //оƬͨ��ʧ��

  smartDelayMs(50);

  for(uint8_t i = 0; i < MPU6500_CONFIG_NUM; i++) { //����Ĵ������ò�У�� �����򷵻ش�����
    MPU6500_Write_Reg(MPU6500_CONFIG_DATA[i][0], MPU6500_CONFIG_DATA[i][1]);
    smartDelayMs(50);

    if(MPU6500_Read_Reg(MPU6500_CONFIG_DATA[i][0]) != MPU6500_CONFIG_DATA[i][1])
      return MPU6500_CONFIG_DATA[i][2];

    smartDelayMs(50);
  }

  //mpu_offset_call();
  return 0;
}

uint8_t IST8310_Init(void) { //��ʼ��������
  IST8310_NSS_Low(); //�����Ƹ�λ
  smartDelayMs(100);
  IST8310_NSS_High(); //�����Ƹ�λȡ��
  smartDelayMs(100);

  for (uint8_t i = 0; i < IST8310_CONFIG_NUM; i++) { //д�������
    IST8310_Reg_Write_By_MPU(IST8310_CONFIG_DATA[i][0], IST8310_CONFIG_DATA[i][1]);
    smartDelayMs(2);
    uint8_t res = IST8310_Reg_Read_By_MPU(IST8310_CONFIG_DATA[i][0]);
    smartDelayMs(2);

    if (res != IST8310_CONFIG_DATA[i][1]) return IST8310_CONFIG_DATA[i][2];
  }

  IST8310_Auto_Update();
  smartDelayMs(100);
  return 0;
}

void IMU_Update_Data(IMUPreDataTypedef* rawData) { //��ȡ�¶ȡ�IMU���ݡ�IST����
  static uint8_t mpu_buff[23];
  MPU6500_Read_Regs(MPU6500_INT_STATUS, mpu_buff, 23); //������ȡ23���Ĵ���

  rawData->imuStatus = (mpu_buff[0]); //IMU״̬�Ĵ���
  rawData->ax = (mpu_buff[0 + REG_VALID_DATA_OFFSET] << 8 | mpu_buff[1 + REG_VALID_DATA_OFFSET]);
  rawData->ay = (mpu_buff[2 + REG_VALID_DATA_OFFSET] << 8 | mpu_buff[3 + REG_VALID_DATA_OFFSET]);
  rawData->az = (mpu_buff[4 + REG_VALID_DATA_OFFSET] << 8 | mpu_buff[5 + REG_VALID_DATA_OFFSET]);
  rawData->gx = (mpu_buff[8 + REG_VALID_DATA_OFFSET] << 8 | mpu_buff[9 + REG_VALID_DATA_OFFSET]);
  rawData->gy = (mpu_buff[10 + REG_VALID_DATA_OFFSET] << 8 | mpu_buff[11 + REG_VALID_DATA_OFFSET]);
  rawData->gz = (mpu_buff[12 + REG_VALID_DATA_OFFSET] << 8 | mpu_buff[13 + REG_VALID_DATA_OFFSET]);
  rawData->mx = (mpu_buff[16 + REG_VALID_DATA_OFFSET] << 8 | mpu_buff[15 + REG_VALID_DATA_OFFSET]); //����һ��״̬�Ĵ�������
  rawData->my = (mpu_buff[18 + REG_VALID_DATA_OFFSET] << 8 | mpu_buff[17 + REG_VALID_DATA_OFFSET]);
  rawData->mz = (mpu_buff[20 + REG_VALID_DATA_OFFSET] << 8 | mpu_buff[19 + REG_VALID_DATA_OFFSET]);
  rawData->temp = mpu_buff[6 + REG_VALID_DATA_OFFSET] << 8 | mpu_buff[7 + REG_VALID_DATA_OFFSET];
}

void IMU_Handle_Data(IMUPreDataTypedef* rawData, IMUTypedef* targetData) { //��λת��������ƫ�ú�У׼
  /*? -> m/s2*/
  rawData->accel[0] = rawData->ax * ACCEL_SEN;
  rawData->accel[1] = rawData->ay * ACCEL_SEN;
  rawData->accel[2] = rawData->az * ACCEL_SEN;
  /*? -> rad/s*/
  rawData->gyro[0] = rawData->gx * GYRO_SEN;
  rawData->gyro[1] = rawData->gy * GYRO_SEN;
  rawData->gyro[2] = rawData->gz * GYRO_SEN;
  /*? -> uT*/
  rawData->mag[0] = rawData->mx * 0.3f;
  rawData->mag[1] = rawData->my * 0.3f;
  rawData->mag[2] = rawData->mz * 0.3f;
  rawData->temp = 21 + rawData->temp / 333.87f;

  //����任������
  for (uint8_t i = 0; i < 3; i++) {
    targetData->gyro[i] = rawData->gyro[0] * Gyro_Scale_Factor[i][0] + rawData->gyro[1] * Gyro_Scale_Factor[i][1] + rawData->gyro[2] * Gyro_Scale_Factor[i][2] + rawData->gyro_offset[i];
    targetData->accel[i] = rawData->accel[0] * Accel_Scale_Factor[i][0] + rawData->accel[1] * Accel_Scale_Factor[i][1] + rawData->accel[2] * Accel_Scale_Factor[i][2] + rawData->accel_offset[i];
    targetData->mag[i] = rawData->mag[0] * Mag_Scale_Factor[i][0] + rawData->mag[1] * Mag_Scale_Factor[i][1] + rawData->mag[2] * Mag_Scale_Factor[i][2] + rawData->mag_offset[i];
  }
}

float inv_sqrt(float x) { //1/Sqrt(x)
  float halfx = 0.5f * x;
  float y     = x;
  long  i     = *(long*)&y;

  i = 0x5f3759df - (i >> 1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));

  return y;
}

void imu_ahrsCalculate(fp32 quat[4], const fp32 timing_time, const fp32 gyro[3], const fp32 accel[3], const fp32 mag[3]) { //��Ԫ�����㣬ʹ��IMU�ʹ�����
  volatile float gx, gy, gz, ax, ay, az, mx, my, mz; //������ԭʼ����
  float norm;
  float hx, hy, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  static float exInt, eyInt, ezInt;
  float tempq0, tempq1, tempq2, tempq3;

  fp32 Ki = 0.01f, Kp = 2.0f;

  float q0 = quat[0], q1 = quat[1], q2 = quat[2], q3 = quat[3];

  float q0q0 = q0 * q0;
  float q0q1 = q0 * q1;
  float q0q2 = q0 * q2;
  float q0q3 = q0 * q3;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q3q3 = q3 * q3;

  gx = gyro[0];   //Gyro X axis
  gy = gyro[1];   //Gyro Y axis
  gz = gyro[2];   //Gyro Z axis
  ax = accel[0];   //Acce X axis
  ay = accel[1];   //Acce Y axis
  az = accel[2];   //Acce Z axis
  mx = mag[0];   //Magn X axis
  my = mag[1];   //Magn Y axis
  mz = mag[2];   //Magn Z axis

  // Normalise accelerometer measurement ��һ�����ٶȼƲ���
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) { //ȫΪ0ʱ�򲻼���
    norm = inv_sqrt(ax * ax + ay * ay + az * az);
    ax *= norm;
    ay *= norm;
    az *= norm;
  }

  // Normalise accelerometer measurement ��һ���شżƲ���
  if(!((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))) { //ȫΪ0ʱ�򲻼���
    norm = inv_sqrt(mx * mx + my * my + mz * mz);
    mx *= norm;
    my *= norm;
    mz *= norm;
  }

  // Reference direction of Earth's magnetic field ����ų��Ĳο�����
  hx = 2.0f * mx * (0.5f - q2q2 - q3q3) + 2.0f * my * (q1q2 - q0q3) + 2.0f * mz * (q1q3 + q0q2);
  hy = 2.0f * mx * (q1q2 + q0q3) + 2.0f * my * (0.5f - q1q1 - q3q3) + 2.0f * mz * (q2q3 - q0q1);
  bx = sqrt((hx * hx) + (hy * hy));
  bz = 2.0f * mx * (q1q3 - q0q2) + 2.0f * my * (q2q3 + q0q1) + 2.0f * mz * (0.5f - q1q1 - q2q2);

  // Estimated direction of gravity and magnetic field ���Ƶ������ʹų�����
  vx = 2.0f * (q1q3 - q0q2);
  vy = 2.0f * (q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
  wx = 2.0f * bx * (0.5f - q2q2 - q3q3) + 2.0f * bz * (q1q3 - q0q2);
  wy = 2.0f * bx * (q1q2 - q0q3) + 2.0f * bz * (q0q1 + q2q3);
  wz = 2.0f * bx * (q0q2 + q1q3) + 2.0f * bz * (0.5f - q1q1 - q2q2);

  /*
   * error is sum of cross product between reference direction
   * of fields and direction measured by sensors
   * ����ǳ��Ĳο�����ʹ����������ķ���֮��Ĳ��֮��
   */
  ex = (ay * vz - az * vy) + (my * wz - mz * wy);
  ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
  ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

  if (Ki > 0.0f) { // accumulate integral error ������
    exInt += ex;
    eyInt += ey;
    ezInt += ez;
  } else { // prevent integral wind up ��������
    exInt = 0.0f;
    eyInt = 0.0f;
    ezInt = 0.0f;
  }

  // Apply feedback terms Ӧ�ø�����
  gx += Kp * ex + Ki * exInt;
  gy += Kp * ey + Ki * eyInt;
  gz += Kp * ez + Ki * ezInt;

  // Integrate rate of change of quaternion ������Ԫ���仯��
  tempq0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * (timing_time / 2.0f);
  tempq1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * (timing_time / 2.0f);
  tempq2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * (timing_time / 2.0f);
  tempq3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * (timing_time / 2.0f);

  // Normalise quaternion ��һ�����
  norm = inv_sqrt(tempq0 * tempq0 + tempq1 * tempq1 + tempq2 * tempq2 + tempq3 * tempq3);
  quat[0] = tempq0 * norm;
  quat[1] = tempq1 * norm;
  quat[2] = tempq2 * norm;
  quat[3] = tempq3 * norm;
}

void imu_ahrsCalculate_IMUonly(fp32 quat[4], const fp32 timing_time, const fp32 gyro[3], const fp32 accel[3]) { //��Ԫ�����㣬��ʹ��IMU�����ô�����
  volatile float gx, gy, gz, ax, ay, az; //������ԭʼ����
  float norm;
  float vx, vy, vz;
  float ex, ey, ez;
  static float exInt, eyInt, ezInt;
  float tempq0, tempq1, tempq2, tempq3;

  fp32 Ki = 0.01f, Kp = 2.0f;

  float q0 = quat[0], q1 = quat[1], q2 = quat[2], q3 = quat[3];

  float q0q0 = q0 * q0;
  float q0q1 = q0 * q1;
  float q0q2 = q0 * q2;
  float q0q3 = q0 * q3;
  float q1q1 = q1 * q1;
  float q1q2 = q1 * q2;
  float q1q3 = q1 * q3;
  float q2q2 = q2 * q2;
  float q2q3 = q2 * q3;
  float q3q3 = q3 * q3;

  gx = gyro[0];   //Gyro X axis
  gy = gyro[1];   //Gyro Y axis
  gz = gyro[2];   //Gyro Z axis
  ax = accel[0];   //Acce X axis
  ay = accel[1];   //Acce Y axis
  az = accel[2];   //Acce Z axis

  // Normalise accelerometer measurement ��һ�����ٶȼƲ���
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) { //ȫΪ0ʱ�򲻼���
    norm = inv_sqrt(ax * ax + ay * ay + az * az);
    ax *= norm;
    ay *= norm;
    az *= norm;
  }

  // Estimated direction of gravity ���Ƶ���������
  vx = 2.0f * (q1q3 - q0q2);
  vy = 2.0f * (q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;

  /*
   * error is sum of cross product between reference direction
   * of fields and direction measured by sensors
   * ����ǳ��Ĳο�����ʹ����������ķ���֮��Ĳ��֮��
   */
  ex = (ay * vz - az * vy);
  ey = (az * vx - ax * vz);
  ez = (ax * vy - ay * vx);

  if (Ki > 0.0f) { // accumulate integral error ������
    exInt += ex;
    eyInt += ey;
    ezInt += ez;
  } else { // prevent integral wind up ��������
    exInt = 0.0f;
    eyInt = 0.0f;
    ezInt = 0.0f;
  }

  // Apply feedback terms Ӧ�ø�����
  gx += Kp * ex + Ki * exInt;
  gy += Kp * ey + Ki * eyInt;
  gz += Kp * ez + Ki * ezInt;

  // Integrate rate of change of quaternion ������Ԫ���仯��
  tempq0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * (timing_time / 2.0f);
  tempq1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * (timing_time / 2.0f);
  tempq2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * (timing_time / 2.0f);
  tempq3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * (timing_time / 2.0f);

  // Normalise quaternion ��һ�����
  norm = inv_sqrt(tempq0 * tempq0 + tempq1 * tempq1 + tempq2 * tempq2 + tempq3 * tempq3);
  quat[0] = tempq0 * norm;
  quat[1] = tempq1 * norm;
  quat[2] = tempq2 * norm;
  quat[3] = tempq3 * norm;
}

void imu_EulerAngleUpdate(const fp32 quat[4], fp32 *yaw, fp32 *pitch, fp32 *roll) { //ŷ���Ǽ���
  float q0 = quat[0], q1 = quat[1], q2 = quat[2], q3 = quat[3];
  /* yaw    -pi----pi */
  *yaw = -atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1);
  /* pitch  -pi/2----pi/2 */
  *pitch = -asin(-2 * q1 * q3 + 2 * q0 * q2);
  /* roll   -pi----pi  */
  *roll =  atan2(2 * q2 * q3 + 2 * q0 * q1, - 2 * q1 * q1 - 2 * q2 * q2 + 1);
}

