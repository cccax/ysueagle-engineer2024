#include "robot.h"

robot_information_t robotInf;

void robotInit() {
  robotInf.robotMode = ROBOT_INIT;
  robotInf.modeStep = ROBOT_INIT_IMU;
}
