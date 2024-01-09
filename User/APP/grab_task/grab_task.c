#include "grab_task.h"
#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "robot.h"
#include "mf9025.h"
#include "messageBoard_B.h"

grabCtrl_t grabTaskStructure;

//取放矿仿真原始数据
float rawData_Servo[RAWDATA_LENGTH]={-88.141,-88.046,-87.952,-87.858,-87.764,-87.67,-87.576,-87.481,-87.387,-87.293,-87.199,-87.105,-87.011,-86.916,-86.822,-86.728,-86.634,-86.54,-86.446,-86.351,-86.257,-86.163,-86.069,-85.975,-85.881,-85.786,-85.692,-85.598,-85.504,-85.41,-85.316,-85.221,-85.127,-85.032,-84.934,-84.836,-84.738,-84.641,-84.543,-84.445,-84.347,-84.25,-84.152,-84.054,-83.956,-83.858,-83.761,-83.663,-83.565,-83.467,-83.369,-83.272,-83.174,-83.076,-82.978,-82.88,-82.783,-82.685,-82.587,-82.489,-82.392,-82.294,-82.196,-82.098,-82,-81.903,-81.808,-81.703,-81.601,-81.499,-81.398,-81.296,-81.194,-81.092,-80.99,-80.889,-80.787,-80.685,-80.583,-80.481,-80.38,-80.278,-80.176,-80.074,-79.972,-79.871,-79.769,-79.667,-79.565,-79.463,-79.362,-79.26,-79.158,-79.056,-78.954,-78.853,-78.751,-78.649,-78.547,-78.445,-78.344,-78.238,-78.132,-78.026,-77.92,-77.814,-77.708,-77.602,-77.496,-77.39,-77.283,-77.177,-77.071,-76.965,-76.859,-76.753,-76.647,-76.541,-76.435,-76.329,-76.223,-76.117,-76.011,-75.905,-75.798,-75.692,-75.586,-75.48,-75.374,-75.268,-75.162,-75.056,-74.95,-74.843,-74.732,-74.622,-74.511,-74.401,-74.291,-74.18,-74.07,-73.959,-73.849,-73.739,-73.628,-73.518,-73.407,-73.297,-73.187,-73.076,-72.966,-72.856,-72.745,-72.635,-72.524,-72.414,-72.304,-72.193,-72.083,-71.972,-71.862,-71.752,-71.641,-71.531,-71.42,-71.31,-71.2,-71.085,-70.971,-70.856,-70.742,-70.627,-70.513,-70.399,-70.284,-70.17,-70.055,-69.941,-69.826,-69.712,-69.597,-69.483,-69.368,-69.254,-69.139,-69.025,-68.911,-68.796,-68.682,-68.567,-68.453,-68.338,-68.224,-68.109,-67.995,-67.88,-67.766,-67.651,-67.537,-67.422,-67.306,-67.188,-67.07,-66.952,-66.834,-66.716,-66.598,-66.48,-66.362,-66.244,-66.126,-66.008,-65.89,-65.772,-65.654,-65.536,-65.418,-65.3,-65.182,-65.064,-64.946,-64.828,-64.711,-64.593,-64.475,-64.357,-64.239,-64.121,-64.003,-63.885,-63.767,-63.649,-63.531,-63.412,-63.292,-63.171,-63.051,-62.93,-62.809,-62.689,-62.568,-62.448,-62.327,-62.207,-62.086,-61.966,-61.845,-61.725,-61.604,-61.484,-61.363,-61.243,-61.122,-61.001,-60.881,-60.76,-60.64,-60.519,-60.399,-60.278,-60.158,-60.037,-59.917,-59.796,-59.676,-59.555,-59.435,-59.313,-59.191,-59.069,-58.947,-58.825,-58.704,-58.582,-58.46,-58.338,-58.216,-58.094,-57.973,-57.851,-57.729,-57.607,-57.485,-57.363,-57.242,-57.12,-56.998,-56.876,-56.754,-56.632,-56.511,-56.389,-56.267,-56.145,-56.023,-55.901,-55.78,-55.658,-55.536,-55.414,-55.292,-55.171,-55.049,-54.927,-54.806,-54.684,-54.563,-54.441,-54.319,-54.198,-54.076,-53.954,-53.833,-53.711,-53.589,-53.468,-53.346,-53.225,-53.103,-52.981,-52.86,-52.738,-52.616,-52.495,-52.373,-52.252,-52.13,-52.008,-51.887,-51.765,-51.643,-51.522,-51.4,-51.279,-51.159,-51.039,-50.92,-50.8,-50.68,-50.56,-50.44,-50.321,-50.201,-50.081,-49.961,-49.842,-49.722,-49.602,-49.482,-49.362,-49.243,-49.123,-49.003,-48.883,-48.763,-48.644,-48.524,-48.404,-48.284,-48.164,-48.045,-47.925,-47.805,-47.685,-47.565,-47.446,-47.326,-47.208,-47.089,-46.971,-46.853,-46.735,-46.617,-46.499,-46.38,-46.262,-46.144,-46.026,-45.908,-45.789,-45.671,-45.553,-45.435,-45.317,-45.199,-45.08,-44.962,-44.844,-44.726,-44.608,-44.489,-44.371,-44.253,-44.135,-44.017,-43.898,-43.78,-43.662,-43.544,-43.426,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382,-43.382};
const float rawData_armmidp[RAWDATA_LENGTH]={27.517,27.436,27.356,27.275,27.194,27.114,27.033,26.952,26.872,26.791,26.71,26.63,26.549,26.468,26.388,26.307,26.226,26.145,26.065,25.984,25.903,25.823,25.742,25.661,25.581,25.5,25.419,25.339,25.258,25.177,25.097,25.016,24.935,24.858,24.787,24.717,24.646,24.575,24.504,24.434,24.363,24.292,24.222,24.151,24.08,24.01,23.939,23.868,23.798,23.727,23.656,23.585,23.515,23.444,23.373,23.303,23.232,23.161,23.091,23.02,22.949,22.878,22.808,22.737,22.666,22.596,22.524,22.465,22.405,22.345,22.285,22.226,22.166,22.106,22.046,21.986,21.926,21.866,21.806,21.746,21.686,21.627,21.567,21.507,21.447,21.387,21.327,21.267,21.207,21.147,21.087,21.028,20.968,20.908,20.848,20.788,20.728,20.668,20.608,20.548,20.498,20.45,20.401,20.353,20.305,20.257,20.208,20.16,20.112,20.064,20.015,19.967,19.919,19.871,19.822,19.774,19.726,19.678,19.629,19.581,19.533,19.485,19.436,19.388,19.34,19.292,19.243,19.195,19.147,19.099,19.05,19.002,18.954,18.909,18.874,18.838,18.802,18.766,18.73,18.695,18.659,18.623,18.587,18.551,18.516,18.48,18.444,18.408,18.373,18.337,18.301,18.265,18.229,18.194,18.158,18.122,18.086,18.05,18.015,17.979,17.943,17.907,17.872,17.836,17.8,17.764,17.728,17.705,17.683,17.66,17.637,17.615,17.592,17.57,17.547,17.524,17.502,17.479,17.456,17.434,17.411,17.388,17.366,17.343,17.321,17.298,17.275,17.253,17.23,17.207,17.185,17.162,17.139,17.117,17.094,17.072,17.049,17.026,17.004,16.981,16.967,16.958,16.949,16.94,16.931,16.922,16.913,16.904,16.895,16.886,16.877,16.868,16.859,16.85,16.841,16.832,16.823,16.814,16.805,16.796,16.787,16.778,16.769,16.76,16.751,16.742,16.733,16.724,16.715,16.706,16.697,16.688,16.679,16.674,16.679,16.684,16.689,16.693,16.698,16.703,16.708,16.713,16.718,16.722,16.727,16.732,16.737,16.742,16.747,16.751,16.756,16.761,16.766,16.771,16.775,16.78,16.785,16.79,16.795,16.8,16.804,16.809,16.814,16.819,16.824,16.829,16.833,16.851,16.87,16.889,16.907,16.926,16.944,16.963,16.981,17,17.018,17.037,17.056,17.074,17.093,17.111,17.13,17.148,17.167,17.185,17.204,17.222,17.241,17.26,17.278,17.297,17.315,17.334,17.352,17.371,17.389,17.408,17.427,17.445,17.472,17.504,17.536,17.568,17.6,17.631,17.663,17.695,17.727,17.759,17.791,17.823,17.855,17.886,17.918,17.95,17.982,18.014,18.046,18.078,18.11,18.141,18.173,18.205,18.237,18.269,18.301,18.333,18.365,18.396,18.428,18.46,18.492,18.528,18.572,18.617,18.661,18.706,18.75,18.795,18.84,18.884,18.929,18.973,19.018,19.062,19.107,19.152,19.196,19.241,19.285,19.33,19.374,19.419,19.463,19.508,19.553,19.597,19.642,19.686,19.731,19.775,19.82,19.865,19.909,19.954,19.998,20.034,20.07,20.106,20.141,20.177,20.213,20.248,20.284,20.32,20.356,20.391,20.427,20.463,20.498,20.534,20.57,20.606,20.641,20.677,20.713,20.748,20.784,20.82,20.855,20.891,20.927,20.963,20.998,21.034,21.07,21.105,21.141,21.177,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19,21.19};
float rawData_armbasep[RAWDATA_LENGTH]={-29.379,-29.392,-29.406,-29.419,-29.433,-29.446,-29.46,-29.473,-29.487,-29.5,-29.514,-29.527,-29.541,-29.554,-29.568,-29.581,-29.595,-29.608,-29.622,-29.635,-29.649,-29.662,-29.676,-29.689,-29.703,-29.716,-29.73,-29.743,-29.757,-29.77,-29.784,-29.797,-29.811,-29.829,-29.856,-29.883,-29.91,-29.937,-29.964,-29.991,-30.018,-30.045,-30.072,-30.1,-30.127,-30.154,-30.181,-30.208,-30.235,-30.262,-30.289,-30.316,-30.343,-30.371,-30.398,-30.425,-30.452,-30.479,-30.506,-30.533,-30.56,-30.587,-30.614,-30.641,-30.669,-30.696,-30.722,-30.765,-30.807,-30.849,-30.89,-30.932,-30.974,-31.016,-31.058,-31.1,-31.142,-31.184,-31.226,-31.268,-31.309,-31.351,-31.393,-31.435,-31.477,-31.519,-31.561,-31.603,-31.645,-31.687,-31.729,-31.77,-31.812,-31.854,-31.896,-31.938,-31.98,-32.022,-32.064,-32.106,-32.156,-32.214,-32.272,-32.33,-32.388,-32.445,-32.503,-32.561,-32.619,-32.677,-32.735,-32.792,-32.85,-32.908,-32.966,-33.024,-33.082,-33.139,-33.197,-33.255,-33.313,-33.371,-33.429,-33.486,-33.544,-33.602,-33.66,-33.718,-33.776,-33.833,-33.891,-33.949,-34.007,-34.07,-34.144,-34.219,-34.294,-34.368,-34.443,-34.517,-34.592,-34.667,-34.741,-34.816,-34.89,-34.965,-35.04,-35.114,-35.189,-35.263,-35.338,-35.413,-35.487,-35.562,-35.636,-35.711,-35.786,-35.86,-35.935,-36.009,-36.084,-36.159,-36.233,-36.308,-36.383,-36.457,-36.532,-36.623,-36.715,-36.807,-36.899,-36.99,-37.082,-37.174,-37.266,-37.358,-37.45,-37.541,-37.633,-37.725,-37.817,-37.909,-38.001,-38.092,-38.184,-38.276,-38.368,-38.46,-38.552,-38.643,-38.735,-38.827,-38.919,-39.011,-39.103,-39.194,-39.286,-39.378,-39.47,-39.562,-39.664,-39.773,-39.882,-39.991,-40.1,-40.209,-40.318,-40.427,-40.536,-40.645,-40.754,-40.863,-40.972,-41.081,-41.19,-41.299,-41.408,-41.517,-41.626,-41.735,-41.844,-41.953,-42.062,-42.171,-42.28,-42.389,-42.498,-42.607,-42.716,-42.825,-42.934,-43.043,-43.152,-43.265,-43.391,-43.516,-43.641,-43.767,-43.892,-44.018,-44.143,-44.268,-44.394,-44.519,-44.644,-44.77,-44.895,-45.021,-45.146,-45.271,-45.397,-45.522,-45.647,-45.773,-45.898,-46.023,-46.149,-46.274,-46.4,-46.525,-46.65,-46.776,-46.901,-47.026,-47.152,-47.277,-47.402,-47.542,-47.683,-47.823,-47.964,-48.104,-48.244,-48.385,-48.525,-48.666,-48.806,-48.946,-49.087,-49.227,-49.367,-49.508,-49.648,-49.789,-49.929,-50.069,-50.21,-50.35,-50.491,-50.631,-50.771,-50.912,-51.052,-51.193,-51.333,-51.473,-51.614,-51.754,-51.895,-52.035,-52.184,-52.337,-52.491,-52.644,-52.798,-52.951,-53.105,-53.258,-53.412,-53.565,-53.719,-53.872,-54.026,-54.179,-54.333,-54.486,-54.64,-54.793,-54.947,-55.1,-55.254,-55.407,-55.561,-55.714,-55.868,-56.021,-56.175,-56.328,-56.482,-56.635,-56.789,-56.942,-57.096,-57.253,-57.417,-57.581,-57.746,-57.91,-58.075,-58.239,-58.403,-58.568,-58.732,-58.896,-59.061,-59.225,-59.389,-59.554,-59.718,-59.883,-60.047,-60.211,-60.376,-60.54,-60.704,-60.869,-61.033,-61.197,-61.362,-61.526,-61.691,-61.855,-62.019,-62.184,-62.348,-62.512,-62.677,-62.831,-62.985,-63.139,-63.293,-63.447,-63.6,-63.754,-63.908,-64.062,-64.216,-64.37,-64.524,-64.678,-64.832,-64.985,-65.139,-65.293,-65.447,-65.601,-65.755,-65.909,-66.063,-66.217,-66.371,-66.524,-66.678,-66.832,-66.986,-67.14,-67.294,-67.448,-67.602,-67.756,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812,-67.812};
	 
void grabBehaviourSelect(void);
void grabCtrlChangeHandle(void);
void grabFeedbackUpdate(void);
void grabCanBusCtrlMotors(s16 parallelMotorCurrent1,s16 parallelMotorCurrent2,
														s16 armbasepMotorCurrent1, s16 armbasepMotorCurrent2, s16 armmidyMotorCurrent,
														s16 xplatMotorCurrent1,s16 yplatMotorCurrent1,  s16 yplatMotorCurrent2, s16 liftoreMotorCurrent1,  s16 liftoreMotorCurrent2);
void grabPidCalc(void);
void grabMotorLimit(void);
void grabBehaviourInit(grab_behaviour_t *initBehavior, grab_behaviour_e num, grab_motor_mode_e armbasepMode, 
	                     grab_motor_mode_e armmidpMode, grab_motor_mode_e singlerotateMode, grab_motor_mode_e xplatMode, grab_motor_mode_e yplatMode, 
                       grab_motor_mode_e liftoreMode, bool_t (*enterBehaviorCondition)(void), bool_t (*outBehaviorCondition)(void), void (*enterBehaviorFun)(void), void (*outBehaviorFun)(void), 
											 void (*behaviorHandleFun)(float *armbaseyExp, float *armbasepExp, float *armmidpExp, float *singlerotateExp, float *xplatExp, float *yplatExp, float *liftoreExp));


void grabTask(void *pvParameters) {
  portTickType currentTime;
	
  grabInit();
	
	valve_Init(); //电磁阀初始化
	
	 //grabTaskStructure.oreServo.angleSet=grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_NORMAL]; //舵机初始化
	
//	MF9025_Motor_Ecd_SetZero(CAN2,1);  //MF9025设置当前为0点
//	MF9025_Motor_Ecd_SetZero(CAN2,2);  //MF9025设置当前为0点

  vTaskDelay(4);
  currentTime = xTaskGetTickCount();
  grabTaskStructure.behaviourStep = 0;

  while(1) {
		
    //更新当前抓取的模式
    grabBehaviourSelect();

    //更新实际值数据 要先做 防止下一步传入错误数值
    grabFeedbackUpdate();

    //电机模式转换处理
    grabCtrlChangeHandle();

    //期望值设定
		float *armbaseyExp;//机械臂 基座yaw轴 旋转电机期望
		float *armbasepExp;//Add：机械臂 基座pitch轴 旋转电机(MF9025)期望
		float *armmidpExp; //机械臂 中部pitch轴 旋转电机期望
		float *singlerotateExp;//夹子期望
		float *xplatExp;//平台X方向平移期望
		float *yplatExp;//Add：平台Y方向平移期望 
		float *liftoreExp;//矿仓提升期望

//    //机械臂底部yaw轴旋转期望
//    if(grabTaskStructure.armbaseyMotor.mode == GRAB_MOTOR_POSITION)//位置双环控制
//      armbaseyExp = &(grabTaskStructure.armbaseyMotor.totalEcdSet);
//    else if(grabTaskStructure.armbaseyMotor.mode == GRAB_MOTOR_SPEED)//速度单环控制
//      armbaseyExp = &(grabTaskStructure.armbaseyMotor.speedSet);
//    else if(grabTaskStructure.armbaseyMotor.mode == GRAB_MOTOR_RAW)//原始电流控制
//      armbaseyExp = &(grabTaskStructure.armbaseyMotor.rawCmdCurrent);

    //Add：机械臂底部pitch轴旋转期望
    if(grabTaskStructure.armbasepMotor1.mode == GRAB_MOTOR_POSITION)//位置双环控制
      armbasepExp = &(grabTaskStructure.armbasepMotor1.totalEcdSet);
    else if(grabTaskStructure.armbasepMotor1.mode == GRAB_MOTOR_SPEED)//速度单环控制
      armbasepExp = &(grabTaskStructure.armbasepMotor1.speedSet);
    else if(grabTaskStructure.armbasepMotor1.mode == GRAB_MOTOR_RAW)//原始电流控制
      armbasepExp = &(grabTaskStructure.armbasepMotor1.rawCmdCurrent);
		
    //机械臂中部pitch轴旋转期望//cjh yaw
    if(grabTaskStructure.armmidyMotor.mode == GRAB_MOTOR_POSITION)//位置双环控制
      armmidpExp = &(grabTaskStructure.armmidyMotor.totalEcdSet);
    else if(grabTaskStructure.armmidyMotor.mode == GRAB_MOTOR_SPEED)//速度单环控制
      armmidpExp = &(grabTaskStructure.armmidyMotor.speedSet);
    else if(grabTaskStructure.armmidyMotor.mode == GRAB_MOTOR_RAW)//原始电流控制
      armmidpExp = &(grabTaskStructure.armmidyMotor.rawCmdCurrent);

    //cjh:并联机构期望
    if(grabTaskStructure.parallelMotor1.mode == GRAB_MOTOR_POSITION)//位置双环控制
      singlerotateExp = &(grabTaskStructure.parallelMotor1.totalEcdSet);
    else if(grabTaskStructure.parallelMotor1.mode == GRAB_MOTOR_SPEED)//速度单环控制
      singlerotateExp = &(grabTaskStructure.parallelMotor1.speedSet);
    else if(grabTaskStructure.parallelMotor1.mode == GRAB_MOTOR_RAW)//原始电流控制
      singlerotateExp = &(grabTaskStructure.parallelMotor1.rawCmdCurrent);
    //爪子旋转期望 //待删除cjh
    if(grabTaskStructure.singlerotateMotor.mode == GRAB_MOTOR_POSITION)//位置双环控制
      singlerotateExp = &(grabTaskStructure.singlerotateMotor.totalEcdSet);
    else if(grabTaskStructure.singlerotateMotor.mode == GRAB_MOTOR_SPEED)//速度单环控制
      singlerotateExp = &(grabTaskStructure.singlerotateMotor.speedSet);
    else if(grabTaskStructure.singlerotateMotor.mode == GRAB_MOTOR_RAW)//原始电流控制
      singlerotateExp = &(grabTaskStructure.singlerotateMotor.rawCmdCurrent);
    //平台X方向移动期望
    if(grabTaskStructure.xplatMotor1.mode == GRAB_MOTOR_POSITION)//位置双环控制
      xplatExp = &(grabTaskStructure.xplatMotor1.totalEcdSet);
    else if(grabTaskStructure.xplatMotor1.mode == GRAB_MOTOR_SPEED)//速度单环控制
      xplatExp = &(grabTaskStructure.xplatMotor1.speedSet);
    else if(grabTaskStructure.xplatMotor1.mode == GRAB_MOTOR_RAW)//原始电流控制
      xplatExp = &(grabTaskStructure.xplatMotor1.rawCmdCurrent);
		
    //Add：平台Y方向移动期望
    if(grabTaskStructure.yplatMotor1.mode == GRAB_MOTOR_POSITION)//位置双环控制
      yplatExp = &(grabTaskStructure.yplatMotor1.totalEcdSet);
    else if(grabTaskStructure.yplatMotor1.mode == GRAB_MOTOR_SPEED)//速度单环控制
      yplatExp = &(grabTaskStructure.yplatMotor1.speedSet);
    else if(grabTaskStructure.yplatMotor1.mode == GRAB_MOTOR_RAW)//原始电流控制
      yplatExp = &(grabTaskStructure.yplatMotor1.rawCmdCurrent);
		
    //平台提升期望
    if(grabTaskStructure.liftoreMotor1.mode == GRAB_MOTOR_POSITION)//位置双环控制
      liftoreExp = &(grabTaskStructure.liftoreMotor1.totalEcdSet);
    else if(grabTaskStructure.liftoreMotor1.mode == GRAB_MOTOR_SPEED)//速度单环控制
      liftoreExp = &(grabTaskStructure.liftoreMotor1.speedSet);
    else if(grabTaskStructure.liftoreMotor1.mode == GRAB_MOTOR_RAW)//原始电流控制
      liftoreExp = &(grabTaskStructure.liftoreMotor1.rawCmdCurrent);
		
    grabTaskStructure.nowBehaviour->behaviorHandleFun(armbaseyExp,armbasepExp,armmidpExp,singlerotateExp,xplatExp,yplatExp,liftoreExp);

    //位置限幅
    grabMotorLimit();

    //更新遥控器参数
    rcDataCopy(&(grabTaskStructure.rc.last));

    //grabCtrlChangeHandle();

    //PID计算
    grabPidCalc();

    //CAN发送
    grabCanBusCtrlMotors(	//grabTaskStructure.singlerotateMotor.baseInf.given_current,
		                      grabTaskStructure.parallelMotor1.baseInf.given_current,
		                      grabTaskStructure.parallelMotor2.baseInf.given_current,
													//grabTaskStructure.armbaseyMotor.baseInf.given_current,
													grabTaskStructure.armbasepMotor1.baseInf.given_current,
													grabTaskStructure.armbasepMotor2.baseInf.given_current,
                          grabTaskStructure.armmidyMotor.baseInf.given_current,
                          grabTaskStructure.xplatMotor1.baseInf.given_current,
													grabTaskStructure.yplatMotor1.baseInf.given_current,//addcjh
													grabTaskStructure.yplatMotor2.baseInf.given_current, //Add
													//grabTaskStructure.yplatMotor2.baseInf.given_current, //Addcjh
                          grabTaskStructure.liftoreMotor1.baseInf.given_current,
                          grabTaskStructure.liftoreMotor2.baseInf.given_current
													);
		
    vTaskDelayUntil(&currentTime, GRAB_TASK_MS);
  }
}

void grabInit() 
{
	grabTaskStructure.currentExchlevel = EXCH_LEVEL_0;//默认等级0
	
  //获取遥控器数据
  grabTaskStructure.rc.now = get_remote_control_point();

  //电机参数初始化
//  motorInit(&(grabTaskStructure.armbaseyMotor.baseInf), 19);									//机械臂底部yaw轴电机
//  motorInit(&(grabTaskStructure.armbasepMotor1.baseInf), 1);                 //Add:MF9025
//	motorInit(&(grabTaskStructure.armbasepMotor2.baseInf), 1);								//Add:MF9025
	motorInit(&(grabTaskStructure.armbasepMotor1.baseInf), 19);
	motorInit(&(grabTaskStructure.armbasepMotor2.baseInf), 19);
	motorInit(&(grabTaskStructure.armmidyMotor.baseInf),  19);								//机械臂中部pitch轴电机
  //motorInit(&(grabTaskStructure.singlerotateMotor.baseInf), 19);								//爪子旋转电机
	motorInit(&(grabTaskStructure.parallelMotor1.baseInf), 36);//差动机构电机cjh
	motorInit(&(grabTaskStructure.parallelMotor2.baseInf), 36);
  motorInit(&(grabTaskStructure.xplatMotor1.baseInf), 19);								//平台X平移电机
  motorInit(&(grabTaskStructure.yplatMotor1.baseInf), 19);              //Add：平台Y平移电机
	motorInit(&(grabTaskStructure.yplatMotor2.baseInf), 19); //cjh
  motorInit(&(grabTaskStructure.liftoreMotor1.baseInf), 19);						//平台升降电机1 
  motorInit(&(grabTaskStructure.liftoreMotor2.baseInf), 19);						//平台升降电机2 
	

  //grabTaskStructure.armbaseyMotor.lastMode = GRAB_MOTOR_RAW;
	grabTaskStructure.armbasepMotor1.lastMode = GRAB_MOTOR_RAW;
	grabTaskStructure.armbasepMotor2.lastMode = GRAB_MOTOR_RAW;
  grabTaskStructure.armmidyMotor.lastMode = GRAB_MOTOR_RAW;
  grabTaskStructure.singlerotateMotor.lastMode = GRAB_MOTOR_RAW;
	grabTaskStructure.parallelMotor1.lastMode = GRAB_MOTOR_RAW;//cjh
	grabTaskStructure.parallelMotor2.lastMode = GRAB_MOTOR_RAW;//cjh
  grabTaskStructure.xplatMotor1.lastMode = GRAB_MOTOR_RAW;
	grabTaskStructure.yplatMotor1.lastMode = GRAB_MOTOR_RAW;//cjh
	grabTaskStructure.yplatMotor2.lastMode = GRAB_MOTOR_RAW;//cjh
  grabTaskStructure.liftoreMotor1.lastMode = GRAB_MOTOR_RAW;
  grabTaskStructure.liftoreMotor2.lastMode = GRAB_MOTOR_RAW;


  //标准位置值初始化 XXX_RESET均为0
	//-----------------电机--------------------------
//  grabTaskStructure.armbaseyMotor.fixedEcd[ARM_BASE_Y_RESET]  = 0;//ok
//	grabTaskStructure.armbaseyMotor.fixedEcd[ARM_BASE_Y_NORMAL]  = 2315;//ok
//	grabTaskStructure.armbaseyMotor.fixedEcd[ARM_BASE_Y_STORE_ORE]  = 10680;//ok
//	grabTaskStructure.armbaseyMotor.fixedEcd[ARM_BASE_Y_MAX] = 11500;//ok

	grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_RESET] = 0;//ok
	grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_MIN] = -25000;//ok
	grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_NORMAL] = -3000;//-4000;//ok
	grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_SMALL_ISLAND] = -5100; //okok
	grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_BIG_ISLAND] = -5100; //
	grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_ALLEY_OPP] = -5100; //ok
	grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_STORE_ORE] = -12000;  //ok
	grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_EXCH_ORE] = -13000;
	grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_STORE_START] = -5461;
	grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_EXCH_LEVEL_01] = -13600;
	grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_EXCH_LEVEL_12] = -10300;
	grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_EXCH_LEVEL_24] = -3300;
	
	
  grabTaskStructure.armmidyMotor.fixedEcd[ARM_MID_Y_RESET]  = 0;// ok
	grabTaskStructure.armmidyMotor.fixedEcd[ARM_MID_Y_NORMAL]  = 0;
	grabTaskStructure.armmidyMotor.fixedEcd[ARM_MID_Y_SMALL_ISLAND] = 0;//300; //okok or 0
	grabTaskStructure.armmidyMotor.fixedEcd[ARM_MID_Y_BIG_ISLAND] = 300;
	grabTaskStructure.armmidyMotor.fixedEcd[ARM_MID_Y_ALLEY_OPP] = 0; //ok
	grabTaskStructure.armmidyMotor.fixedEcd[ARM_MID_Y_STORE_ORE] = 0;//2800;//2537;  //ok
	grabTaskStructure.armmidyMotor.fixedEcd[ARM_MID_Y_EXCH_ORE] = 2000;
	grabTaskStructure.armmidyMotor.fixedEcd[ARM_MID_Y_PRE_ORE] = 1000;
	grabTaskStructure.armmidyMotor.fixedEcd[ARM_MID_Y_EXCH_LEVEL_01] = 3650;
	grabTaskStructure.armmidyMotor.fixedEcd[ARM_MID_Y_EXCH_LEVEL_12] = 3320;
	grabTaskStructure.armmidyMotor.fixedEcd[ARM_MID_Y_EXCH_LEVEL_24] = 3650;
  grabTaskStructure.armmidyMotor.fixedEcd[ARM_MID_Y_MAX] = 3800;// ok

  grabTaskStructure.singlerotateMotor.fixedEcd[ROTATE_SINGLE_RESET] = 0;// 
	grabTaskStructure.singlerotateMotor.fixedEcd[ROTATE_SINGLE_NORMAL] = 2250;// ok
	grabTaskStructure.singlerotateMotor.fixedEcd[ROTATE_SINGLE_MAX] = 4500;

  grabTaskStructure.xplatMotor1.fixedEcd[PLAT_X_RESET] = 0;//ok
  grabTaskStructure.xplatMotor1.fixedEcd[PLAT_X_NORMAL] = -24000;//ok
	grabTaskStructure.xplatMotor1.fixedEcd[PLAT_X_INIT_RIGHT] = -37000;//ok
	grabTaskStructure.xplatMotor1.fixedEcd[PLAT_X_RIGHT_LITTLE] = -7000; //ok
	grabTaskStructure.xplatMotor1.fixedEcd[PLAT_X_MIN] = -45000; //ok
	
	grabTaskStructure.yplatMotor1.fixedEcd[PLAT_Y_MIN] = 1500;//ok
	grabTaskStructure.yplatMotor1.fixedEcd[PLAT_Y_FRONT_LITTLE_INIT] = 5000;//ok
	grabTaskStructure.yplatMotor1.fixedEcd[PLAT_Y_FRONT_LITTLE_EXCH] = 9000;
	grabTaskStructure.yplatMotor1.fixedEcd[PLAT_Y_NORMAL] = 2000;//ok
	grabTaskStructure.yplatMotor1.fixedEcd[PLAT_Y_SMALL_ISLAND] = 10000;
	grabTaskStructure.yplatMotor1.fixedEcd[PLAT_Y_SMALL_ISLAND_LIFT] = 3000;
	grabTaskStructure.yplatMotor1.fixedEcd[PLAT_Y_BIG_ISLAND] = 10000;
	grabTaskStructure.yplatMotor1.fixedEcd[PLAT_Y_ALLEY_OPP] = 2000;
	grabTaskStructure.yplatMotor1.fixedEcd[PLAT_Y_STORE_ORE] = 20000;//24000;  //ok
	grabTaskStructure.yplatMotor1.fixedEcd[PLAT_Y_EXCH_ORE] = 2200;  //ok
	grabTaskStructure.yplatMotor1.fixedEcd[PLAT_Y_FRONT_MAX] = 29000;//ok
//------------------cjh-------------------
	grabTaskStructure.yplatMotor2.fixedEcd[PLAT_Y_MIN] = 1500;//ok
	grabTaskStructure.yplatMotor2.fixedEcd[PLAT_Y_FRONT_LITTLE_INIT] = 5000;//ok
	grabTaskStructure.yplatMotor2.fixedEcd[PLAT_Y_FRONT_LITTLE_EXCH] = 9000;
	grabTaskStructure.yplatMotor2.fixedEcd[PLAT_Y_NORMAL] = 2000;//ok
	grabTaskStructure.yplatMotor2.fixedEcd[PLAT_Y_SMALL_ISLAND] = 10000;
	grabTaskStructure.yplatMotor2.fixedEcd[PLAT_Y_SMALL_ISLAND_LIFT] = 3000;
	grabTaskStructure.yplatMotor2.fixedEcd[PLAT_Y_BIG_ISLAND] = 10000;
	grabTaskStructure.yplatMotor2.fixedEcd[PLAT_Y_ALLEY_OPP] = 2000;
	grabTaskStructure.yplatMotor2.fixedEcd[PLAT_Y_STORE_ORE] = 20000;//24000;  //ok
	grabTaskStructure.yplatMotor2.fixedEcd[PLAT_Y_EXCH_ORE] = 2200;  //ok
	grabTaskStructure.yplatMotor2.fixedEcd[PLAT_Y_FRONT_MAX] = 29000;//ok
	//-----------------------
	grabTaskStructure.liftoreMotor1.fixedEcd[LIFT_RESET] = 0;
	grabTaskStructure.liftoreMotor1.fixedEcd[LIFT_NORMAL] = 0; //ok
	grabTaskStructure.liftoreMotor1.fixedEcd[LIFT_SMALL_ISLAND] = 0;
	grabTaskStructure.liftoreMotor1.fixedEcd[LIFT_BIG_ISLAND] = 5000;
	grabTaskStructure.liftoreMotor1.fixedEcd[LIFT_ALLEY_OPP] = 9200;
	grabTaskStructure.liftoreMotor1.fixedEcd[LIFT_LITTLE_UP] = 3000;
  grabTaskStructure.liftoreMotor1.fixedEcd[LIFT_MAX] = 9200; //ok

	//---------------------舵机-----------------
//	grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_DIRECT_GROUND] = 250; //ok
//	grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_NORMAL] = 150; //ok
//	grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_NORMAL_COMP] = 130;
//	grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_SMALL_ISLAND_COMP] = 100;
//	grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_STORE_ORE] = 60;
//	grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_ALLEY_OPP] = 60;
//	grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_EXCH_LEVEL_01] = 130;
//	grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_EXCH_LEVEL_12] = 150;
//	grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_EXCH_LEVEL_24] = 195;
//	grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_DIRECT_STORE] = 90;
//	grabTaskStructure.oreServo.fixedAngle[SERVO_ORE_DIRECT_EXCH] = 100;


  //void PIDInit(PidTypeDef *pid,float kp,float ki,float kd,float ka,float max_out,float dead_band,float i_band,float max_input, float i_maxout, pid_mode_e model)
  //																																	   kp	,		ki	,	 kd	, ka, max_out	,dead_band,i_band	,max_input, i_maxout, pid_mode_e model
//  PIDInit(&(grabTaskStructure.armbaseyMotor.pidParameter[INNER])			, 70	, 12.5	, 300, 0	, 10000		,		0	  	,	0	,			-1	,		3000	, SPEED);		//OK
//  PIDInit(&(grabTaskStructure.armbaseyMotor.pidParameter[OUTER])			, 0.4	, 0.006	, 0.3	, 0	, 10000		,		5			, 200	, 		-1	, 	500			, SPEED);
	//armbasey备份
//	PIDInit(&(grabTaskStructure.armbaseyMotor.pidParameter[INNER])			, 70	, 12.5	, 300, 0	, 10000		,		0	  	,	0	,			-1	,		3000	, SPEED);		//OK
//  PIDInit(&(grabTaskStructure.armbaseyMotor.pidParameter[OUTER])			, 0.4	, 0.005	, 0.1	, 0	, 10000		,		5			, 200	, 		-1	, 	500			, SPEED);
	//---------------------------------------------------------------------------------------------------------------------------------------
  PIDInit(&(grabTaskStructure.armbasepMotor1.pidParameter[INNER])    ,  3 , 0 , 8   , 0 , 1500	,	 0	, 0 ,	-1	, 5000	, SPEED);
  PIDInit(&(grabTaskStructure.armbasepMotor1.pidParameter[OUTER])    ,0.2, 0.001   , 0.08 , 0 , 1500	,	 0	, 300	,	-1	, 2000	, SPEED);


//armbasep1备份	PIDInit(&(grabTaskStructure.armbasepMotor1.pidParameter[INNER])    ,  3 , 800 , 8   , 0 , 1000	,	 0	, 0 ,	-1	, 5000	, SPEED);
//  PIDInit(&(grabTaskStructure.armbasepMotor1.pidParameter[OUTER])    ,0.25, 0   , 0.1 , 0 , 1000	,	 0	, 200	,	-1	, 5000	, SPEED);
	//armbasepMotor2用不到
  PIDInit(&(grabTaskStructure.armbasepMotor2.pidParameter[INNER])    ,  3 , 800 , 8   , 0 , 1000	,	 0	, 0	,	-1	, 5000	, SPEED);
  PIDInit(&(grabTaskStructure.armbasepMotor2.pidParameter[OUTER])    ,0.25, 0   , 0.1 , 0 , 1000	,	 0	, 0	,	-1	, 5000	, SPEED);
 	//---------------------------------------------------------------------------------------------------------------------------------------
  PIDInit(&(grabTaskStructure.armmidyMotor.pidParameter[INNER])		   , 150	, 0	, 200   ,0, 15000	,		0	,	0	,	-1	,	5000	, SPEED);	
  PIDInit(&(grabTaskStructure.armmidyMotor.pidParameter[OUTER])		   , 0.15, 0.008, 0.2	  , 0 , 15000   , 	0	, 200	,	-1  , 	500		, SPEED);
	//---------------------------------------------------------------------------------------------------------------------------------------
  PIDInit(&(grabTaskStructure.singlerotateMotor.pidParameter[INNER])	, 180	 , 1		, 200 , 0	, 10000		,		0			,	0	,			-1	,		3000	, SPEED);		//OK
  PIDInit(&(grabTaskStructure.singlerotateMotor.pidParameter[OUTER])	, 0.1  ,  0.001  , 0.2	, 0	, 10000		, 	0			, 500			,		 	-1	, 	1000			, SPEED);
	//---------------------------------------------------------------------------------------------------------------------------------------
  PIDInit(&(grabTaskStructure.xplatMotor1.pidParameter[INNER])  ,150  , 20 ,  300  , 0 , 10000	,	 0	, 0	, -1	, 3000	, SPEED);
  PIDInit(&(grabTaskStructure.xplatMotor1.pidParameter[OUTER])  , 0.1 , 0.005 ,  0.4   , 0 , 10000	,	 10	, 300	,	-1	, 5000	, SPEED);
  //----------------------cjh------------------------------------------
	PIDInit(&(grabTaskStructure.yplatMotor2.pidParameter[INNER])	, 120	, 0			, 1200, 0	, 10000		,		0			,	4000	,			-1	,		5000	, SPEED);		
  PIDInit(&(grabTaskStructure.yplatMotor2.pidParameter[OUTER])	, 0.5 , 0     , 0.5	, 0	, 10000		, 	0			, 0			,		 	-1	, 	0			, SPEED);
	
	//Additional-----------------------------------------------------------------------------------------------------------------------------
  PIDInit(&(grabTaskStructure.yplatMotor1.pidParameter[INNER])	, 120	, 0			, 1200, 0	, 10000		,		0			,	4000	,			-1	,		5000	, SPEED);		
  PIDInit(&(grabTaskStructure.yplatMotor1.pidParameter[OUTER])	, 0.5 , 0     , 0.5	, 0	, 10000		, 	0			, 0			,		 	-1	, 	0			, SPEED);
	//---------------------------------------------------------------------------------------------------------------------------------------
  PIDInit(&(grabTaskStructure.liftoreMotor1.pidParameter[INNER])    , 250  , 0.1 , 250  , 0 , 15000	,	 0	, 0 ,	-1	, 5000	, SPEED);
  PIDInit(&(grabTaskStructure.liftoreMotor1.pidParameter[OUTER])    , 0.2, 0.001 , 0.5   , 0 , 15000	,	 0	, 400	,	-1	, 5000	, SPEED);
  PIDInit(&(grabTaskStructure.liftoreMotor2.pidParameter[INNER])    , 250  , 0.1 , 250  , 0 , 15000	,	 0	, 0	,	-1	, 5000	, SPEED);
  PIDInit(&(grabTaskStructure.liftoreMotor2.pidParameter[OUTER])    , 0.2, 0.001 , 0.5   , 0 , 15000	,	 0	, 400	,	-1	, 5000	, SPEED);


  //卡尔曼电机速度滤波
  //KalmanCreate(&(grabTaskStructure.armbaseyMotor.klmFiller)		, 1, 40);//
	KalmanCreate(&(grabTaskStructure.armbasepMotor1.klmFiller)	, 1, 40);//Add
	KalmanCreate(&(grabTaskStructure.armbasepMotor2.klmFiller)	, 1, 40);//Add
  KalmanCreate(&(grabTaskStructure.armmidyMotor.klmFiller)	, 1, 40);//
  //KalmanCreate(&(grabTaskStructure.singlerotateMotor.klmFiller)		, 1, 40);//
	KalmanCreate(&(grabTaskStructure.parallelMotor1.klmFiller), 1, 40);//cjh
	KalmanCreate(&(grabTaskStructure.parallelMotor2.klmFiller), 1, 40);//cjh

	KalmanCreate(&(grabTaskStructure.xplatMotor1.klmFiller), 1, 40);
	KalmanCreate(&(grabTaskStructure.yplatMotor2.klmFiller), 1, 40);
	KalmanCreate(&(grabTaskStructure.yplatMotor1.klmFiller), 1, 40);//Additional
  KalmanCreate(&(grabTaskStructure.liftoreMotor1.klmFiller), 1, 40);
  KalmanCreate(&(grabTaskStructure.liftoreMotor2.klmFiller), 1, 40);
	

  //行为初始化
  grabBehaviourInit(grabTaskStructure.behaviorList + GRAB_DEBUG, GRAB_DEBUG, DEBUG_GRAB_ARMBASEP_TYPE, DEBUG_GRAB_ARMMIDP_TYPE, DEBUG_GRAB_SINGLEROTATE_TYPE, DEBUG_GRAB_XPLAT_TYPE, DEBUG_GRAB_YPLAT_TYPE, DEBUG_LIFTORE_TYPE, grabDebugBehaviourEnterCondition, grabDebugBehaviourOutCondition, NULL, NULL, grabDebugBehaviourHandleFun);
  grabBehaviourInit(grabTaskStructure.behaviorList + GRAB_ZERO_FORCE, GRAB_ZERO_FORCE, GRAB_MOTOR_RAW, GRAB_MOTOR_RAW, GRAB_MOTOR_RAW, GRAB_MOTOR_RAW, GRAB_MOTOR_RAW, GRAB_MOTOR_RAW, grabZeroForceBehaviourEnterCondition, grabZeroForceBehaviourOutCondition, NULL, NULL, grabZeroForceBehaviourHandleFun);
  grabBehaviourInit(grabTaskStructure.behaviorList + GRAB_INIT, GRAB_INIT, GRAB_MOTOR_RAW, GRAB_MOTOR_RAW, GRAB_MOTOR_RAW, GRAB_MOTOR_RAW, GRAB_MOTOR_RAW, GRAB_MOTOR_RAW, grabInitBehaviourEnterCondition, grabInitBehaviourOutCondition, NULL, NULL, grabInitBehaviourHandleFun);
  //grabBehaviourInit(grabTaskStructure.behaviorList + GRAB_PREPARE, GRAB_PREPARE, GRAB_MOTOR_POSITION, GRAB_MOTOR_POSITION, GRAB_MOTOR_POSITION, GRAB_MOTOR_POSITION,GRAB_MOTOR_POSITION, GRAB_MOTOR_POSITION, GRAB_MOTOR_POSITION, grabPrepareBehaviourEnterCondition, grabPrepareBehaviourOutCondition, NULL, NULL, grabPrepareBehaviourHandleFun);
  grabBehaviourInit(grabTaskStructure.behaviorList + GRAB_TAKE, GRAB_TAKE, GRAB_MOTOR_POSITION, GRAB_MOTOR_POSITION, GRAB_MOTOR_POSITION,GRAB_MOTOR_POSITION, GRAB_MOTOR_POSITION, GRAB_MOTOR_POSITION, grabTakeBehaviourEnterCondition, grabTakeBehaviourOutCondition, NULL, NULL, grabTakeBehaviourHandleFun);
  grabBehaviourInit(grabTaskStructure.behaviorList + GRAB_EXCHANGE, GRAB_EXCHANGE, GRAB_MOTOR_POSITION, GRAB_MOTOR_POSITION, GRAB_MOTOR_POSITION,GRAB_MOTOR_POSITION, GRAB_MOTOR_POSITION, GRAB_MOTOR_POSITION, grabExchangeBehaviourEnterCondition, grabExchangeBehaviourOutCondition, NULL, NULL, grabExchangeBehaviourHandleFun);
  grabBehaviourInit(grabTaskStructure.behaviorList + GRAB_NORMAL, GRAB_NORMAL, GRAB_MOTOR_POSITION, GRAB_MOTOR_POSITION, GRAB_MOTOR_POSITION,GRAB_MOTOR_POSITION, GRAB_MOTOR_POSITION, GRAB_MOTOR_POSITION, grabNormalBehaviourEnterCondition, grabNormalBehaviourOutCondition, NULL, NULL, grabNormalBehaviourHandleFun);

  grabTaskStructure.nowBehaviour = grabTaskStructure.behaviorList + GRAB_ZERO_FORCE;

}

//void grabBehaviourInit(grab_behaviour_t *initBehavior, grab_behaviour_e num, grab_motor_mode_e armbaseyMode, grab_motor_mode_e armbasepMode, grab_motor_mode_e armmidpMode, 
//											grab_motor_mode_e singlerotateMode, grab_motor_mode_e xplatMode, grab_motor_mode_e yplatMode, grab_motor_mode_e liftoreMode,  
//											bool_t (*enterBehaviorCondition)(void), bool_t (*outBehaviorCondition)(void), void (*enterBehaviorFun)(void), void (*outBehaviorFun)(void), 
//											void (*behaviorHandleFun)(float *armbaseyExp, float *armbasepExp, float *armmidpExp, float *singlerotateExp, float *xplatExp, float *yplatExp, float *liftoreExp)) 
void grabBehaviourInit(grab_behaviour_t *initBehavior, grab_behaviour_e num, grab_motor_mode_e armbasepMode, grab_motor_mode_e armmidyMode, 
											grab_motor_mode_e parallelMode, grab_motor_mode_e xplatMode, grab_motor_mode_e yplatMode, grab_motor_mode_e liftoreMode,  
											bool_t (*enterBehaviorCondition)(void), bool_t (*outBehaviorCondition)(void), void (*enterBehaviorFun)(void), void (*outBehaviorFun)(void), 
											void (*behaviorHandleFun)(float *armbaseyExp, float *armbasepExp, float *armmidpExp, float *singlerotateExp, float *xplatExp, float *yplatExp, float *liftoreExp)) 
{
if(initBehavior == NULL || num >= GRAB_BEHAVIOUR_LENGTH || enterBehaviorCondition == NULL || outBehaviorCondition == NULL || behaviorHandleFun == NULL)
    return;

  initBehavior->num = num;
  initBehavior->behaviorHandleFun = behaviorHandleFun;
  initBehavior->enterBehaviorCondition = enterBehaviorCondition;
  initBehavior->outBehaviorCondition = outBehaviorCondition;
  initBehavior->enterBehaviorFun = enterBehaviorFun;
  initBehavior->outBehaviorFun = outBehaviorFun;

  //initBehavior->armbaseyMode = armbaseyMode;
	initBehavior->armbasepMode = armbasepMode;
  initBehavior->armmidyMode = armmidyMode;
  //initBehavior->singlerotateMode = singlerotateMode;
  initBehavior->parallelMode = parallelMode;
  initBehavior->xplatMode = xplatMode;
	initBehavior->yplatMode = yplatMode;
  initBehavior->liftoreMode = liftoreMode;
}

void grabBehaviourSelect() {
  //查看有没有优先级比当前行为高的行为的进入条件得到满足
  for(grab_behaviour_t *iterator = grabTaskStructure.behaviorList;
      iterator < grabTaskStructure.nowBehaviour; iterator++) {
    //如果有优先级高的进入行为得到满足，则进入那个行为的模式
    if(iterator->enterBehaviorCondition()) {
      grabBehaviorChange(iterator);
      break;
    }
  }

  if(grabTaskStructure.nowBehaviour->outBehaviorCondition()) {
    //查看当前行为是否已经达到了退出的条件，如果达到，则寻找下一个可以进入的行为
    for(grab_behaviour_t *iterator = grabTaskStructure.nowBehaviour;
        iterator < grabTaskStructure.behaviorList + GRAB_BEHAVIOUR_LENGTH; iterator++) {
      //按照优先级，如果有满足的行为，那么进入这个行为
      if(iterator->enterBehaviorCondition()) {
        grabBehaviorChange(iterator);
        return;
      }
    }
  } else
    return;

  //如果没有满足所有行为的进入条件，那么进入无力模式
  grabBehaviorChange(grabTaskStructure.behaviorList + GRAB_ZERO_FORCE);
}

void grabBehaviorChange(grab_behaviour_t *next) {
  //执行上一个behavior的退出函数
  if(grabTaskStructure.nowBehaviour->outBehaviorFun != NULL)
    grabTaskStructure.nowBehaviour->outBehaviorFun();

  //将当前行为切换成函数传入的行为
  grabTaskStructure.nowBehaviour = next;
  grabTaskStructure.behaviourStep = 0;
  grabTaskStructure.behaviourTime = 0;
  //grabTaskStructure.armbaseyMotor.mode = grabTaskStructure.nowBehaviour->armbaseyMode; //提升电机
	grabTaskStructure.armbasepMotor1.mode = grabTaskStructure.nowBehaviour->armbasepMode; //提升电机
  grabTaskStructure.armmidyMotor.mode = grabTaskStructure.nowBehaviour->armmidyMode; //推拉电机
  //grabTaskStructure.singlerotateMotor.mode = grabTaskStructure.nowBehaviour->singlerotateMode; //旋转电机
	grabTaskStructure.parallelMotor1.mode = grabTaskStructure.nowBehaviour->parallelMode; //旋转电机cjh
  grabTaskStructure.parallelMotor2.mode = grabTaskStructure.nowBehaviour->parallelMode;
  grabTaskStructure.xplatMotor1.mode = grabTaskStructure.nowBehaviour->xplatMode;
	grabTaskStructure.yplatMotor1.mode = grabTaskStructure.nowBehaviour->yplatMode; //Add
	grabTaskStructure.yplatMotor2.mode = grabTaskStructure.nowBehaviour->yplatMode; //Add//cjh
  grabTaskStructure.liftoreMotor1.mode = grabTaskStructure.nowBehaviour->liftoreMode;

  for(grab_behaviour_e i = GRAB_DEBUG; i < GRAB_BEHAVIOUR_LENGTH; i++) {
    if(grabTaskStructure.behaviorList + i == next) {
      grabTaskStructure.nowBehaviorName = i;
      break;
    }
  }
  //如果有进入该行为的处理函数，则执行
  if(grabTaskStructure.nowBehaviour->enterBehaviorFun != NULL)
    grabTaskStructure.nowBehaviour->enterBehaviorFun();
}

void grabCtrlChangeHandle() { //电机控制方式改变处理函数
  //机械臂底座yaw轴
//  if(grabTaskStructure.armbaseyMotor.lastMode != grabTaskStructure.armbaseyMotor.mode) {
//    if(grabTaskStructure.armbaseyMotor.mode == GRAB_MOTOR_POSITION)//位置双环控制
//      grabTaskStructure.armbaseyMotor.totalEcdSet = grabTaskStructure.armbaseyMotor.totalEcd;
//    else if(grabTaskStructure.armbaseyMotor.mode == GRAB_MOTOR_SPEED)//速度单环控制
//      grabTaskStructure.armbaseyMotor.speedSet = 0;
//    else if(grabTaskStructure.armbaseyMotor.mode == GRAB_MOTOR_RAW)//原始电流控制
//      grabTaskStructure.armbaseyMotor.rawCmdCurrent = 0;

//    grabTaskStructure.armbaseyMotor.lastMode = grabTaskStructure.armbaseyMotor.mode;
//  }

  //Add：机械臂底座pitch轴
  if(grabTaskStructure.armbasepMotor1.lastMode != grabTaskStructure.armbasepMotor1.mode) {
    if(grabTaskStructure.armbasepMotor1.mode == GRAB_MOTOR_POSITION)//位置双环控制
      grabTaskStructure.armbasepMotor1.totalEcdSet = grabTaskStructure.armbasepMotor1.totalEcd;
    else if(grabTaskStructure.armbasepMotor1.mode == GRAB_MOTOR_SPEED)//速度单环控制
      grabTaskStructure.armbasepMotor1.speedSet = 0;
    else if(grabTaskStructure.armbasepMotor1.mode == GRAB_MOTOR_RAW)//原始电流控制
      grabTaskStructure.armbasepMotor1.rawCmdCurrent = 0;

    grabTaskStructure.armbasepMotor1.lastMode = grabTaskStructure.armbasepMotor1.mode;
  }
	
  //机械臂中部pitch轴
  if(grabTaskStructure.armmidyMotor.lastMode != grabTaskStructure.armmidyMotor.mode) {
    if(grabTaskStructure.armmidyMotor.mode == GRAB_MOTOR_POSITION)//位置双环控制
      grabTaskStructure.armmidyMotor.totalEcdSet = grabTaskStructure.armmidyMotor.totalEcd;
    else if(grabTaskStructure.armmidyMotor.mode == GRAB_MOTOR_SPEED)//速度单环控制
      grabTaskStructure.armmidyMotor.speedSet = 0;
    else if(grabTaskStructure.armmidyMotor.mode == GRAB_MOTOR_RAW)//原始电流控制
      grabTaskStructure.armmidyMotor.rawCmdCurrent = 0;

    grabTaskStructure.armmidyMotor.lastMode = grabTaskStructure.armmidyMotor.mode;
  }

//  //爪子旋转期望
//  if(grabTaskStructure.singlerotateMotor.lastMode != grabTaskStructure.singlerotateMotor.mode) {
//    if(grabTaskStructure.singlerotateMotor.mode == GRAB_MOTOR_POSITION)//位置双环控制
//      grabTaskStructure.singlerotateMotor.totalEcdSet = grabTaskStructure.singlerotateMotor.totalEcd;
//    else if(grabTaskStructure.singlerotateMotor.mode == GRAB_MOTOR_SPEED)//速度单环控制
//      grabTaskStructure.singlerotateMotor.speedSet = 0;
//    else if(grabTaskStructure.singlerotateMotor.mode == GRAB_MOTOR_RAW)//原始电流控制
//      grabTaskStructure.singlerotateMotor.rawCmdCurrent = 0;

//    grabTaskStructure.singlerotateMotor.lastMode = grabTaskStructure.singlerotateMotor.mode;
//  }
  //差动机构期望cjh
  if(grabTaskStructure.parallelMotor1.lastMode != grabTaskStructure.parallelMotor1.mode) {
    if(grabTaskStructure.parallelMotor1.mode == GRAB_MOTOR_POSITION)//位置双环控制
      grabTaskStructure.parallelMotor1.totalEcdSet = grabTaskStructure.parallelMotor1.totalEcd;
    else if(grabTaskStructure.parallelMotor1.mode == GRAB_MOTOR_SPEED)//速度单环控制
      grabTaskStructure.parallelMotor1.speedSet = 0;
    else if(grabTaskStructure.parallelMotor1.mode == GRAB_MOTOR_RAW)//原始电流控制
      grabTaskStructure.parallelMotor1.rawCmdCurrent = 0;

    grabTaskStructure.parallelMotor1.lastMode = grabTaskStructure.parallelMotor1.mode;
  }
  //平台X平移期望
  if(grabTaskStructure.xplatMotor1.lastMode != grabTaskStructure.xplatMotor1.mode) {
    if(grabTaskStructure.xplatMotor1.mode == GRAB_MOTOR_POSITION)//位置双环控制
      grabTaskStructure.xplatMotor1.totalEcdSet = grabTaskStructure.xplatMotor1.totalEcd;
    else if(grabTaskStructure.xplatMotor1.mode == GRAB_MOTOR_SPEED)//速度单环控制
      grabTaskStructure.xplatMotor1.speedSet = 0;
    else if(grabTaskStructure.xplatMotor1.mode == GRAB_MOTOR_RAW)//原始电流控制
      grabTaskStructure.xplatMotor1.rawCmdCurrent = 0;

    grabTaskStructure.xplatMotor1.lastMode = grabTaskStructure.xplatMotor1.mode;
  }
  //Additional：平台Y平移
  if(grabTaskStructure.yplatMotor1.lastMode != grabTaskStructure.yplatMotor1.mode) {
    if(grabTaskStructure.yplatMotor1.mode == GRAB_MOTOR_POSITION)//位置双环控制
      grabTaskStructure.yplatMotor1.totalEcdSet = grabTaskStructure.yplatMotor1.totalEcd;
    else if(grabTaskStructure.yplatMotor1.mode == GRAB_MOTOR_SPEED)//速度单环控制
      grabTaskStructure.yplatMotor1.speedSet = 0;
    else if(grabTaskStructure.yplatMotor1.mode == GRAB_MOTOR_RAW)//原始电流控制
      grabTaskStructure.yplatMotor1.rawCmdCurrent = 0;

    grabTaskStructure.yplatMotor1.lastMode = grabTaskStructure.yplatMotor1.mode;
  }
  //提升电机
  if(grabTaskStructure.liftoreMotor1.lastMode != grabTaskStructure.liftoreMotor1.mode) {
    if(grabTaskStructure.liftoreMotor1.mode == GRAB_MOTOR_POSITION)//位置双环控制
      grabTaskStructure.liftoreMotor1.totalEcdSet = grabTaskStructure.liftoreMotor1.totalEcd;
    else if(grabTaskStructure.liftoreMotor1.mode == GRAB_MOTOR_SPEED)//速度单环控制
      grabTaskStructure.liftoreMotor1.speedSet = 0;
    else if(grabTaskStructure.liftoreMotor1.mode == GRAB_MOTOR_RAW)//原始电流控制
      grabTaskStructure.liftoreMotor1.rawCmdCurrent = 0;

    grabTaskStructure.liftoreMotor1.lastMode = grabTaskStructure.liftoreMotor1.mode;
  }

	
}

void grabFeedbackUpdate() {
  //速度更新
  //grabTaskStructure.armbaseyMotor.filterSpeed = KalmanFilter(&(grabTaskStructure.armbaseyMotor.klmFiller), grabTaskStructure.armbaseyMotor.baseInf.real_speed_rpm);			//
  grabTaskStructure.armbasepMotor1.filterSpeed = KalmanFilter(&(grabTaskStructure.armbasepMotor1.klmFiller), grabTaskStructure.armbasepMotor1.baseInf.real_speed_rpm);   //Add
  grabTaskStructure.armbasepMotor2.filterSpeed = KalmanFilter(&(grabTaskStructure.armbasepMotor2.klmFiller), grabTaskStructure.armbasepMotor2.baseInf.real_speed_rpm);   //Add
	grabTaskStructure.armmidyMotor.filterSpeed = KalmanFilter(&(grabTaskStructure.armmidyMotor.klmFiller), grabTaskStructure.armmidyMotor.baseInf.real_speed_rpm);	//
  //grabTaskStructure.singlerotateMotor.filterSpeed = KalmanFilter(&(grabTaskStructure.singlerotateMotor.klmFiller), grabTaskStructure.singlerotateMotor.baseInf.real_speed_rpm);			//夹子旋转电机
	grabTaskStructure.parallelMotor1.filterSpeed = KalmanFilter(&(grabTaskStructure.parallelMotor1.klmFiller), grabTaskStructure.parallelMotor1.baseInf.real_speed_rpm);   //Addcjh
  grabTaskStructure.parallelMotor2.filterSpeed = KalmanFilter(&(grabTaskStructure.parallelMotor2.klmFiller), grabTaskStructure.parallelMotor2.baseInf.real_speed_rpm);   //Addcjh
	grabTaskStructure.xplatMotor1.filterSpeed = KalmanFilter(&(grabTaskStructure.xplatMotor1.klmFiller), grabTaskStructure.xplatMotor1.baseInf.real_speed_rpm);
  grabTaskStructure.yplatMotor1.filterSpeed = KalmanFilter(&(grabTaskStructure.yplatMotor1.klmFiller), grabTaskStructure.yplatMotor1.baseInf.real_speed_rpm);   //Add
  grabTaskStructure.yplatMotor2.filterSpeed = KalmanFilter(&(grabTaskStructure.yplatMotor2.klmFiller), grabTaskStructure.yplatMotor2.baseInf.real_speed_rpm);   //Addcjh
  grabTaskStructure.liftoreMotor1.filterSpeed = KalmanFilter(&(grabTaskStructure.liftoreMotor1.klmFiller), grabTaskStructure.liftoreMotor1.baseInf.real_speed_rpm);
  grabTaskStructure.liftoreMotor2.filterSpeed = KalmanFilter(&(grabTaskStructure.liftoreMotor2.klmFiller), grabTaskStructure.liftoreMotor2.baseInf.real_speed_rpm);

	//上一次位置的更新
  //grabTaskStructure.armbaseyMotor.last_totalEcd = grabTaskStructure.armbaseyMotor.totalEcd;		
	grabTaskStructure.armbasepMotor1.last_totalEcd = grabTaskStructure.armbasepMotor1.totalEcd; //Add
	grabTaskStructure.armbasepMotor2.last_totalEcd = grabTaskStructure.armbasepMotor2.totalEcd; //Add
  grabTaskStructure.armmidyMotor.last_totalEcd = grabTaskStructure.armmidyMotor.totalEcd;	//机械臂电机
  //grabTaskStructure.singlerotateMotor.last_totalEcd = grabTaskStructure.singlerotateMotor.totalEcd;		//夹子旋转电机
	grabTaskStructure.parallelMotor1.last_totalEcd = grabTaskStructure.parallelMotor1.totalEcd;	//cjh
	grabTaskStructure.parallelMotor2.last_totalEcd = grabTaskStructure.parallelMotor2.totalEcd;//cjh
	grabTaskStructure.xplatMotor1.last_totalEcd = grabTaskStructure.xplatMotor1.totalEcd;	
	grabTaskStructure.yplatMotor1.last_totalEcd = grabTaskStructure.yplatMotor1.totalEcd;			//Addcjh
	grabTaskStructure.yplatMotor2.last_totalEcd = grabTaskStructure.yplatMotor2.totalEcd;    //cjh
  grabTaskStructure.liftoreMotor1.last_totalEcd = grabTaskStructure.liftoreMotor1.totalEcd;
  grabTaskStructure.liftoreMotor2.last_totalEcd = grabTaskStructure.liftoreMotor2.totalEcd;
	
  //位置更新
  //grabTaskStructure.armbaseyMotor.totalEcd = grabTaskStructure.armbaseyMotor.baseInf.relativeEcd;		
	grabTaskStructure.armbasepMotor1.totalEcd = grabTaskStructure.armbasepMotor1.baseInf.relativeEcd; //Add
	grabTaskStructure.armbasepMotor2.totalEcd = grabTaskStructure.armbasepMotor2.baseInf.relativeEcd; //Add
  grabTaskStructure.armmidyMotor.totalEcd = grabTaskStructure.armmidyMotor.baseInf.relativeEcd;	//机械臂电机
  //grabTaskStructure.singlerotateMotor.totalEcd = grabTaskStructure.singlerotateMotor.baseInf.relativeEcd;		//夹子旋转电机
	grabTaskStructure.xplatMotor1.totalEcd = grabTaskStructure.xplatMotor1.baseInf.relativeEcd;	
	grabTaskStructure.yplatMotor1.totalEcd = grabTaskStructure.yplatMotor1.baseInf.relativeEcd;			//Add
	grabTaskStructure.yplatMotor2.totalEcd = grabTaskStructure.yplatMotor2.baseInf.relativeEcd;			//Addcjh
  grabTaskStructure.liftoreMotor1.totalEcd = grabTaskStructure.liftoreMotor1.baseInf.relativeEcd;
  grabTaskStructure.liftoreMotor2.totalEcd = grabTaskStructure.liftoreMotor2.baseInf.relativeEcd;

}

void grabMotorLimit() {
	if(grabTaskStructure.inited == 1) //初始化之后机械限位位置
	{
		//armbasey
//		if(grabTaskStructure.armbaseyMotor.totalEcdSet > grabTaskStructure.armbaseyMotor.fixedEcd[ARM_BASE_Y_MAX])
//			grabTaskStructure.armbaseyMotor.totalEcdSet = grabTaskStructure.armbaseyMotor.fixedEcd[ARM_BASE_Y_MAX];
//		else if(grabTaskStructure.armbaseyMotor.totalEcdSet < grabTaskStructure.armbaseyMotor.fixedEcd[ARM_BASE_Y_RESET])
//			grabTaskStructure.armbaseyMotor.totalEcdSet = grabTaskStructure.armbaseyMotor.fixedEcd[ARM_BASE_Y_RESET];
		//armbasep
		if(grabTaskStructure.armbasepMotor1.totalEcdSet > grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_RESET])
			grabTaskStructure.armbasepMotor1.totalEcdSet = grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_RESET];
		else if(grabTaskStructure.armbasepMotor1.totalEcdSet < grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_MIN])
			grabTaskStructure.armbasepMotor1.totalEcdSet = grabTaskStructure.armbasepMotor1.fixedEcd[ARM_BASE_P_MIN];
		//armmidp
		if(grabTaskStructure.armmidyMotor.totalEcdSet > grabTaskStructure.armmidyMotor.fixedEcd[ARM_MID_Y_MAX])
    grabTaskStructure.armmidyMotor.totalEcdSet = grabTaskStructure.armmidyMotor.fixedEcd[ARM_MID_Y_MAX];
			else if(grabTaskStructure.armmidyMotor.totalEcdSet < grabTaskStructure.armmidyMotor.fixedEcd[ARM_MID_Y_RESET])
    grabTaskStructure.armmidyMotor.totalEcdSet = grabTaskStructure.armmidyMotor.fixedEcd[ARM_MID_Y_RESET];
		//singlerotate
//		if(grabTaskStructure.singlerotateMotor.totalEcdSet > grabTaskStructure.singlerotateMotor.fixedEcd[ROTATE_SINGLE_MAX])
//    grabTaskStructure.singlerotateMotor.totalEcdSet = grabTaskStructure.singlerotateMotor.fixedEcd[ROTATE_SINGLE_MAX];
//			else if(grabTaskStructure.singlerotateMotor.totalEcdSet < grabTaskStructure.singlerotateMotor.fixedEcd[ROTATE_SINGLE_RESET])
//    grabTaskStructure.singlerotateMotor.totalEcdSet = grabTaskStructure.singlerotateMotor.fixedEcd[ROTATE_SINGLE_RESET];
					//singlerotate
		if(grabTaskStructure.parallelMotor1.totalEcdSet > grabTaskStructure.parallelMotor1.fixedEcd[ROTATE_SINGLE_MAX])
    grabTaskStructure.parallelMotor1.totalEcdSet = grabTaskStructure.parallelMotor1.fixedEcd[ROTATE_SINGLE_MAX];
			else if(grabTaskStructure.parallelMotor1.totalEcdSet < grabTaskStructure.parallelMotor1.fixedEcd[ROTATE_SINGLE_RESET])
    grabTaskStructure.parallelMotor1.totalEcdSet = grabTaskStructure.parallelMotor1.fixedEcd[ROTATE_SINGLE_RESET];
		//xplat   CJH
		if(grabTaskStructure.xplatMotor1.totalEcdSet > grabTaskStructure.xplatMotor1.fixedEcd[PLAT_X_RESET])
    grabTaskStructure.xplatMotor1.totalEcdSet = grabTaskStructure.xplatMotor1.fixedEcd[PLAT_X_RESET];
			else if(grabTaskStructure.xplatMotor1.totalEcdSet < grabTaskStructure.xplatMotor1.fixedEcd[PLAT_X_MIN])
    grabTaskStructure.xplatMotor1.totalEcdSet = grabTaskStructure.xplatMotor1.fixedEcd[PLAT_X_MIN];
		//yplat   CJH
		if(grabTaskStructure.yplatMotor1.totalEcdSet > grabTaskStructure.yplatMotor1.fixedEcd[PLAT_Y_FRONT_MAX])
    grabTaskStructure.yplatMotor1.totalEcdSet = grabTaskStructure.yplatMotor1.fixedEcd[PLAT_Y_FRONT_MAX];
			else if(grabTaskStructure.yplatMotor1.totalEcdSet < grabTaskStructure.yplatMotor1.fixedEcd[PLAT_Y_MIN])
    grabTaskStructure.yplatMotor1.totalEcdSet = grabTaskStructure.yplatMotor1.fixedEcd[PLAT_Y_MIN];
		//lift
		if(grabTaskStructure.liftoreMotor1.totalEcdSet > grabTaskStructure.liftoreMotor1.fixedEcd[LIFT_MAX])
    grabTaskStructure.liftoreMotor1.totalEcdSet = grabTaskStructure.liftoreMotor1.fixedEcd[LIFT_MAX];
			else if(grabTaskStructure.liftoreMotor1.totalEcdSet < grabTaskStructure.liftoreMotor1.fixedEcd[LIFT_RESET])
    grabTaskStructure.liftoreMotor1.totalEcdSet = grabTaskStructure.liftoreMotor1.fixedEcd[LIFT_RESET];
	}
}
//调参变量定义
int16_t setspeed_grab, realspeed_grab,setEcd,realEcd;

void grabPidCalc() { //PID计算函数
	
  //机械臂底座yaw轴电机
//  if(grabTaskStructure.armbaseyMotor.mode == GRAB_MOTOR_RAW) //原始电流控制
//    grabTaskStructure.armbaseyMotor.currentSet = grabTaskStructure.armbaseyMotor.rawCmdCurrent;
//  else if(grabTaskStructure.armbaseyMotor.mode == GRAB_MOTOR_SPEED) { //速度单环控制
//    PID_Calc(&(grabTaskStructure.armbaseyMotor.pidParameter[INNER]), grabTaskStructure.armbaseyMotor.filterSpeed, grabTaskStructure.armbaseyMotor.speedSet);
//    grabTaskStructure.armbaseyMotor.currentSet = grabTaskStructure.armbaseyMotor.pidParameter[INNER].output;
//  } else if(grabTaskStructure.armbaseyMotor.mode == GRAB_MOTOR_POSITION) { //位置双环控制
//    PID_Calc(&(grabTaskStructure.armbaseyMotor.pidParameter[OUTER]), grabTaskStructure.armbaseyMotor.totalEcd, grabTaskStructure.armbaseyMotor.totalEcdSet);
//	    PID_Calc(&(grabTaskStructure.armbaseyMotor.pidParameter[INNER]), grabTaskStructure.armbaseyMotor.filterSpeed, grabTaskStructure.armbaseyMotor.pidParameter[OUTER].output);
//    grabTaskStructure.armbaseyMotor.currentSet = grabTaskStructure.armbaseyMotor.pidParameter[INNER].output;
//  } else
//    grabTaskStructure.armbaseyMotor.currentSet = 0;
	
  //机械臂底部pitch轴电机（MF9025） 
  if(grabTaskStructure.armbasepMotor1.mode == GRAB_MOTOR_RAW) //原始电流控制
    grabTaskStructure.armbasepMotor1.currentSet = grabTaskStructure.armbasepMotor1.rawCmdCurrent;
  else if(grabTaskStructure.armbasepMotor1.mode == GRAB_MOTOR_SPEED) { //速度单环控制
    PID_Calc(&(grabTaskStructure.armbasepMotor1.pidParameter[INNER]), grabTaskStructure.armbasepMotor1.filterSpeed, grabTaskStructure.armbasepMotor1.speedSet);
    grabTaskStructure.armbasepMotor1.currentSet = -grabTaskStructure.armbasepMotor1.pidParameter[INNER].output;
  } else if(grabTaskStructure.armbasepMotor1.mode == GRAB_MOTOR_POSITION) { //位置双环控制
    PID_Calc(&(grabTaskStructure.armbasepMotor1.pidParameter[OUTER]), grabTaskStructure.armbasepMotor1.totalEcd, grabTaskStructure.armbasepMotor1.totalEcdSet);
    PID_Calc(&(grabTaskStructure.armbasepMotor1.pidParameter[INNER]), grabTaskStructure.armbasepMotor1.filterSpeed,-grabTaskStructure.armbasepMotor1.pidParameter[OUTER].output);
    grabTaskStructure.armbasepMotor1.currentSet = -grabTaskStructure.armbasepMotor1.pidParameter[INNER].output;
  } else
    grabTaskStructure.armbasepMotor1.currentSet = 0;
	//motor2用不到了
  if(grabTaskStructure.armbasepMotor1.mode == GRAB_MOTOR_RAW) //原始电流控制
    grabTaskStructure.armbasepMotor2.currentSet = grabTaskStructure.armbasepMotor1.rawCmdCurrent;
  else if(grabTaskStructure.armbasepMotor1.mode == GRAB_MOTOR_SPEED) { //速度单环控制
    PID_Calc(&(grabTaskStructure.armbasepMotor2.pidParameter[INNER]), grabTaskStructure.armbasepMotor2.filterSpeed, -grabTaskStructure.armbasepMotor1.speedSet);
    grabTaskStructure.armbasepMotor2.currentSet = -grabTaskStructure.armbasepMotor2.pidParameter[INNER].output;
  } else if(grabTaskStructure.armbasepMotor1.mode == GRAB_MOTOR_POSITION) { //位置双环控制
    PID_Calc(&(grabTaskStructure.armbasepMotor2.pidParameter[OUTER]), grabTaskStructure.armbasepMotor2.totalEcd, -grabTaskStructure.armbasepMotor1.totalEcdSet);
    PID_Calc(&(grabTaskStructure.armbasepMotor2.pidParameter[INNER]), grabTaskStructure.armbasepMotor2.filterSpeed, -grabTaskStructure.armbasepMotor2.pidParameter[OUTER].output);
    grabTaskStructure.armbasepMotor2.currentSet = -grabTaskStructure.armbasepMotor2.pidParameter[INNER].output;
  } else
    grabTaskStructure.armbasepMotor2.currentSet = 0;

  //机械臂中部pitch轴电机
  if(grabTaskStructure.armmidyMotor.mode == GRAB_MOTOR_RAW) //原始电流控制
    grabTaskStructure.armmidyMotor.currentSet = grabTaskStructure.armmidyMotor.rawCmdCurrent;
  else if(grabTaskStructure.armmidyMotor.mode == GRAB_MOTOR_SPEED) { //速度单环控制
    PID_Calc(&(grabTaskStructure.armmidyMotor.pidParameter[INNER]), grabTaskStructure.armmidyMotor.filterSpeed, grabTaskStructure.armmidyMotor.speedSet);
    grabTaskStructure.armmidyMotor.currentSet = grabTaskStructure.armmidyMotor.pidParameter[INNER].output;
  } else if(grabTaskStructure.armmidyMotor.mode == GRAB_MOTOR_POSITION) { //位置双环控制
    PID_Calc(&(grabTaskStructure.armmidyMotor.pidParameter[OUTER]), grabTaskStructure.armmidyMotor.totalEcd, grabTaskStructure.armmidyMotor.totalEcdSet);
    PID_Calc(&(grabTaskStructure.armmidyMotor.pidParameter[INNER]), grabTaskStructure.armmidyMotor.filterSpeed, grabTaskStructure.armmidyMotor.pidParameter[OUTER].output);
    grabTaskStructure.armmidyMotor.currentSet = grabTaskStructure.armmidyMotor.pidParameter[INNER].output;
  } else
    grabTaskStructure.armmidyMotor.currentSet = 0;

//  //夹子旋转期望
//  if(grabTaskStructure.singlerotateMotor.mode == GRAB_MOTOR_RAW) //原始电流控制
//    grabTaskStructure.singlerotateMotor.currentSet = grabTaskStructure.singlerotateMotor.rawCmdCurrent;
//  else if(grabTaskStructure.singlerotateMotor.mode == GRAB_MOTOR_SPEED) { //速度单环控制
//    PID_Calc(&(grabTaskStructure.singlerotateMotor.pidParameter[INNER]), grabTaskStructure.singlerotateMotor.filterSpeed, grabTaskStructure.singlerotateMotor.speedSet);
//    grabTaskStructure.singlerotateMotor.currentSet = grabTaskStructure.singlerotateMotor.pidParameter[INNER].output;
//  } else if(grabTaskStructure.singlerotateMotor.mode == GRAB_MOTOR_POSITION) { //位置双环控制
//    PID_Calc(&(grabTaskStructure.singlerotateMotor.pidParameter[OUTER]), grabTaskStructure.singlerotateMotor.totalEcd, grabTaskStructure.singlerotateMotor.totalEcdSet);
//    PID_Calc(&(grabTaskStructure.singlerotateMotor.pidParameter[INNER]), grabTaskStructure.singlerotateMotor.filterSpeed, grabTaskStructure.singlerotateMotor.pidParameter[OUTER].output);
//    grabTaskStructure.singlerotateMotor.currentSet = grabTaskStructure.singlerotateMotor.pidParameter[INNER].output;
//  } else
//    grabTaskStructure.singlerotateMotor.currentSet = 0;
  //差动机构期望
  if(grabTaskStructure.parallelMotor1.mode == GRAB_MOTOR_RAW) //原始电流控制
    grabTaskStructure.parallelMotor1.currentSet = grabTaskStructure.parallelMotor1.rawCmdCurrent;
  else if(grabTaskStructure.parallelMotor1.mode == GRAB_MOTOR_SPEED) { //速度单环控制
    PID_Calc(&(grabTaskStructure.parallelMotor1.pidParameter[INNER]), grabTaskStructure.parallelMotor1.filterSpeed, grabTaskStructure.parallelMotor1.speedSet);
    grabTaskStructure.parallelMotor1.currentSet = grabTaskStructure.parallelMotor1.pidParameter[INNER].output;
  } else if(grabTaskStructure.parallelMotor1.mode == GRAB_MOTOR_POSITION) { //位置双环控制
    PID_Calc(&(grabTaskStructure.parallelMotor1.pidParameter[OUTER]), grabTaskStructure.parallelMotor1.totalEcd, grabTaskStructure.parallelMotor1.totalEcdSet);
    PID_Calc(&(grabTaskStructure.parallelMotor1.pidParameter[INNER]), grabTaskStructure.parallelMotor1.filterSpeed, grabTaskStructure.parallelMotor1.pidParameter[OUTER].output);
    grabTaskStructure.parallelMotor1.currentSet = grabTaskStructure.parallelMotor1.pidParameter[INNER].output;
  } else
    grabTaskStructure.parallelMotor1.currentSet = 0;
  //平台X方向平移电机
  if(grabTaskStructure.xplatMotor1.mode == GRAB_MOTOR_RAW) //原始电流控制
    grabTaskStructure.xplatMotor1.currentSet = grabTaskStructure.xplatMotor1.rawCmdCurrent;
  else if(grabTaskStructure.xplatMotor1.mode == GRAB_MOTOR_SPEED) { //速度单环控制
    PID_Calc(&(grabTaskStructure.xplatMotor1.pidParameter[INNER]), grabTaskStructure.xplatMotor1.filterSpeed, grabTaskStructure.xplatMotor1.speedSet);
    grabTaskStructure.xplatMotor1.currentSet = grabTaskStructure.xplatMotor1.pidParameter[INNER].output;
  } else if(grabTaskStructure.xplatMotor1.mode == GRAB_MOTOR_POSITION) { //位置双环控制
    PID_Calc(&(grabTaskStructure.xplatMotor1.pidParameter[OUTER]), grabTaskStructure.xplatMotor1.totalEcd, grabTaskStructure.xplatMotor1.totalEcdSet);
    PID_Calc(&(grabTaskStructure.xplatMotor1.pidParameter[INNER]), grabTaskStructure.xplatMotor1.filterSpeed, grabTaskStructure.xplatMotor1.pidParameter[OUTER].output);
    grabTaskStructure.xplatMotor1.currentSet = grabTaskStructure.xplatMotor1.pidParameter[INNER].output;
  } else
    grabTaskStructure.xplatMotor1.currentSet = 0;

	//Add 平台Y方向平移电机
  if(grabTaskStructure.yplatMotor1.mode == GRAB_MOTOR_RAW) //原始电流控制
    grabTaskStructure.yplatMotor1.currentSet = grabTaskStructure.yplatMotor1.rawCmdCurrent;
  else if(grabTaskStructure.yplatMotor1.mode == GRAB_MOTOR_SPEED) { //速度单环控制
    PID_Calc(&(grabTaskStructure.yplatMotor1.pidParameter[INNER]), grabTaskStructure.yplatMotor1.filterSpeed, grabTaskStructure.yplatMotor1.speedSet);
    grabTaskStructure.yplatMotor1.currentSet = grabTaskStructure.yplatMotor1.pidParameter[INNER].output;
  } else if(grabTaskStructure.yplatMotor1.mode == GRAB_MOTOR_POSITION) { //位置双环控制
    PID_Calc(&(grabTaskStructure.yplatMotor1.pidParameter[OUTER]), grabTaskStructure.yplatMotor1.totalEcd, grabTaskStructure.yplatMotor1.totalEcdSet);
    PID_Calc(&(grabTaskStructure.yplatMotor1.pidParameter[INNER]), grabTaskStructure.yplatMotor1.filterSpeed, grabTaskStructure.yplatMotor1.pidParameter[OUTER].output);
    grabTaskStructure.yplatMotor1.currentSet = grabTaskStructure.yplatMotor1.pidParameter[INNER].output;
  } else
    grabTaskStructure.yplatMotor1.currentSet = 0;

  //矿仓提升电机 期望 1和2电机共用模式、期望，仅改变方向
  if(grabTaskStructure.liftoreMotor1.mode == GRAB_MOTOR_RAW) //原始电流控制
    grabTaskStructure.liftoreMotor1.currentSet = grabTaskStructure.liftoreMotor1.rawCmdCurrent;
  else if(grabTaskStructure.liftoreMotor1.mode == GRAB_MOTOR_SPEED) { //速度单环控制
    PID_Calc(&(grabTaskStructure.liftoreMotor1.pidParameter[INNER]), grabTaskStructure.liftoreMotor1.filterSpeed, grabTaskStructure.liftoreMotor1.speedSet);
    grabTaskStructure.liftoreMotor1.currentSet = grabTaskStructure.liftoreMotor1.pidParameter[INNER].output;
  } else if(grabTaskStructure.liftoreMotor1.mode == GRAB_MOTOR_POSITION) { //位置双环控制
    PID_Calc(&(grabTaskStructure.liftoreMotor1.pidParameter[OUTER]), grabTaskStructure.liftoreMotor1.totalEcd, grabTaskStructure.liftoreMotor1.totalEcdSet);
    PID_Calc(&(grabTaskStructure.liftoreMotor1.pidParameter[INNER]), grabTaskStructure.liftoreMotor1.filterSpeed, grabTaskStructure.liftoreMotor1.pidParameter[OUTER].output);
    grabTaskStructure.liftoreMotor1.currentSet = grabTaskStructure.liftoreMotor1.pidParameter[INNER].output;
  } else
    grabTaskStructure.liftoreMotor1.currentSet = 0;

  if(grabTaskStructure.liftoreMotor1.mode == GRAB_MOTOR_RAW) //原始电流控制
    grabTaskStructure.liftoreMotor2.currentSet = grabTaskStructure.liftoreMotor2.rawCmdCurrent;
  else if(grabTaskStructure.liftoreMotor1.mode == GRAB_MOTOR_SPEED) { //速度单环控制
    PID_Calc(&(grabTaskStructure.liftoreMotor2.pidParameter[INNER]), grabTaskStructure.liftoreMotor2.filterSpeed, -grabTaskStructure.liftoreMotor1.speedSet);
    grabTaskStructure.liftoreMotor2.currentSet = grabTaskStructure.liftoreMotor2.pidParameter[INNER].output;
  } else if(grabTaskStructure.liftoreMotor1.mode == GRAB_MOTOR_POSITION) { //位置双环控制
    PID_Calc(&(grabTaskStructure.liftoreMotor2.pidParameter[OUTER]), grabTaskStructure.liftoreMotor2.totalEcd, -grabTaskStructure.liftoreMotor1.totalEcdSet);
    PID_Calc(&(grabTaskStructure.liftoreMotor2.pidParameter[INNER]), grabTaskStructure.liftoreMotor2.filterSpeed, grabTaskStructure.liftoreMotor2.pidParameter[OUTER].output);
    grabTaskStructure.liftoreMotor2.currentSet = grabTaskStructure.liftoreMotor2.pidParameter[INNER].output;
  } else
    grabTaskStructure.liftoreMotor2.currentSet = 0;


	
	
 // grabTaskStructure.armbaseyMotor.baseInf.given_current = (s16)(grabTaskStructure.armbaseyMotor.currentSet);
	grabTaskStructure.armbasepMotor1.baseInf.given_current = (s16)(grabTaskStructure.armbasepMotor1.currentSet);//Add
	grabTaskStructure.armbasepMotor2.baseInf.given_current = (s16)(-grabTaskStructure.armbasepMotor1.currentSet);//Add
  grabTaskStructure.armmidyMotor.baseInf.given_current = (s16)(grabTaskStructure.armmidyMotor.currentSet);
	//grabTaskStructure.singlerotateMotor.baseInf.given_current = (s16)(grabTaskStructure.singlerotateMotor.currentSet);
	grabTaskStructure.parallelMotor1.baseInf.given_current = (s16)(grabTaskStructure.parallelMotor1.currentSet);//Addcjh
	grabTaskStructure.parallelMotor2.baseInf.given_current = (s16)(grabTaskStructure.parallelMotor2.currentSet);//Addcjh
  grabTaskStructure.yplatMotor1.baseInf.given_current = (s16)(grabTaskStructure.yplatMotor1.currentSet);//Add
	grabTaskStructure.xplatMotor1.baseInf.given_current = (s16)(grabTaskStructure.xplatMotor1.currentSet);
  grabTaskStructure.yplatMotor2.baseInf.given_current = (s16)(grabTaskStructure.yplatMotor2.currentSet);//Addcjh
  grabTaskStructure.liftoreMotor1.baseInf.given_current = (s16)(grabTaskStructure.liftoreMotor1.currentSet);
  grabTaskStructure.liftoreMotor2.baseInf.given_current = (s16)(grabTaskStructure.liftoreMotor2.currentSet);


 
	
//---------------------------------------------------------------------------------------------------
	//调参 
	#define debug_grabName armbasepMotor1
  setspeed_grab = grabTaskStructure.debug_grabName.speedSet;
  realspeed_grab = grabTaskStructure.debug_grabName.filterSpeed;
	setEcd = grabTaskStructure.debug_grabName.totalEcdSet;
	realEcd = grabTaskStructure.debug_grabName.totalEcd;

}



void grabCanBusCtrlMotors(	s16 parallelMotorCurrent1,s16 parallelMotorCurrent2,
														s16 armbasepMotorCurrent1, s16 armbasepMotorCurrent2, s16 armmidyMotorCurrent,
														s16 xplatMotorCurrent1,s16 yplatMotorCurrent1,  s16 yplatMotorCurrent2, s16 liftoreMotorCurrent1,  s16 liftoreMotorCurrent2
												)
{
  s16 grabCurrent[4];
	
//-------------------------------------------------------------------------------------
  grabCurrent[0] = parallelMotorCurrent1;//singlerotateMotorCurrent;//0x201 夹子旋转电机
  grabCurrent[1] = parallelMotorCurrent2;//armbaseyMotorCurrent;//0x202  armbasey
  grabCurrent[2] = armmidyMotorCurrent;//0x203  底座yaw轴旋转电机//修改cjh 中部yaw
  grabCurrent[3] = xplatMotorCurrent1;//0x204  平台前后平移电机

  if(toe_is_error(DBUSTOE)) {
    grabCurrent[0] = 0;
    grabCurrent[1] = 0;
    grabCurrent[2] = 0;
    grabCurrent[3] = 0;
  }

  djiMotorCurrentSendQueue(CAN2, GRAB_CANBUS_SEND_HEADER14, grabCurrent, 4);//CAN2

//-------------------------------------------------------------------------------------
  grabCurrent[0] = yplatMotorCurrent1;//0x205 平台左右平移电机
  grabCurrent[1] = yplatMotorCurrent2;//basicTaskStructure.camtopMotor.baseInf.given_current; //0x206 图传电机//y轴电机2
  grabCurrent[2] = armbasepMotorCurrent1;//0x207
  grabCurrent[3] = armbasepMotorCurrent2;//0x208

  if(toe_is_error(DBUSTOE)) {
    grabCurrent[0] = 0;
    grabCurrent[1] = 0;
    grabCurrent[2] = 0;
    grabCurrent[3] = 0;
  }

  djiMotorCurrentSendQueue(CAN2, GRAB_CANBUS_SEND_HEADER58, grabCurrent, 4);//CAN2
	
//-------------------------------------------------------------------------------------
//	if(toe_is_error(DBUSTOE)) 
//	{
//		//无力
//		MF9025_CurrentSendQueue(CAN2,1,0);
//		MF9025_CurrentSendQueue(CAN2,2,0);
//	}
//	else 
//	{
//		MF9025_CurrentSendQueue(CAN2,1,armbasepMotorCurrent1);
//		MF9025_CurrentSendQueue(CAN2,2,armbasepMotorCurrent2);
//	}
				
	//----------------------------------------------------------------------------------
	
	
	
											  																								//CAN2
//-------------------------------------------------------------------------------------
  grabCurrent[0] = 0;
  grabCurrent[1] = 0;
  grabCurrent[2] = liftoreMotorCurrent1;//0x203  升降
  grabCurrent[3] = liftoreMotorCurrent2;//0x204  升降

  if(toe_is_error(DBUSTOE)) {
    grabCurrent[0] = 0;
    grabCurrent[1] = 0;
    grabCurrent[2] = 0;
    grabCurrent[3] = 0;
  }

  djiMotorCurrentSendQueue(CAN1, GRAB_CANBUS_SEND_HEADER14, grabCurrent, 4);//CAN1
	
//---------------------------------------------------------------------------------
	

}
