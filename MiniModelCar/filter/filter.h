#ifndef __FILTER_H
#define __FILTER_H
  /**************************************************************************
项目：MiniModelCar
说明：标准两驱模型智能小车
**************************************************************************/
extern float angle, angle_dot; 	
void Kalman_Filter(float Accel,float Gyro);		
void Yijielvbo(float angle_m, float gyro_m);
#endif
