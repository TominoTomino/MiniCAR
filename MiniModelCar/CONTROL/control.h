#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
  /**************************************************************************
项目：MiniModelCar
说明：标准两驱模型智能小车
**************************************************************************/
#define PI 3.14159265 
#define DIFFERENCE 100
extern	int Balance_Pwm,Velocity_Pwm,Turn_Pwm;


int EXTI9_5_IRQHandler(void);
void Set_Pwm(int motor_a,int motor_b);
void Kinematic_Analysis(float Vy,float Vz);
void Kinematic_Analysis2(float Vy,float Vz);
void Key(void);
void Xianfu_Pwm(int amplitude);
void Xianfu_Velocity(int amplitude_A,int amplitude_B);
u8 Turn_Off( int voltage);
u32 myabs(long int a);
int Incremental_PI_A (int Encoder,float Target);
int Incremental_PI_B (int Encoder,float Target);
void Get_RC(void);
//void Position_Control(void);
int Position_PID_A(int Encoder, int Target);
int Position_PID_B(int Encoder, int Target);

void Track_Logic(void);
void C_Speed_Response(void);
void C_Speed_Response_1(void);
void U_Speed_Response(void);
void U_Speed_Response_1(void);
void Data_Execute(void);


void Trail_Not_Stop(void);

void m_speed_change(void);

float calcPid(float input);
void m_speed_change1(void);

void PID(int left_val,int right_val);
void P(int left_val,int right_val);
void PD(int left_val,int right_val);


int Incremental_PI_C (int Encoder_Left ,int Encoder_Right, int Target);

	
#endif
