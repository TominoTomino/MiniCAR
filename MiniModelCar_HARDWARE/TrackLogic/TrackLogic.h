#ifndef _TRACKLOGIC_H__
#define _TRACKLOGIC_H__

#include "sys.h"

/**************************************************************************
外部变量声明
**************************************************************************/


/****************外部变量声明*****************/
extern u16 fADCDatum0[5],mADCDatum0[4],bADCDatum0[5];
extern int fSensorRead[5];        //存储车头传感器状态，从左到右存储方便写逻辑
extern int mSensorRead[4];       //存储中间传感器状态
extern int bSensorRead[5];       //存储车尾传感器状态
extern int fSensorReadSum, mSensorReadSum, bSensorReadSum;

extern float error;
extern int TimeFlag;
extern int TestFlag,LastTestFlag;
extern u8 Auto_Back_Flag;
extern int Mod;


void ReadSensor01(void);
void TrackContor1(void);
void TrackContor2(void);   //循迹黑线
void GetSetParament(void);
void Track_Rx_Logic(void);
void AutoBackTrack(void);
void AutoBackTrack2(void);
void ReadSensor02(void);
u8 CheckFlashTrackData(void);



#endif

