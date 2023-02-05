#ifndef _TRACKLOGIC_H__
#define _TRACKLOGIC_H__

#include "sys.h"

/**************************************************************************
�ⲿ��������
**************************************************************************/


/****************�ⲿ��������*****************/
extern u16 fADCDatum0[5],mADCDatum0[4],bADCDatum0[5];
extern int fSensorRead[5];        //�洢��ͷ������״̬�������Ҵ洢����д�߼�
extern int mSensorRead[4];       //�洢�м䴫����״̬
extern int bSensorRead[5];       //�洢��β������״̬
extern int fSensorReadSum, mSensorReadSum, bSensorReadSum;

extern float error;
extern int TimeFlag;
extern int TestFlag,LastTestFlag;
extern u8 Auto_Back_Flag;
extern int Mod;


void ReadSensor01(void);
void TrackContor1(void);
void TrackContor2(void);   //ѭ������
void GetSetParament(void);
void Track_Rx_Logic(void);
void AutoBackTrack(void);
void AutoBackTrack2(void);
void ReadSensor02(void);
u8 CheckFlashTrackData(void);



#endif

