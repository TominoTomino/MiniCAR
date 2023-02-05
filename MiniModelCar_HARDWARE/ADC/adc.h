#ifndef __ADC_H
#define __ADC_H	
#include "sys.h"

//����bool������Ҫ��ӵ�ͷ�ļ�
#include <stdint.h>
#include <stdbool.h>

#define Battery_Ch 8

#define N 5 //ÿͨ����5��
#define M 16 //Ϊ16��ͨ��

void Dly_us(void);
void Adc_Init(void);
u16 Get_Adc(u8 ch);
int Get_battery_volt(void); 
int Get_chip_temp(void);
void  Trail_ADC_Init(void);
void  Trail_ADC_DMA_Init(void);
void filter(void);
u16 constrain( u16 val , u16 min_val , u16 max_val);
bool bit_map( u16 x , u16 a , bool b);
u8 bit_map2( u16 x , u16 a , u16 b );
u8 bit_map_scope( u16 x ,  u16 a , u16 b , u16 c );

#endif 












