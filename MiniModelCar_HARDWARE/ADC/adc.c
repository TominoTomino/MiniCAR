
#include "adc.h"

//����bool������Ҫ��ӵ�ͷ�ļ�
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#define TSL_SI    PCout(3)   //SI  
#define TSL_CLK   PCout(2)   //CLK 

vu16 AD_Value[N][M]; //�������ADCת�������Ҳ��DMA��Ŀ���ַ
float After_filter[M]; //���������ƽ��ֵ֮��Ľ��

/**************************************************************************
�������ܣ���ʱ
��ڲ�������
����  ֵ����
��    Ŀ��MiniModelCar
**************************************************************************/
void Dly_us(void)
{
   int ii;    
   for(ii=0;ii<10;ii++);      
}

void  Adc_Init(void)
{    
 	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |RCC_APB2Periph_ADC1	, ENABLE );	  //ʹ��ADC1ͨ��ʱ��
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M
	//����ģ��ͨ����������                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
	ADC_DeInit(ADC1);  //��λADC1,������ ADC1 ��ȫ���Ĵ�������Ϊȱʡֵ
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC����ģʽ:ADC1��ADC2�����ڶ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//ģ��ת�������ڵ�ͨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//ģ��ת�������ڵ���ת��ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//ת��������������ⲿ��������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 1;	//˳����й���ת����ADCͨ������Ŀ
	ADC_Init(ADC1, &ADC_InitStructure);	//����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ���   
	ADC_Cmd(ADC1, ENABLE);	//ʹ��ָ����ADC1
	ADC_ResetCalibration(ADC1);	//ʹ�ܸ�λУ׼  	 
	while(ADC_GetResetCalibrationStatus(ADC1));	//�ȴ���λУ׼����	
	ADC_StartCalibration(ADC1);	 //����ADУ׼
	while(ADC_GetCalibrationStatus(ADC1));	 //�ȴ�У׼����
}		

/**************************************************************************
�������ܣ�AD����
��ڲ�����ADC1 ��ͨ��
����  ֵ��ADת�����
**************************************************************************/
u16 Get_Adc(u8 ch)   
{
	  	//����ָ��ADC�Ĺ�����ͨ����һ�����У�����ʱ��
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADCͨ��,����ʱ��Ϊ239.5����	  			     
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//ʹ��ָ����ADC1�����ת����������		 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//�ȴ�ת������
	return ADC_GetConversionValue(ADC1);	//�������һ��ADC1�������ת�����
}


/**************************************************************************
�������ܣ���ȡ��ص�ѹ 
��ڲ�������
����  ֵ����ص�ѹ ��λMV
**************************************************************************/
int Get_battery_volt(void)   
{  
	int Volt;//��ص�ѹ
	
	#if ADC_NO_DMA		   //====Ϊ�٣���ִ��
				Volt=Get_Adc(Battery_Ch)*3.3*11*100/4096;	//�����ѹ���������ԭ��ͼ�򵥷������Եõ�	
	#elif ADC_IS_DMA       //====Ϊ�棬ִ��
				Volt=After_filter[8]*3.3*11*100/4096;	//�����ѹ���������ԭ��ͼ�򵥷������Եõ�	
	#endif
	
	
	return Volt;
}

int Get_chip_temp(void)
{
	float temperate;
	temperate = After_filter[15]*3.3/4096;
	temperate = (1.43 - temperate) / 0.0043 +25;
	return temperate * 100;
}



/**************************************************************************
�������ܣ�Ѳ�ߴ�����ADC������ʼ��
��ڲ�������
����  ֵ����
��    Ŀ��MiniModelCar
**************************************************************************/
void  Trail_ADC_Init(void)
{    
 	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC |RCC_APB2Periph_ADC1	, ENABLE );	  //ʹ��ADC1ͨ��ʱ��
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M
	//����ģ��ͨ����������  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(GPIOB, &GPIO_InitStructure);	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
	

	
	ADC_DeInit(ADC1);  //��λADC1,������ ADC1 ��ȫ���Ĵ�������Ϊȱʡֵ
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC����ģʽ:ADC1��ADC2�����ڶ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//ģ��ת�������ڵ�ͨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//ģ��ת�������ڵ���ת��ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//ת��������������ⲿ��������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = 1;	//˳����й���ת����ADCͨ������Ŀ
	ADC_Init(ADC1, &ADC_InitStructure);	//����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ���   
	ADC_Cmd(ADC1, ENABLE);	//ʹ��ָ����ADC1
	ADC_ResetCalibration(ADC1);	//ʹ�ܸ�λУ׼  	 
	while(ADC_GetResetCalibrationStatus(ADC1));	//�ȴ���λУ׼����	
	ADC_StartCalibration(ADC1);	 //����ADУ׼
	while(ADC_GetCalibrationStatus(ADC1));	 //�ȴ�У׼����
}	

/**************************************************************************
�������ܣ�Ѳ�ߴ�����ADC����DMA�����ʼ��
��ڲ�������
����  ֵ����
��    Ŀ��MiniModelCar
**************************************************************************/
void  Trail_ADC_DMA_Init(void)
{    
 	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	ErrorStatus HSEStartUpStatus;

	RCC_DeInit(); //RCC ϵͳ��λ
	RCC_HSEConfig(RCC_HSE_ON); //����HSE
	HSEStartUpStatus = RCC_WaitForHSEStartUp(); //�ȴ�HSE׼����
	if(HSEStartUpStatus == SUCCESS)
	{
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable); //Enable Prefetch Buffer
		FLASH_SetLatency(FLASH_Latency_2); //Set 2 Latency cycles
		RCC_HCLKConfig(RCC_SYSCLK_Div1); //AHB clock = SYSCLK
		RCC_PCLK2Config(RCC_HCLK_Div1); //APB2 clock = HCLK
		RCC_PCLK1Config(RCC_HCLK_Div2); //APB1 clock = HCLK/2
		
		RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_9); //PLLCLK = 12MHz * 6 = 72 MHz
		
		RCC_PLLCmd(ENABLE); //Enable PLL
		
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET); //Wait till PLL is ready
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK); //Select PLL as system clock source
		while(RCC_GetSYSCLKSource() != 0x08); //Wait till PLL is used as system clock source

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB
		| RCC_APB2Periph_GPIOC |RCC_APB2Periph_ADC1 | RCC_APB2Periph_AFIO |RCC_APB2Periph_USART1, ENABLE ); //ʹ��ADC1ͨ��ʱ�ӣ������ܽ�ʱ��

		RCC_ADCCLKConfig(RCC_PCLK2_Div6); //72M/6=12,ADC���ʱ�䲻�ܳ���14M
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //ʹ��DMA����

	}
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC |RCC_APB2Periph_ADC1	, ENABLE );	  //ʹ��ADC1ͨ��ʱ��
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //����ADC��Ƶ����6 72M/6=12,ADC���ʱ�䲻�ܳ���14M
	
	//����ģ��ͨ����������--��ѹ����ADC����                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
	
	
		//����Ѳ�ߴ���ģ��ͨ����������                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(GPIOB, &GPIO_InitStructure);	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//ģ����������
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
	
	DMA_DeInit(DMA1_Channel1); //��DMA��ͨ��1�Ĵ�������Ϊȱʡֵ
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR; //DMA����ADC����ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&AD_Value; //DMA�ڴ����ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; //�ڴ���Ϊ���ݴ����Ŀ�ĵ�
	DMA_InitStructure.DMA_BufferSize = N*M; //DMAͨ����DMA����Ĵ�С
	
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //�����ַ�Ĵ�������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //�ڴ��ַ�Ĵ�������
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //���ݿ��Ϊ16λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //���ݿ��Ϊ16λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; //������ѭ������ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; //DMAͨ�� xӵ�и����ȼ�
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; //DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
	DMA_Init(DMA1_Channel1, &DMA_InitStructure); //����DMA_InitStruct��ָ���Ĳ�����ʼ��DMA��ͨ��
	
	
	ADC_DeInit(ADC1);  //��λADC1,������ ADC1 ��ȫ���Ĵ�������Ϊȱʡֵ
	ADC_TempSensorVrefintCmd(ENABLE); //���ڲ�������ͨ���������¶�
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC����ģʽ:ADC1��ADC2�����ڶ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;	//ģ��ת�������ڶ�ͨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//ģ��ת������������ת��ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//ת��������������ⲿ��������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = M;	//˳����й���ת����ADCͨ������Ŀ
	ADC_Init(ADC1, &ADC_InitStructure);	//����ADC_InitStruct��ָ���Ĳ�����ʼ������ADCx�ļĴ��� 

	//ADC1,ADCͨ��,����ʱ��Ϊ71.5����
	ADC_RegularChannelConfig(ADC1 , ADC_Channel_0 , 1 , ADC_SampleTime_71Cycles5);  //PA0
	ADC_RegularChannelConfig(ADC1 , ADC_Channel_1 , 2 , ADC_SampleTime_71Cycles5);  //PA1
	ADC_RegularChannelConfig(ADC1 , ADC_Channel_2 , 3 , ADC_SampleTime_71Cycles5);  //PA2
	ADC_RegularChannelConfig(ADC1 , ADC_Channel_3 , 4 , ADC_SampleTime_71Cycles5);  //PA3
	ADC_RegularChannelConfig(ADC1 , ADC_Channel_4 , 5 , ADC_SampleTime_71Cycles5); //PA4
	ADC_RegularChannelConfig(ADC1 , ADC_Channel_5 , 6 , ADC_SampleTime_71Cycles5); //PA5
	ADC_RegularChannelConfig(ADC1 , ADC_Channel_6 , 7 , ADC_SampleTime_71Cycles5); //PA6
	ADC_RegularChannelConfig(ADC1 , ADC_Channel_7 , 8 , ADC_SampleTime_71Cycles5); //PA7
	ADC_RegularChannelConfig(ADC1 , ADC_Channel_8 , 9 , ADC_SampleTime_71Cycles5); //PB0
	ADC_RegularChannelConfig(ADC1 , ADC_Channel_9 , 10 , ADC_SampleTime_71Cycles5);  //PB1
	ADC_RegularChannelConfig(ADC1 , ADC_Channel_10 , 11 , ADC_SampleTime_71Cycles5);  //PC0
	ADC_RegularChannelConfig(ADC1 , ADC_Channel_11 , 12 , ADC_SampleTime_71Cycles5);  //PC1
	ADC_RegularChannelConfig(ADC1 , ADC_Channel_12 , 13 , ADC_SampleTime_71Cycles5);  //PC2
	ADC_RegularChannelConfig(ADC1 , ADC_Channel_13 , 14 , ADC_SampleTime_71Cycles5); //PC3
	ADC_RegularChannelConfig(ADC1 , ADC_Channel_14 , 15 , ADC_SampleTime_71Cycles5); //PC4
	ADC_RegularChannelConfig(ADC1 , ADC_Channel_16 , 16 , ADC_SampleTime_71Cycles5);  //оƬ�ڲ��¶ȴ���


	//оƬ�ڲ����м��㹫ʽ�� T���棩={��V25-Vsense��/Avg_Slope}+25  STM32F103RCT6�����¶�ϵ��������
	//V25=Vsense �� 25 ��ʱ����ֵ������ֵΪ��1.43����
	//Avg_Slope=�¶���Vsense���ߵ�ƽ��б�ʣ���λ��mv/���uv/�棩������ֵ��4.3mv/�棩��

	// ����ADC��DMA֧�֣�Ҫʵ��DMA���ܣ������������DMAͨ���Ȳ�����
	ADC_DMACmd(ADC1, ENABLE);
	
	ADC_Cmd(ADC1, ENABLE);	//ʹ��ָ����ADC1
	
	ADC_ResetCalibration(ADC1);	//ʹ�ܸ�λУ׼  	 
	while(ADC_GetResetCalibrationStatus(ADC1));	//�ȴ���λУ׼����	
	ADC_StartCalibration(ADC1);	 //����ADУ׼
	while(ADC_GetCalibrationStatus(ADC1));	 //�ȴ�У׼����
	
	ADC_SoftwareStartConvCmd(ADC1, ENABLE); //ʹ��ADC���ת��
	DMA_Cmd(DMA1_Channel1, ENABLE); //����DMAͨ��
}

void filter(void)
{
	int sum = 0;
	u8 count,i;
	for(i=0;i<M;i++)
	{
		for ( count=0;count<N;count++)
		{
			sum += AD_Value[count][i];
		}
		//After_filter[i]=(float)sum/N;
		
		After_filter[i]=constrain((float)sum/N , 0 , 4095);  //��������ֵԼ����0-4095

		sum=0;
	}

}


//�������ܣ�Լ������,����ֵԼ����ĳ����Χ
u16 constrain( u16 val , u16 min_val , u16 max_val)
{
	if(val < min_val) val = min_val;
	if(val > max_val) val = max_val;
  return val;
	
}


//�������ܣ�����������ֵ����Ϊ0��1��x����������ֵ��a��ֵ��b��0��1
bool bit_map( u16 x , u16 a , bool b)
{
//	if (  x < a &&  x > 200) return !b;
//	else if( x > a &&  x < 3900) return b;
	
	if(x >= a) return b;
	else return !b;
	
}


//�������ܣ�����������ֵ����Ϊ0��1��x����������ֵ��a��ֵ-��ɫɫ�飬b��ֵ,���ص�λ0��1
u8 bit_map2( u16 x , u16 a , u16 b )
{
	u16 min_threshold = 0;
	if((x >= min_threshold) &&  (x<=a)) return 0;
	else if((x>a) && (x<=b)) return 1;
	else if(x>b) return 2;
	
}

//�������ܣ�����������ֵ����Ϊ0��1��x����������ֵ��a��ֵ��b��ֵ,���ص�λ0��1��2
u8 bit_map3( u16 x , u16 a , u16 b )
{
	u16 min_threshold = 0;
	if((x >= min_threshold) &&  (x<=a)) return 0;
	else if((x>a) && (x<=b)) return 1;
	else if(x>b) return 2;
	
}

//�������ܣ�����������ֵ����Ϊ0��1��x����������ֵ��a����ֵ��b����ֵ,����c--0��1
u8 bit_map_scope( u16 x ,  u16 a , u16 b , u16 c )
{
	if(x >= a && x<= b) return c;
	else return !c;
	
}




