
#include "adc.h"

//定义bool变量需要添加的头文件
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#define TSL_SI    PCout(3)   //SI  
#define TSL_CLK   PCout(2)   //CLK 

vu16 AD_Value[N][M]; //用来存放ADC转换结果，也是DMA的目标地址
float After_filter[M]; //用来存放求平均值之后的结果

/**************************************************************************
函数功能：延时
入口参数：无
返回  值：无
项    目：MiniModelCar
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
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC |RCC_APB2Periph_ADC1	, ENABLE );	  //使能ADC1通道时钟
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M
	//设置模拟通道输入引脚                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
	ADC_DeInit(ADC1);  //复位ADC1,将外设 ADC1 的全部寄存器重设为缺省值
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC工作模式:ADC1和ADC2工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//模数转换工作在单通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//模数转换工作在单次转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;	//顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitStructure);	//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器   
	ADC_Cmd(ADC1, ENABLE);	//使能指定的ADC1
	ADC_ResetCalibration(ADC1);	//使能复位校准  	 
	while(ADC_GetResetCalibrationStatus(ADC1));	//等待复位校准结束	
	ADC_StartCalibration(ADC1);	 //开启AD校准
	while(ADC_GetCalibrationStatus(ADC1));	 //等待校准结束
}		

/**************************************************************************
函数功能：AD采样
入口参数：ADC1 的通道
返回  值：AD转换结果
**************************************************************************/
u16 Get_Adc(u8 ch)   
{
	  	//设置指定ADC的规则组通道，一个序列，采样时间
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );	//ADC1,ADC通道,采样时间为239.5周期	  			     
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能		 
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));//等待转换结束
	return ADC_GetConversionValue(ADC1);	//返回最近一次ADC1规则组的转换结果
}


/**************************************************************************
函数功能：读取电池电压 
入口参数：无
返回  值：电池电压 单位MV
**************************************************************************/
int Get_battery_volt(void)   
{  
	int Volt;//电池电压
	
	#if ADC_NO_DMA		   //====为假，不执行
				Volt=Get_Adc(Battery_Ch)*3.3*11*100/4096;	//电阻分压，具体根据原理图简单分析可以得到	
	#elif ADC_IS_DMA       //====为真，执行
				Volt=After_filter[8]*3.3*11*100/4096;	//电阻分压，具体根据原理图简单分析可以得到	
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
函数功能：巡线传感器ADC采样初始化
入口参数：无
返回  值：无
项    目：MiniModelCar
**************************************************************************/
void  Trail_ADC_Init(void)
{    
 	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC |RCC_APB2Periph_ADC1	, ENABLE );	  //使能ADC1通道时钟
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M
	//设置模拟通道输入引脚  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOB, &GPIO_InitStructure);	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
	

	
	ADC_DeInit(ADC1);  //复位ADC1,将外设 ADC1 的全部寄存器重设为缺省值
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC工作模式:ADC1和ADC2工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//模数转换工作在单通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//模数转换工作在单次转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;	//顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitStructure);	//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器   
	ADC_Cmd(ADC1, ENABLE);	//使能指定的ADC1
	ADC_ResetCalibration(ADC1);	//使能复位校准  	 
	while(ADC_GetResetCalibrationStatus(ADC1));	//等待复位校准结束	
	ADC_StartCalibration(ADC1);	 //开启AD校准
	while(ADC_GetCalibrationStatus(ADC1));	 //等待校准结束
}	

/**************************************************************************
函数功能：巡线传感器ADC采样DMA传输初始化
入口参数：无
返回  值：无
项    目：MiniModelCar
**************************************************************************/
void  Trail_ADC_DMA_Init(void)
{    
 	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	ErrorStatus HSEStartUpStatus;

	RCC_DeInit(); //RCC 系统复位
	RCC_HSEConfig(RCC_HSE_ON); //开启HSE
	HSEStartUpStatus = RCC_WaitForHSEStartUp(); //等待HSE准备好
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
		| RCC_APB2Periph_GPIOC |RCC_APB2Periph_ADC1 | RCC_APB2Periph_AFIO |RCC_APB2Periph_USART1, ENABLE ); //使能ADC1通道时钟，各个管脚时钟

		RCC_ADCCLKConfig(RCC_PCLK2_Div6); //72M/6=12,ADC最大时间不能超过14M
		RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //使能DMA传输

	}
	
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC |RCC_APB2Periph_ADC1	, ENABLE );	  //使能ADC1通道时钟
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M
	
	//设置模拟通道输入引脚--电压测量ADC引脚                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
	
	
	
		//设置巡线传感模拟通道输入引脚                         
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOB, &GPIO_InitStructure);	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
	
	DMA_DeInit(DMA1_Channel1); //将DMA的通道1寄存器重设为缺省值
	
	DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&ADC1->DR; //DMA外设ADC基地址
	DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&AD_Value; //DMA内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; //内存作为数据传输的目的地
	DMA_InitStructure.DMA_BufferSize = N*M; //DMA通道的DMA缓存的大小
	
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable; //内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //数据宽度为16位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //数据宽度为16位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular; //工作在循环缓存模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_High; //DMA通道 x拥有高优先级
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; //DMA通道x没有设置为内存到内存传输
	DMA_Init(DMA1_Channel1, &DMA_InitStructure); //根据DMA_InitStruct中指定的参数初始化DMA的通道
	
	
	ADC_DeInit(ADC1);  //复位ADC1,将外设 ADC1 的全部寄存器重设为缺省值
	ADC_TempSensorVrefintCmd(ENABLE); //打开内部传感器通道，测量温度
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC工作模式:ADC1和ADC2工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;	//模数转换工作在多通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;	//模数转换工作在连续转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = M;	//顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitStructure);	//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器 

	//ADC1,ADC通道,采样时间为71.5周期
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
	ADC_RegularChannelConfig(ADC1 , ADC_Channel_16 , 16 , ADC_SampleTime_71Cycles5);  //芯片内部温度传感


	//芯片内部传感计算公式： T（℃）={（V25-Vsense）/Avg_Slope}+25  STM32F103RCT6是逆温度系数传感器
	//V25=Vsense 在 25 度时的数值（典型值为：1.43）。
	//Avg_Slope=温度与Vsense曲线的平均斜率（单位：mv/℃或uv/℃）（典型值：4.3mv/℃）。

	// 开启ADC的DMA支持（要实现DMA功能，还需独立配置DMA通道等参数）
	ADC_DMACmd(ADC1, ENABLE);
	
	ADC_Cmd(ADC1, ENABLE);	//使能指定的ADC1
	
	ADC_ResetCalibration(ADC1);	//使能复位校准  	 
	while(ADC_GetResetCalibrationStatus(ADC1));	//等待复位校准结束	
	ADC_StartCalibration(ADC1);	 //开启AD校准
	while(ADC_GetCalibrationStatus(ADC1));	 //等待校准结束
	
	ADC_SoftwareStartConvCmd(ADC1, ENABLE); //使能ADC软件转换
	DMA_Cmd(DMA1_Channel1, ENABLE); //启动DMA通道
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
		
		After_filter[i]=constrain((float)sum/N , 0 , 4095);  //将传感数值约束在0-4095

		sum=0;
	}

}


//函数功能：约束函数,将数值约束在某个范围
u16 constrain( u16 val , u16 min_val , u16 max_val)
{
	if(val < min_val) val = min_val;
	if(val > max_val) val = max_val;
  return val;
	
}


//函数功能：将传感器的值设置为0或1；x：传感器的值，a阈值，b：0或1
bool bit_map( u16 x , u16 a , bool b)
{
//	if (  x < a &&  x > 200) return !b;
//	else if( x > a &&  x < 3900) return b;
	
	if(x >= a) return b;
	else return !b;
	
}


//函数功能：将传感器的值设置为0或1；x：传感器的值，a阈值-白色色块，b阈值,返回挡位0、1
u8 bit_map2( u16 x , u16 a , u16 b )
{
	u16 min_threshold = 0;
	if((x >= min_threshold) &&  (x<=a)) return 0;
	else if((x>a) && (x<=b)) return 1;
	else if(x>b) return 2;
	
}

//函数功能：将传感器的值设置为0或1；x：传感器的值，a阈值，b阈值,返回挡位0、1、2
u8 bit_map3( u16 x , u16 a , u16 b )
{
	u16 min_threshold = 0;
	if((x >= min_threshold) &&  (x<=a)) return 0;
	else if((x>a) && (x<=b)) return 1;
	else if(x>b) return 2;
	
}

//函数功能：将传感器的值设置为0或1；x：传感器的值，a低阈值，b高阈值,返回c--0或1
u8 bit_map_scope( u16 x ,  u16 a , u16 b , u16 c )
{
	if(x >= a && x<= b) return c;
	else return !c;
	
}




