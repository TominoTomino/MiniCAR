#include "stm32f10x.h"
#include "sys.h"
/**************************************************************************
项目：MiniModelCar
说明：标准两驱模型智能小车
**************************************************************************/
u8 Flag_Stop = 1; //停止标志位和 显示标志位 默认停止 显示打开  0打开，1关闭
int Encoder_A, Encoder_B; //编码器的脉冲计数
long int Position_A, Position_B, Rate_A, Rate_B; //PID控制相关变量
long int Motor_A, Motor_B; //电机PWM变量 , Motor_A是右电机，Motor_B是左电机
long int Target_A, Target_B; //电机目标值
u16 delay_50, delay_flag; //延时相关变量
u8 Run_Flag = 0; //速度或者位置模式指示变量
int UrxSt, UtxSt;   //串口控制相关变量
float Pitch, Roll, Yaw, Gryo_Z, Move_X, Move_Y, Move_Z; //三轴角度 Z轴陀螺仪和XYZ轴目标速度
float	Position_KP = 14, Position_KI = 0, Position_KD = 25; //位置控制PID参数
//0.1 1 2 6 8 16
float Velocity_KP = 10, Velocity_KI = 10, Velocity_KD = 0;	  //速度控制PID参数
float Read_Speed_Left, Read_Speed_Right;


float Speed; //速度

extern float After_filter[M];


/*==========================================================小车性能&问题
问题：
1.小车整体往右偏移，在右电机增量PID算法上给目标速度做微补偿且取消左电机微分，达到沙盘路径直线段尽可能平直

性能：
1.编码器计数为10ms车轮转动的脉冲数
2.车轮转动一圈输出=编码器线数7*减速比42*4倍频 = 1176  ，车轮每秒转的圈数为（编码器计数*100）/1176 圈


参考值基准设置：
  1. 参考值 = （白 + 黑）/2.5   （不稳定，不适用）
	2.采集白色色块加一定偏移量作为参考值，参考值 = 白 + X

*/

/*==========================================================屏幕软件参数
1.显示尺寸：长2160mm 宽1220mm
2.分辨率：3840x2160   长1.77778像素/mm  宽1.77049像素/mm


*/

/* ==========================================================小车参数
电机物理参数：
1.电压：DC 12V ，功率：0.2W，空载电流：≤65mA，空载转速：550rpm，额定电流：≤0.15A，额定转速：390rpm，减速比：1：118，
2.输出信号类型：方波AB相
3.响应频率：100KHz
4.基础脉冲数：7PPR ， 脉冲数：294 ，7x减速比
5.磁环触发极数：14极，7对极

小车物理参数：
1.尺寸规格：长129mm  宽45mm  高47mm ； 最大离地高度  mm，最低离地高度  mm；传感离地高度  mm
2.巡线传感间距： mm


*/



char send_buf[32];


int main(void)
{


    delay_init();	    	            //=====延时函数初始化
    //JTAG_Set(JTAG_SWD_DISABLE);     //=====关闭JTAG接口
    JTAG_Set(SWD_ENABLE);           //=====打开SWD接口 可以利用主板的SWD接口调试
    LED_Init();                     //=====初始化与 LED 连接的硬件接口
    KEY_Init();                     //=====按键初始化
    MY_NVIC_PriorityGroupConfig(2);	//=====设置中断分组
    MiniModelCar_PWM_Init(7199, 0);  //=====初始化PWM 10KHZ，用于驱动电机 如需初始化电调接口
    uart3_init(9600);             //=====串口3初始化
    Encoder_Init_TIM2();            //=====编码器接口
    Encoder_Init_TIM8();            //=====初始化编码器

    // TIM3_Cap_Init(99, 7199); //10Khz的计数频率，计数到100为10ms
    TIM3_Cap_Init(499, 7199); //10Khz的计数频率，计数到500为50ms  118减速比稳定版
	 // TIM3_Cap_Init(1499, 7199); //10Khz的计数频率，计数到1500为150ms   42减速比

    delay_ms(500);                  //=====延时,初始化MPU6050前需延时一段时间，确保初始化成功
    IIC_Init();                     //=====IIC初始化
    MPU6050_initialize();           //=====MPU6050初始化
    DMP_Init();                     //=====初始化DMP

#if ADC_NO_DMA		   //====为假，不执行
    Adc_Init();                     //=====adc初始化
    Trail_ADC_Init();								//=====巡线传感adc初始化
#elif ADC_IS_DMA       //====为真，执行
    Trail_ADC_DMA_Init();
#endif

    MiniModelCar_EXTI_Init();        //=====MPU6050 5ms定时中断初始化



    while(1)
    {


        GetSetParament(); //循迹传感器基准参数设置&行车信息打印

        /*  //固定曲率不循迹遥控泊车出入库
        		switch(Flag_Turn)
        		{
        			case 1:
        				Move_Y = 900;
        				Move_Z = -120;
        			break;
        			case 2:
        				Move_Y = -900;
        				 Move_Z = 116;
        			break;
        			case -1:
        				Move_Y = 0;
        				Move_Z = 0;
        			break;

        		}
        		*/

        /*    //固定曲率不循迹遥控泊车出入库
              	Move_Y = 900;
        				Move_Z = -120;
              SysTick_Delay_ms (7500);

        				Move_Y = 0;
        				Move_Z = 0;
        		 SysTick_Delay_ms(5000);

        				 Move_Y = -900;
        				 Move_Z = 117;
        		 SysTick_Delay_ms(7300);

        				Move_Y = 0;
        				Move_Z = 0;

        SysTick_Delay_ms(5000);
        		*/



    }

}

