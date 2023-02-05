#include "control.h"
#include "filter.h"
#include "adc.h"
/**************************************************************************
项目：MiniModelCar
说明：标准两驱模型智能小车
**************************************************************************/

u8 Flag_Target, Flag_Change; //相关标志位

int Voltage;//电池电压采样相关的变量
float Voltage_Count, Voltage_All; //电压采样相关变量
int Temp_Chip;//电池电压采样相关的变量
float Temp_Count, Temp_All; //温度采样相关变量

float Gyro_K = 0.004;     //陀螺仪比例系数
int j, sum;
//#define a_PARAMETER          (0.079f)
#define a_PARAMETER          (0.5f)
#define JG_OFFSET_A 0//6//3    //前进走直线补偿  (1号车3，2号车6)
#define JG_OFFSET_B  0 //3//0//3    //前进走直线补偿

int Sensor_Left, Sensor_Middle, Sensor_Right, Sensor; //传感巡线相关
int S_Input_Read, S_D_Input_Read, S_M_Input_Read, S_D_M_Input_Read; //传感器位赋值变量
int Flag_S = 0 ; //标记加减速匀速状态
int Flag_state = 0, Flag_Run = 0;

int Flag_Accelerate = 0; //标记加减速，0：默认打开 ，1：关闭


extern float After_filter[M];


float Left_Angle = 0.5, Right_Angle = 0.01;

#define Y_PARAMETER           (sqrt(3)/2.f)
#define L_PARAMETER            (1.0f)


#define TRACK_SPEED    (Speed_Num = Speed_Num)
#define GetCount()  (Count)
#define SetCount(num) (Count=(num))
#define ReloadCount() (Count=0)


#define SetDelay_Flag(num) (delay_flag=(num))


/**************************************************************************
函数功能：小车运动数学模型
入口参数：X Y Z 三轴速度或者位置
返回  值：无
**************************************************************************/
void Kinematic_Analysis(float Vy, float Vz)
{
    //    Target_A   = Vy + Vz * (a_PARAMETER);
    //    Target_B   = Vy - Vz * (a_PARAMETER);

    Target_A = (Vy + Vz);
    Target_B = (Vy - Vz);


    //    if( Target_B!=0)Target_B=(Target_B+3)*0.1;
    //      else Target_B = Target_B*0.1;




    //  Target_A   =  Y_PARAMETER*Vy + a_PARAMETER*Vz+gyro[2]*Gyro_K;
    //  Target_B   =  Y_PARAMETER*Vy - a_PARAMETER*Vz+gyro[2]*Gyro_K;



}
/**************************************************************************
函数功能：获取位置控制过程速度值
入口参数：X Y Z 三轴位置变化量
返回  值：无
**************************************************************************/
void Kinematic_Analysis2(float Vy, float Vz)
{
    Rate_A   = Vy - Vz * (a_PARAMETER) * 10;
    Rate_B   = Vy - Vz * (a_PARAMETER) * 10;

}


/**************************************************************************
函数功能：所有的控制代码都在这里面
         5ms定时中断由MPU6050的INT引脚触发
         严格保证采样和数据处理的时间同步
**************************************************************************/
int EXTI9_5_IRQHandler(void)//用于数据采集&接收
{
    if(INT == 0)
    {
        EXTI->PR = 1 << 5; //清除LINE5上的中断标志位

        Flag_Target = !Flag_Target;
        if(Flag_Target == 1) //5ms读取一次陀螺仪和加速度计的值
        {
            //    Read_DMP();   //===更新姿态
            return 0;
        }

        //  Read_DMP();  //===更新姿态

        filter();  //===巡线传感DMA传输数据采集
        ReadSensor01();//===传感数据基准转换，读取巡线传感挡位 ，放在这里是为了方便基准检查时关闭电机后也能读取校准后的数据

        ++delay_50;
        ++delay_flag;

        Voltage_All += Get_battery_volt(); //多次采样累积
        if(++Voltage_Count == 100) Voltage = Voltage_All / 100, Voltage_All = 0, Voltage_Count = 0; //求平均值 获取电池电压

        Temp_All += Get_chip_temp(); //多次累积采样温度
        if(++Temp_Count == 100) Temp_Chip = Temp_All / 100, Temp_All = 0, Temp_Count = 0; //求平均值 获取芯片温度



    }
    return 0;
}


int TIM3_IRQHandler(void)   //TIM3中断，单独用于运动控制以保证运动更加精确
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //清除TIMx的中断待处理位:TIM 中断源

        //===10ms控制一次，为了保证M法测速的时间基准，首先读取编码器数据
        Encoder_A = Read_Encoder(2); //===读取编码器的值
        Encoder_B = -Read_Encoder(8); //===读取编码器的值，调整PA0 PA1线序，即调整电机AB相线序打到电机同方向旋转效果

        Track_Rx_Logic();  //===更新串口通信数据

        //filter();  //===巡线传感DMA传输数据采集
        //ReadSensor01();//===传感数据基准转换，读取巡线传感挡位 ，放在这里是为了方便基准检查时关闭电机后也能读取校准后的数据



        if(Turn_Off(Voltage) == 1) Flag_Stop = 1; //===如果电池电压存在异常则关闭电机熄火,开机默认会进入关闭电机状态，需要指令打开

        if(Flag_Stop == 0)
        {
            Get_RC();

            Motor_B = Incremental_PI_B(Encoder_B, Target_B); //===速度闭环控制计算电机B最终PWM
            Motor_A = Incremental_PI_A(Encoder_A, Target_A); //===速度闭环控制计算电机A最终PWM

            Led_Flash(100);  //===LED闪烁;常规模式 1s改变一次指示灯的状态


            Xianfu_Pwm(6900);                     //===PWM限幅
            Set_Pwm(Motor_A, Motor_B);    //===赋值给PWM寄存器
        }






    }

    return 0;
}
/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motor_a, int motor_b)
{
    if(motor_a > 0)        PWMA1 = 7200, PWMA2 = 7200 - motor_a;
    else                PWMA2 = 7200, PWMA1 = 7200 + motor_a;

    if(motor_b > 0)         PWMB1 = 7200, PWMB2 = 7200 - motor_b;
    else                PWMB2 = 7200, PWMB1 = 7200 + motor_b;

}
/**************************************************************************
函数功能：限制PWM赋值
入口参数：幅值
返回  值：无
**************************************************************************/
void Xianfu_Pwm(int amplitude)
{
    if(Motor_A < -amplitude) Motor_A = -amplitude;
    if(Motor_A > amplitude)  Motor_A = amplitude;
    if(Motor_B < -amplitude) Motor_B = -amplitude;
    if(Motor_B > amplitude)  Motor_B = amplitude;

}
/**************************************************************************
函数功能：位置PID控制过程中速度的设置
入口参数：无、幅值
返回  值：无
**************************************************************************/
void Xianfu_Velocity(int amplitude_A, int amplitude_B)
{
    if(Motor_A < -amplitude_A) Motor_A = -amplitude_A;  //位置控制模式中，A电机的运行速度
    if(Motor_A > amplitude_A)  Motor_A = amplitude_A;    //位置控制模式中，A电机的运行速度
    if(Motor_B < -amplitude_B) Motor_B = -amplitude_B;  //位置控制模式中，B电机的运行速度
    if(Motor_B > amplitude_B)  Motor_B = amplitude_B;       //位置控制模式中，B电机的运行速度

}
/**************************************************************************
函数功能：按键修改小车运行状态
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)
{
    vu8 tmp;
    tmp = click_N_Double(50);
    //if(tmp==1)Flag_Stop=!Flag_Stop;//单击控制电机启停
}
/**************************************************************************
函数功能：异常关闭电机
入口参数：电压
返回  值：1：异常  0：正常
**************************************************************************/
u8 Turn_Off( int voltage)
{
    u8 temp;
    // if(voltage < 1110 || Flag_Stop == 1) //电池电压低于11.1V关闭电机
    if(voltage < 1000 || Flag_Stop == 1) //电池电压低于10V关闭电机
    {
        temp = 1;
        PWMA1 = 0; //电机控制位清零
        PWMB1 = 0; //电机控制位清零
        PWMA2 = 0; //电机控制位清零
        PWMB2 = 0; //电机控制位清零
    }
    else
        temp = 0;

    if(voltage < 1150 || Flag_Stop == 1)  //电池电压低于11.5V打开充电
    {
        CHARGE = 1; //打开充电
    }
    else if(voltage > 1200)   CHARGE = 0; //关闭充电


    return temp;
}

/**************************************************************************
函数功能：绝对值函数
入口参数：long int
返回  值：unsigned int
**************************************************************************/
u32 myabs(long int a)
{
    u32 temp;
    if(a < 0)  temp = -a;
    else temp = a;
    return temp;
}
/**************************************************************************
函数功能：增量PI控制器
入口参数：编码器测量值，目标速度
返回  值：电机PWM
根据增量式离散PID公式
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差
e(k-1)代表上一次的偏差  以此类推
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
先调 I，再调 P，再调 D，最后再统一微调
**************************************************************************/
int Incremental_PI_A (int Encoder, float Target)
{
    static float Bias, Pwm, Last_bias, Prev_bias;
    // static int Bias, Pwm, Last_bias;
    if( Target != 0)Bias = Encoder - ( Target + JG_OFFSET_A) * 0.1;
    else Bias = Encoder - Target * 0.1;
    // Bias = Encoder - Target*0.1;            //计算偏差
    //   Bias = Target - Encoder;            //计算偏差
    Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias + Velocity_KD * (Bias - 2 * Last_bias + Prev_bias); //增量式PI控制器
    //  Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
    if(Pwm > 7200)Pwm = 7200;
    if(Pwm < -7200)Pwm = -7200;
    Prev_bias = Last_bias; //保存上上次偏差
    Last_bias = Bias;                    //保存上一次偏差
    return Pwm;
}
int Incremental_PI_B (int Encoder, float Target)
{

    static float Bias, Pwm, Last_bias, Prev_bias;
    //    static int Bias, Pwm, Last_bias;
    if( Target != 0)Bias = Encoder - ( Target + JG_OFFSET_B) * 0.1;
    else Bias = Encoder - Target * 0.1;
    //      else if(Target<0)Bias=Encoder-( Target-3 )*0.1;                //计算偏差
    //      else Target =0;

    // Bias = Encoder - Target;            //计算偏差
    //Bias = Encoder - (Target+3)*0.1;          //计算偏差
    Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias  + Velocity_KD * (Bias - 2 * Last_bias + Prev_bias); //增量式PI控制器
    if(Pwm > 7200)Pwm = 7200;
    if(Pwm < -7200)Pwm = -7200;
    Prev_bias = Last_bias; //保存上上次偏差
    Last_bias = Bias;                    //保存上一次偏差
    return Pwm;                         //增量输出
}

int Incremental_PI_C (int Encoder_Left, int Encoder_Right, int Target)
{
    static float Velocity, Encoder_Least, Encoder;
    static float Encoder_Integral;
    //=============速度PI控制器=======================//
    Encoder_Least = (Encoder_Left + Encoder_Right) - Target;               //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零）
    Encoder *= 0.8;                                                   //===一阶低通滤波器
    Encoder += Encoder_Least * 0.2;                                   //===一阶低通滤波器
    Encoder_Integral += Encoder;                                      //===积分出位移 积分时间：40ms
    Encoder_Integral = Encoder_Integral - Target;                   //===接收遥控器数据，控制前进后退
    if (Encoder_Integral > 10000)    Encoder_Integral = 10000;        //===积分限,控制最高速度
    if (Encoder_Integral < -10000) Encoder_Integral = -10000;         //===积分限幅

    Velocity = Encoder * Velocity_KP + Encoder_Integral * Velocity_KI;                  //===速度PI控制
    // if (Turn_Off(KalFilter.angle, Battery_Voltage) == 1 || Flag_Stop == 1)    Encoder_Integral = 0;//小车停止的时候积分清零
    return Velocity;
}

int Position_PID_A(int Encoder, int Target)
{
    static float Bias, Pwm, Integral_bias, Last_Bias, PwmKI = 0;
    Bias = Encoder - Target;                                     //计算偏差
    Integral_bias += Bias;                                 //求出偏差的积分
    PwmKI = Position_KI * Integral_bias;
    if(PwmKI > 3000) Integral_bias = 3000 / Position_KI;
    Pwm = Position_KP * Bias + PwmKI + Position_KD * (Bias - Last_Bias); //位置式PID控制器
    Last_Bias = Bias;                                        //保存上一次偏差
    return Pwm;
}
int Position_PID_B(int Encoder, int Target)
{
    static float Bias, Pwm, Integral_bias, Last_Bias, PwmKI = 0;
    Bias = Encoder - Target;                                     //计算偏差
    Integral_bias += Bias;                                 //求出偏差的积分
    PwmKI = Position_KI * Integral_bias;
    if(PwmKI > 3000) Integral_bias = 3000 / Position_KI;
    Pwm = Position_KP * Bias + PwmKI + Position_KD * (Bias - Last_Bias); //位置式PID控制器
    Last_Bias = Bias;                                        //保存上一次偏差
    return Pwm;
}

/**************************************************************************
函数功能：通过指令对小车进行遥控
入口参数：串口指令
返回  值：无
**************************************************************************/
void Get_RC(void)
{

    TrackContor1();

    //  Kinematic_Analysis(50,10);//得到控制目标值，进行运动学分析,固定曲率行驶
    Kinematic_Analysis(Move_Y, Move_Z); //得到控制目标值，进行运动学分析
}





//int D_AD_VALUE=-1710;     //确定左右传感器差值

//void GetParament(void)
//{
//  int DValue1=0;
//  int DValue2=0;
//
//  int Left1_AD,Left2_AD,Right1_AD,Right2_AD,Mid_AD;//定义左右中传感器AD值变量
//
//  static int Left1Max=0,Left2Max=0;
//  static int Right1Max=0,Right2Max=0;
//  static int MidMax=0;
//  static int Left1_Thersh=0,Left2_Thersh=0;
//  static int Right1_Thersh=0,Right2_Thersh=0;
//  static int Left1_Span=0,Left2_Span=0;
//  static int Right1_Span=0,Right2_Span=0;
//
//  Right1_AD=After_filter[3];  //右传感器获取的AD值
//  Right2_AD=After_filter[3];
//  Mid_AD=After_filter[2];     //中间传感器获取的AD值
//  Left1_AD=After_filter[1];       //左传感器获取的AD值
//  Left2_AD=After_filter[0];
//
//  if(Left1_AD>Left1Max)
//  {
//      Left1Max=Left1_AD;
//      Left1_Thersh=Mid_AD;
//      Left1_Span=(2*Left1Max-Left1_AD)*2-(Left1_AD-Right1_AD+D_AD_VALUE);
//
//  }
//  if(Left2_AD>Left2Max)
//  {
//      Left2Max=Left2_AD;
//      Left2_Thersh=Mid_AD;
//      Left2_Span=(2*Left2Max-Left2_AD)*2-(Left2_AD-Right2_AD+D_AD_VALUE);
//
//  }
//  if(Right1_AD>Right1Max)
//  {
//      Right1Max=Right1_AD;
//      Right1_Thersh=Mid_AD;
//      Right1_Span=(Right1_AD-2*Right1Max)*2-(Left1_AD-Right1_AD+D_AD_VALUE);
//  }
//  if(Right2_AD>Right2Max)
//  {
//      Right2Max=Right2_AD;
//      Right2_Thersh=Mid_AD;
//      Right2_Span=(Right2_AD-2*Right2Max)*2-(Left2_AD-Right2_AD+D_AD_VALUE);
//  }

//  if(Mid_AD>MidMax)
//  {
//      MidMax=Mid_AD;
//  }
//  DValue1=Right1_AD-Left1_AD;//差值，右传感器减左传感器
//  DValue2= Right2_AD+Left2_AD;
//
//  printf("Right1_AD:%d Right2_AD:%d Mid_AD:%d Left1_AD:%d Left2_AD:%d\r\n",Right1_AD,Right2_AD,Mid_AD,Left1_AD,Left2_AD);
//  printf("D_AD_VALUE1:%d Left1Max:%d MidMax:%d Right1Max:%d  Left1_Thersh:%d Right1_Thersh:%d Left1_Span:%d Right1_Span:%d\r\n",DValue1,Left1Max,MidMax,Right1Max,Left1_Thersh,Right1_Thersh,Left1_Span,Right1_Span);
//  printf("D_AD_VALUE2:%d Left2Max:%d MidMax:%d Right2Max:%d  Left2_Thersh:%d Right2_Thersh:%d Left2_Span:%d Right2_Span:%d\r\n",DValue2,Left2Max,MidMax,Right2Max,Left2_Thersh,Right2_Thersh,Left2_Span,Right2_Span);
//}

