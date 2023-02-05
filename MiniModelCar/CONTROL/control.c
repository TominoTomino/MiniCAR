#include "control.h"
#include "filter.h"
#include "adc.h"
/**************************************************************************
��Ŀ��MiniModelCar
˵������׼����ģ������С��
**************************************************************************/

u8 Flag_Target, Flag_Change; //��ر�־λ

int Voltage;//��ص�ѹ������صı���
float Voltage_Count, Voltage_All; //��ѹ������ر���
int Temp_Chip;//��ص�ѹ������صı���
float Temp_Count, Temp_All; //�¶Ȳ�����ر���

float Gyro_K = 0.004;     //�����Ǳ���ϵ��
int j, sum;
//#define a_PARAMETER          (0.079f)
#define a_PARAMETER          (0.5f)
#define JG_OFFSET_A 0//6//3    //ǰ����ֱ�߲���  (1�ų�3��2�ų�6)
#define JG_OFFSET_B  0 //3//0//3    //ǰ����ֱ�߲���

int Sensor_Left, Sensor_Middle, Sensor_Right, Sensor; //����Ѳ�����
int S_Input_Read, S_D_Input_Read, S_M_Input_Read, S_D_M_Input_Read; //������λ��ֵ����
int Flag_S = 0 ; //��ǼӼ�������״̬
int Flag_state = 0, Flag_Run = 0;

int Flag_Accelerate = 0; //��ǼӼ��٣�0��Ĭ�ϴ� ��1���ر�


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
�������ܣ�С���˶���ѧģ��
��ڲ�����X Y Z �����ٶȻ���λ��
����  ֵ����
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
�������ܣ���ȡλ�ÿ��ƹ����ٶ�ֵ
��ڲ�����X Y Z ����λ�ñ仯��
����  ֵ����
**************************************************************************/
void Kinematic_Analysis2(float Vy, float Vz)
{
    Rate_A   = Vy - Vz * (a_PARAMETER) * 10;
    Rate_B   = Vy - Vz * (a_PARAMETER) * 10;

}


/**************************************************************************
�������ܣ����еĿ��ƴ��붼��������
         5ms��ʱ�ж���MPU6050��INT���Ŵ���
         �ϸ�֤���������ݴ����ʱ��ͬ��
**************************************************************************/
int EXTI9_5_IRQHandler(void)//�������ݲɼ�&����
{
    if(INT == 0)
    {
        EXTI->PR = 1 << 5; //���LINE5�ϵ��жϱ�־λ

        Flag_Target = !Flag_Target;
        if(Flag_Target == 1) //5ms��ȡһ�������Ǻͼ��ٶȼƵ�ֵ
        {
            //    Read_DMP();   //===������̬
            return 0;
        }

        //  Read_DMP();  //===������̬

        filter();  //===Ѳ�ߴ���DMA�������ݲɼ�
        ReadSensor01();//===�������ݻ�׼ת������ȡѲ�ߴ��е�λ ������������Ϊ�˷����׼���ʱ�رյ����Ҳ�ܶ�ȡУ׼�������

        ++delay_50;
        ++delay_flag;

        Voltage_All += Get_battery_volt(); //��β����ۻ�
        if(++Voltage_Count == 100) Voltage = Voltage_All / 100, Voltage_All = 0, Voltage_Count = 0; //��ƽ��ֵ ��ȡ��ص�ѹ

        Temp_All += Get_chip_temp(); //����ۻ������¶�
        if(++Temp_Count == 100) Temp_Chip = Temp_All / 100, Temp_All = 0, Temp_Count = 0; //��ƽ��ֵ ��ȡоƬ�¶�



    }
    return 0;
}


int TIM3_IRQHandler(void)   //TIM3�жϣ����������˶������Ա�֤�˶����Ӿ�ȷ
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //���ָ����TIM�жϷ������:TIM �ж�Դ
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //���TIMx���жϴ�����λ:TIM �ж�Դ

        //===10ms����һ�Σ�Ϊ�˱�֤M�����ٵ�ʱ���׼�����ȶ�ȡ����������
        Encoder_A = Read_Encoder(2); //===��ȡ��������ֵ
        Encoder_B = -Read_Encoder(8); //===��ȡ��������ֵ������PA0 PA1���򣬼��������AB������򵽵��ͬ������תЧ��

        Track_Rx_Logic();  //===���´���ͨ������

        //filter();  //===Ѳ�ߴ���DMA�������ݲɼ�
        //ReadSensor01();//===�������ݻ�׼ת������ȡѲ�ߴ��е�λ ������������Ϊ�˷����׼���ʱ�رյ����Ҳ�ܶ�ȡУ׼�������



        if(Turn_Off(Voltage) == 1) Flag_Stop = 1; //===�����ص�ѹ�����쳣��رյ��Ϩ��,����Ĭ�ϻ����رյ��״̬����Ҫָ���

        if(Flag_Stop == 0)
        {
            Get_RC();

            Motor_B = Incremental_PI_B(Encoder_B, Target_B); //===�ٶȱջ����Ƽ�����B����PWM
            Motor_A = Incremental_PI_A(Encoder_A, Target_A); //===�ٶȱջ����Ƽ�����A����PWM

            Led_Flash(100);  //===LED��˸;����ģʽ 1s�ı�һ��ָʾ�Ƶ�״̬


            Xianfu_Pwm(6900);                     //===PWM�޷�
            Set_Pwm(Motor_A, Motor_B);    //===��ֵ��PWM�Ĵ���
        }






    }

    return 0;
}
/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ�����PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int motor_a, int motor_b)
{
    if(motor_a > 0)        PWMA1 = 7200, PWMA2 = 7200 - motor_a;
    else                PWMA2 = 7200, PWMA1 = 7200 + motor_a;

    if(motor_b > 0)         PWMB1 = 7200, PWMB2 = 7200 - motor_b;
    else                PWMB2 = 7200, PWMB1 = 7200 + motor_b;

}
/**************************************************************************
�������ܣ�����PWM��ֵ
��ڲ�������ֵ
����  ֵ����
**************************************************************************/
void Xianfu_Pwm(int amplitude)
{
    if(Motor_A < -amplitude) Motor_A = -amplitude;
    if(Motor_A > amplitude)  Motor_A = amplitude;
    if(Motor_B < -amplitude) Motor_B = -amplitude;
    if(Motor_B > amplitude)  Motor_B = amplitude;

}
/**************************************************************************
�������ܣ�λ��PID���ƹ������ٶȵ�����
��ڲ������ޡ���ֵ
����  ֵ����
**************************************************************************/
void Xianfu_Velocity(int amplitude_A, int amplitude_B)
{
    if(Motor_A < -amplitude_A) Motor_A = -amplitude_A;  //λ�ÿ���ģʽ�У�A����������ٶ�
    if(Motor_A > amplitude_A)  Motor_A = amplitude_A;    //λ�ÿ���ģʽ�У�A����������ٶ�
    if(Motor_B < -amplitude_B) Motor_B = -amplitude_B;  //λ�ÿ���ģʽ�У�B����������ٶ�
    if(Motor_B > amplitude_B)  Motor_B = amplitude_B;       //λ�ÿ���ģʽ�У�B����������ٶ�

}
/**************************************************************************
�������ܣ������޸�С������״̬
��ڲ�������
����  ֵ����
**************************************************************************/
void Key(void)
{
    vu8 tmp;
    tmp = click_N_Double(50);
    //if(tmp==1)Flag_Stop=!Flag_Stop;//�������Ƶ����ͣ
}
/**************************************************************************
�������ܣ��쳣�رյ��
��ڲ�������ѹ
����  ֵ��1���쳣  0������
**************************************************************************/
u8 Turn_Off( int voltage)
{
    u8 temp;
    // if(voltage < 1110 || Flag_Stop == 1) //��ص�ѹ����11.1V�رյ��
    if(voltage < 1000 || Flag_Stop == 1) //��ص�ѹ����10V�رյ��
    {
        temp = 1;
        PWMA1 = 0; //�������λ����
        PWMB1 = 0; //�������λ����
        PWMA2 = 0; //�������λ����
        PWMB2 = 0; //�������λ����
    }
    else
        temp = 0;

    if(voltage < 1150 || Flag_Stop == 1)  //��ص�ѹ����11.5V�򿪳��
    {
        CHARGE = 1; //�򿪳��
    }
    else if(voltage > 1200)   CHARGE = 0; //�رճ��


    return temp;
}

/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����long int
����  ֵ��unsigned int
**************************************************************************/
u32 myabs(long int a)
{
    u32 temp;
    if(a < 0)  temp = -a;
    else temp = a;
    return temp;
}
/**************************************************************************
�������ܣ�����PI������
��ڲ���������������ֵ��Ŀ���ٶ�
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)������ƫ��
e(k-1)������һ�ε�ƫ��  �Դ�����
pwm�����������
�����ǵ��ٶȿ��Ʊջ�ϵͳ���棬ֻʹ��PI����
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)
�ȵ� I���ٵ� P���ٵ� D�������ͳһ΢��
**************************************************************************/
int Incremental_PI_A (int Encoder, float Target)
{
    static float Bias, Pwm, Last_bias, Prev_bias;
    // static int Bias, Pwm, Last_bias;
    if( Target != 0)Bias = Encoder - ( Target + JG_OFFSET_A) * 0.1;
    else Bias = Encoder - Target * 0.1;
    // Bias = Encoder - Target*0.1;            //����ƫ��
    //   Bias = Target - Encoder;            //����ƫ��
    Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias + Velocity_KD * (Bias - 2 * Last_bias + Prev_bias); //����ʽPI������
    //  Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias;
    if(Pwm > 7200)Pwm = 7200;
    if(Pwm < -7200)Pwm = -7200;
    Prev_bias = Last_bias; //�������ϴ�ƫ��
    Last_bias = Bias;                    //������һ��ƫ��
    return Pwm;
}
int Incremental_PI_B (int Encoder, float Target)
{

    static float Bias, Pwm, Last_bias, Prev_bias;
    //    static int Bias, Pwm, Last_bias;
    if( Target != 0)Bias = Encoder - ( Target + JG_OFFSET_B) * 0.1;
    else Bias = Encoder - Target * 0.1;
    //      else if(Target<0)Bias=Encoder-( Target-3 )*0.1;                //����ƫ��
    //      else Target =0;

    // Bias = Encoder - Target;            //����ƫ��
    //Bias = Encoder - (Target+3)*0.1;          //����ƫ��
    Pwm += Velocity_KP * (Bias - Last_bias) + Velocity_KI * Bias  + Velocity_KD * (Bias - 2 * Last_bias + Prev_bias); //����ʽPI������
    if(Pwm > 7200)Pwm = 7200;
    if(Pwm < -7200)Pwm = -7200;
    Prev_bias = Last_bias; //�������ϴ�ƫ��
    Last_bias = Bias;                    //������һ��ƫ��
    return Pwm;                         //�������
}

int Incremental_PI_C (int Encoder_Left, int Encoder_Right, int Target)
{
    static float Velocity, Encoder_Least, Encoder;
    static float Encoder_Integral;
    //=============�ٶ�PI������=======================//
    Encoder_Least = (Encoder_Left + Encoder_Right) - Target;               //===��ȡ�����ٶ�ƫ��==�����ٶȣ����ұ�����֮�ͣ�-Ŀ���ٶȣ��˴�Ϊ�㣩
    Encoder *= 0.8;                                                   //===һ�׵�ͨ�˲���
    Encoder += Encoder_Least * 0.2;                                   //===һ�׵�ͨ�˲���
    Encoder_Integral += Encoder;                                      //===���ֳ�λ�� ����ʱ�䣺40ms
    Encoder_Integral = Encoder_Integral - Target;                   //===����ң�������ݣ�����ǰ������
    if (Encoder_Integral > 10000)    Encoder_Integral = 10000;        //===������,��������ٶ�
    if (Encoder_Integral < -10000) Encoder_Integral = -10000;         //===�����޷�

    Velocity = Encoder * Velocity_KP + Encoder_Integral * Velocity_KI;                  //===�ٶ�PI����
    // if (Turn_Off(KalFilter.angle, Battery_Voltage) == 1 || Flag_Stop == 1)    Encoder_Integral = 0;//С��ֹͣ��ʱ���������
    return Velocity;
}

int Position_PID_A(int Encoder, int Target)
{
    static float Bias, Pwm, Integral_bias, Last_Bias, PwmKI = 0;
    Bias = Encoder - Target;                                     //����ƫ��
    Integral_bias += Bias;                                 //���ƫ��Ļ���
    PwmKI = Position_KI * Integral_bias;
    if(PwmKI > 3000) Integral_bias = 3000 / Position_KI;
    Pwm = Position_KP * Bias + PwmKI + Position_KD * (Bias - Last_Bias); //λ��ʽPID������
    Last_Bias = Bias;                                        //������һ��ƫ��
    return Pwm;
}
int Position_PID_B(int Encoder, int Target)
{
    static float Bias, Pwm, Integral_bias, Last_Bias, PwmKI = 0;
    Bias = Encoder - Target;                                     //����ƫ��
    Integral_bias += Bias;                                 //���ƫ��Ļ���
    PwmKI = Position_KI * Integral_bias;
    if(PwmKI > 3000) Integral_bias = 3000 / Position_KI;
    Pwm = Position_KP * Bias + PwmKI + Position_KD * (Bias - Last_Bias); //λ��ʽPID������
    Last_Bias = Bias;                                        //������һ��ƫ��
    return Pwm;
}

/**************************************************************************
�������ܣ�ͨ��ָ���С������ң��
��ڲ���������ָ��
����  ֵ����
**************************************************************************/
void Get_RC(void)
{

    TrackContor1();

    //  Kinematic_Analysis(50,10);//�õ�����Ŀ��ֵ�������˶�ѧ����,�̶�������ʻ
    Kinematic_Analysis(Move_Y, Move_Z); //�õ�����Ŀ��ֵ�������˶�ѧ����
}





//int D_AD_VALUE=-1710;     //ȷ�����Ҵ�������ֵ

//void GetParament(void)
//{
//  int DValue1=0;
//  int DValue2=0;
//
//  int Left1_AD,Left2_AD,Right1_AD,Right2_AD,Mid_AD;//���������д�����ADֵ����
//
//  static int Left1Max=0,Left2Max=0;
//  static int Right1Max=0,Right2Max=0;
//  static int MidMax=0;
//  static int Left1_Thersh=0,Left2_Thersh=0;
//  static int Right1_Thersh=0,Right2_Thersh=0;
//  static int Left1_Span=0,Left2_Span=0;
//  static int Right1_Span=0,Right2_Span=0;
//
//  Right1_AD=After_filter[3];  //�Ҵ�������ȡ��ADֵ
//  Right2_AD=After_filter[3];
//  Mid_AD=After_filter[2];     //�м䴫������ȡ��ADֵ
//  Left1_AD=After_filter[1];       //�󴫸�����ȡ��ADֵ
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
//  DValue1=Right1_AD-Left1_AD;//��ֵ���Ҵ��������󴫸���
//  DValue2= Right2_AD+Left2_AD;
//
//  printf("Right1_AD:%d Right2_AD:%d Mid_AD:%d Left1_AD:%d Left2_AD:%d\r\n",Right1_AD,Right2_AD,Mid_AD,Left1_AD,Left2_AD);
//  printf("D_AD_VALUE1:%d Left1Max:%d MidMax:%d Right1Max:%d  Left1_Thersh:%d Right1_Thersh:%d Left1_Span:%d Right1_Span:%d\r\n",DValue1,Left1Max,MidMax,Right1Max,Left1_Thersh,Right1_Thersh,Left1_Span,Right1_Span);
//  printf("D_AD_VALUE2:%d Left2Max:%d MidMax:%d Right2Max:%d  Left2_Thersh:%d Right2_Thersh:%d Left2_Span:%d Right2_Span:%d\r\n",DValue2,Left2Max,MidMax,Right2Max,Left2_Thersh,Right2_Thersh,Left2_Span,Right2_Span);
//}

