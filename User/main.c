#include "stm32f10x.h"
#include "sys.h"
/**************************************************************************
��Ŀ��MiniModelCar
˵������׼����ģ������С��
**************************************************************************/
u8 Flag_Stop = 1; //ֹͣ��־λ�� ��ʾ��־λ Ĭ��ֹͣ ��ʾ��  0�򿪣�1�ر�
int Encoder_A, Encoder_B; //���������������
long int Position_A, Position_B, Rate_A, Rate_B; //PID������ر���
long int Motor_A, Motor_B; //���PWM���� , Motor_A���ҵ����Motor_B������
long int Target_A, Target_B; //���Ŀ��ֵ
u16 delay_50, delay_flag; //��ʱ��ر���
u8 Run_Flag = 0; //�ٶȻ���λ��ģʽָʾ����
int UrxSt, UtxSt;   //���ڿ�����ر���
float Pitch, Roll, Yaw, Gryo_Z, Move_X, Move_Y, Move_Z; //����Ƕ� Z�������Ǻ�XYZ��Ŀ���ٶ�
float	Position_KP = 14, Position_KI = 0, Position_KD = 25; //λ�ÿ���PID����
//0.1 1 2 6 8 16
float Velocity_KP = 10, Velocity_KI = 10, Velocity_KD = 0;	  //�ٶȿ���PID����
float Read_Speed_Left, Read_Speed_Right;


float Speed; //�ٶ�

extern float After_filter[M];


/*==========================================================С������&����
���⣺
1.С����������ƫ�ƣ����ҵ������PID�㷨�ϸ�Ŀ���ٶ���΢������ȡ������΢�֣��ﵽɳ��·��ֱ�߶ξ�����ƽֱ

���ܣ�
1.����������Ϊ10ms����ת����������
2.����ת��һȦ���=����������7*���ٱ�42*4��Ƶ = 1176  ������ÿ��ת��Ȧ��Ϊ������������*100��/1176 Ȧ


�ο�ֵ��׼���ã�
  1. �ο�ֵ = ���� + �ڣ�/2.5   �����ȶ��������ã�
	2.�ɼ���ɫɫ���һ��ƫ������Ϊ�ο�ֵ���ο�ֵ = �� + X

*/

/*==========================================================��Ļ�������
1.��ʾ�ߴ磺��2160mm ��1220mm
2.�ֱ��ʣ�3840x2160   ��1.77778����/mm  ��1.77049����/mm


*/

/* ==========================================================С������
������������
1.��ѹ��DC 12V �����ʣ�0.2W�����ص�������65mA������ת�٣�550rpm�����������0.15A���ת�٣�390rpm�����ٱȣ�1��118��
2.����ź����ͣ�����AB��
3.��ӦƵ�ʣ�100KHz
4.������������7PPR �� ��������294 ��7x���ٱ�
5.�Ż�����������14����7�Լ�

С�����������
1.�ߴ��񣺳�129mm  ��45mm  ��47mm �� �����ظ߶�  mm�������ظ߶�  mm��������ظ߶�  mm
2.Ѳ�ߴ��м�ࣺ mm


*/



char send_buf[32];


int main(void)
{


    delay_init();	    	            //=====��ʱ������ʼ��
    //JTAG_Set(JTAG_SWD_DISABLE);     //=====�ر�JTAG�ӿ�
    JTAG_Set(SWD_ENABLE);           //=====��SWD�ӿ� �������������SWD�ӿڵ���
    LED_Init();                     //=====��ʼ���� LED ���ӵ�Ӳ���ӿ�
    KEY_Init();                     //=====������ʼ��
    MY_NVIC_PriorityGroupConfig(2);	//=====�����жϷ���
    MiniModelCar_PWM_Init(7199, 0);  //=====��ʼ��PWM 10KHZ������������� �����ʼ������ӿ�
    uart3_init(9600);             //=====����3��ʼ��
    Encoder_Init_TIM2();            //=====�������ӿ�
    Encoder_Init_TIM8();            //=====��ʼ��������

    // TIM3_Cap_Init(99, 7199); //10Khz�ļ���Ƶ�ʣ�������100Ϊ10ms
    TIM3_Cap_Init(499, 7199); //10Khz�ļ���Ƶ�ʣ�������500Ϊ50ms  118���ٱ��ȶ���
	 // TIM3_Cap_Init(1499, 7199); //10Khz�ļ���Ƶ�ʣ�������1500Ϊ150ms   42���ٱ�

    delay_ms(500);                  //=====��ʱ,��ʼ��MPU6050ǰ����ʱһ��ʱ�䣬ȷ����ʼ���ɹ�
    IIC_Init();                     //=====IIC��ʼ��
    MPU6050_initialize();           //=====MPU6050��ʼ��
    DMP_Init();                     //=====��ʼ��DMP

#if ADC_NO_DMA		   //====Ϊ�٣���ִ��
    Adc_Init();                     //=====adc��ʼ��
    Trail_ADC_Init();								//=====Ѳ�ߴ���adc��ʼ��
#elif ADC_IS_DMA       //====Ϊ�棬ִ��
    Trail_ADC_DMA_Init();
#endif

    MiniModelCar_EXTI_Init();        //=====MPU6050 5ms��ʱ�жϳ�ʼ��



    while(1)
    {


        GetSetParament(); //ѭ����������׼��������&�г���Ϣ��ӡ

        /*  //�̶����ʲ�ѭ��ң�ز��������
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

        /*    //�̶����ʲ�ѭ��ң�ز��������
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

