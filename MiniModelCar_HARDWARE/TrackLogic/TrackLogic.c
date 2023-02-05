/**************************************************************************
�ļ�˵�������ļ�Ϊ����ʵ��ѭ���߼�����
��ڲ�������
����  ֵ����
**************************************************************************/
#include "TrackLogic.h"
#include "adc.h"

/**************************************************************************
                      ������������
**************************************************************************/
#define FLASH_SAVE_ADDR  0X0800E000     //����FLASH �����ַ(����Ϊż��������ֵҪ���ڱ�������ռ��FLASH�Ĵ�С+0X08000000)
#define BLACK_COLOR_DELAY 30  //δ����״̬��ʱֹͣ BLACK_COLOR_DELAY*50 ms
#define TRACK_INIT_SPEED  100   //Ѳ�߳�ʼ�ٶ�


//�ɼ�����ֵ
//int fADCDatum0[5] = {1463, 1902, 1744, 1627, 1558}; //�ɼ���ͷ������ɫ���Եֵ��Ϊ��׼�����ҵ��󣬴�ǰ������
//int bADCDatum0[5] = {1596, 1631, 1691, 1321, 1662}; //�ɼ���β������ɫ���Եֵ��Ϊ��׼�����ҵ��󣬴�ǰ������
//int mADCDatum0[4] = {1618, 1632, 1862, 1827}; //�ɼ��м䴫����ɫ���Եֵ��Ϊ��׼
/*
//����ɼ����ݣ�/2
ADCDatum0[0-4]:2105 ,2529 ,2341 ,2174 ,2172 ,
mADCDatum0[0-3]:1828 ,1880 ,2148 ,2168 ,
bADCDatum0[0-4]:1668 ,1834 ,2031 ,1494 ,2003 ,


*/

// ѭ������
//1277,1580,1968,1337,1322

//u16 fADCDatum0[5] = { 2000, 2000, 2000, 2000, 2000}; //�ɼ���ͷ������ɫ���Եֵ��Ϊ��׼�����ҵ��󣬴�ǰ������
//u16 mADCDatum0[4] = {2000, 2000, 2000, 2000}; //�ɼ��м䴫����ɫ���Եֵ��Ϊ��׼
////u16 bADCDatum0[5] = {1200,1000,1200,1200,1000};
//u16 bADCDatum0[5] = {1200,1200,1300,1200,1100};
//�ֳ�����ɼ�
//u16 bADCDatum0[5] = {1500, 1500, 1600, 1600, 1500};
//u16 bADCDatum0[5] = {1183, 1725, 1633, 1475, 1355}; //�ɼ���β������ɫ���Եֵ��Ϊ��׼�����ҵ��󣬴�ǰ������

//������촫�л�׼  05/26�ɼ�  /2.2
//int fADCDatum0[5] = {2105 ,2529 ,2341 ,2174 ,2172 }; //�ɼ���ͷ������ɫ���Եֵ��Ϊ��׼�����ҵ��󣬴�ǰ������
//u16 fADCDatum0[5] = { 1605 ,1775 ,1773 ,1645 ,1775 }; //�ɼ���ͷ������ɫ���Եֵ��Ϊ��׼�����ҵ��󣬴�ǰ������
//1017.001127.00646.001289.00608.00
//2697.003190.002803.003105.001676.00

/*
//�ɼ�ɫ����С��׼��ֵ   1�ų�
u16 fADCDatum0[5] = {1600 ,1800 ,1800 ,1000 ,1900};
//u16 fADCDatum0[5] = {1300, 1800, 1500, 1300, 1400}; //�ɼ���ͷ������ɫ���Եֵ��Ϊ��׼�����ҵ��󣬴�ǰ������
u16 mADCDatum0[4] = {1882 ,1895 ,2089 ,2128}; //�ɼ��м䴫����ɫ���Եֵ��Ϊ��׼
//u16 bADCDatum0[5] = {2057,2223,2289,1948,2267}; //�ɼ���β������ɫ���Եֵ��Ϊ��׼�����ҵ��󣬴�ǰ������
//u16 bADCDatum0[5] = {1678 ,1842 ,1948 ,1534 ,1992};
u16 bADCDatum0[5] = {1900 ,1842 ,1948 ,1834 ,1992}; //0605

u16 fADCDatum0[5] = {1600 ,1800 ,1800 ,1000 ,1900};
u16 mADCDatum0[4] = {1882 ,1895 ,2089 ,2128}; //�ɼ��м䴫����ɫ���Եֵ��Ϊ��׼
u16 bADCDatum0[5] = {2057,2223,2289,1948,2267}; //�ɼ���β������ɫ���Եֵ��Ϊ��׼�����ҵ��󣬴�ǰ������

*/


////�ɼ�ɫ����С��׼��ֵ 2�ų�
//u16 fADCDatum0[5] = {1600, 1600, 1600, 1000, 1600};
//u16 mADCDatum0[4] = {1882, 1895, 2089, 2128}; //�ɼ��м䴫����ɫ���Եֵ��Ϊ��׼
//u16 bADCDatum0[5] = {1678, 1842, 1948, 1534, 1992};

/*
//�ɼ�ɫ����С��׼��ֵ 1�ų�  �ֳ�
u16 fADCDatum0[5] = {2310 ,2249 ,2173 ,1406 ,2088};
u16 mADCDatum0[4] = {2044 ,2178 ,2477 ,2118}; //�ɼ��м䴫����ɫ���Եֵ��Ϊ��׼
u16 bADCDatum0[5] = {1678, 1842, 1948, 1534, 1992};
*/
//u16 fADCDatum0[5] = {2206 ,2204 ,2133 ,1350 ,1883};
//u16 mADCDatum0[4] = {2300, 2300, 2300,1900 }; //�ɼ��м䴫����ɫ���Եֵ��Ϊ��׼
//u16 bADCDatum0[5] = {1711 ,2099 ,2160 ,1708 ,2113 };


//u16 fADCDatum0[5] = {2367 ,2388 ,2320 ,1488 ,2100};
//u16 mADCDatum0[4] = {2144 ,2233 ,2639 ,2261}; //�ɼ��м䴫����ɫ���Եֵ��Ϊ��׼
//u16 bADCDatum0[5] = {1897 ,2340 ,2485 ,2082 ,2585};

/*
//�칫�һ���С��2-���� +800
u16 fADCDatum0[5] = {1968 ,1961 ,1778 ,1725 ,1901 };
u16 mADCDatum0[4] = {1631 ,1780 ,1712 ,1521}; //�ɼ��м䴫����ɫ���Եֵ��Ϊ��׼
u16 bADCDatum0[5] = {1627 ,1891 ,1704 ,1646 ,1637};
*/

/*
//�칫�һ���С��2-������ +800
u16 fADCDatum0[5] = {1700 ,1700 ,1600 ,2100 ,1800 };
u16 mADCDatum0[4] = {1585 ,1529 ,1868 ,1932}; //�ɼ��м䴫����ɫ���Եֵ��Ϊ��׼
u16 bADCDatum0[5] = {1518 ,1748 ,1774 ,1507 ,1953};
*/

/*
//�칫�һ���С��1-���� +800 300LUX
u16 fADCDatum0[5] = {1654 ,1628 ,1483 ,1954 ,1782};
u16 mADCDatum0[4] = {1565 ,1505 ,1808 ,1884 }; //�ɼ��м䴫����ɫ���Եֵ��Ϊ��׼
u16 bADCDatum0[5] = {1493 ,1700 ,1726 ,1481 ,1882};
*/


/*
//�칫�һ���С��1-����+900  300LUX
u16 fADCDatum0[5] = {1751 ,1711 ,1585 ,2141 ,1893};
u16 mADCDatum0[4] = {1681 ,1619 ,1952 ,2021 }; //�ɼ��м䴫����ɫ���Եֵ��Ϊ��׼
u16 bADCDatum0[5] = {1618 ,1835 ,1861 ,1586 ,1997};
*/



/*
//�칫�һ���С��1-����+900   2000LUX
u16 fADCDatum0[5] = {1186 ,1155 ,1072 ,1419 ,1263};
u16 mADCDatum0[4] = {1135 ,1101 ,1279 ,1302 }; //�ɼ��м䴫����ɫ���Եֵ��Ϊ��׼
u16 bADCDatum0[5] = {1111 ,1213 ,1214 ,1089 ,1300};
*/

/*
//�칫�һ���С��1-����+300   2000LUX
u16 fADCDatum0[5] = {681 ,657 ,574 ,917 ,766};
u16 mADCDatum0[4] = {623 ,600 ,777 ,806 }; //�ɼ��м䴫����ɫ���Եֵ��Ϊ��׼
u16 bADCDatum0[5] = {609 ,713 ,713 ,591 ,813};
*/



/*
//�ɼ�ɫ����С��׼��ֵ 2�ų�  �ֳ��������ڹ�
u16 fADCDatum0[5] = {1100 ,1100 ,1000 ,1100 ,1100};
u16 mADCDatum0[4] = {1100 ,1100 ,1000 ,1100 }; //�ɼ��м䴫����ɫ���Եֵ��Ϊ��׼
u16 bADCDatum0[5] = {1100 ,1100 ,1000 ,1100 ,1100};
*/

/*
//�ɼ�ɫ����С��׼��ֵ 1�ų�  �ֳ�
u16 fADCDatum0[5] = {1600, 1600, 1600, 1600, 1900};
u16 mADCDatum0[4] = {2300, 2300, 2300,1900 }; //�ɼ��м䴫����ɫ���Եֵ��Ϊ��׼
u16 bADCDatum0[5] = {1678, 1842, 1948, 1534, 1992};
*/

//�ɼ�ɫ������׼��ֵ
int fADCDatum1[5] = {1886, 1784, 1632, 2317, 1932}; //�ɼ���ͷ������ɫ���Եֵ��Ϊ��׼�����ҵ��󣬴�ǰ������
int mADCDatum1[4] = {1708, 1683, 2043, 2087}; //�ɼ��м䴫����ɫ���Եֵ��Ϊ��׼
int bADCDatum1[5] = {1703, 1896, 1943, 1638, 2185}; //�ɼ���β������ɫ���Եֵ��Ϊ��׼�����ҵ��󣬴�ǰ������




//�������ϴ��л�׼
//int fADCDatum0[5] = {1600, 2100, 1900, 1800, 1700}; //�ɼ���ͷ������ɫ���Եֵ��Ϊ��׼�����ҵ��󣬴�ǰ������
//int bADCDatum0[5] = {1596, 1631, 1691, 1321, 1662}; //�ɼ���β������ɫ���Եֵ��Ϊ��׼�����ҵ��󣬴�ǰ������
//int mADCDatum0[4] = {1618, 1632, 1862, 1827}; //�ɼ��м䴫����ɫ���Եֵ��Ϊ��׼

//int fADCDatum0[5] = {1309, 1979, 1611, 1266, 1386}; //�ɼ���ͷ������ɫ���Եֵ��Ϊ��׼�����ҵ��󣬴�ǰ������
//int bADCDatum0[5] = {1522, 1708, 1905, 1429, 1855}; //�ɼ���β������ɫ���Եֵ��Ϊ��׼�����ҵ��󣬴�ǰ������
//int mADCDatum0[4] = {1671, 1663, 1928, 1992}; //�ɼ��м䴫����ɫ���Եֵ��Ϊ��׼


#if ( CAR_NUM == 1 )
/*
//�칫�һ���С��1-������+900  300LUX �ȶ��ܲ���(���ڴ���)����β�������ɼ�
u16 fADCDatum0[5] = {1884 ,1797 ,1638 ,2302 ,1947};  //��ͷ������+900 
u16 mADCDatum0[4] = {1710 ,1640 ,2013 ,2082}; //�м䴫����+900
u16 bADCDatum0[5] = {1786 ,1972 ,1993 ,1714 ,2215}; //��β��������1000
*/

//�ֳ�����С��1-������+900  300LUX �ȶ��ܲ���(���ڴ���)����β�������ɼ�
//u16 fADCDatum0[5] = {1893 ,1788 ,1635 ,2282 ,1898};  //��ͷ������+900 
//u16 mADCDatum0[4] = {1727 ,1664 ,2013 ,2039}; //�м䴫����+900
//u16 bADCDatum0[5] = {1765 ,1986 ,2030 ,1720 ,2162}; //��β��������1000

u16 fADCDatum0[5] = {1870 ,1801 ,1641 ,2290 ,1902};  //��ͷ������+900 
u16 mADCDatum0[4] = {1695 ,1648 ,1999 ,2031}; //�м䴫����+900
u16 bADCDatum0[5] = {1661 ,1877 ,1910 ,1634 ,2122}; //��β��������1000


#elif ( CAR_NUM == 2 )

/*
//�칫�һ���С��2-����+900  300LUX�������������ȶ��ܲ���(Ҫ���ڴ��������߾���)
u16 fADCDatum0[5] = {1977, 1995, 1892, 1816, 1973}; //��ͷ������+900
u16 mADCDatum0[4] = {1724, 1881, 1817, 1724}; //�м䴫����+900
u16 bADCDatum0[5] = {1815, 2081, 1890, 1807, 1796};  //��β��������1000
*/

/*
//�칫�һ���С��2-����+900  300LUX���ȶ��ܲ���(���ڴ���)����β�������ɼ�
u16 fADCDatum0[5] = {1823 ,1895 ,1893 ,1703 ,1874}; //��ͷ������+700 
u16 mADCDatum0[4] = {1862 ,2035 ,1999 ,1861}; //�м䴫����+1000
//u16 bADCDatum0[5] = {1762 ,2105 ,1863 ,1720 ,1689};  //��β������+900 
u16 bADCDatum0[5] = {1762 ,2105 ,1863 ,1720 ,1589};  //��β������+900
*/

/*
//�칫�һ���С��2-����+900  300LUX���ȶ��ܲ�������β�������ɼ�
u16 fADCDatum0[5] = {2003 ,2087 ,2033 ,2007 ,2228}; //��ͷ������+900 
u16 mADCDatum0[4] = {1764 ,2004 ,1960 ,1793}; //�м䴫����+900
u16 bADCDatum0[5] = {1943 ,2297 ,2112 ,2031 ,2007};  //��β������+1000
*/


//�칫�һ���С��2-����+900  300LUX���ȶ��ܲ�������β�������ɼ�
//u16 fADCDatum0[5] = {2003 ,2087 ,2033 ,2007 ,2228}; //��ͷ������+900 
//u16 mADCDatum0[4] = {1764 ,2004 ,1960 ,1793}; //�м䴫����+900
//u16 bADCDatum0[5] = {1943 ,2297 ,2112 ,2031 ,1600};
//u16 bADCDatum0[5] = {1900 ,2297 ,2112 ,1900 ,1600};  //��β������+1000

u16 fADCDatum0[5] = {1750 ,1801 ,1641 ,2290 ,1800};  //��ͷ������+900 
u16 mADCDatum0[4] = {1595 ,1648 ,1999 ,1900}; //�м䴫����+900
//u16 bADCDatum0[5] = {1600 ,1877 ,1910 ,1634 ,2122}; //��β��������1000
u16 bADCDatum0[5] = {1900 ,2000 ,2010 ,2000 ,2322}; //��β��������1000


#elif ( CAR_NUM == 3 )  //������


#elif ( CAR_NUM == 4 )

//�칫�һ���С��4-����+900  300LUX��������������������������β�������ɼ���
u16 fADCDatum0[5] = {1872 ,1840 ,1729 ,1660 ,1922}; //��ͷ������+900
u16 mADCDatum0[4] = {1580 ,1521 ,1798 ,1798}; //�м䴫����+900
u16 bADCDatum0[5] = {1843 ,1866 ,1680 ,1848 ,1743};  //��β��������1000


#endif



int fSensorRead[5] = {0, 0, 0, 0, 0};    //�洢��ͷ������״̬�������Ҵ洢����д�߼�
int bSensorRead[5] = {0, 0, 0, 0, 0};    //�洢��β������״̬
int mSensorRead[4] = {0, 0, 0, 0};     //�洢�м䴫����״̬


//================================��������ر���
int TimeFlag = 0 ;    // ��ʱ��δ���ְ��߼�ʱ��־λ
int F_Not_Find_White_STOP = 0;
int fSensorReadSum = 0, mSensorReadSum = 0, bSensorReadSum = 0;
int TestFlag, LastTestFlag ;


//================================�г���ر���
u8 Auto_Back_Flag = 0; //����ǰ������״̬0 ǰ��  1����
//��λ���ã����ֲ��ٺ�������ٶȲ�С��6���������״򻬺�˦β
float bGear0 = 0, bGear1 = 1, bGear2 = 2, bGear3 = 3, bGear4 = 4; //������λ����
//float fGear0 = 0, fGear1 = 1, fGear2 = 2, fGear3 = 3, fGear4 = 4; //ǰ����λ����
float error;
int iTime = 0, error_time, error_time1, flag_print = 0;



//================================����������У׼��ر���
int Mod = 0; //Mod=0 ��ӪѲ��ģʽ��  Mod=1 ���������ݻ�׼�ɼ���  Mod=2 ��������׼���
float Menchmark_Coefficient = 900;//2.2;//2; //���봫�������ݻ�׼�ɼ�ʱʹ�ã����ڵ��ڴ������ɼ���ֵ�ķ�Χ�����ȣ�ԽСԽ������һ�㷶Χ(2~4.5)
float White_line_datum[15], Black_road_datum[15];  //���ڱ���ɼ���׼����
int c = 0, aa = 0; //���û�׼������أ�Ĭ��ֵΪ0�������޸Ļ�������;
int jishi = 0, jishi_50 = 0;

float pid_i, pid_d, pid_out;

//================================ͨѶЭ����ر���
int Flag_Turn = 0; //�����ʻ�仯״̬
int iUartCommand = 0x00;//���洮���ֽ�

//================================�ⲿ������ر���
extern float After_filter[M];


/**************************************************************************
�������ܣ�ѭ����������׼��ֵ��׼�ɼ������á���ѯ
ģʽ���ã�
    Ĭ��Ѳ��ģʽ��Mod=0  0xf0��
    ��������׼�ɼ�ģʽ,����رգ�Mod=1  0xf1��0xf3�ɼ���ɫɫ������ 0xf4�ɼ���·���� 0xf5��׼����  0xf6��׼ֵ��ѯ
    ͨ����׼���ݲ�ѯ�����ڴ������ֿ��������Ӧ���飬��������EEPROM�Զ����湦��
    ��������׼���ģʽ������رգ�Mod=2  0xf2��

ע�⣺�ڲɼ�ģʽ�´��ڲ�Ҫ����������ɼ��޸�ָ����Ųɼ�
**************************************************************************/
void DebugDrivingInformation(void)  //�г�������Ϣ����Ӫʱ���ε�����ֹ��������ͨѶ
{
    if (Auto_Back_Flag == 0  ) //ǰ��ģʽ
    {


        switch( jishi)
        {

        case 40:
            usart3_send_head();//ǰ������ ����,�г���ͷ������Ϣ
            //                  printf("time:%d \r\n",error_time);
            // printf("\r\n iTime%d,LastTestFlag:%d,TestFlag:%d,fSensorReadSum:%d",iTime,LastTestFlag,TestFlag,fSensorReadSum);
            //              printf("\r\n pid_d:%.2f , pid_out:%.2f",pid_d , pid_out);
            printf("\r\nfS: %d%d%d%d%d Y:%.2f,Z:%.2f,A:%d,B:%d,error:%.2f", fSensorRead[0], fSensorRead[1], fSensorRead[2], fSensorRead[3], fSensorRead[4], Move_Y, Move_Z, Encoder_A, Encoder_B, error);
			   	//printf("\r\nmS: %d%d%d%d Y:%.2f,Z:%.2f,A:%d,B:%d,error:%.2f", mSensorRead[3], mSensorRead[2], mSensorRead[1], mSensorRead[0], Move_Y, Move_Z, Encoder_A, Encoder_B, error);
            //  usart3_send_head();
            //   printf("\t mSensorRead[0-3]:%d%d%d%d\r\n", mSensorRead[0], mSensorRead[1], mSensorRead[2], mSensorRead[3]);

            //  MyPrintf("\r\nfS: %d%d%d%d%d Y:%.2f,Z:%.2f,A:%d,B:%d,T:%d,mSensorRead[0-3]:%d%d", fSensorRead[0], fSensorRead[1], fSensorRead[2], fSensorRead[3], fSensorRead[4], Move_Y, Move_Z, Encoder_A, Encoder_B, error_time, mSensorRead[0], mSensorRead[1]);
            //            printf("\r\nfSensorRead[0-4]:%d%d%d%d%d\r\n ", fSensorRead[0], fSensorRead[1], fSensorRead[2], fSensorRead[3], fSensorRead[4]);
            //                      printf("Move_Y:%.2f,Move_Z:%.2f,Encoder_A:%d,Encoder_B:%d",Move_Y, Move_Z, Encoder_A, Encoder_B);
            //  delay_ms(100);
            //printf("%c%c",Encoder_A,Encoder_B);
            jishi = 0;
            //  delay_flag = 0;
            break;
        case 80:
            usart3_send_head();
            printf("\r\nV:%d , Temp:%d ,FT:%d,pd:%.2f, pi:%.2f,Flag_Stop:%d", Voltage, Temp_Chip, Flag_Turn, pid_d, pid_i, Flag_Stop);

            jishi = 0;
            delay_flag = 0;
            break;

        //        case 20: //ǰ������ ����,�г����д�����Ϣ
        //            usart3_send_head();
        //            printf("mSensorRead[0-3]:%d%d%d%d\r\n", mSensorRead[0], mSensorRead[1], mSensorRead[2], mSensorRead[3]);
        //            jishi = 0;
        //            break;
        //        case 30:  //ǰ������  ����,�г���β������Ϣ
        //            usart3_send_head();
        //            printf("bSensorRead[4-0]:%d%d%d%d%d\r\n", bSensorRead[4], bSensorRead[3], bSensorRead[2], bSensorRead[1], bSensorRead[0]);
        //            jishi = 0;
        //            break;
        //        case 40:  //�г��ٶ���Ϣ
        //            usart3_send_head();
        //           //      printf("TestFlag:%d Motor_A��%d Motor_B:%d Move_Y:%.2f,Move_Z:%.2f,Encoder_A:%d,Encoder_B:%d\r\n", TestFlag, Motor_A,Motor_B,Move_Y, Move_Z, Encoder_A, Encoder_B);
        //              printf("  Move_Y:%.2f,Move_Z:%.2f,Encoder_A:%d,Encoder_B:%d\r\n", Move_Y, Move_Z, Encoder_A, Encoder_B);
        //           // printf(" Motor_A��%d Motor_B:%d ,Encoder_A:%d,Encoder_B:%d\r\n", Motor_A,Motor_B, Encoder_A, Encoder_B);
        //case 200:
        //            jishi = 0;
        //            delay_flag = 0;
        //          break;

        default:
            break;




        }

        /*
        switch( jishi)
        {

        case 40:
        usart3_send_head();//ǰ������ ����,�г���ͷ������Ϣ
        printf("\r\nAfS[0-4]: %.2f%.2f%.2f%.2f%.2f ", After_filter[0], After_filter[1], After_filter[2], After_filter[3], After_filter[4]);
        jishi = 0;
        break;
        case 80:
        usart3_send_head();
            printf("\r\nAmS[5-9]: %.2f%.2f%.2f%.2f ", After_filter[5], After_filter[9], After_filter[7], After_filter[6]);
                jishi = 0;
            break;
        case 120:
        usart3_send_head();//ǰ������ ����,�г���ͷ������Ϣ
        printf("\r\nAbS[10-14]: %.2f%.2f%.2f%.2f%.2f ", After_filter[10], After_filter[11], After_filter[12], After_filter[13], After_filter[14]);
        jishi = 0;
        delay_flag = 0;
        break;
        default:
        break;

        }
            */


        //              for( c= 0 ;c<15;c++)
        //            {
        //                             usart3_send_head();
        //                  printf("After_filter[%d]:%d  \r\n",c,(int)After_filter[c]);
        //                            delay_ms(1000);
        //                      //if(c>=14) printf("\r\n");
        //
        //            }

    }
    else if (Auto_Back_Flag == 1) //����ģʽ
    {
        switch( jishi)
        {
        case 40:
            usart3_send_head();//ǰ������ ����,�г���ͷ������Ϣ
            //                  printf("time:%d \r\n",error_time);
            // printf("\r\n iTime%d,LastTestFlag:%d,TestFlag:%d,fSensorReadSum:%d",iTime,LastTestFlag,TestFlag,fSensorReadSum);
            //              printf("\r\n pid_d:%.2f , pid_out:%.2f",pid_d , pid_out);
            printf("\r\nbS: %d%d%d%d%d Y:%.2f,Z:%.2f,A:%d,B:%d,error:%.2f", bSensorRead[0], bSensorRead[1], bSensorRead[2], bSensorRead[3], bSensorRead[4], Move_Y, Move_Z, Encoder_A, Encoder_B, error);
           //	printf("\r\nmS: %d%d%d%d Y:%.2f,Z:%.2f,A:%d,B:%d,error:%.2f", mSensorRead[3], mSensorRead[2], mSensorRead[1], mSensorRead[0], Move_Y, Move_Z, Encoder_A, Encoder_B, error);
				jishi = 0;
            break;
        case 80:
            usart3_send_head();
            printf("\r\nV:%d , Temp_Chip:%d ", Voltage, Temp_Chip);

            jishi = 0;
            delay_flag = 0;
            break;

        default:
            break;

        }
    }

}

int a = 0, b = 0;
void GetSetParament(void)
{
    if(delay_flag == 10)jishi = delay_flag;
    else if(delay_flag == 20)jishi = delay_flag;
    else if(delay_flag == 30)jishi = delay_flag;
    else if(delay_flag == 40)jishi = delay_flag;
    else if(delay_flag == 50)jishi = delay_flag;
    else if(delay_flag == 60)jishi = delay_flag;
    else if(delay_flag == 70)jishi = delay_flag;
    else if(delay_flag == 80)jishi = delay_flag;
    else if(delay_flag == 100)jishi = delay_flag;
    else if(delay_flag >= 200)jishi = delay_flag, delay_flag = 0;
    //       else if(delay_flag == 300)jishi =delay_flag;
    //       else if(delay_flag == 400)jishi =delay_flag;
    //     else if(delay_flag == 500)jishi =delay_flag;
    //     else if(delay_flag >= 600)jishi =delay_flag,delay_flag=0;//jishi =0 ,delay_flag=0;

    if(delay_50 == 1000)jishi_50 = delay_50; // ����������Ԥ��ʹ��




    if(Mod == 1)  //��������׼�ɼ�
    {


        if(UrxSt == 0xc3) //�ɼ���������
        {
            for( ; a < 15 ; a++)
            {
                White_line_datum[a] = After_filter[a];
                if(a >= 14)
                {
                    usart3_send_head();
                    printf("White_line_datum Collection completed\r\n");
                }
            }


            if(jishi == 20)
            {

                usart3_send_head();
                printf("White_line_datum[%d]:%d  ", c, (int)White_line_datum[c]);
                ++c;
                jishi = 0;
                delay_flag = 0;
                if(c > 14) UrxSt = NULL, printf("\r\n"), c = 0;
            }


        }
        else if(UrxSt == 0xc4) //�ɼ���·����
        {

            for( ; b < 15 ; b++)
            {
                Black_road_datum[b] = After_filter[b];
                if(b >= 14)
                {
                    usart3_send_head();
                    printf("Black_road_datum Collection completed\r\n");
                }
            }

            if(jishi == 20)
            {
                usart3_send_head();
                printf("Black_road_datum[%d]:%d  ", c, (int)Black_road_datum[c]);
                ++c;
                jishi = 0;
                delay_flag = 0;
                if(c > 14) UrxSt = NULL, printf("\r\n"), c = 0;
            }

        }
        else if(UrxSt == 0xc5) //��׼���ݼ���
        {
            if( c < 5) //���򣺴�ǰ��������2��1����1��2����ͷ����������PA0-PA4,
            {
                if(jishi == 20)
                {
                    fADCDatum0[c] = White_line_datum[c] + Menchmark_Coefficient;
                    // STMFLASH_Write(FLASH_SAVE_ADDR,(u16*)fADCDatum0,10);  //������Ļ�׼д��FLASH��
                    // STMFLASH_Read(FLASH_SAVE_ADDR,(u16*)fADCDatum0,10);   //��Flash�ж�ȡ
                    usart3_send_head();
                    printf("fADCDatum0[%d]:%d  ", c, (int)fADCDatum0[c]);
                    if(c > 4) printf("\r\n");
                    ++c;
                    jishi = 0;
                    delay_flag = 0;
                }

            }
            else if( c >= 5 && c < 10)
            {
                if(jishi == 20)
                {
                    usart3_send_head();
                    if(c == 5)
                    {
                        mADCDatum0[0] = White_line_datum[c] + Menchmark_Coefficient;
                        // STMFLASH_Write(FLASH_SAVE_ADDR+12,(u16*)mADCDatum0,10);  //������Ļ�׼д��FLASH��
                        // STMFLASH_Read(FLASH_SAVE_ADDR+12,(u16*)mADCDatum0,10);       //��Flash�ж�ȡ
                        printf("mADCDatum0[%d]:%d c=%d ", 0, (int)mADCDatum0[0], c);

                    }
                    else if(c == 9)
                    {
                        mADCDatum0[1] = White_line_datum[c] + Menchmark_Coefficient;
                        // STMFLASH_Write(FLASH_SAVE_ADDR+12,(u16*)mADCDatum0,10);  //������Ļ�׼д��FLASH��
                        // STMFLASH_Read(FLASH_SAVE_ADDR+12,(u16*)mADCDatum0,10);       //��Flash�ж�ȡ
                        printf("mADCDatum0[%d]:%d c=%d ", 1, (int)mADCDatum0[1], c);
                    }
                    else if(c == 7)
                    {
                        mADCDatum0[2] = White_line_datum[c] + Menchmark_Coefficient;
                        // STMFLASH_Write(FLASH_SAVE_ADDR+12,(u16*)mADCDatum0,10);  //������Ļ�׼д��FLASH��
                        // STMFLASH_Read(FLASH_SAVE_ADDR+12,(u16*)mADCDatum0,10);       //��Flash�ж�ȡ
                        printf("mADCDatum0[%d]:%d c=%d ", 2, (int)mADCDatum0[2], c);
                    }
                    else if(c == 6)
                    {
                        mADCDatum0[3] = White_line_datum[c] + Menchmark_Coefficient;
                        // STMFLASH_Write(FLASH_SAVE_ADDR+12,(u16*)mADCDatum0,10);  //������Ļ�׼д��FLASH��
                        // STMFLASH_Read(FLASH_SAVE_ADDR+12,(u16*)mADCDatum0,10);       //��Flash�ж�ȡ
                        printf("mADCDatum0[%d]:%d c=%d ", 3, (int)mADCDatum0[3], c);
                    }
                    if(c > 9) printf("\r\n");
                    ++c;
                    jishi = 0;
                    delay_flag = 0;
                }

            }
            else if( c >= 10 && c < 15)
            {
                if(jishi == 20)
                {
                    bADCDatum0[c - 10] = White_line_datum[c] + Menchmark_Coefficient;
                    // STMFLASH_Write(FLASH_SAVE_ADDR+24,(u16*)bADCDatum0,10);  //������Ļ�׼д��FLASH��
                    // STMFLASH_Read(FLASH_SAVE_ADDR+24,(u16*)bADCDatum0,10);       //��Flash�ж�ȡ
                    usart3_send_head();
                    printf("bADCDatum0[%d]:%d c=%d ", c - 10, (int)bADCDatum0[c - 10], c);
                    ++c;
                    jishi = 0;
                    delay_flag = 0;
                    if(c > 14) UrxSt = NULL, printf("\r\n"), c = 0;
                }

            }


        }
        else if(UrxSt == 0xc6) //��׼���ݲ�ѯ�����ڴ������ֿ��������Ӧ���飬��������EEPROM�Զ����湦��
        {
            if(jishi == 20)
            {
                usart3_send_head();
                if(aa == 0)
                {
                    if(c == 0)/*STMFLASH_Read(FLASH_SAVE_ADDR,(u16*)fADCDatum0,10),*/printf("fADCDatum0[0-4]:%d ,", (int)fADCDatum0[c]);
                    else
                        printf("%d ,", (int)fADCDatum0[c]);
                    ++c;
                    if(c > 4) ++aa, c = 0, printf("\r\n");
                }
                else if(aa == 1)
                {
                    if(c == 0)    /*STMFLASH_Read(FLASH_SAVE_ADDR+12,(u16*)mADCDatum0,10),*/printf("mADCDatum0[0-3]:%d ,", (int)mADCDatum0[c]);
                    else
                        printf("%d ,", (int)mADCDatum0[c]);
                    ++c;
                    if(c > 3) ++aa, c = 0, printf("\r\n");
                }
                else if(aa == 2)
                {
                    if(c == 0)/*STMFLASH_Read(FLASH_SAVE_ADDR+24,(u16*)bADCDatum0,10), */printf("bADCDatum0[0-4]:%d ,", (int)bADCDatum0[c]);
                    else
                        printf("%d ,", (int)bADCDatum0[c]);
                    ++c;
                    if(c > 4) UrxSt = NULL, aa = 0, c = 0, printf("\r\n");
                }

                jishi = 0;
                delay_flag = 0;

            }

        }

    }
    else if(Mod == 2)  //��������׼���
    {

        switch( jishi)
        {
        case 20:
            usart3_send_head();//ǰ������ ����
            printf("\r\nfSensorRead[0-4]:%d%d%d%d%d\r\n", fSensorRead[0], fSensorRead[1], fSensorRead[2], fSensorRead[3], fSensorRead[4]);
            jishi = 0;
            break;
        case 40: //ǰ������ ����
            usart3_send_head();
            printf("mSensorRead[0-3]:%d%d%d%d\r\n", mSensorRead[0], mSensorRead[1], mSensorRead[2], mSensorRead[3]);
            jishi = 0;
            break;
        case 60:  //ǰ������  ����
            usart3_send_head();
            printf("bSensorRead[4-0]:%d%d%d%d%d\r\n", bSensorRead[4], bSensorRead[3], bSensorRead[2], bSensorRead[1], bSensorRead[0]);
            jishi = 0;
            delay_flag = 0;
            break;

        default:
            break;

        }


    }
    else if(Mod == 3) //���������ݲɼ�
    {

        usart3_send_head();//ǰ������ ����,�г���ͷ������Ϣ
        printf("\r\n��ͷ����[0-4]: %.0f %.0f %.0f %.0f %.0f ", After_filter[4], After_filter[3], After_filter[2], After_filter[1], After_filter[0]);
        SysTick_Delay_ms(150);

        usart3_send_head();
        printf("\r\n������ǰ���Һ�ǰ[5-9]:  %.0f %.0f %.0f %.0f ", After_filter[6], After_filter[7], After_filter[9], After_filter[5]);
        SysTick_Delay_ms(150);

        usart3_send_head();//ǰ������ ����,�г���ͷ������Ϣ
        printf("\r\n��β����[10-14]: %.0f %.0f %.0f %.0f %.0f  ", After_filter[10], After_filter[11], After_filter[12], After_filter[13], After_filter[14]);
        SysTick_Delay_ms(150);
    }
    else if(Mod == 4) //�г���Ϣ
    {
        DebugDrivingInformation(); //�г�������Ϣ
    }

    //��ص�ѹ����10.5Vʱ֪ͨ���ϵͳ��ʱ�ص����վ���
    if( Voltage < 1050 ) //������Ԥ������
    {
        if(jishi_50 == 1000) //10S Ԥ��һ��
        {
            usart3_send_head();
            printf("\r\n��ص����ͣ��뼰ʱ�ص����վ��磬����10V��ǿ��Ϩ��ͣ������ǰ��ѹ�� %.2f ", (float)Voltage / 100);

            jishi_50 = 0;
            delay_50 = 0;
        }

    }


}


/**************************************************************************
�������ܣ����Flash���Ƿ�洢ѭ����ػ�׼����
��ڲ�������
����  ֵ����Ź����ݷ���1�����򷵻�0
**************************************************************************/
u8 CheckFlashTrackData(void)
{
    STMFLASH_Read(FLASH_SAVE_ADDR, (u16 *)fADCDatum0, 10);
    STMFLASH_Read(FLASH_SAVE_ADDR + 12, (u16 *)mADCDatum0, 10);
    STMFLASH_Read(FLASH_SAVE_ADDR + 124, (u16 *)bADCDatum0, 10);
    if((fADCDatum0[1] != 0) && (fADCDatum0[3] != 0) && (mADCDatum0[1] != 0) && (mADCDatum0[2] != 0) && ( bADCDatum0[2] != 0) && (bADCDatum0[3] != 0))
    {
        return 1;
    }

    return 0;

}



/**************************************************************************
�������ܣ���λ����������������ɺ�����ÿλ��2��, ��λ,2^5, ÿλ3��, ��λ,3^5
         dfs(0); �������
��ڲ�������
����  ֵ����
**************************************************************************/
int a1[10];
void dfs(int d)
{
    int i;
    if(d == 5)
    {
        for( i = 0; i < d; i++) printf("%d ", a1[i]);
        puts("");
        return;
    }
    a1[d] = 0;
    dfs(d + 1);
    a1[d] = 1;
    dfs(d + 1);
}

/**************************************************************************
�������ܣ�ѭ�����ж�ȡ����������������Ļ̫�������Էֵ�ʱʹ��01
��ڲ�������
����  ֵ����
**************************************************************************/
void ReadSensor01(void)
{
    //if(CheckFlashTrackData())
    {
        // ��ͷ����  ����ǰǰ������
        fSensorRead[4] = bit_map(After_filter[0], fADCDatum0[0], 1);   //��2
        fSensorRead[3] = bit_map(After_filter[1], fADCDatum0[1], 1);    //��1
        fSensorRead[2] = bit_map(After_filter[2], fADCDatum0[2], 1);    //��
        fSensorRead[1] = bit_map(After_filter[3], fADCDatum0[3], 1);    //��1
        fSensorRead[0] = bit_map(After_filter[4], fADCDatum0[4], 1);    //��2

        //  fSensorRead[0] = bit_map2(After_filter[0] , fADCDatum0[0] , fADCDatum1[0]);  //��2
        //  fSensorRead[1] = bit_map2(After_filter[1] , fADCDatum0[1] , fADCDatum1[1]);  //��1
        //  fSensorRead[2] = bit_map2(After_filter[2] , fADCDatum0[2] , fADCDatum1[2]);  //��
        //  fSensorRead[3] = bit_map2(After_filter[3] , fADCDatum0[3] , fADCDatum1[3]);  //��1
        //  fSensorRead[4] = bit_map2(After_filter[4] , fADCDatum0[4] , fADCDatum1[4]);  //��2


        //��β���� �ӵ���ǰ������
        //  bSensorRead[0] = bit_map2(After_filter[10] , bADCDatum0[0] , bADCDatum1[0]); //��2
        //  bSensorRead[1] = bit_map2(After_filter[11] , bADCDatum0[1] , bADCDatum1[1]); //��1
        //  bSensorRead[2] = bit_map2(After_filter[12] , bADCDatum0[2] , bADCDatum1[2]); //��
        //  bSensorRead[3] = bit_map2(After_filter[13] , bADCDatum0[3] , bADCDatum1[3]); //��1
        //  bSensorRead[4] = bit_map2(After_filter[14] , bADCDatum0[4] , bADCDatum1[4]); //��2

        //��β���� �ӵ���ǰ������
        bSensorRead[4] = bit_map(After_filter[10], bADCDatum0[0], 1);   //��2
        bSensorRead[3] = bit_map(After_filter[11], bADCDatum0[1], 1);    //��1
        bSensorRead[2] = bit_map(After_filter[12], bADCDatum0[2], 1);    //��
        bSensorRead[1] = bit_map(After_filter[13], bADCDatum0[3], 1);    //��1
        bSensorRead[0] = bit_map(After_filter[14], bADCDatum0[4], 1);    //��2

        //���м䲿��, ����ǰǰ������
        mSensorRead[3] = bit_map(After_filter[5], mADCDatum0[0], 1 );    //ǰ��1
        mSensorRead[2] = bit_map(After_filter[9], mADCDatum0[1], 1);    //����1
        mSensorRead[1] = bit_map(After_filter[7], mADCDatum0[2], 1);    //����1
        mSensorRead[0] = bit_map(After_filter[6], mADCDatum0[3], 1);    //ǰ��1
    }

}

void ReadSensor02(void)
{

    // ��ͷ����  ����ǰǰ������
    fSensorRead[4] = bit_map_scope(After_filter[0], fADCDatum0[0], fADCDatum1[0], 1);  //��2
    fSensorRead[3] = bit_map_scope(After_filter[1], fADCDatum0[1], fADCDatum1[1], 1);   //��1
    fSensorRead[2] = bit_map_scope(After_filter[2], fADCDatum0[2], fADCDatum1[2], 1);   //��
    fSensorRead[1] = bit_map_scope(After_filter[3], fADCDatum0[3], fADCDatum1[3], 1);   //��1
    fSensorRead[0] = bit_map_scope(After_filter[4], fADCDatum0[4], fADCDatum1[4], 1);   //��2

    //��β���� �ӵ���ǰ������
    bSensorRead[4] = bit_map_scope(After_filter[10], bADCDatum0[0], bADCDatum1[0], 1);  //��2
    bSensorRead[3] = bit_map_scope(After_filter[11], bADCDatum0[1], bADCDatum1[1], 1);   //��1
    bSensorRead[2] = bit_map_scope(After_filter[12], bADCDatum0[2], bADCDatum1[2], 1);   //��
    bSensorRead[1] = bit_map_scope(After_filter[13], bADCDatum0[3], bADCDatum1[3], 1);   //��1
    bSensorRead[0] = bit_map_scope(After_filter[14], bADCDatum0[4], bADCDatum1[4], 1);   //��2

    //���м䲿��, ����ǰǰ������
    mSensorRead[3] = bit_map_scope(After_filter[5], mADCDatum0[0], mADCDatum1[0], 1);    //ǰ��1
    mSensorRead[2] = bit_map_scope(After_filter[9], mADCDatum0[1], mADCDatum1[1], 1);    //����1
    mSensorRead[1] = bit_map_scope(After_filter[7], mADCDatum0[2], mADCDatum1[2], 1);    //����1
    mSensorRead[0] = bit_map_scope(After_filter[6], mADCDatum0[3], mADCDatum1[3], 1);    //ǰ��1


}



/**************************************************************************
�������ܣ����ߴ���ͨ���߼����յ���λ������ָ����ش���λ������ָ��
Э���ʽ��Ĭ��ǰ׺��0x00,0x01,0x17 ����ָ�0xfe;ʾ����0x00,0x01,0x17,0xfe
�����豸����0xa- ǰ����ʻ0xf-  ������ʻ0xb-  ����0x1-  ���0x2-  ����0x3-  ת��0x4-  ���ݲɼ�0xc-

����Э�鲿��
1���豸����:0xaa �� �豸�رգ�0xab (Ĭ�Ϲرյ������Ҫ�豸����ָ����߹�Ϊ����ʼ���ָ��򿪵��)
2��ǰ����ʻ��0xf1
3���˶�ֹͣ��0xf0

С������У׼����(�������У׼�豸����ر�״̬����)
1����������ɼ�ģʽ��0xc1
2�������ɼ�ģʽ�£�0xc3���ɼ���ɫɫ�����ݣ�0xc4���ɼ���·���ݣ�0xc5����������׼���㣬0xc6����������׼��ѯ
3����������׼���ģʽ��0xc2
4��0xc7��������ԭʼ���ݲɼ���0xc8�����ڴ�ӡ�г���Ϣ

���վ����--�������վ����
1��ǰ�������վ��0xf2 --����ǰ��ģʽ
2�����������վ��0xb2 --���뵹��ģʽ
3�����վ��ֹͣ��0xab

��������--ң�ز�������
1��ǰ����ʻ��0xf1   --����ǰ��ģʽ
2���������⣺0x12
2��������⣺0x22   --���뵹��ģʽ
3���˶�ֹͣ��0xf0

��������--����ѭ��������
1���������⣺0x13   --���뵹��ģʽ
2��ǰ����ʻ��0xf1   --����ǰ��ģʽ
3��ǰ��խ·���ϣ�0x31
4������խ·���ϣ�0x32 --���뵹��ģʽ
5��ת��1��0x41
6��ת��2��0x42
7��������⣺0x22  --���뵹��ģʽ
8���˶�ֹͣ��0xf0

��������--���䲴������
1���������⣺0x14   --���뵹��ģʽ
2���˶�ֹͣ��0xf0
//3��ǰ����ʻ��0xf1    --����ǰ��ģʽ
4�����б���1��0x33
5������ת�䣨������㳵�⣩��0x43
6�����ͣ����0x23
7�����⸴λ��0x15
8�����⸴λת�䣺0x44
9�����⸴λͣ����0x22

��������--һ������
1�����㳵�⵽խ·���ϣ�0x34  --����ǰ��ģʽ

���ٳ���--NDA�����캽����ϵͳ
1��NDA�����캽����ϵͳ��0xf3  --����ǰ��ģʽ
2�����ٳ���������⣺0x22 --���뵹��ģʽ




����˵����
1��Flag_Stop������򿪺͹رգ�
2��Auto_Back_Flag��0��ǰ��ģʽ��1������ģʽ��
3��Flag_Turn��
            0��ǰ��ֱ�ߣ�
            1��ң�ز������⣬2:ң�ز�����⣬
            3������ѭ���������⣬4������ѭ��ǰ��խ·���ϣ� 5������ѭ������խ·���ϣ� 6������ѭ��ǰ��ת��1�� 7������ѭ��ǰ��ת��2�� 8������ѭ��������⣬
            9�����䲴���������⣬10�����䲴�����б���1��11�����䲴������ת�䣬12�����䲴���������ͣ����13�����䲴�����⸴λ
            14��һ��������㳵�⵽խ·���� ��15�������վ �� 16�������վ �� 17�����ٳ���



���ʣ�
������������
������1--ң�ز�����ң�ز�����������=2/1.5  ң�ز����������=
������2--����ѭ��������ѭ��������������=2   ����ѭ��խ·ǰ������=   ����ѭ��խ·��������=   ����ѭ��ת��1����=   ����ѭ��ת��2����=   ����ѭ�������������=
������3--���䲴�������䲴����������=     ���䲴�����б�������=    ���䲴������ת������=   ���䲴�������������=    ���䲴��ǰ����������=
���ٳ�������
���г�������

**************************************************************************/


void Track_Rx_Logic(void)  //����ͨѶ�߼����ڴ��ļ����������ʹ��
{
    switch( UrxSt )
    {
    //У׼��������ص������г�ģʽ
    case 0xc0:
        iUartCommand = UrxSt;
        Mod = 0;
        Flag_Stop = 0;//��Ǵ򿪵��
        break;
    //���봫�������ݲɼ�У׼ģʽ
    case 0xc1:
        iUartCommand = UrxSt;
        Mod = 1;
        Flag_Stop = 1; //��ǹرյ��
        break;
    //��������׼���ģʽ
    case 0xc2:
        iUartCommand = UrxSt;
        Mod = 2;
        // Flag_Stop = 1; //��ǹرյ��
        break;
    //���������ݲɼ�
    case 0xc7:
        iUartCommand = UrxSt;
        Mod = 3;
        Flag_Stop = 1; //��ǹرյ��
        break;
    //�г���Ϣ���ڴ�ӡ
    case 0xc8:
        iUartCommand = UrxSt;
        Mod = 4;
        break;

    //�豸�������򿪵��
    case 0xaa:
        iUartCommand = UrxSt;
        Flag_Stop = 0;//��Ǵ򿪵��
        break;
    //Ϩ��ͣ���豸�رգ��رյ��
    case 0xab:
        iUartCommand = UrxSt;
        Flag_Stop = 1; //��ǹرյ��
        Flag_Turn = -1;
        break;


    //�˶�ֹͣ
    case 0xf0:
        iUartCommand = UrxSt;
        Move_Y = 0;
        Move_Z = 0;
        Flag_Turn = -1;
        break;
    //ǰ����ʻ
    case 0xf1:
        iUartCommand = UrxSt;
        Auto_Back_Flag = 0;
        Flag_Turn = 0;
        break;

    //ң�ز�������
    case 0x12:
        iUartCommand = UrxSt;
        Auto_Back_Flag = 0;
        Flag_Turn = 1;
        Flag_Stop = 0;//��Ǵ򿪵��
        break;
    //ң�ز������
    case 0x22:
        iUartCommand = UrxSt;
        Auto_Back_Flag = 1;
        Flag_Turn = 2;
        break;

    //����ѭ����������
    case 0x13:
        iUartCommand = UrxSt;
        Auto_Back_Flag = 1;
        Flag_Turn = 3;
        Flag_Stop = 0;//��Ǵ򿪵��
        break;
    //����ѭ��ǰ��խ·����
    case 0x31:
        iUartCommand = UrxSt;
        Auto_Back_Flag = 0;
        Flag_Turn = 4;
        break;
    //����ѭ������խ·����
    case 0x32:
        iUartCommand = UrxSt;
        Auto_Back_Flag = 1;
        Flag_Turn = 5;
        break;
    //����ѭ��ת��1
    case 0x41:
        iUartCommand = UrxSt;
        Auto_Back_Flag = 0;
        Flag_Turn = 6;
        break;
    //����ѭ��ת��2
    case 0x42:
        iUartCommand = UrxSt;
        Auto_Back_Flag = 0;
        Flag_Turn = 7;
        break;
    //����ѭ���������
    case 0x23:
        iUartCommand = UrxSt;
        Auto_Back_Flag = 1;
        Flag_Turn = 8;
        break;

    //���䲴����������
    case 0x14:
        iUartCommand = UrxSt;
        Auto_Back_Flag = 1;
        Flag_Turn = 9;
        Flag_Stop = 0;//��Ǵ򿪵��
        break;
    //���䲴�����б���1
    case 0x33:
        iUartCommand = UrxSt;
        Auto_Back_Flag = 0;
        Flag_Turn = 10;
        break;
    //���䲴������ת��
    case 0x43:
        iUartCommand = UrxSt;
        Auto_Back_Flag = 0;
        Flag_Turn = 11;
        break;
    //���䲴���������ͣ��
    case 0x24:
        iUartCommand = UrxSt;
        Auto_Back_Flag = 1;
        Flag_Turn = 12;
        break;
    //���䲴�����⸴λ
    case 0x15:
        iUartCommand = UrxSt;
        Auto_Back_Flag = 0;
        Flag_Turn = 13;
				Speed = 915;
        break;
    //һ��������㳵�⵽խ·����
    case 0x34:
        iUartCommand = UrxSt;
        Auto_Back_Flag = 0;
        Flag_Turn = 14;
        Flag_Stop = 0;//��Ǵ򿪵��
        break;
		
    //ǰ�������վ
    case 0xf2:
        iUartCommand = UrxSt;
        Auto_Back_Flag = 0;
        Flag_Turn = 15;
        Flag_Stop = 0;//��Ǵ򿪵��
        break;
    case 0xb2:
        //���������վ
        iUartCommand = UrxSt;
        Auto_Back_Flag = 1;
        Flag_Turn = 16;
        break;
		
		//���ٳ���
    case 0xf3:
        iUartCommand = UrxSt;
        Auto_Back_Flag = 0;
        Flag_Turn = 17;
        Flag_Stop = 0;//��Ǵ򿪵��
        break;

    default:

        break;
    }


}


/**************************************************************************
�������ܣ�Ѳ���ٶ��޷�����
��ڲ�������
����  ֵ����
**************************************************************************/
void Xianfu_Track_Speed(int amplitude)
{
    //    if(Speed < -amplitude) Speed = -amplitude;
    //    if(Speed > amplitude)  Speed = amplitude;
    //    if(error < -amplitude) error = -amplitude;
    //    if(error > amplitude)  error = amplitude;

    if(Move_Y < -amplitude) Move_Y = -amplitude;
    if(Move_Y > amplitude)  Move_Y = amplitude;
    if(Move_Z < -amplitude) Move_Z = -amplitude;
    if(Move_Z > amplitude)  Move_Z = amplitude;

}


/**************************************************************************
�������ܣ��Ӽ����߼�����
��ڲ�������
����  ֵ����
**************************************************************************/

float V1_Step = 10, V2_Step = 50, V3_Step = 300 ;
void U_Speed_Response(void)  //���S��ֱ�߼����߼�
{

    if (Speed >= 800)
    {
        Speed = Speed + V1_Step;
    }
    else if (Speed >= 600 && Speed <= 800)
    {
        Speed = Speed + V2_Step;
    }
    else if (Speed >= 0 && Speed <= 600 )
    {
        Speed = Speed + V3_Step;
    }

    if(Speed > 950)
    {
        Speed = 950;  //950;
    }
}

void C_Speed_Response(void)  //���S��ֱ�߼����߼�
{
    if (Speed >= 800 && Speed <= 900)
    {
        Speed = Speed - V1_Step;
    }
    else if ((Speed >= 0 && Speed <= 300) || (Speed >= 600 && Speed <= 800))
    {
        Speed = Speed - V2_Step;
    }
    else if (Speed >= 300 && Speed <= 600)
    {
        Speed = Speed - V3_Step;
    }
    else if (Speed > 950)
    {
        Speed = 920;
    }
    if (Speed <= 0 )
    {
        Speed = 0;
    }

}


//�����ƣ���Ҫ���ǵ�С���Ӽ��ٴ���ƫ�Ƴ��ƶ���ɫɫ��ʱ״̬���Ҳ�������Ч�Ⲩ����������Ӱ��
void Acc_And_Dec_Logic(void)
{
    if (Auto_Back_Flag == 0)  //ǰ��
    {

        //                  ǰ��1                   ����1                 ����1                   ǰ��1
        mSensorReadSum = (mSensorRead[0] << 3) + (mSensorRead[1] << 2) + (mSensorRead[2] << 1) + (mSensorRead[3] << 0) ;
        //                   ��2                    ��1                   ��                       ��1                 ��2
        bSensorReadSum = (bSensorRead[0] << 0) + (bSensorRead[1] << 1) + (bSensorRead[2] << 2) + (bSensorRead[3] << 3) + (bSensorRead[4] << 4) ;

        if((bSensorRead[0] && bSensorRead[1] && bSensorRead[2] && bSensorRead[3] && bSensorRead[4]) == 0 )  //ǰ��״̬����β��������һ��ʶ�𵽽��м���
        {

#if( CAR_NUM == 1 )
            switch(Flag_Turn)  
            {

            case 9: //���䲴����������
            case 10://���䲴�����б���1 
            case 11://���䲴������ת��
                Speed = 920;
                break;
            case 12://���䲴���������ͣ��
                Speed = 850;
                break;

            default:
                C_Speed_Response();
								if (Speed <= 0 && Flag_Turn == -1) Speed = 0;
                else Speed = 900; //�����ٵ�0��ʱ���ѯ���Ƿ�����ʻ״̬���ǵĻ������Ǹ��ŵ��µģ������ٶȼ�����ʻ
                break;
            }

#elif( CAR_NUM == 2 )
            switch(Flag_Turn)  
            {
													
            case 9://���䲴����������
            case 10://���䲴�����б���1
            case 11://���䲴������ת��
               // Speed = 930;// 950;//920;
                break;
            case 12://���䲴���������ͣ��
                Speed = 900;//850;
                break;
						
						case 17:  //���ٳ���������
								//Speed = 950; 
								break;
						
						case 6:
						case 7:
							 break;

            default:
//                C_Speed_Response();
//                if (Speed <= 0 && Flag_Turn == -1) Speed = 0;
//                else Speed = 900; //�����ٵ�0��ʱ���ѯ���Ƿ�����ʻ״̬���ǵĻ������Ǹ��ŵ��µģ������ٶȼ�����ʻ
                break;
            }
#endif


        }
       // else if( (bSensorRead[2] == 1) && ((mSensorRead[0] && mSensorRead[1] && mSensorRead[2] && mSensorRead[3]) == 0) ) //ǰ��״̬���м�Ӽ�������һ��ʶ�𵽽��м���
           else if(  ((mSensorRead[0] && mSensorRead[1] && mSensorRead[2] && mSensorRead[3]) == 0) ) //ǰ��״̬���м�Ӽ�������һ��ʶ�𵽽��м���
        {

#if( CAR_NUM == 1 )
            switch(Flag_Turn)  
            {
            case 9://���䲴����������
            case 10://���䲴�����б���1
            case 11://���䲴������ת��
                Speed = 950;//920;
                break;
            case 12://���䲴���������ͣ��
                Speed = 850;
                break;
						case 17:  //���ٳ������ٸ���Щ
						    Speed = 920;
								break;
												
            default:
                U_Speed_Response();
                if(Speed >= 950) Speed = 930;
                break;
            }
#elif( CAR_NUM == 2 )

            switch(Flag_Turn)  
            {
            case 9://���䲴����������
            case 10://���䲴�����б���1
            case 11://���䲴������ת��

//							if(mSensorRead[1]==0 &&  mSensorRead[2] == 0 )
//                Speed = 950;//950;//920;
//							else 
//								Speed = 930;
							
							//else if((mSensorRead[1]&&mSensorRead[2] ) == 0)
                break;
            case 12://���䲴���������ͣ��
                Speed = 900;//850;
                break;
						
						case 17:  //���ٳ������ٸ���Щ
								//U_Speed_Response(); 
//								if( ((mSensorRead[0] && mSensorRead[3]) == 0)) Speed = 940;  //�м����ǰ������ʶ�𵽸�������ٶ�
//								else Speed = 910;
						    Speed = 920;
						     //if(Speed >= 900) Speed = 920;
								break;
						
            default:
                U_Speed_Response();
               // if(Speed >= 950) Speed = 930;
                break;
            }
#endif

        }

    }

    else if (Auto_Back_Flag == 1) //����
    {
        //                   ��2                    ��1                   ��                       ��1                 ��2
        fSensorReadSum = (fSensorRead[0] << 0) + (fSensorRead[1] << 1) + (fSensorRead[2] << 2) + (fSensorRead[3] << 3) + (fSensorRead[4] << 4) ;
        //                  ǰ��1                   ����1                 ����1                   ǰ��1
        mSensorReadSum = (mSensorRead[2] << 3) + (mSensorRead[3] << 2) + (mSensorRead[0] << 1) + (mSensorRead[1] << 0) ;

        if((fSensorRead[0] && fSensorRead[1] && fSensorRead[2] && fSensorRead[3] && fSensorRead[4]) == 0 )  //����״̬����ͷ��������һ��ʶ�𵽽��м���
        {


#if( CAR_NUM == 1 )
            switch(Flag_Turn)  
            {

            case 9://���䲴����������
                Speed = 930;
                break;
            case 12://���䲴���������ͣ��
                Speed = 900;
                break;
            default:
                C_Speed_Response();
                //if(Speed >= 850) Speed=820;
                break;
            }

#elif( CAR_NUM == 2 )
            switch(Flag_Turn)  
            {

            case 5: //����ѭ����խ·���ϵ�����ʻ
                C_Speed_Response();
                break;
						
						case 6:
						case 7:
							 break;

            case 9://���䲴����������  
                //Speed = 900;
                break;
            case 12://���䲴���������ͣ��
               // Speed = 850;
                break;

            default:
                C_Speed_Response();
                if(Speed >= 900) Speed = 900;
                break;
            }

#endif

        }
        else if( (fSensorRead[2] == 1) && ((mSensorRead[0] && mSensorRead[1] && mSensorRead[2] && mSensorRead[3]) == 0) ) //����״̬���м�Ӽ�������һ��ʶ�𵽽��м���
            // else if( ((mSensorRead[0] && mSensorRead[1] && mSensorRead[2] && mSensorRead[3]) == 0) ) //����״̬���м�Ӽ�������һ��ʶ�𵽽��м���

        {

#if( CAR_NUM == 1 )
            switch(Flag_Turn)  
            {

            case 9://���䲴����������
                Speed = 930;
                break;
            case 12://���䲴���������ͣ��
                Speed = 900;
                break;
            default:
                U_Speed_Response();
                //if(Speed >= 900) Speed=820;
                break;
            }

#elif( CAR_NUM == 2 )
            switch(Flag_Turn)  
            {
							
            case 5: //����ѭ����խ·���ϵ�����ʻ��ֱ�߽϶࣬���ٶȿ�Щ
                U_Speed_Response();
                break;

            case 9://���䲴����������
              //  Speed = 900;
                break;
            case 12://���䲴���������ͣ��
               // Speed = 850;
                break;


            default:
                U_Speed_Response();
                //if(Speed >= 900) Speed = 900;
                break;
            }
#endif

        }
    }

}



/**************************************************************************
�������ܣ�Ѳ���߼�����
��ڲ�������
����  ֵ����
**************************************************************************/
//============================================PID�㷨

/*
С����ֱ�����Ҷ���Ƶ��������Kp��ֵ�����Kdֵ
С���������ʱ��ת����ȥ������Kp��ֵ������Kiֵ
С�����������ֱ��ʱ������ߣ�����Kd��ֵ�����Kiֵ

��Ҫ��Ӧ���죬����P��СI��
��Ҫ��Ӧ��������СP����I��
�������̫�󣬻�����ϵͳ�𵴣�
�������̫�󣬻�����ϵͳ�ٶ�

KP���ã�ת��Ƕȣ�ֵԽ��ת��Ƕ�Խ��
KI���ã��ۻ�ת��������
KD���ã�ת������,kdԽ������Խ����£���ã���������
*/



#define F1_MAX 30
#define F2_MAX 25
//float fGear0 = 0, fGear1 = 8, fGear2 = 10, fGear3 = 15, fGear4 = 20; //ǰ����λ����
float fGear0 = 0, fGear1 = 2, fGear2 = 4, fGear3 = 8, fGear4 = 16; //ǰ����λ����

/*
//�������ȶ���ȫ�̲���
//ǰ����ʻPID����
//float Kp0 = 2, Ki0 = 0, Kd0 = 2.1;  //ֱ����ʻ  0xf1
float Kp0 = 70, Ki0 = 0, Kd0 = 2;  //ֱ����ʻ  0xf1 ����������ȫ�̸���
float Kp1 = 15, Ki1 = 0, Kd1 = 10.5; //ң�ز�������  0x12
float Kp4 = 15, Ki4 = 0, Kd4 = 8;  //����ѭ��խ·���� 0x31
float Kp6 = 35, Ki6 = 0, Kd6 = 10;  //����ѭ��ǰ��ת��1 0x41
float Kp7 = 35, Ki7 = 0, Kd7 = 10;  //����ѭ��ǰ��ת��2 0x42
float Kp10 = 50, Ki10 = 0, Kd10 = 10; //10; //���䲴�����б���1 0x33
float Kp11 = 50, Ki11 = 0, Kd11 = 10; //10;  //���䲴������ת��  0x43
float Kp13 = 30, Ki13 = 0, Kd13 = 20; //���䲴�����⸴λ  0x44
float Kp14 = 30, Ki14 = 0, Kd14 = 20; //һ��������㳵�⵽խ·����  0x34
float Kp15 = 30, Ki15 = 0, Kd15 = 20; //ǰ�������վ  0xf2


//������ʻPID����
float Kp2 = 30, Ki2 = 0, Kd2 = 10.5;
*/

//ǰ����ʻPID����
float Kp0 = 70, Ki0 = 0, Kd0 = 2;  //ֱ����ʻ  0xf1 ����������ȫ�̸���
float Kp1 = 60, Ki1 = 0, Kd1 = 10; //ң�ز�������  0x12
//float Kp4 = 60, Ki4 = 0, Kd4 = 10;//2;  //����ѭ��խ·���� 0x31
float Kp4 = 60, Ki4 = 0, Kd4 = 10;//2;  //����ѭ��խ·���� 0x31
float Kp6 = 60, Ki6 = 0, Kd6 = 10;//2;  //����ѭ��ǰ��ת��1 0x41
float Kp7 = 60, Ki7 = 0, Kd7 = 10;//2;  //����ѭ��ǰ��ת��2 0x42
float Kp10 = 60, Ki10 = 0, Kd10 = 2; //10; //���䲴�����б���1 0x33
float Kp11 = 60, Ki11 = 0, Kd11 = 2; //10;  //���䲴������ת��  0x43
float Kp13 = 60, Ki13 = 0, Kd13 = 2; //���䲴�����⸴λ  0x44
float Kp14 = 60, Ki14 = 0, Kd14 = 2; //һ��������㳵�⵽խ·����  0x34
float Kp15 = 60, Ki15 = 0, Kd15 = 2; //ǰ�������վ  0xf2
float Kp17 = 60, Ki17 = 0, Kd17 = 2; //���ٳ���  0xf3


//������ʻPID����
//float Kp2 = 10, Ki2 = 0, Kd2 = 2;  //������
//float Kp2 = 15, Ki2 = 0, Kd2 = 2;
float Kp2 = 15, Ki2 = 0, Kd2 = 2;


float  fP = 0, fI = 0, fD = 0, PID_value = 0;  //pidֱ������
float previous_error = 0, previous_I = 0;
float xf = 300;   //500;//300;//250;  //һ���������ٶ�1/3����Ϊת�����
float offset = 3;

float middle_offset = -6;//-2;
float fI_offset = 5;


void calc_pid()
{
    fP = error;
    fI = fI + error;



    if((error - previous_error) > 0)
    {
        fD = error - previous_error;
        //  if(fSensorReadSum == 27) fD = fD - middle_offset;
    }
    if((error - previous_error) < 0)
    {
        fD = error - previous_error;
        if(fSensorReadSum == 27) fD = fD - middle_offset;
    }
    if(fSensorReadSum == 31) fD = 0;


    if (Auto_Back_Flag == 0)
    {
        if(fI > F1_MAX) fI = 10;
        switch(Flag_Turn)
        {
        case 0:  //ǰ��ֱ��
            PID_value = (Kp0 * fP) + (Ki0 * fI) + (Kd0 * fD);
            break;
        case 1:  //ң�ز�������
            PID_value = (Kp1 * fP) + (Ki1 * fI) + (Kd1 * fD);
            break;
        case 4:  //����ѭ��խ·����
            PID_value = (Kp4 * fP) + (Ki4 * fI) + (Kd4 * fD);
            break;
        case 6:  //����ѭ��ǰ��ת��1
            PID_value = (Kp6 * fP) + (Ki6 * fI) + (Kd6 * fD);
            break;
        case 7:  //����ѭ��ǰ��ת��2
            PID_value = (Kp7 * fP) + (Ki7 * fI) + (Kd7 * fD);
            break;
        case 10:  //���䲴�����б���1
            PID_value = (Kp10 * fP) + (Ki10 * fI) + (Kd10 * fD);
            break;
        case 11:  //���䲴������ת��
            PID_value = (Kp11 * fP) + (Ki11 * fI) + (Kd11 * fD);
            break;
        case 13:  //���䲴�����⸴λ
            PID_value = (Kp13 * fP) + (Ki13 * fI) + (Kd13 * fD);
            break;
        case 14:  //һ��������㳵�⵽խ·����
            PID_value = (Kp14 * fP) + (Ki14 * fI) + (Kd14 * fD);
        case 15:  //ǰ�������վ
            PID_value = (Kp15 * fP) + (Ki15 * fI) + (Kd15 * fD);
            break;
				case 17:  //���ٳ���
            PID_value = (Kp17 * fP) + (Ki17 * fI) + (Kd17 * fD);
            break;

        }

    }
    else
    {
        if(fI > F2_MAX) fI = 15;
        PID_value = (Kp2 * fP) + (Ki2 * fI) + (Kd2 * fD);
    }


    pid_d = fD, pid_i = fI;



    if(PID_value < -xf) PID_value = -xf;
    if(PID_value > xf)  PID_value = xf;

    previous_error = error;
}

int fStopSensor , bStopSensor , Last_Flag_Turn;

void TrackContor1(void)
{
	
		

    if (Auto_Back_Flag == 0)
    {

        //                   ��2                    ��1                   ��                       ��1                 ��2
        fSensorReadSum = (fSensorRead[0] << 4) + (fSensorRead[1] << 3) + (fSensorRead[2] << 2) + (fSensorRead[3] << 1) + (fSensorRead[4] << 0) ;
        //                  ǰ��1                   ����1                 ����1                   ǰ��1
        mSensorReadSum = (mSensorRead[0] << 3) + (mSensorRead[1] << 2) + (mSensorRead[2] << 1) + (mSensorRead[3] << 0) ;
        //                   ��2                    ��1                   ��                       ��1                 ��2
        bSensorReadSum = (bSensorRead[0] << 0) + (bSensorRead[1] << 1) + (bSensorRead[2] << 2) + (bSensorRead[3] << 3) + (bSensorRead[4] << 4) ;
			
				fStopSensor = bSensorRead[0] + bSensorRead[1] + bSensorRead[2] + bSensorRead[3] + bSensorRead[4];

        //============================================���¼Ӽ��ٴ����߼�=====================================
			
			 // if( fSensorReadSum != 31 || fSensorReadSum != 0 )
         Acc_And_Dec_Logic();
				
			      switch(Flag_Turn)  
            {
            case 10://���䲴�����б���1
            case 11://���䲴������ת��

						if(mSensorRead[1]==0 &&  mSensorRead[2] == 0 )
							Speed = 930;//950;//950;//920;
						else if(mSensorRead[0]==0 ||  mSensorRead[3] == 0 )
							Speed = 950;//960;
						else 
							Speed = 915;
							
                break;
						case -1:
							break;
						
            default:
//								if((bSensorRead[0] && bSensorRead[1] && bSensorRead[2] && bSensorRead[3] && bSensorRead[4]) == 0 )  //ǰ��״̬����β��������һ��ʶ�𵽽��м���
//									C_Speed_Response();
//								else if(  ((mSensorRead[0] && mSensorRead[1] && mSensorRead[2] && mSensorRead[3]) == 0) ) //ǰ��״̬���м�Ӽ�������һ��ʶ�𵽽��м���
//									 U_Speed_Response();
//								else
//								  Speed = 915;
                break;
            }


        //============================================��������ѭ�������߼�=====================================

        switch(fSensorReadSum)
        {

        case B10011:
            error = -fGear1;
            break;
        case B10111:
            error = -fGear2;
            break;
        case B00111:
            error = -fGear3;
            break;
        case B01111:
            error = -fGear4;
            break;

        case B11011:
            error = fGear0;
            break;

        case B11001:
            error = fGear1;
            break;
        case B11101:
            error = fGear2;
            break;
        case B11100:
            error = fGear3;
            break;
        case B11110:
            error = fGear4;
            break;


        // case B00000:
        //     break;
        // case B00001:
        //     break;
        // case B00010:
        //     break;
        // case B00011:
        //     break;
        // case B00100:
        //     break;
        // case B00101:
        //     break;
        // case B00110:
        //     break;
        // case B01000:
        //     break;
        // case B01001:
        //     break;
        // case B01010:
        //     break;
        // case B01011:
        //     break;
        // case B01100:
        //     break;
        // case B01101:
        //     break;
        // case B01110:
        //     break;
        // case B10000:
        //     break;
        // case B10010:
        //     break;
        // case B10100:
        //     break;
        // case B10101:
        //     break;
        // case B10110:
        //     break;
        // case B10001:
        //   break;
        // case B11000:
        //     break;
        // case B11010:
        //     break;
        //case B11111:
        //break;
        default:
            break;
        }

        calc_pid();
        Move_Y = Speed;
        Move_Z = PID_value;
				
				
				


        if(fSensorReadSum == B11111 || fSensorReadSum == B00000 || iUartCommand == 0xf0 )  //��ȫ�׻���ȫ��ʱ��Ϊ������״̬
        {
            ++TimeFlag;
            if (TimeFlag >= BLACK_COLOR_DELAY)//30*50=1500ms��ʱֹͣ
            {
                TimeFlag = 0;
                F_Not_Find_White_STOP = 1;      // ����ʱ��δ���ְ��߱�־λ����1
            }

        }
        else
        {
            TimeFlag = 0; // �����δ���ְ��߼�ʱ��,��ʱ�ڶ�ʱ���жϺ�����
            F_Not_Find_White_STOP = 0;  // �������ʱ��δ���ְ��߱�־λ��
        }
				
				
				if(fStopSensor < 2 && iUartCommand == 0xf0)   F_Not_Find_White_STOP = 1; 



        if(F_Not_Find_White_STOP == 1) //ʹ�ô��߼��ж����Ӵ�������Ϊ�ֽ����ܹ�Ӱ��Ŀ�������
         //     if(fSensorReadSum == B11111 || iUartCommand == 0xf0)
        {

            Move_Y = 0;
            Move_Z = 0;
            Speed  = 0;
            error = 0;
            PID_value = 0;
            fP = 0, fI = 0, fD = 0;


        }
        else
        {
            // Speed = 950;//300;
            Move_Y = Move_Y;
						//Move_Y = 915;
            if(Move_Y == 0) Move_Z = 0;
            else Move_Z = Move_Z;
        }



    }
    else if (Auto_Back_Flag == 1)
    {
        AutoBackTrack();
    }





}







void AutoBackTrack(void)
{

    //                   ��2                    ��1                   ��                       ��1                 ��2
    fSensorReadSum = (fSensorRead[0] << 0) + (fSensorRead[1] << 1) + (fSensorRead[2] << 2) + (fSensorRead[3] << 3) + (fSensorRead[4] << 4) ;
    //                  ǰ��1                   ����1                 ����1                   ǰ��1
    mSensorReadSum = (mSensorRead[2] << 3) + (mSensorRead[3] << 2) + (mSensorRead[0] << 1) + (mSensorRead[1] << 0) ;
    //                   ��2                    ��1                   ��                       ��1                 ��2
    bSensorReadSum = (bSensorRead[0] << 4) + (bSensorRead[1] << 3) + (bSensorRead[2] << 2) + (bSensorRead[3] << 1) + (bSensorRead[4] << 0) ;
	
	 bStopSensor = fSensorRead[0] + fSensorRead[1] + fSensorRead[2] + fSensorRead[3] + fSensorRead[4];

    //============================================���¼Ӽ��ٴ����߼�=====================================
	 //if( bSensorReadSum != 31 || bSensorReadSum != 0 )
    Acc_And_Dec_Logic();
	

	//	u16 bADCDatum0[5] = {1943 ,2297 ,2112 ,2031 ,1600};
		switch(Flag_Turn)  
		{
			
		case 9://���䲴����������
				if(mSensorRead[0]==0 ||  mSensorRead[3] == 0 )
					Speed = 950;//950;//920;
				else if(mSensorRead[1]==0 ||  mSensorRead[2] == 0 )
					Speed = 960;
				else 
					Speed = 800;
			//  Speed = 900;
				
				//bADCDatum0[0] = 1600;
				break;
		case 12://���䲴���������ͣ��
				if(mSensorRead[0]==0 &&  mSensorRead[3] == 0 )
					Speed = 900;//950;//920;
				else if(mSensorRead[1]==0 ||  mSensorRead[2] == 0 )
					Speed = 920;
				else 
					Speed = 850;
			 // Speed = 850;
				
				//bADCDatum0[4] = 1000;
				break;
		case -1:
				break;
		
		//9�����䲴���������⣬10�����䲴�����б���1��11�����䲴������ת�䣬12�����䲴���������ͣ����13�����䲴�����⸴λ
		
//		case 2:
//			bADCDatum0[0] = 1700;
//		  bADCDatum0[4] =  1000;
//		break;

		default:			
//			bADCDatum0[0] = 1943;
//		  bADCDatum0[4] =  1600;
////				if((fSensorRead[0] && fSensorRead[1] && fSensorRead[2] && fSensorRead[3] && fSensorRead[4]) == 0 )  //����״̬����ͷ��������һ��ʶ�𵽽��м���
//					C_Speed_Response();
//				else if(  ((mSensorRead[0] && mSensorRead[1] && mSensorRead[2] && mSensorRead[3]) == 0) ) //����״̬���м�Ӽ�������һ��ʶ�𵽽��м���
//					 U_Speed_Response();
//				else
//					Speed = 915;
				break;
		}


    //============================================��������ѭ�������߼�=====================================

    switch(bSensorReadSum)
    {

    case B10011:
        error = -fGear1;
        break;
    case B10111:
        error = -fGear2;
        break;
    case B00111:
        error = -fGear3;
        break;
    case B01111:
        error = -fGear4;
        break;

    case B11011:
        error = fGear0;
        break;

    case B11001:
        error = fGear1;
        break;
    case B11101:
        error = fGear2;
        break;
    case B11100:
        error = fGear3;
        break;
    case B11110:
        error = fGear4;
        break;


    // case B00000:
    //     break;
    // case B00001:
    //     break;
    // case B00010:
    //     break;
    // case B00011:
    //     break;
    // case B00100:
    //     break;
    // case B00101:
    //     break;
    // case B00110:
    //     break;
    // case B01000:
    //     break;
    // case B01001:
    //     break;
    // case B01010:
    //     break;
    // case B01011:
    //     break;
    // case B01100:
    //     break;
    // case B01101:
    //     break;
    // case B01110:
    //     break;
    // case B10000:
    //     break;
    // case B10010:
    //     break;
    // case B10100:
    //     break;
    // case B10101:
    //     break;
    // case B10110:
    //     break;
    // case B10001:
    //   break;
    // case B11000:
    //     break;
    // case B11010:
    //     break;
    //case B11111:
    //break;
    default:
        break;
    }

    calc_pid();
    Move_Y = -Speed;
    Move_Z = PID_value;

    if(bSensorReadSum == B11111 || bSensorReadSum == B00000 || iUartCommand == 0xf0) //��ȫ�׻���ȫ��ʱ��Ϊ������״̬
    {
        TimeFlag++;
        if (TimeFlag >= BLACK_COLOR_DELAY)//30*50=1500ms��ʱֹͣ
        {
            TimeFlag = 0;
            F_Not_Find_White_STOP = 1;      // ����ʱ��δ���ְ��߱�־λ����1
        }
    }
    else
    {
        TimeFlag = 0; // �����δ���ְ��߼�ʱ��,��ʱ�ڶ�ʱ���жϺ�����
        F_Not_Find_White_STOP = 0;  // �������ʱ��δ���ְ��߱�־λ��
    }

		
		
		if(bStopSensor < 2 && iUartCommand == 0xf0)   F_Not_Find_White_STOP = 1; 
		
	
    
    if(F_Not_Find_White_STOP == 1 || (Last_Flag_Turn==2 && Flag_Turn == -1))  //ʹ�ô��߼��ж����Ӵ�������Ϊ�ֽ����ܹ�Ӱ��Ŀ�������
    //    if(bSensorReadSum == B11111 || iUartCommand == 0xf0)
    {
        Move_Y = 0;
        Move_Z = 0;
        Speed  = 0;
        error = 0;
        PID_value = 0;
        fP = 0, fI = 0, fD = 0;
    }

    else
    {
        // Speed = -950;
        Move_Y = Move_Y;
		//	Move_Y = -915;
        if(Move_Y == 0) Move_Z = 0;
        else Move_Z = Move_Z;

    }


		Last_Flag_Turn = Flag_Turn;
}


























