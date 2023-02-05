/**************************************************************************
文件说明：此文件为关于实现循迹逻辑函数
入口参数：无
返回  值：无
**************************************************************************/
#include "TrackLogic.h"
#include "adc.h"

/**************************************************************************
                      变量声明部分
**************************************************************************/
#define FLASH_SAVE_ADDR  0X0800E000     //设置FLASH 保存地址(必须为偶数，且其值要大于本代码所占用FLASH的大小+0X08000000)
#define BLACK_COLOR_DELAY 30  //未在线状态延时停止 BLACK_COLOR_DELAY*50 ms
#define TRACK_INIT_SPEED  100   //巡线初始速度


//采集白线值
//int fADCDatum0[5] = {1463, 1902, 1744, 1627, 1558}; //采集车头传感器色块边缘值作为基准，从右到左，从前进方向看
//int bADCDatum0[5] = {1596, 1631, 1691, 1321, 1662}; //采集车尾传感器色块边缘值作为基准，从右到左，从前进方向看
//int mADCDatum0[4] = {1618, 1632, 1862, 1827}; //采集中间传感器色块边缘值作为基准
/*
//下午采集数据，/2
ADCDatum0[0-4]:2105 ,2529 ,2341 ,2174 ,2172 ,
mADCDatum0[0-3]:1828 ,1880 ,2148 ,2168 ,
bADCDatum0[0-4]:1668 ,1834 ,2031 ,1494 ,2003 ,


*/

// 循迹黑线
//1277,1580,1968,1337,1322

//u16 fADCDatum0[5] = { 2000, 2000, 2000, 2000, 2000}; //采集车头传感器色块边缘值作为基准，从右到左，从前进方向看
//u16 mADCDatum0[4] = {2000, 2000, 2000, 2000}; //采集中间传感器色块边缘值作为基准
////u16 bADCDatum0[5] = {1200,1000,1200,1200,1000};
//u16 bADCDatum0[5] = {1200,1200,1300,1200,1100};
//现场白天采集
//u16 bADCDatum0[5] = {1500, 1500, 1600, 1600, 1500};
//u16 bADCDatum0[5] = {1183, 1725, 1633, 1475, 1355}; //采集车尾传感器色块边缘值作为基准，从右到左，从前进方向看

//适配白天传感基准  05/26采集  /2.2
//int fADCDatum0[5] = {2105 ,2529 ,2341 ,2174 ,2172 }; //采集车头传感器色块边缘值作为基准，从右到左，从前进方向看
//u16 fADCDatum0[5] = { 1605 ,1775 ,1773 ,1645 ,1775 }; //采集车头传感器色块边缘值作为基准，从右到左，从前进方向看
//1017.001127.00646.001289.00608.00
//2697.003190.002803.003105.001676.00

/*
//采集色块最小基准阈值   1号车
u16 fADCDatum0[5] = {1600 ,1800 ,1800 ,1000 ,1900};
//u16 fADCDatum0[5] = {1300, 1800, 1500, 1300, 1400}; //采集车头传感器色块边缘值作为基准，从右到左，从前进方向看
u16 mADCDatum0[4] = {1882 ,1895 ,2089 ,2128}; //采集中间传感器色块边缘值作为基准
//u16 bADCDatum0[5] = {2057,2223,2289,1948,2267}; //采集车尾传感器色块边缘值作为基准，从右到左，从前进方向看
//u16 bADCDatum0[5] = {1678 ,1842 ,1948 ,1534 ,1992};
u16 bADCDatum0[5] = {1900 ,1842 ,1948 ,1834 ,1992}; //0605

u16 fADCDatum0[5] = {1600 ,1800 ,1800 ,1000 ,1900};
u16 mADCDatum0[4] = {1882 ,1895 ,2089 ,2128}; //采集中间传感器色块边缘值作为基准
u16 bADCDatum0[5] = {2057,2223,2289,1948,2267}; //采集车尾传感器色块边缘值作为基准，从右到左，从前进方向看

*/


////采集色块最小基准阈值 2号车
//u16 fADCDatum0[5] = {1600, 1600, 1600, 1000, 1600};
//u16 mADCDatum0[4] = {1882, 1895, 2089, 2128}; //采集中间传感器色块边缘值作为基准
//u16 bADCDatum0[5] = {1678, 1842, 1948, 1534, 1992};

/*
//采集色块最小基准阈值 1号车  现场
u16 fADCDatum0[5] = {2310 ,2249 ,2173 ,1406 ,2088};
u16 mADCDatum0[4] = {2044 ,2178 ,2477 ,2118}; //采集中间传感器色块边缘值作为基准
u16 bADCDatum0[5] = {1678, 1842, 1948, 1534, 1992};
*/
//u16 fADCDatum0[5] = {2206 ,2204 ,2133 ,1350 ,1883};
//u16 mADCDatum0[4] = {2300, 2300, 2300,1900 }; //采集中间传感器色块边缘值作为基准
//u16 bADCDatum0[5] = {1711 ,2099 ,2160 ,1708 ,2113 };


//u16 fADCDatum0[5] = {2367 ,2388 ,2320 ,1488 ,2100};
//u16 mADCDatum0[4] = {2144 ,2233 ,2639 ,2261}; //采集中间传感器色块边缘值作为基准
//u16 bADCDatum0[5] = {1897 ,2340 ,2485 ,2082 ,2585};

/*
//办公室环境小车2-白天 +800
u16 fADCDatum0[5] = {1968 ,1961 ,1778 ,1725 ,1901 };
u16 mADCDatum0[4] = {1631 ,1780 ,1712 ,1521}; //采集中间传感器色块边缘值作为基准
u16 bADCDatum0[5] = {1627 ,1891 ,1704 ,1646 ,1637};
*/

/*
//办公室环境小车2-晚上天 +800
u16 fADCDatum0[5] = {1700 ,1700 ,1600 ,2100 ,1800 };
u16 mADCDatum0[4] = {1585 ,1529 ,1868 ,1932}; //采集中间传感器色块边缘值作为基准
u16 bADCDatum0[5] = {1518 ,1748 ,1774 ,1507 ,1953};
*/

/*
//办公室环境小车1-白天 +800 300LUX
u16 fADCDatum0[5] = {1654 ,1628 ,1483 ,1954 ,1782};
u16 mADCDatum0[4] = {1565 ,1505 ,1808 ,1884 }; //采集中间传感器色块边缘值作为基准
u16 bADCDatum0[5] = {1493 ,1700 ,1726 ,1481 ,1882};
*/


/*
//办公室环境小车1-晚上+900  300LUX
u16 fADCDatum0[5] = {1751 ,1711 ,1585 ,2141 ,1893};
u16 mADCDatum0[4] = {1681 ,1619 ,1952 ,2021 }; //采集中间传感器色块边缘值作为基准
u16 bADCDatum0[5] = {1618 ,1835 ,1861 ,1586 ,1997};
*/



/*
//办公室环境小车1-晚上+900   2000LUX
u16 fADCDatum0[5] = {1186 ,1155 ,1072 ,1419 ,1263};
u16 mADCDatum0[4] = {1135 ,1101 ,1279 ,1302 }; //采集中间传感器色块边缘值作为基准
u16 bADCDatum0[5] = {1111 ,1213 ,1214 ,1089 ,1300};
*/

/*
//办公室环境小车1-晚上+300   2000LUX
u16 fADCDatum0[5] = {681 ,657 ,574 ,917 ,766};
u16 mADCDatum0[4] = {623 ,600 ,777 ,806 }; //采集中间传感器色块边缘值作为基准
u16 bADCDatum0[5] = {609 ,713 ,713 ,591 ,813};
*/



/*
//采集色块最小基准阈值 2号车  现场，不加遮光
u16 fADCDatum0[5] = {1100 ,1100 ,1000 ,1100 ,1100};
u16 mADCDatum0[4] = {1100 ,1100 ,1000 ,1100 }; //采集中间传感器色块边缘值作为基准
u16 bADCDatum0[5] = {1100 ,1100 ,1000 ,1100 ,1100};
*/

/*
//采集色块最小基准阈值 1号车  现场
u16 fADCDatum0[5] = {1600, 1600, 1600, 1600, 1900};
u16 mADCDatum0[4] = {2300, 2300, 2300,1900 }; //采集中间传感器色块边缘值作为基准
u16 bADCDatum0[5] = {1678, 1842, 1948, 1534, 1992};
*/

//采集色块最大基准阈值
int fADCDatum1[5] = {1886, 1784, 1632, 2317, 1932}; //采集车头传感器色块边缘值作为基准，从右到左，从前进方向看
int mADCDatum1[4] = {1708, 1683, 2043, 2087}; //采集中间传感器色块边缘值作为基准
int bADCDatum1[5] = {1703, 1896, 1943, 1638, 2185}; //采集车尾传感器色块边缘值作为基准，从右到左，从前进方向看




//适配晚上传感基准
//int fADCDatum0[5] = {1600, 2100, 1900, 1800, 1700}; //采集车头传感器色块边缘值作为基准，从右到左，从前进方向看
//int bADCDatum0[5] = {1596, 1631, 1691, 1321, 1662}; //采集车尾传感器色块边缘值作为基准，从右到左，从前进方向看
//int mADCDatum0[4] = {1618, 1632, 1862, 1827}; //采集中间传感器色块边缘值作为基准

//int fADCDatum0[5] = {1309, 1979, 1611, 1266, 1386}; //采集车头传感器色块边缘值作为基准，从右到左，从前进方向看
//int bADCDatum0[5] = {1522, 1708, 1905, 1429, 1855}; //采集车尾传感器色块边缘值作为基准，从右到左，从前进方向看
//int mADCDatum0[4] = {1671, 1663, 1928, 1992}; //采集中间传感器色块边缘值作为基准


#if ( CAR_NUM == 1 )
/*
//办公室环境小车1-白天上+900  300LUX 稳定跑参数(不遮窗帘)，车尾朝窗户采集
u16 fADCDatum0[5] = {1884 ,1797 ,1638 ,2302 ,1947};  //车头传感器+900 
u16 mADCDatum0[4] = {1710 ,1640 ,2013 ,2082}; //中间传感器+900
u16 bADCDatum0[5] = {1786 ,1972 ,1993 ,1714 ,2215}; //车尾传感器加1000
*/

//现场环境小车1-晚上上+900  300LUX 稳定跑参数(不遮窗帘)，车尾朝窗户采集
//u16 fADCDatum0[5] = {1893 ,1788 ,1635 ,2282 ,1898};  //车头传感器+900 
//u16 mADCDatum0[4] = {1727 ,1664 ,2013 ,2039}; //中间传感器+900
//u16 bADCDatum0[5] = {1765 ,1986 ,2030 ,1720 ,2162}; //车尾传感器加1000

u16 fADCDatum0[5] = {1870 ,1801 ,1641 ,2290 ,1902};  //车头传感器+900 
u16 mADCDatum0[4] = {1695 ,1648 ,1999 ,2031}; //中间传感器+900
u16 bADCDatum0[5] = {1661 ,1877 ,1910 ,1634 ,2122}; //车尾传感器加1000


#elif ( CAR_NUM == 2 )

/*
//办公室环境小车2-白天+900  300LUX，泊车场景，稳定跑参数(要求遮窗帘，光线均匀)
u16 fADCDatum0[5] = {1977, 1995, 1892, 1816, 1973}; //车头传感器+900
u16 mADCDatum0[4] = {1724, 1881, 1817, 1724}; //中间传感器+900
u16 bADCDatum0[5] = {1815, 2081, 1890, 1807, 1796};  //车尾传感器加1000
*/

/*
//办公室环境小车2-白天+900  300LUX，稳定跑参数(不遮窗帘)，车尾朝窗户采集
u16 fADCDatum0[5] = {1823 ,1895 ,1893 ,1703 ,1874}; //车头传感器+700 
u16 mADCDatum0[4] = {1862 ,2035 ,1999 ,1861}; //中间传感器+1000
//u16 bADCDatum0[5] = {1762 ,2105 ,1863 ,1720 ,1689};  //车尾传感器+900 
u16 bADCDatum0[5] = {1762 ,2105 ,1863 ,1720 ,1589};  //车尾传感器+900
*/

/*
//办公室环境小车2-晚上+900  300LUX，稳定跑参数，车尾朝窗户采集
u16 fADCDatum0[5] = {2003 ,2087 ,2033 ,2007 ,2228}; //车头传感器+900 
u16 mADCDatum0[4] = {1764 ,2004 ,1960 ,1793}; //中间传感器+900
u16 bADCDatum0[5] = {1943 ,2297 ,2112 ,2031 ,2007};  //车尾传感器+1000
*/


//办公室环境小车2-白天+900  300LUX，稳定跑参数，车尾朝窗户采集
//u16 fADCDatum0[5] = {2003 ,2087 ,2033 ,2007 ,2228}; //车头传感器+900 
//u16 mADCDatum0[4] = {1764 ,2004 ,1960 ,1793}; //中间传感器+900
//u16 bADCDatum0[5] = {1943 ,2297 ,2112 ,2031 ,1600};
//u16 bADCDatum0[5] = {1900 ,2297 ,2112 ,1900 ,1600};  //车尾传感器+1000

u16 fADCDatum0[5] = {1750 ,1801 ,1641 ,2290 ,1800};  //车头传感器+900 
u16 mADCDatum0[4] = {1595 ,1648 ,1999 ,1900}; //中间传感器+900
//u16 bADCDatum0[5] = {1600 ,1877 ,1910 ,1634 ,2122}; //车尾传感器加1000
u16 bADCDatum0[5] = {1900 ,2000 ,2010 ,2000 ,2322}; //车尾传感器加1000


#elif ( CAR_NUM == 3 )  //待补充


#elif ( CAR_NUM == 4 )

//办公室环境小车4-白天+900  300LUX，泊车场景，（不车窗帘，车尾朝窗户采集）
u16 fADCDatum0[5] = {1872 ,1840 ,1729 ,1660 ,1922}; //车头传感器+900
u16 mADCDatum0[4] = {1580 ,1521 ,1798 ,1798}; //中间传感器+900
u16 bADCDatum0[5] = {1843 ,1866 ,1680 ,1848 ,1743};  //车尾传感器加1000


#endif



int fSensorRead[5] = {0, 0, 0, 0, 0};    //存储车头传感器状态，从左到右存储方便写逻辑
int bSensorRead[5] = {0, 0, 0, 0, 0};    //存储车尾传感器状态
int mSensorRead[4] = {0, 0, 0, 0};     //存储中间传感器状态


//================================传感器相关变量
int TimeFlag = 0 ;    // 长时间未发现白线计时标志位
int F_Not_Find_White_STOP = 0;
int fSensorReadSum = 0, mSensorReadSum = 0, bSensorReadSum = 0;
int TestFlag, LastTestFlag ;


//================================行车相关变量
u8 Auto_Back_Flag = 0; //设置前进倒车状态0 前进  1倒车
//挡位设置，保持差速后电机最低速度不小于6，否则容易打滑和甩尾
float bGear0 = 0, bGear1 = 1, bGear2 = 2, bGear3 = 3, bGear4 = 4; //倒车挡位参数
//float fGear0 = 0, fGear1 = 1, fGear2 = 2, fGear3 = 3, fGear4 = 4; //前进挡位参数
float error;
int iTime = 0, error_time, error_time1, flag_print = 0;



//================================传感器数据校准相关变量
int Mod = 0; //Mod=0 运营巡线模式，  Mod=1 传感器数据基准采集，  Mod=2 传感器基准检查
float Menchmark_Coefficient = 900;//2.2;//2; //用与传感器数据基准采集时使用，用于调节传感器采集阈值的范围灵敏度，越小越灵敏，一般范围(2~4.5)
float White_line_datum[15], Black_road_datum[15];  //用于保存采集基准数据
int c = 0, aa = 0; //设置基准参数相关，默认值为0，不能修改或其他用途
int jishi = 0, jishi_50 = 0;

float pid_i, pid_d, pid_out;

//================================通讯协议相关变量
int Flag_Turn = 0; //标记行驶变化状态
int iUartCommand = 0x00;//保存串口字节

//================================外部调用相关变量
extern float After_filter[M];


/**************************************************************************
函数功能：循迹传感器基准阈值基准采集、设置、查询
模式配置：
    默认巡线模式（Mod=0  0xf0）
    传感器基准采集模式,电机关闭（Mod=1  0xf1）0xf3采集白色色块数据 0xf4采集道路数据 0xf5基准计算  0xf6基准值查询
    通过基准数据查询，可在串口助手拷贝填入对应数组，后续完善EEPROM自动保存功能
    传感器基准检查模式，电机关闭（Mod=2  0xf2）

注意：在采集模式下串口不要发送其他与采集无干指令干扰采集
**************************************************************************/
void DebugDrivingInformation(void)  //行车调试信息，运营时屏蔽掉，防止干扰主机通讯
{
    if (Auto_Back_Flag == 0  ) //前进模式
    {


        switch( jishi)
        {

        case 40:
            usart3_send_head();//前进方向看 左到右,行车车头传感信息
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

        //        case 20: //前进方向看 左到右,行车车中传感信息
        //            usart3_send_head();
        //            printf("mSensorRead[0-3]:%d%d%d%d\r\n", mSensorRead[0], mSensorRead[1], mSensorRead[2], mSensorRead[3]);
        //            jishi = 0;
        //            break;
        //        case 30:  //前进方向看  左到右,行车车尾传感信息
        //            usart3_send_head();
        //            printf("bSensorRead[4-0]:%d%d%d%d%d\r\n", bSensorRead[4], bSensorRead[3], bSensorRead[2], bSensorRead[1], bSensorRead[0]);
        //            jishi = 0;
        //            break;
        //        case 40:  //行车速度信息
        //            usart3_send_head();
        //           //      printf("TestFlag:%d Motor_A：%d Motor_B:%d Move_Y:%.2f,Move_Z:%.2f,Encoder_A:%d,Encoder_B:%d\r\n", TestFlag, Motor_A,Motor_B,Move_Y, Move_Z, Encoder_A, Encoder_B);
        //              printf("  Move_Y:%.2f,Move_Z:%.2f,Encoder_A:%d,Encoder_B:%d\r\n", Move_Y, Move_Z, Encoder_A, Encoder_B);
        //           // printf(" Motor_A：%d Motor_B:%d ,Encoder_A:%d,Encoder_B:%d\r\n", Motor_A,Motor_B, Encoder_A, Encoder_B);
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
        usart3_send_head();//前进方向看 左到右,行车车头传感信息
        printf("\r\nAfS[0-4]: %.2f%.2f%.2f%.2f%.2f ", After_filter[0], After_filter[1], After_filter[2], After_filter[3], After_filter[4]);
        jishi = 0;
        break;
        case 80:
        usart3_send_head();
            printf("\r\nAmS[5-9]: %.2f%.2f%.2f%.2f ", After_filter[5], After_filter[9], After_filter[7], After_filter[6]);
                jishi = 0;
            break;
        case 120:
        usart3_send_head();//前进方向看 左到右,行车车头传感信息
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
    else if (Auto_Back_Flag == 1) //倒车模式
    {
        switch( jishi)
        {
        case 40:
            usart3_send_head();//前进方向看 左到右,行车车头传感信息
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

    if(delay_50 == 1000)jishi_50 = delay_50; // 单独给电量预警使用




    if(Mod == 1)  //传感器基准采集
    {


        if(UrxSt == 0xc3) //采集白线数据
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
        else if(UrxSt == 0xc4) //采集道路数据
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
        else if(UrxSt == 0xc5) //基准数据计算
        {
            if( c < 5) //方向：从前进方向看左2左1中右1右2，车头传感器引脚PA0-PA4,
            {
                if(jishi == 20)
                {
                    fADCDatum0[c] = White_line_datum[c] + Menchmark_Coefficient;
                    // STMFLASH_Write(FLASH_SAVE_ADDR,(u16*)fADCDatum0,10);  //将计算的基准写入FLASH中
                    // STMFLASH_Read(FLASH_SAVE_ADDR,(u16*)fADCDatum0,10);   //从Flash中读取
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
                        // STMFLASH_Write(FLASH_SAVE_ADDR+12,(u16*)mADCDatum0,10);  //将计算的基准写入FLASH中
                        // STMFLASH_Read(FLASH_SAVE_ADDR+12,(u16*)mADCDatum0,10);       //从Flash中读取
                        printf("mADCDatum0[%d]:%d c=%d ", 0, (int)mADCDatum0[0], c);

                    }
                    else if(c == 9)
                    {
                        mADCDatum0[1] = White_line_datum[c] + Menchmark_Coefficient;
                        // STMFLASH_Write(FLASH_SAVE_ADDR+12,(u16*)mADCDatum0,10);  //将计算的基准写入FLASH中
                        // STMFLASH_Read(FLASH_SAVE_ADDR+12,(u16*)mADCDatum0,10);       //从Flash中读取
                        printf("mADCDatum0[%d]:%d c=%d ", 1, (int)mADCDatum0[1], c);
                    }
                    else if(c == 7)
                    {
                        mADCDatum0[2] = White_line_datum[c] + Menchmark_Coefficient;
                        // STMFLASH_Write(FLASH_SAVE_ADDR+12,(u16*)mADCDatum0,10);  //将计算的基准写入FLASH中
                        // STMFLASH_Read(FLASH_SAVE_ADDR+12,(u16*)mADCDatum0,10);       //从Flash中读取
                        printf("mADCDatum0[%d]:%d c=%d ", 2, (int)mADCDatum0[2], c);
                    }
                    else if(c == 6)
                    {
                        mADCDatum0[3] = White_line_datum[c] + Menchmark_Coefficient;
                        // STMFLASH_Write(FLASH_SAVE_ADDR+12,(u16*)mADCDatum0,10);  //将计算的基准写入FLASH中
                        // STMFLASH_Read(FLASH_SAVE_ADDR+12,(u16*)mADCDatum0,10);       //从Flash中读取
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
                    // STMFLASH_Write(FLASH_SAVE_ADDR+24,(u16*)bADCDatum0,10);  //将计算的基准写入FLASH中
                    // STMFLASH_Read(FLASH_SAVE_ADDR+24,(u16*)bADCDatum0,10);       //从Flash中读取
                    usart3_send_head();
                    printf("bADCDatum0[%d]:%d c=%d ", c - 10, (int)bADCDatum0[c - 10], c);
                    ++c;
                    jishi = 0;
                    delay_flag = 0;
                    if(c > 14) UrxSt = NULL, printf("\r\n"), c = 0;
                }

            }


        }
        else if(UrxSt == 0xc6) //基准数据查询，可在串口助手拷贝填入对应数组，后续完善EEPROM自动保存功能
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
    else if(Mod == 2)  //传感器基准检查
    {

        switch( jishi)
        {
        case 20:
            usart3_send_head();//前进方向看 左到右
            printf("\r\nfSensorRead[0-4]:%d%d%d%d%d\r\n", fSensorRead[0], fSensorRead[1], fSensorRead[2], fSensorRead[3], fSensorRead[4]);
            jishi = 0;
            break;
        case 40: //前进方向看 左到右
            usart3_send_head();
            printf("mSensorRead[0-3]:%d%d%d%d\r\n", mSensorRead[0], mSensorRead[1], mSensorRead[2], mSensorRead[3]);
            jishi = 0;
            break;
        case 60:  //前进方向看  左到右
            usart3_send_head();
            printf("bSensorRead[4-0]:%d%d%d%d%d\r\n", bSensorRead[4], bSensorRead[3], bSensorRead[2], bSensorRead[1], bSensorRead[0]);
            jishi = 0;
            delay_flag = 0;
            break;

        default:
            break;

        }


    }
    else if(Mod == 3) //传感器数据采集
    {

        usart3_send_head();//前进方向看 左到右,行车车头传感信息
        printf("\r\n车头左到右[0-4]: %.0f %.0f %.0f %.0f %.0f ", After_filter[4], After_filter[3], After_filter[2], After_filter[1], After_filter[0]);
        SysTick_Delay_ms(150);

        usart3_send_head();
        printf("\r\n车中左前后，右后前[5-9]:  %.0f %.0f %.0f %.0f ", After_filter[6], After_filter[7], After_filter[9], After_filter[5]);
        SysTick_Delay_ms(150);

        usart3_send_head();//前进方向看 左到右,行车车头传感信息
        printf("\r\n车尾左到右[10-14]: %.0f %.0f %.0f %.0f %.0f  ", After_filter[10], After_filter[11], After_filter[12], After_filter[13], After_filter[14]);
        SysTick_Delay_ms(150);
    }
    else if(Mod == 4) //行车信息
    {
        DebugDrivingInformation(); //行车调试信息
    }

    //电池电压低于10.5V时通知软件系统及时回到充电站充电
    if( Voltage < 1050 ) //电量低预警功能
    {
        if(jishi_50 == 1000) //10S 预警一次
        {
            usart3_send_head();
            printf("\r\n电池电量低，请及时回到充电站充电，低于10V将强制熄火停车，当前电压： %.2f ", (float)Voltage / 100);

            jishi_50 = 0;
            delay_50 = 0;
        }

    }


}


/**************************************************************************
函数功能：检查Flash中是否存储循迹相关基准数据
入口参数：无
返回  值：存放过数据返回1，否则返回0
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
函数功能：五位传感器排列组合生成函数，每位有2种, 五位,2^5, 每位3种, 五位,3^5
         dfs(0); 生成组合
入口参数：无
返回  值：无
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
函数功能：循迹传感读取，当传感器距离屏幕太近不足以分档时使用01
入口参数：无
返回  值：无
**************************************************************************/
void ReadSensor01(void)
{
    //if(CheckFlashTrackData())
    {
        // 车头部分  从向前前进方向看
        fSensorRead[4] = bit_map(After_filter[0], fADCDatum0[0], 1);   //左2
        fSensorRead[3] = bit_map(After_filter[1], fADCDatum0[1], 1);    //左1
        fSensorRead[2] = bit_map(After_filter[2], fADCDatum0[2], 1);    //中
        fSensorRead[1] = bit_map(After_filter[3], fADCDatum0[3], 1);    //右1
        fSensorRead[0] = bit_map(After_filter[4], fADCDatum0[4], 1);    //右2

        //  fSensorRead[0] = bit_map2(After_filter[0] , fADCDatum0[0] , fADCDatum1[0]);  //右2
        //  fSensorRead[1] = bit_map2(After_filter[1] , fADCDatum0[1] , fADCDatum1[1]);  //右1
        //  fSensorRead[2] = bit_map2(After_filter[2] , fADCDatum0[2] , fADCDatum1[2]);  //中
        //  fSensorRead[3] = bit_map2(After_filter[3] , fADCDatum0[3] , fADCDatum1[3]);  //左1
        //  fSensorRead[4] = bit_map2(After_filter[4] , fADCDatum0[4] , fADCDatum1[4]);  //左2


        //车尾部分 从倒车前进方向看
        //  bSensorRead[0] = bit_map2(After_filter[10] , bADCDatum0[0] , bADCDatum1[0]); //右2
        //  bSensorRead[1] = bit_map2(After_filter[11] , bADCDatum0[1] , bADCDatum1[1]); //右1
        //  bSensorRead[2] = bit_map2(After_filter[12] , bADCDatum0[2] , bADCDatum1[2]); //中
        //  bSensorRead[3] = bit_map2(After_filter[13] , bADCDatum0[3] , bADCDatum1[3]); //左1
        //  bSensorRead[4] = bit_map2(After_filter[14] , bADCDatum0[4] , bADCDatum1[4]); //左2

        //车尾部分 从倒车前进方向看
        bSensorRead[4] = bit_map(After_filter[10], bADCDatum0[0], 1);   //左2
        bSensorRead[3] = bit_map(After_filter[11], bADCDatum0[1], 1);    //左1
        bSensorRead[2] = bit_map(After_filter[12], bADCDatum0[2], 1);    //中
        bSensorRead[1] = bit_map(After_filter[13], bADCDatum0[3], 1);    //右1
        bSensorRead[0] = bit_map(After_filter[14], bADCDatum0[4], 1);    //右2

        //车中间部分, 从向前前进方向看
        mSensorRead[3] = bit_map(After_filter[5], mADCDatum0[0], 1 );    //前右1
        mSensorRead[2] = bit_map(After_filter[9], mADCDatum0[1], 1);    //后右1
        mSensorRead[1] = bit_map(After_filter[7], mADCDatum0[2], 1);    //后左1
        mSensorRead[0] = bit_map(After_filter[6], mADCDatum0[3], 1);    //前左1
    }

}

void ReadSensor02(void)
{

    // 车头部分  从向前前进方向看
    fSensorRead[4] = bit_map_scope(After_filter[0], fADCDatum0[0], fADCDatum1[0], 1);  //左2
    fSensorRead[3] = bit_map_scope(After_filter[1], fADCDatum0[1], fADCDatum1[1], 1);   //左1
    fSensorRead[2] = bit_map_scope(After_filter[2], fADCDatum0[2], fADCDatum1[2], 1);   //中
    fSensorRead[1] = bit_map_scope(After_filter[3], fADCDatum0[3], fADCDatum1[3], 1);   //右1
    fSensorRead[0] = bit_map_scope(After_filter[4], fADCDatum0[4], fADCDatum1[4], 1);   //右2

    //车尾部分 从倒车前进方向看
    bSensorRead[4] = bit_map_scope(After_filter[10], bADCDatum0[0], bADCDatum1[0], 1);  //左2
    bSensorRead[3] = bit_map_scope(After_filter[11], bADCDatum0[1], bADCDatum1[1], 1);   //左1
    bSensorRead[2] = bit_map_scope(After_filter[12], bADCDatum0[2], bADCDatum1[2], 1);   //中
    bSensorRead[1] = bit_map_scope(After_filter[13], bADCDatum0[3], bADCDatum1[3], 1);   //右1
    bSensorRead[0] = bit_map_scope(After_filter[14], bADCDatum0[4], bADCDatum1[4], 1);   //右2

    //车中间部分, 从向前前进方向看
    mSensorRead[3] = bit_map_scope(After_filter[5], mADCDatum0[0], mADCDatum1[0], 1);    //前右1
    mSensorRead[2] = bit_map_scope(After_filter[9], mADCDatum0[1], mADCDatum1[1], 1);    //后右1
    mSensorRead[1] = bit_map_scope(After_filter[7], mADCDatum0[2], mADCDatum1[2], 1);    //后左1
    mSensorRead[0] = bit_map_scope(After_filter[6], mADCDatum0[3], mADCDatum1[3], 1);    //前左1


}



/**************************************************************************
函数功能：无线串口通信逻辑，收到上位机功能指令则回传上位机功能指令
协议格式：默认前缀：0x00,0x01,0x17 功能指令：0xfe;示例：0x00,0x01,0x17,0xfe
规则：设备开关0xa- 前进行驶0xf-  倒车行驶0xb-  出库0x1-  入库0x2-  避障0x3-  转弯0x4-  数据采集0xc-

基础协议部分
1、设备启动:0xaa 、 设备关闭：0xab (默认关闭电机，需要设备启动指令或者归为后起始点的指令打开电机)
2、前进行驶：0xf1
3、运动停止：0xf0

小车参数校准部分(进入参数校准设备进入关闭状态进行)
1、进入参数采集模式：0xc1
2、参数采集模式下，0xc3：采集白色色块数据，0xc4：采集道路数据，0xc5：传感器基准计算，0xc6：传感器基准查询
3、传感器基准检查模式：0xc2
4、0xc7：传感器原始数据采集，0xc8：串口打印行车信息

充电站场景--进出充电站部分
1、前进出充电站：0xf2 --进入前进模式
2、倒车进充电站：0xb2 --进入倒车模式
3、充电站内停止：0xab

泊车场景--遥控泊车部分
1、前进行驶：0xf1   --进入前进模式
2、泊车出库：0x12
2、泊车入库：0x22   --进入倒车模式
3、运动停止：0xf0

泊车场景--倒车循迹车部分
1、倒车出库：0x13   --进入倒车模式
2、前进行驶：0xf1   --进入前进模式
3、前进窄路避障：0x31
4、倒车窄路避障：0x32 --进入倒车模式
5、转弯1：0x41
6、转弯2：0x42
7、倒车入库：0x22  --进入倒车模式
8、运动停止：0xf0

泊车场景--记忆泊车部分
1、倒车出库：0x14   --进入倒车模式
2、运动停止：0xf0
//3、前进行驶：0xf1    --进入前进模式
4、绕行避障1：0x33
5、绕行转弯（进入二层车库）：0x43
6、入库停车：0x23
7、出库复位：0x15
8、出库复位转弯：0x44
9、出库复位停车：0x22

泊车场景--一键讲解
1、二层车库到窄路避障：0x34  --进入前进模式

高速场景--NDA智能领航辅助系统
1、NDA智能领航辅助系统：0xf3  --进入前进模式
2、高速场景倒车入库：0x22 --进入倒车模式




参数说明：
1、Flag_Stop：电机打开和关闭；
2、Auto_Back_Flag：0：前进模式，1：倒车模式；
3、Flag_Turn：
            0：前进直线，
            1：遥控泊车出库，2:遥控泊车入库，
            3：倒车循迹倒车出库，4：倒车循迹前进窄路避障， 5：倒车循迹倒车窄路避障， 6：倒车循迹前进转弯1， 7：倒车循迹前进转弯2， 8：倒车循迹倒车入库，
            9：记忆泊车倒车出库，10：记忆泊车绕行避障1，11：记忆泊车绕行转弯，12：记忆泊车倒车入库停车，13：记忆泊车出库复位
            14：一键讲解二层车库到窄路避障 ，15：出充电站 ， 16：进充电站 ， 17：高速场景



曲率：
泊车场景部分
技术点1--遥控泊车：遥控泊车出库曲率=2/1.5  遥控泊车入库曲率=
技术点2--倒车循迹：倒车循迹倒车出库曲率=2   倒车循迹窄路前进曲率=   倒车循迹窄路倒车曲率=   倒车循迹转弯1曲率=   倒车循迹转弯2曲率=   倒车循迹倒车入库曲率=
技术点3--记忆泊车：记忆泊车出库曲率=     记忆泊车绕行避障曲率=    记忆泊车绕行转弯曲率=   记忆泊车倒车入库曲率=    记忆泊车前进出库曲率=
高速场景部分
城市场景部分

**************************************************************************/


void Track_Rx_Logic(void)  //串口通讯逻辑放在此文件，方便调试使用
{
    switch( UrxSt )
    {
    //校准传感器后回到正常行车模式
    case 0xc0:
        iUartCommand = UrxSt;
        Mod = 0;
        Flag_Stop = 0;//标记打开电机
        break;
    //进入传感器数据采集校准模式
    case 0xc1:
        iUartCommand = UrxSt;
        Mod = 1;
        Flag_Stop = 1; //标记关闭电机
        break;
    //传感器基准检查模式
    case 0xc2:
        iUartCommand = UrxSt;
        Mod = 2;
        // Flag_Stop = 1; //标记关闭电机
        break;
    //传感器数据采集
    case 0xc7:
        iUartCommand = UrxSt;
        Mod = 3;
        Flag_Stop = 1; //标记关闭电机
        break;
    //行车信息串口打印
    case 0xc8:
        iUartCommand = UrxSt;
        Mod = 4;
        break;

    //设备启动，打开电机
    case 0xaa:
        iUartCommand = UrxSt;
        Flag_Stop = 0;//标记打开电机
        break;
    //熄火停车设备关闭，关闭电机
    case 0xab:
        iUartCommand = UrxSt;
        Flag_Stop = 1; //标记关闭电机
        Flag_Turn = -1;
        break;


    //运动停止
    case 0xf0:
        iUartCommand = UrxSt;
        Move_Y = 0;
        Move_Z = 0;
        Flag_Turn = -1;
        break;
    //前进行驶
    case 0xf1:
        iUartCommand = UrxSt;
        Auto_Back_Flag = 0;
        Flag_Turn = 0;
        break;

    //遥控泊车出库
    case 0x12:
        iUartCommand = UrxSt;
        Auto_Back_Flag = 0;
        Flag_Turn = 1;
        Flag_Stop = 0;//标记打开电机
        break;
    //遥控泊车入库
    case 0x22:
        iUartCommand = UrxSt;
        Auto_Back_Flag = 1;
        Flag_Turn = 2;
        break;

    //倒车循迹倒车出库
    case 0x13:
        iUartCommand = UrxSt;
        Auto_Back_Flag = 1;
        Flag_Turn = 3;
        Flag_Stop = 0;//标记打开电机
        break;
    //倒车循迹前进窄路避障
    case 0x31:
        iUartCommand = UrxSt;
        Auto_Back_Flag = 0;
        Flag_Turn = 4;
        break;
    //倒车循迹倒车窄路避障
    case 0x32:
        iUartCommand = UrxSt;
        Auto_Back_Flag = 1;
        Flag_Turn = 5;
        break;
    //倒车循迹转弯1
    case 0x41:
        iUartCommand = UrxSt;
        Auto_Back_Flag = 0;
        Flag_Turn = 6;
        break;
    //倒车循迹转弯2
    case 0x42:
        iUartCommand = UrxSt;
        Auto_Back_Flag = 0;
        Flag_Turn = 7;
        break;
    //倒车循迹倒车入库
    case 0x23:
        iUartCommand = UrxSt;
        Auto_Back_Flag = 1;
        Flag_Turn = 8;
        break;

    //记忆泊车倒车出库
    case 0x14:
        iUartCommand = UrxSt;
        Auto_Back_Flag = 1;
        Flag_Turn = 9;
        Flag_Stop = 0;//标记打开电机
        break;
    //记忆泊车绕行避障1
    case 0x33:
        iUartCommand = UrxSt;
        Auto_Back_Flag = 0;
        Flag_Turn = 10;
        break;
    //记忆泊车绕行转弯
    case 0x43:
        iUartCommand = UrxSt;
        Auto_Back_Flag = 0;
        Flag_Turn = 11;
        break;
    //记忆泊车倒车入库停车
    case 0x24:
        iUartCommand = UrxSt;
        Auto_Back_Flag = 1;
        Flag_Turn = 12;
        break;
    //记忆泊车出库复位
    case 0x15:
        iUartCommand = UrxSt;
        Auto_Back_Flag = 0;
        Flag_Turn = 13;
				Speed = 915;
        break;
    //一键讲解二层车库到窄路避障
    case 0x34:
        iUartCommand = UrxSt;
        Auto_Back_Flag = 0;
        Flag_Turn = 14;
        Flag_Stop = 0;//标记打开电机
        break;
		
    //前进出充电站
    case 0xf2:
        iUartCommand = UrxSt;
        Auto_Back_Flag = 0;
        Flag_Turn = 15;
        Flag_Stop = 0;//标记打开电机
        break;
    case 0xb2:
        //倒车进充电站
        iUartCommand = UrxSt;
        Auto_Back_Flag = 1;
        Flag_Turn = 16;
        break;
		
		//高速场景
    case 0xf3:
        iUartCommand = UrxSt;
        Auto_Back_Flag = 0;
        Flag_Turn = 17;
        Flag_Stop = 0;//标记打开电机
        break;

    default:

        break;
    }


}


/**************************************************************************
函数功能：巡线速度限幅函数
入口参数：无
返回  值：无
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
函数功能：加减速逻辑函数
入口参数：无
返回  值：无
**************************************************************************/

float V1_Step = 10, V2_Step = 50, V3_Step = 300 ;
void U_Speed_Response(void)  //简版S型直线加速逻辑
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

void C_Speed_Response(void)  //简版S型直线减速逻辑
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


//待完善：需要考虑当小车加减速传感偏移出移动黑色色块时状态，且不能受特效光波等其他因素影响
void Acc_And_Dec_Logic(void)
{
    if (Auto_Back_Flag == 0)  //前进
    {

        //                  前左1                   后左1                 后右1                   前右1
        mSensorReadSum = (mSensorRead[0] << 3) + (mSensorRead[1] << 2) + (mSensorRead[2] << 1) + (mSensorRead[3] << 0) ;
        //                   左2                    左1                   中                       右1                 右2
        bSensorReadSum = (bSensorRead[0] << 0) + (bSensorRead[1] << 1) + (bSensorRead[2] << 2) + (bSensorRead[3] << 3) + (bSensorRead[4] << 4) ;

        if((bSensorRead[0] && bSensorRead[1] && bSensorRead[2] && bSensorRead[3] && bSensorRead[4]) == 0 )  //前进状态，车尾传感任意一个识别到进行减速
        {

#if( CAR_NUM == 1 )
            switch(Flag_Turn)  
            {

            case 9: //记忆泊车倒车出库
            case 10://记忆泊车绕行避障1 
            case 11://记忆泊车绕行转弯
                Speed = 920;
                break;
            case 12://记忆泊车倒车入库停车
                Speed = 850;
                break;

            default:
                C_Speed_Response();
								if (Speed <= 0 && Flag_Turn == -1) Speed = 0;
                else Speed = 900; //当减速到0的时候查询下是否还在行驶状态，是的话可能是干扰导致的，给个速度继续行驶
                break;
            }

#elif( CAR_NUM == 2 )
            switch(Flag_Turn)  
            {
													
            case 9://记忆泊车倒车出库
            case 10://记忆泊车绕行避障1
            case 11://记忆泊车绕行转弯
               // Speed = 930;// 950;//920;
                break;
            case 12://记忆泊车倒车入库停车
                Speed = 900;//850;
                break;
						
						case 17:  //高速场景不减速
								//Speed = 950; 
								break;
						
						case 6:
						case 7:
							 break;

            default:
//                C_Speed_Response();
//                if (Speed <= 0 && Flag_Turn == -1) Speed = 0;
//                else Speed = 900; //当减速到0的时候查询下是否还在行驶状态，是的话可能是干扰导致的，给个速度继续行驶
                break;
            }
#endif


        }
       // else if( (bSensorRead[2] == 1) && ((mSensorRead[0] && mSensorRead[1] && mSensorRead[2] && mSensorRead[3]) == 0) ) //前进状态，中间加减速任意一个识别到进行加速
           else if(  ((mSensorRead[0] && mSensorRead[1] && mSensorRead[2] && mSensorRead[3]) == 0) ) //前进状态，中间加减速任意一个识别到进行加速
        {

#if( CAR_NUM == 1 )
            switch(Flag_Turn)  
            {
            case 9://记忆泊车倒车出库
            case 10://记忆泊车绕行避障1
            case 11://记忆泊车绕行转弯
                Speed = 950;//920;
                break;
            case 12://记忆泊车倒车入库停车
                Speed = 850;
                break;
						case 17:  //高速场景加速更快些
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
            case 9://记忆泊车倒车出库
            case 10://记忆泊车绕行避障1
            case 11://记忆泊车绕行转弯

//							if(mSensorRead[1]==0 &&  mSensorRead[2] == 0 )
//                Speed = 950;//950;//920;
//							else 
//								Speed = 930;
							
							//else if((mSensorRead[1]&&mSensorRead[2] ) == 0)
                break;
            case 12://记忆泊车倒车入库停车
                Speed = 900;//850;
                break;
						
						case 17:  //高速场景加速更快些
								//U_Speed_Response(); 
//								if( ((mSensorRead[0] && mSensorRead[3]) == 0)) Speed = 940;  //中间加速前面两个识别到给更快的速度
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

    else if (Auto_Back_Flag == 1) //倒车
    {
        //                   左2                    左1                   中                       右1                 右2
        fSensorReadSum = (fSensorRead[0] << 0) + (fSensorRead[1] << 1) + (fSensorRead[2] << 2) + (fSensorRead[3] << 3) + (fSensorRead[4] << 4) ;
        //                  前左1                   后左1                 后右1                   前右1
        mSensorReadSum = (mSensorRead[2] << 3) + (mSensorRead[3] << 2) + (mSensorRead[0] << 1) + (mSensorRead[1] << 0) ;

        if((fSensorRead[0] && fSensorRead[1] && fSensorRead[2] && fSensorRead[3] && fSensorRead[4]) == 0 )  //倒车状态，车头传感任意一个识别到进行减速
        {


#if( CAR_NUM == 1 )
            switch(Flag_Turn)  
            {

            case 9://记忆泊车倒车出库
                Speed = 930;
                break;
            case 12://记忆泊车倒车入库停车
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

            case 5: //倒车循迹，窄路避障倒车行驶
                C_Speed_Response();
                break;
						
						case 6:
						case 7:
							 break;

            case 9://记忆泊车倒车出库  
                //Speed = 900;
                break;
            case 12://记忆泊车倒车入库停车
               // Speed = 850;
                break;

            default:
                C_Speed_Response();
                if(Speed >= 900) Speed = 900;
                break;
            }

#endif

        }
        else if( (fSensorRead[2] == 1) && ((mSensorRead[0] && mSensorRead[1] && mSensorRead[2] && mSensorRead[3]) == 0) ) //倒车状态，中间加减速任意一个识别到进行加速
            // else if( ((mSensorRead[0] && mSensorRead[1] && mSensorRead[2] && mSensorRead[3]) == 0) ) //倒车状态，中间加减速任意一个识别到进行加速

        {

#if( CAR_NUM == 1 )
            switch(Flag_Turn)  
            {

            case 9://记忆泊车倒车出库
                Speed = 930;
                break;
            case 12://记忆泊车倒车入库停车
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
							
            case 5: //倒车循迹，窄路避障倒车行驶，直线较多，让速度快些
                U_Speed_Response();
                break;

            case 9://记忆泊车倒车出库
              //  Speed = 900;
                break;
            case 12://记忆泊车倒车入库停车
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
函数功能：巡线逻辑控制
入口参数：无
返回  值：无
**************************************************************************/
//============================================PID算法

/*
小车在直线左右抖动频繁，减少Kp的值或减少Kd值
小车在弯道的时候转不过去，增大Kp的值或增大Ki值
小车经过弯道回直线时冲出黑线，增大Kd的值或减少Ki值

若要反应增快，增大P减小I；
若要反应减慢，减小P增大I；
如果比例太大，会引起系统震荡；
如果积分太大，会引起系统迟钝

KP作用：转向角度，值越大，转向角度越大
KI作用：累积转向误差并消除
KD作用：转向阻力,kd越大阻力越大，收拢作用，减弱惯性
*/



#define F1_MAX 30
#define F2_MAX 25
//float fGear0 = 0, fGear1 = 8, fGear2 = 10, fGear3 = 15, fGear4 = 20; //前进挡位参数
float fGear0 = 0, fGear1 = 2, fGear2 = 4, fGear3 = 8, fGear4 = 16; //前进挡位参数

/*
//可完整稳定跑全程参数
//前进行驶PID参数
//float Kp0 = 2, Ki0 = 0, Kd0 = 2.1;  //直线行驶  0xf1
float Kp0 = 70, Ki0 = 0, Kd0 = 2;  //直线行驶  0xf1 ，可以跑完全程高速
float Kp1 = 15, Ki1 = 0, Kd1 = 10.5; //遥控泊车出库  0x12
float Kp4 = 15, Ki4 = 0, Kd4 = 8;  //倒车循迹窄路避障 0x31
float Kp6 = 35, Ki6 = 0, Kd6 = 10;  //倒车循迹前进转弯1 0x41
float Kp7 = 35, Ki7 = 0, Kd7 = 10;  //倒车循迹前进转弯2 0x42
float Kp10 = 50, Ki10 = 0, Kd10 = 10; //10; //记忆泊车绕行避障1 0x33
float Kp11 = 50, Ki11 = 0, Kd11 = 10; //10;  //记忆泊车绕行转弯  0x43
float Kp13 = 30, Ki13 = 0, Kd13 = 20; //记忆泊车出库复位  0x44
float Kp14 = 30, Ki14 = 0, Kd14 = 20; //一键讲解二层车库到窄路避障  0x34
float Kp15 = 30, Ki15 = 0, Kd15 = 20; //前进出充电站  0xf2


//倒车行驶PID参数
float Kp2 = 30, Ki2 = 0, Kd2 = 10.5;
*/

//前进行驶PID参数
float Kp0 = 70, Ki0 = 0, Kd0 = 2;  //直线行驶  0xf1 ，可以跑完全程高速
float Kp1 = 60, Ki1 = 0, Kd1 = 10; //遥控泊车出库  0x12
//float Kp4 = 60, Ki4 = 0, Kd4 = 10;//2;  //倒车循迹窄路避障 0x31
float Kp4 = 60, Ki4 = 0, Kd4 = 10;//2;  //倒车循迹窄路避障 0x31
float Kp6 = 60, Ki6 = 0, Kd6 = 10;//2;  //倒车循迹前进转弯1 0x41
float Kp7 = 60, Ki7 = 0, Kd7 = 10;//2;  //倒车循迹前进转弯2 0x42
float Kp10 = 60, Ki10 = 0, Kd10 = 2; //10; //记忆泊车绕行避障1 0x33
float Kp11 = 60, Ki11 = 0, Kd11 = 2; //10;  //记忆泊车绕行转弯  0x43
float Kp13 = 60, Ki13 = 0, Kd13 = 2; //记忆泊车出库复位  0x44
float Kp14 = 60, Ki14 = 0, Kd14 = 2; //一键讲解二层车库到窄路避障  0x34
float Kp15 = 60, Ki15 = 0, Kd15 = 2; //前进出充电站  0xf2
float Kp17 = 60, Ki17 = 0, Kd17 = 2; //高速场景  0xf3


//倒车行驶PID参数
//float Kp2 = 10, Ki2 = 0, Kd2 = 2;  //入库居中
//float Kp2 = 15, Ki2 = 0, Kd2 = 2;
float Kp2 = 15, Ki2 = 0, Kd2 = 2;


float  fP = 0, fI = 0, fD = 0, PID_value = 0;  //pid直道参数
float previous_error = 0, previous_I = 0;
float xf = 300;   //500;//300;//250;  //一般设置在速度1/3左右为转弯幅度
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
        case 0:  //前进直线
            PID_value = (Kp0 * fP) + (Ki0 * fI) + (Kd0 * fD);
            break;
        case 1:  //遥控泊车出库
            PID_value = (Kp1 * fP) + (Ki1 * fI) + (Kd1 * fD);
            break;
        case 4:  //倒车循迹窄路避障
            PID_value = (Kp4 * fP) + (Ki4 * fI) + (Kd4 * fD);
            break;
        case 6:  //倒车循迹前进转弯1
            PID_value = (Kp6 * fP) + (Ki6 * fI) + (Kd6 * fD);
            break;
        case 7:  //倒车循迹前进转弯2
            PID_value = (Kp7 * fP) + (Ki7 * fI) + (Kd7 * fD);
            break;
        case 10:  //记忆泊车绕行避障1
            PID_value = (Kp10 * fP) + (Ki10 * fI) + (Kd10 * fD);
            break;
        case 11:  //记忆泊车绕行转弯
            PID_value = (Kp11 * fP) + (Ki11 * fI) + (Kd11 * fD);
            break;
        case 13:  //记忆泊车出库复位
            PID_value = (Kp13 * fP) + (Ki13 * fI) + (Kd13 * fD);
            break;
        case 14:  //一键讲解二层车库到窄路避障
            PID_value = (Kp14 * fP) + (Ki14 * fI) + (Kd14 * fD);
        case 15:  //前进出充电站
            PID_value = (Kp15 * fP) + (Ki15 * fI) + (Kd15 * fD);
            break;
				case 17:  //高速场景
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

        //                   左2                    左1                   中                       右1                 右2
        fSensorReadSum = (fSensorRead[0] << 4) + (fSensorRead[1] << 3) + (fSensorRead[2] << 2) + (fSensorRead[3] << 1) + (fSensorRead[4] << 0) ;
        //                  前左1                   后左1                 后右1                   前右1
        mSensorReadSum = (mSensorRead[0] << 3) + (mSensorRead[1] << 2) + (mSensorRead[2] << 1) + (mSensorRead[3] << 0) ;
        //                   左2                    左1                   中                       右1                 右2
        bSensorReadSum = (bSensorRead[0] << 0) + (bSensorRead[1] << 1) + (bSensorRead[2] << 2) + (bSensorRead[3] << 3) + (bSensorRead[4] << 4) ;
			
				fStopSensor = bSensorRead[0] + bSensorRead[1] + bSensorRead[2] + bSensorRead[3] + bSensorRead[4];

        //============================================以下加减速处理逻辑=====================================
			
			 // if( fSensorReadSum != 31 || fSensorReadSum != 0 )
         Acc_And_Dec_Logic();
				
			      switch(Flag_Turn)  
            {
            case 10://记忆泊车绕行避障1
            case 11://记忆泊车绕行转弯

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
//								if((bSensorRead[0] && bSensorRead[1] && bSensorRead[2] && bSensorRead[3] && bSensorRead[4]) == 0 )  //前进状态，车尾传感任意一个识别到进行减速
//									C_Speed_Response();
//								else if(  ((mSensorRead[0] && mSensorRead[1] && mSensorRead[2] && mSensorRead[3]) == 0) ) //前进状态，中间加减速任意一个识别到进行加速
//									 U_Speed_Response();
//								else
//								  Speed = 915;
                break;
            }


        //============================================以下正常循迹处理逻辑=====================================

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
				
				
				


        if(fSensorReadSum == B11111 || fSensorReadSum == B00000 || iUartCommand == 0xf0 )  //当全白或者全黑时，为非正常状态
        {
            ++TimeFlag;
            if (TimeFlag >= BLACK_COLOR_DELAY)//30*50=1500ms延时停止
            {
                TimeFlag = 0;
                F_Not_Find_White_STOP = 1;      // “长时间未发现白线标志位”置1
            }

        }
        else
        {
            TimeFlag = 0; // 清除“未发现白线计时”,计时在定时器中断函数中
            F_Not_Find_White_STOP = 0;  // 清除“长时间未发现白线标志位”
        }
				
				
				if(fStopSensor < 2 && iUartCommand == 0xf0)   F_Not_Find_White_STOP = 1; 



        if(F_Not_Find_White_STOP == 1) //使用此逻辑判断增加传感器因为分界线受光影响的抗干扰性
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

    //                   左2                    左1                   中                       右1                 右2
    fSensorReadSum = (fSensorRead[0] << 0) + (fSensorRead[1] << 1) + (fSensorRead[2] << 2) + (fSensorRead[3] << 3) + (fSensorRead[4] << 4) ;
    //                  前左1                   后左1                 后右1                   前右1
    mSensorReadSum = (mSensorRead[2] << 3) + (mSensorRead[3] << 2) + (mSensorRead[0] << 1) + (mSensorRead[1] << 0) ;
    //                   左2                    左1                   中                       右1                 右2
    bSensorReadSum = (bSensorRead[0] << 4) + (bSensorRead[1] << 3) + (bSensorRead[2] << 2) + (bSensorRead[3] << 1) + (bSensorRead[4] << 0) ;
	
	 bStopSensor = fSensorRead[0] + fSensorRead[1] + fSensorRead[2] + fSensorRead[3] + fSensorRead[4];

    //============================================以下加减速处理逻辑=====================================
	 //if( bSensorReadSum != 31 || bSensorReadSum != 0 )
    Acc_And_Dec_Logic();
	

	//	u16 bADCDatum0[5] = {1943 ,2297 ,2112 ,2031 ,1600};
		switch(Flag_Turn)  
		{
			
		case 9://记忆泊车倒车出库
				if(mSensorRead[0]==0 ||  mSensorRead[3] == 0 )
					Speed = 950;//950;//920;
				else if(mSensorRead[1]==0 ||  mSensorRead[2] == 0 )
					Speed = 960;
				else 
					Speed = 800;
			//  Speed = 900;
				
				//bADCDatum0[0] = 1600;
				break;
		case 12://记忆泊车倒车入库停车
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
		
		//9：记忆泊车倒车出库，10：记忆泊车绕行避障1，11：记忆泊车绕行转弯，12：记忆泊车倒车入库停车，13：记忆泊车出库复位
		
//		case 2:
//			bADCDatum0[0] = 1700;
//		  bADCDatum0[4] =  1000;
//		break;

		default:			
//			bADCDatum0[0] = 1943;
//		  bADCDatum0[4] =  1600;
////				if((fSensorRead[0] && fSensorRead[1] && fSensorRead[2] && fSensorRead[3] && fSensorRead[4]) == 0 )  //倒车状态，车头传感任意一个识别到进行减速
//					C_Speed_Response();
//				else if(  ((mSensorRead[0] && mSensorRead[1] && mSensorRead[2] && mSensorRead[3]) == 0) ) //倒车状态，中间加减速任意一个识别到进行加速
//					 U_Speed_Response();
//				else
//					Speed = 915;
				break;
		}


    //============================================以下正常循迹处理逻辑=====================================

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

    if(bSensorReadSum == B11111 || bSensorReadSum == B00000 || iUartCommand == 0xf0) //当全白或者全黑时，为非正常状态
    {
        TimeFlag++;
        if (TimeFlag >= BLACK_COLOR_DELAY)//30*50=1500ms延时停止
        {
            TimeFlag = 0;
            F_Not_Find_White_STOP = 1;      // “长时间未发现白线标志位”置1
        }
    }
    else
    {
        TimeFlag = 0; // 清除“未发现白线计时”,计时在定时器中断函数中
        F_Not_Find_White_STOP = 0;  // 清除“长时间未发现白线标志位”
    }

		
		
		if(bStopSensor < 2 && iUartCommand == 0xf0)   F_Not_Find_White_STOP = 1; 
		
	
    
    if(F_Not_Find_White_STOP == 1 || (Last_Flag_Turn==2 && Flag_Turn == -1))  //使用此逻辑判断增加传感器因为分界线受光影响的抗干扰性
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


























