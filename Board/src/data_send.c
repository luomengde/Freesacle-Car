#include "data_send.h"
/*
 * Function Name: CRC_CHECK
 * Input Param  : 需要校验的数组
 * Output Param : 返回校验后的值
 * Description  : 虚拟示波器校验函数
 * Author Data  : 2014/5/20 星期二, by Liu
 */
static unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT)
{
    unsigned short CRC_Temp;
    unsigned char i,j;
    CRC_Temp = 0xffff;

    for (i=0;i<CRC_CNT; i++){
        CRC_Temp ^= Buf[i];
        for (j=0;j<8;j++) {
            if (CRC_Temp & 0x01)
                CRC_Temp = (CRC_Temp >>1 ) ^ 0xa001;
            else
                CRC_Temp = CRC_Temp >> 1;
        }
    }
    return(CRC_Temp);
}
/*
 * Function Name: VisualScope_Output
 * Input Param  : 需要发送的数据，没用到的置零
 * Output Param : 经过校验后的数据
 * Description  : 虚拟示波器输出函数
 * Author Data  : 2014/5/20 星期二, by Liu
 */
void VisualScope_Output(float data1 ,float data2 ,float data3 ,float data4)
{
  int temp[4] = {0};
  unsigned int temp1[4] = {0};
  unsigned char databuf[10] = {0};
  unsigned char i;
  unsigned short CRC16 = 0;

  temp[0] = (int)data1;
  temp[1] = (int)data2;
  temp[2] = (int)data3;
  temp[3] = (int)data4;

  temp1[0] = (unsigned int)temp[0] ;
  temp1[1] = (unsigned int)temp[1];
  temp1[2] = (unsigned int)temp[2];
  temp1[3] = (unsigned int)temp[3];

  for(i=0;i<4;i++)
  {
    databuf[i*2]   = (unsigned char)(temp1[i]%256);
    databuf[i*2+1] = (unsigned char)(temp1[i]/256);
  }

  CRC16 = CRC_CHECK(databuf,8);
  databuf[8] = CRC16%256;
  databuf[9] = CRC16/256;
  for(i=0;i<10;i++)
    uart_putchar(UART3,(char)databuf[i]);
//    printf("%d",databuf[i]);
}