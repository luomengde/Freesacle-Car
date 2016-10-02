/*!
 *     COPYbRIGHT NOTICE
 *     Copyright (c) 2013,ɽ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�ɽ����̳ http://www.vcan123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
 *
 * @file       main.c
 * @brief      ɽ��K60 ƽ̨������
 * @author     ɽ��Ƽ�
 * @version    v5.0
 * @date       2013-08-28
 */

#include "common.h"
#include "include.h"
#include "outputdata.h"

#define nrf_speed_debug  0             //�������ߵ����Ƿ���
#define nrf_img_debug    0
#define nrf_yaokongqi    0

/*���ü������λ,�п����õ���*/
//#define NVIC_SystemReset()    {
//             __DSB();          /* Ensure all outstanding memory accesses includedbuffered write are completed before reset */
//             SCB_AIRCR  = ((0x5FA << SCB_AIRCR_VECTKEY_SHIFT)      |
//             SCB_AIRCR_SYSRESETREQ_MASK);
//             __DSB();                                                     /* Ensure completion of memory access */
//             while(1);                                                    /* wait until reset */
//             }


//����ͷ��ر���
uint8 *imgbuff = (uint8 *)(((uint8 *)&nrf_tx_buff) + COM_LEN);
//����洢����ͼ�������,2400=160*120/8
uint8  nrf_tx_buff[CAMERA_SIZE + 2*COM_LEN + DATA_PACKET];
//uint8  nrf_rx_buff[CAMERA_SIZE + 2*COM_LEN + DATA_PACKET];         //Ԥ��   �����˷���Դ

//uint8 imgbuff[CAMERA_SIZE];                             //����洢����ͼ�������

uint8  img[160*120];                                     //��ѹ���ͼ��
uint16 PIT_count=0;                                     //���ж�ѭ������������ʱ��Ƭ
uint8  Duoji_control_cnt=0;
uint32 Moto_speed_out;                              //����ٶȵ����ֵ
float  float_speed;                                 //��ʱʹ��
uint32 set_speed;                                   //����ٶȵ��趨ֵ
uint8  temp_count=0;
float  speed_nrf;                             //nrf�����ٶ�ʱ�ı���
uint32 speed_nrf_n;
float  img_curvature;                         //ͼ������
uint32 zhenshu_count;
uint8  time_chip_second;                      //�����������    60������
uint32  qibu_time_second;
uint8  qibu_speed_flag=1;            //���qibu_flag Ϊ1��ʱ��Ϊ�� ���Գ�ʼ��Ϊ1
uint32 camera_zhenshu;
static Site_t  site_zhenshu={128-16,128-16};      //��ʾ����ͷ֡��
extern uint8  podao_flag;
static uint16 podao_overlook_count;
uint8  pretect_stop_car_count;             //������������ֹ����
uint8  car_begin_count;
uint8  car_stop_count;
uint8  auto_car_flag;        //�Զ���ʻģʽ
uint8  hand_car_flag;        //�ֶ���ʻģʽ
uint8  forward_step;         //�ֶ�ģʽ�µ�����ָ��
uint8  back_step;
uint8  left_step;
uint8  right_step;
/*****************************Main***************************/
/*!
 *  @brief      main����
 *  @since      v5.0
 *  @note       mengde������ͷ������
 */
uint32 pit1_ms;
 void  main(void)
{
    uint8 count_nrf=0;
    uint8 buff[DATA_PACKET];
	uint8 buff1[32];
    uint8 *str1 = "";
	uint8 buff_yaokong[32];
	static uint16 var;
//    uint32 buff[1];
	uint8 relen;
	Init_system();                              //ϵͳ�ܳ�ʼ��
//	WDOG_Init();
    while(1)
    {
        pit_time_start(PIT1);
        camera_get_img();                       //����ͷ��ȡͼ��,ʹ�õ���120*160�Ĳɼ�
        img_extract(img,imgbuff);			    //������ͷͼ���ѹ
        median_line_extract(img);			    //��ȡ����ͷ����
        quan_dir_duty_new();				    //���ÿһ֡ͼ�������Ȩ��
        deal_key_event();                       //������Ϣ����
        pit1_ms=pit_time_get_ms(PIT1);
        pit_close(PIT1);
//        OutData[0]=speed_current;
//		OutData[1]=speed_duty;
//		OutData[2]=speed_err;
//		OutData[3]=speed_want;
//        OutPut_Data();                      //�������
//        WDOG_Feed();
//        printf("%d ",speed_current);
//        vcan_sendimg(imgbuff,120*160/8);

/*******************************NRF����*******************************/
#if nrf_speed_debug    //���ߵ����ٶ�
      //������Щ�����Լ��޸ļ��ɹ۲��ٶ�PID���Σ�������ٶȱջ�֮��Ϳ�������
      //�������ǵ��ڵ��PID
      //������һ���ط���Щ���⣬���Ǵ����ٶȹ��������Կ��ǽ�NRF�ŵ��ж��ÿ����һ��
      //�ٶȾͿ��Է���һ�Σ��������ܸ���׼ȷһ��

//      speed_nrf_n=speed_duty;
//      speed_nrf_n=speed_current;     //����PWM���ֵ
      speed_nrf_n=(ABS(se_du)*100)/9500;
//	  speed_nrf_n=(uint32)speed_control_output;
	  sprintf((char *)buff1,"%s%d",str1,speed_nrf_n);       //��str��i�ϲ���һ���ַ�����buff��ٽ��з���
                            //����NRF�޷����͸�����  ԭ��δ֪!!!   �������޷����͸�����
	  if(nrf_tx(buff1,DATA_PACKET) == 1 )            //����һ�����ݰ���buff����Ϊ32�ֽڣ�
      {
            //�ȴ����͹����У��˴����Լ��봦������
            while(nrf_tx_state() == NRF_TXING);         //�ȴ��������
      }
#endif

#if nrf_img_debug     //���ߵ����ٶ�
        /*********************** ���߷���ͼ�� ***********************/
        nrf_msg_tx(COM_IMG,nrf_tx_buff);
        while(nrf_tx_state() == NRF_TXING);             //�ȴ��������
#endif
/*ң�������Է�����ͣ��*/
#if nrf_yaokongqi
	 relen = nrf_rx(buff,DATA_PACKET);
	 if(relen != 0)
	 {
         if(buff[0]=='1'&&buff[1]=='1')     //11������
         {
			 speed_want=flash_speed_want;
			 Speed_protect=0;
			 enable_irq(PIT2_IRQn);
		 }
         if(buff[0]=='2'&&buff[1]=='2')    //22����ͣ��
         {
			 speed_want=0;
			 Speed_protect=1;
		 }
	 }
#endif
/******************************NRF����END*****************************/
    }
}
/*************************END OF MAIN************************/


/*!
 *  @brief      UART3�жϷ�����
 *  @since      v5.0
 */
void uart3_handler(void)
{
    char ch;
    if(uart_query    (UART3) == 1)   //�������ݼĴ�����    //'\v'    '\0'
    {
        //�û���Ҫ�����������
        uart_getchar   (UART3, &ch);                    //���޵ȴ�����1���ֽ�
        if(ch==0x01)                  //ǰ
        {
            car_begin_count++;
			if(car_begin_count>=1&&auto_car_flag==0&&hand_car_flag==0)  //��һ�η���
			{
				Speed_protect=0;				 //����
				auto_car_flag=1;
				car_begin_count=0;
				enable_irq(PIT2_IRQn);
			}
			if(car_begin_count>=1&&hand_car_flag==1)
			{
                forward_step=1;
				if(car_begin_count>=3)   //�������ΰ�ǰ���ʾҪ�����Զ�ģʽ
				{
				    auto_car_flag=1;
                    hand_car_flag=0;
					Speed_protect=0;
					car_begin_count=0;
				}
			}
		}
		else if(ch==0x02)           //��
		{
		    car_stop_count++;
			car_begin_count=0;
			if(car_stop_count>=1&&hand_car_flag==1)
			{
                back_step=1;
				car_stop_count=0;
			}
			else if(car_stop_count>=1&&auto_car_flag==1)
			{
               Speed_protect=1;                  //ͣ��   ��ת���ֶ�ģʽ
               car_stop_count=0;
               hand_car_flag=1;
			   auto_car_flag=0;
			}
			else
			{}
		}
		else if(ch==0x03)          //��
		{
		    car_begin_count=0;    //ֻҪ����һ�η����������
	        if(hand_car_flag==1)
                left_step=1;
		}
		else if(ch==0x04)         //��
		{
		    car_begin_count=0;   //ֻҪ����һ�η����������
			if(hand_car_flag==1)
                right_step=1;
		}
		else
		{}

    }

}


void PIT2_IRQHandler()        //��Ҫ�ж�ѭ��
{
    zhenshu_count++;
	PIT_count++;               //��PIT_countΪʱ��Ƭ  ����Ϊ5ms
    qibu_time_second++;
	if(qibu_time_second>=1500)
	{
        qibu_time_second=0;
	}

    if(zhenshu_count==1000)    //1������ʾһ�Σ���Ϊ��ǰ֡��
    {
        LCD_num_BC(site_zhenshu,camera_zhenshu,2,BLUE,RED);    //�����½���ʾ����ͷ֡��
        camera_zhenshu=0;
        zhenshu_count=0;
		time_chip_second++;
		if(time_chip_second>=60)
			time_chip_second=0;
    }
	if(PIT_count==2)
	{
        GetMotoPulse();         //��ȡ����������
	}
	else if(PIT_count==3)
	{
        SpeedControl();         //����ٶȿ�����
	}
	else if(PIT_count==4)
	{
        SpeedControlOutput();   //�ٶ����
	}
	else if(PIT_count==5)
	{
	    if(begin_the_frost_point==1)
	    {
            gpio_set(PTD4,1);
		}
		else
		{
            gpio_set(PTD4,0);
		}
		PIT_count=0;             //���PIT_count
	}
	else if(PIT_count==1)                //�ȿ��ƶ��
	{
        Duoji_control_cnt++;
		if(Duoji_control_cnt==2)                //10ms����һ�ζ��  ��whileѭ�����ڸ����һ��   ����ÿһ��
		{
            DJ_PID();
			Duoji_control_cnt=0;
		}
	}
	PIT_Flag_Clear(PIT2);
}

void PIT0_IRQHandler()      //����ɨ���ж�
{
     stop_count++;
	if(stop_count>=400)
	{
        stop_count=0;
	}
	start_line_stop(stop_count);
    if(out_circle_time_flag==1)
    {
        out_circle_count++;
		if(out_circle_count>=200)
		{
		    begin_the_frost_point=1;      //��ʼ������Ϸ��ĵ�
            out_circle_time_flag=0;
			out_circle_count=0;
		}
	}

    key_IRQHandler();
    PIT_Flag_Clear(PIT0);
}


/*!
 *  @brief      DMA0�жϷ�����
 *  @since      v5.0
 */
void DMA0_IRQHandler()
{
    camera_dma();
}
/*!
 *  @brief      PORTA�жϷ�����
 *  @since      v5.0
 */
void PORTA_IRQHandler()
{
    uint8  n;    //���ź�
    uint32 flag;

    while(!PORTA_ISFR);
    flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //���жϱ�־λ

    n = 29;                                             //���ж�
    if(flag & (1 << n))                                 //PTA29�����ж�
    {
        camera_vsync();
    }
#if ( CAMERA_USE_HREF == 1 )                            //ʹ�����ж�
    n = 28;
    if(flag & (1 << n))                                 //PTA28�����ж�
    {
        camera_href();
    }
#endif

}

/*!
 *  @brief      PORTE�жϷ�����
 *  @since      v5.0
 */
void PORTE_IRQHandler()                          //���������շ��� ����ʱʹ��
{
    uint8  n;    //���ź�
    uint32 flag;

    flag = PORTE_ISFR;
    PORTE_ISFR  = ~0;                                   //���жϱ�־λ

    n = 27;
    if(flag & (1 << n))                                 //PTE27�����ж�
    {
        nrf_handler();
    }
}
/*���Ź�����*/
void WDOG_Init(void)
{
    WDOG_UNLOCK = 0xC520;
    WDOG_UNLOCK = 0xD928;               //�������Ź��Ĵ���������д��ǰ�������������д�벻�ܳ���20��ʱ������

    WDOG_PRESC = 0x0700;                //��7+1����Ƶ

    WDOG_TOVALH = 0x0110;              //���Ź���ʱʱ������Ϊ3s
    WDOG_TOVALL = 0x5E90;
}

void WDOG_Feed(void)
{
    WDOG_REFRESH = 0xA602;
    WDOG_REFRESH = 0xB480;            //ι��
}
void WDOG_Close(void)
{
    WDOG_UNLOCK = 0xC520u;
    WDOG_UNLOCK = 0xD928u;
    WDOG_STCTRLH &= ~0x0001;          //�رտ��Ź� WDOTEN 0
}
void WDOG_Open(void)
{
    WDOG_UNLOCK = 0xC520;
    WDOG_UNLOCK = 0xD928;
    WDOG_STCTRLH |= 0x0001;          //�򿪿��Ź� WDOTEN 1
}










