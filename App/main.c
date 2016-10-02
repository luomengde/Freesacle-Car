/*!
 *     COPYbRIGHT NOTICE
 *     Copyright (c) 2013,山外科技
 *     All rights reserved.
 *     技术讨论：山外论坛 http://www.vcan123.com
 *
 *     除注明出处外，以下所有内容版权均属山外科技所有，未经允许，不得用于商业用途，
 *     修改内容时必须保留山外科技的版权声明。
 *
 * @file       main.c
 * @brief      山外K60 平台主程序
 * @author     山外科技
 * @version    v5.0
 * @date       2013-08-28
 */

#include "common.h"
#include "include.h"
#include "outputdata.h"

#define nrf_speed_debug  0             //决定无线调试是否开启
#define nrf_img_debug    0
#define nrf_yaokongqi    0

/*调用即软件复位,有可能用得上*/
//#define NVIC_SystemReset()    {
//             __DSB();          /* Ensure all outstanding memory accesses includedbuffered write are completed before reset */
//             SCB_AIRCR  = ((0x5FA << SCB_AIRCR_VECTKEY_SHIFT)      |
//             SCB_AIRCR_SYSRESETREQ_MASK);
//             __DSB();                                                     /* Ensure completion of memory access */
//             while(1);                                                    /* wait until reset */
//             }


//摄像头相关变量
uint8 *imgbuff = (uint8 *)(((uint8 *)&nrf_tx_buff) + COM_LEN);
//定义存储接收图像的数组,2400=160*120/8
uint8  nrf_tx_buff[CAMERA_SIZE + 2*COM_LEN + DATA_PACKET];
//uint8  nrf_rx_buff[CAMERA_SIZE + 2*COM_LEN + DATA_PACKET];         //预多   定义浪费资源

//uint8 imgbuff[CAMERA_SIZE];                             //定义存储接收图像的数组

uint8  img[160*120];                                     //解压后的图像
uint16 PIT_count=0;                                     //主中断循环计数，控制时间片
uint8  Duoji_control_cnt=0;
uint32 Moto_speed_out;                              //电机速度的输出值
float  float_speed;                                 //暂时使用
uint32 set_speed;                                   //电机速度的设定值
uint8  temp_count=0;
float  speed_nrf;                             //nrf调试速度时的变量
uint32 speed_nrf_n;
float  img_curvature;                         //图像曲率
uint32 zhenshu_count;
uint8  time_chip_second;                      //开车后的秒数    60秒重置
uint32  qibu_time_second;
uint8  qibu_speed_flag=1;            //这个qibu_flag 为1的时候为起步 所以初始化为1
uint32 camera_zhenshu;
static Site_t  site_zhenshu={128-16,128-16};      //显示摄像头帧数
extern uint8  podao_flag;
static uint16 podao_overlook_count;
uint8  pretect_stop_car_count;             //保护计数，防止误判
uint8  car_begin_count;
uint8  car_stop_count;
uint8  auto_car_flag;        //自动驾驶模式
uint8  hand_car_flag;        //手动驾驶模式
uint8  forward_step;         //手动模式下的四种指令
uint8  back_step;
uint8  left_step;
uint8  right_step;
/*****************************Main***************************/
/*!
 *  @brief      main函数
 *  @since      v5.0
 *  @note       mengde的摄像头主函数
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
	Init_system();                              //系统总初始化
//	WDOG_Init();
    while(1)
    {
        pit_time_start(PIT1);
        camera_get_img();                       //摄像头获取图像,使用的是120*160的采集
        img_extract(img,imgbuff);			    //将摄像头图像解压
        median_line_extract(img);			    //提取摄像头中线
        quan_dir_duty_new();				    //算出每一帧图像的中线权重
        deal_key_event();                       //按键消息处理
        pit1_ms=pit_time_get_ms(PIT1);
        pit_close(PIT1);
//        OutData[0]=speed_current;
//		OutData[1]=speed_duty;
//		OutData[2]=speed_err;
//		OutData[3]=speed_want;
//        OutPut_Data();                      //输出波形
//        WDOG_Feed();
//        printf("%d ",speed_current);
//        vcan_sendimg(imgbuff,120*160/8);

/*******************************NRF调试*******************************/
#if nrf_speed_debug    //无线调试速度
      //底下这些代码稍加修改即可观察速度PID波形，在完成速度闭环之后就可以用了
      //辅组我们调节电机PID
      //现在有一个地方有些问题，就是传输速度过慢，可以考虑将NRF放到中断里，每计算一次
      //速度就可以发送一次，这样可能更加准确一点

//      speed_nrf_n=speed_duty;
//      speed_nrf_n=speed_current;     //发送PWM输出值
      speed_nrf_n=(ABS(se_du)*100)/9500;
//	  speed_nrf_n=(uint32)speed_control_output;
	  sprintf((char *)buff1,"%s%d",str1,speed_nrf_n);       //把str和i合并成一个字符串到buff里，再进行发送
                            //这里NRF无法发送浮点数  原因未知!!!   本来就无法发送浮点数
	  if(nrf_tx(buff1,DATA_PACKET) == 1 )            //发送一个数据包：buff（包为32字节）
      {
            //等待发送过程中，此处可以加入处理任务
            while(nrf_tx_state() == NRF_TXING);         //等待发送完成
      }
#endif

#if nrf_img_debug     //无线调试速度
        /*********************** 无线发送图像 ***********************/
        nrf_msg_tx(COM_IMG,nrf_tx_buff);
        while(nrf_tx_state() == NRF_TXING);             //等待发送完成
#endif
/*遥控器调试发车与停车*/
#if nrf_yaokongqi
	 relen = nrf_rx(buff,DATA_PACKET);
	 if(relen != 0)
	 {
         if(buff[0]=='1'&&buff[1]=='1')     //11代表开车
         {
			 speed_want=flash_speed_want;
			 Speed_protect=0;
			 enable_irq(PIT2_IRQn);
		 }
         if(buff[0]=='2'&&buff[1]=='2')    //22代表停车
         {
			 speed_want=0;
			 Speed_protect=1;
		 }
	 }
#endif
/******************************NRF调试END*****************************/
    }
}
/*************************END OF MAIN************************/


/*!
 *  @brief      UART3中断服务函数
 *  @since      v5.0
 */
void uart3_handler(void)
{
    char ch;
    if(uart_query    (UART3) == 1)   //接收数据寄存器满    //'\v'    '\0'
    {
        //用户需要处理接收数据
        uart_getchar   (UART3, &ch);                    //无限等待接受1个字节
        if(ch==0x01)                  //前
        {
            car_begin_count++;
			if(car_begin_count>=1&&auto_car_flag==0&&hand_car_flag==0)  //第一次发车
			{
				Speed_protect=0;				 //发车
				auto_car_flag=1;
				car_begin_count=0;
				enable_irq(PIT2_IRQn);
			}
			if(car_begin_count>=1&&hand_car_flag==1)
			{
                forward_step=1;
				if(car_begin_count>=3)   //连续三次按前则表示要进入自动模式
				{
				    auto_car_flag=1;
                    hand_car_flag=0;
					Speed_protect=0;
					car_begin_count=0;
				}
			}
		}
		else if(ch==0x02)           //后
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
               Speed_protect=1;                  //停车   并转入手动模式
               car_stop_count=0;
               hand_car_flag=1;
			   auto_car_flag=0;
			}
			else
			{}
		}
		else if(ch==0x03)          //左
		{
		    car_begin_count=0;    //只要按了一次方向键则清零
	        if(hand_car_flag==1)
                left_step=1;
		}
		else if(ch==0x04)         //右
		{
		    car_begin_count=0;   //只要按了一次方向键则清零
			if(hand_car_flag==1)
                right_step=1;
		}
		else
		{}

    }

}


void PIT2_IRQHandler()        //主要中断循环
{
    zhenshu_count++;
	PIT_count++;               //以PIT_count为时间片  周期为5ms
    qibu_time_second++;
	if(qibu_time_second>=1500)
	{
        qibu_time_second=0;
	}

    if(zhenshu_count==1000)    //1秒钟显示一次，即为当前帧数
    {
        LCD_num_BC(site_zhenshu,camera_zhenshu,2,BLUE,RED);    //在右下角显示摄像头帧数
        camera_zhenshu=0;
        zhenshu_count=0;
		time_chip_second++;
		if(time_chip_second>=60)
			time_chip_second=0;
    }
	if(PIT_count==2)
	{
        GetMotoPulse();         //获取编码器脉冲
	}
	else if(PIT_count==3)
	{
        SpeedControl();         //算出速度控制量
	}
	else if(PIT_count==4)
	{
        SpeedControlOutput();   //速度输出
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
		PIT_count=0;             //清空PIT_count
	}
	else if(PIT_count==1)                //先控制舵机
	{
        Duoji_control_cnt++;
		if(Duoji_control_cnt==2)                //10ms控制一次舵机  跟while循环周期更相近一点   现在每一次
		{
            DJ_PID();
			Duoji_control_cnt=0;
		}
	}
	PIT_Flag_Clear(PIT2);
}

void PIT0_IRQHandler()      //按键扫描中断
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
		    begin_the_frost_point=1;      //开始检测最上方的点
            out_circle_time_flag=0;
			out_circle_count=0;
		}
	}

    key_IRQHandler();
    PIT_Flag_Clear(PIT0);
}


/*!
 *  @brief      DMA0中断服务函数
 *  @since      v5.0
 */
void DMA0_IRQHandler()
{
    camera_dma();
}
/*!
 *  @brief      PORTA中断服务函数
 *  @since      v5.0
 */
void PORTA_IRQHandler()
{
    uint8  n;    //引脚号
    uint32 flag;

    while(!PORTA_ISFR);
    flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //清中断标志位

    n = 29;                                             //场中断
    if(flag & (1 << n))                                 //PTA29触发中断
    {
        camera_vsync();
    }
#if ( CAMERA_USE_HREF == 1 )                            //使用行中断
    n = 28;
    if(flag & (1 << n))                                 //PTA28触发中断
    {
        camera_href();
    }
#endif

}

/*!
 *  @brief      PORTE中断服务函数
 *  @since      v5.0
 */
void PORTE_IRQHandler()                          //用于无线收发用 调试时使用
{
    uint8  n;    //引脚号
    uint32 flag;

    flag = PORTE_ISFR;
    PORTE_ISFR  = ~0;                                   //清中断标志位

    n = 27;
    if(flag & (1 << n))                                 //PTE27触发中断
    {
        nrf_handler();
    }
}
/*看门狗函数*/
void WDOG_Init(void)
{
    WDOG_UNLOCK = 0xC520;
    WDOG_UNLOCK = 0xD928;               //解锁看门狗寄存器（重新写入前必须解锁）两次写入不能超过20个时钟周期

    WDOG_PRESC = 0x0700;                //（7+1）分频

    WDOG_TOVALH = 0x0110;              //看门狗超时时间设置为3s
    WDOG_TOVALL = 0x5E90;
}

void WDOG_Feed(void)
{
    WDOG_REFRESH = 0xA602;
    WDOG_REFRESH = 0xB480;            //喂狗
}
void WDOG_Close(void)
{
    WDOG_UNLOCK = 0xC520u;
    WDOG_UNLOCK = 0xD928u;
    WDOG_STCTRLH &= ~0x0001;          //关闭看门狗 WDOTEN 0
}
void WDOG_Open(void)
{
    WDOG_UNLOCK = 0xC520;
    WDOG_UNLOCK = 0xD928;
    WDOG_STCTRLH |= 0x0001;          //打开看门狗 WDOTEN 1
}










