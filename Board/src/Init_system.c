#include "include.h"
#include "common.h"
#include "VCAN_key_event.h"
uint32 KEY_text;
/*切记参数里的图像偏移量一定要临场修改，一定不要忘记!!!!!!!!!!!!不然图像都是歪的怎么跑*/
float key_num_init1[16]={126.00,12.00,520.0,250.00,25.00,8.00,120.00,9.50,128,14,100,50,0,0,0,0};      //拨码开关控制三档速度 开关1 2 3
float key_num_init2[16]={125.00,11.50,520.0,250.00,25.00,8.00,108.00,8.00,125,14,90,45,0,0,0,0};
float key_num_init3[16]={250.00,18.00,600.0,250.00,25.00,8.00,170.00,8.00,90,17,90,100,0,0,0,0};
float key_num_init4[16]={130.00,12.50,560.0,250.00,22.00,8.00,170.00,8.00,130,14,105,50,0,0,0,0};
//                                                                        |      |  |-->加减速系数
//                                                                        |      |-->最低速度
//                                                                        |-->期望速度
/*比赛时四个拨码开关档位控制说明
**BM1** 高速挡-->冲一冲 能够在实验室跑到2.8~2.9的速度  这个速度过连续大S过不去
**BM2** 中速挡-->稳一稳 在工大的西部决赛赛道跑到2.76的速度 过弯速度比较小和加减速系数特别大  过弯好一点，比较循迹，D值比较小
**BM3** 低速挡-->保一保 力求在绝境的时候能够跑下来，尽量尽量不要用它  综合速度才2.6m左右    //最低只能是这个了，不能再低了,再低一些就无价值了
**BM4** 国赛冲刺挡 速度比较高
//拨码开关说明
拨码开关拨下来时每次开机都会写入预定参数，而当拨码开关拨上去的时候则就变成了普通调
试，参数可以修改，并且写入flash存储
比赛这几天，一定要适应这种拨码开关方式  不要有病乱拨开关!!!!!!!!!!!!!!!!!!!!!!!
*/
//1.一挡 二挡期望速度绝对不能低于120!!!!!!!!!!!!!!!!   不然就会进入另一个程序
//2.每一挡的Kp2一定记得加0.5!!!!!!!!!!!!!!别忘记!!!!!!!!!!
//3.图像偏移量别忘了修改
//4.每换一个档位需要拨两次，千万别忘了!!!!!!
/******************************************************************************
* Function Name: init();
* Input Param  : 无
* Output Param : 无
* Description  : 系统初始化
* Author Data  : 2015.11.28
******************************************************************************/
void Init_system()
{
#if defined(SHANWAI)
	/*FTM模块初始化*/		 //放到前面，防止疯转
	ftm_pwm_init(FTM0, FTM_CH0,10000,0);	//电机正     5000HZ加减速比2000效果好得多
	ftm_pwm_init(FTM0, FTM_CH1,10000,0);	//电机反
	ftm_pwm_init(FTM1, FTM_CH1,300,49500); 	//初始化舵机，初始时打角在中间//49100     //旧车48600
	ftm_quad_init(FTM2);					//FTM2 正交解码初始化
	/*Flash初始化*/
	flash_init();
	DELAY_MS(100);
	/*nrf无线调试*/
//	nrf_init();
//    nrf_msg_init();                      //无线模块消息初始化
	/*I/O初始化*/
	gpio_init (PTE8, GPI,1);	//初始化 拨码开关1
	gpio_init (PTE10, GPI,1);	//初始化 拨码开关2
	gpio_init (PTE11, GPI,1);	//初始化 拨码开关3
	gpio_init (PTE12, GPI,1);	//初始化 拨码开关4
	gpio_init (PTD4, GPO,0);	//蜂鸣器   //高电平有效 由硬件决定
	/*各模块功初始化*/
	camera_init(imgbuff);					//初始化摄像头
    LCD_init();                             //LCD初始化要放到按键消息处理前面，因为按键消息要在小液晶里显示信息
	key_event_init();						//按键消息初始化   按键消息要尽早初始化，里面涉及到很多flash变量的读取问题
	led_init(LED_MAX); 					    //LCD初始化
	BM_key();                              //拨码开关程序
	DELAY_MS(10);
	if(BM1==1)
		init_flash_write1();
	if(BM2==1)
		init_flash_write2();
	if(BM3==1)
		init_flash_write3();
	if(BM4==1)
		init_flash_write4();
//	adc_init(ADC1_SE14);              //ADC初始化   测量电池电压
	/*中断优先级配置*/
	NVIC_SetPriorityGrouping(5);			//设置优先级分组,4bit 抢占优先级,没有亚优先级
	NVIC_SetPriority(PORTA_IRQn,0); 		//配置优先级
	NVIC_SetPriority(DMA0_IRQn,1);			//配置优先级
	NVIC_SetPriority(PIT2_IRQn,2);			//配置优先级
	NVIC_SetPriority(PIT0_IRQn,3);			//配置优先级
	NVIC_SetPriority(UART3_RX_TX_IRQn,4);			//配置优先级
	NVIC_SetPriority(PORTE_IRQn,5);         //无线调试 	   //经常无线调试是关闭的
	pit_init_ms(PIT2,1);					//pit1 定时中断 1ms中断，主中断
	pit_init_ms(PIT0,10);					//pit0 定时中断(用于按键定时扫描)
	//配置中断服务函数
	set_vector_handler(PORTA_VECTORn , PORTA_IRQHandler);	//设置 PORTA 的中断服务函数为 PORTA_IRQHandler
	set_vector_handler(DMA0_VECTORn , DMA0_IRQHandler); 	//设置 DMA0 的中断服务函数为 PORTA_IRQHandler
	set_vector_handler(PIT0_VECTORn , PIT0_IRQHandler); 	//设置 PIT0 的中断复位函数为 PIT0_IRQHandler
	set_vector_handler(PIT2_VECTORn , PIT2_IRQHandler); 	//设置 PIT2 的中断复位函数为 PIT1_IRQHandler
	set_vector_handler(PORTE_VECTORn ,PORTE_IRQHandler);	//设置 PORTE 的中断服务函数为 PORTE_VECTORn
	set_vector_handler(UART3_RX_TX_VECTORn ,uart3_handler);         //设置串口接收中断
	/*使能中断*/
	enable_irq(PORTE_IRQn); 		  //仅仅调试时使用
	enable_irq(PIT0_IRQn); 
	uart_rx_irq_en (UART3);     //4s之后才打开
//	enable_irq(PIT2_IRQn);



#elif defined(LONGQIU)
	/*FTM模块初始化*/		 //放到前面，防止疯转
	ftm_pwm_init(FTM1, FTM_CH0,5000,0);	//电机正     5000HZ加减速比2000效果好得多
	ftm_pwm_init(FTM1, FTM_CH1,5000,0);	//电机反
	ftm_pwm_init(FTM0, FTM_CH4,100,14500); 	//初始化舵机，初始时打角在中间
	ftm_quad_init(FTM2);					//FTM2 正交解码初始化
	/*Flash初始化*/
	flash_init();
	DELAY_MS(150);
	/*I/O初始化*/
	gpio_init (PTC3, GPI,0);	//初始化 拨码开关1
	gpio_init (PTC1, GPI,0);	//初始化 拨码开关2
	gpio_init (PTB23, GPI,0);	//初始化 拨码开关3
	gpio_init (PTB21, GPI,0);	//初始化 拨码开关4
	gpio_init (PTE11, GPO,0);	//蜂鸣器   //低电平有效 由硬件决定
	/*各模块功初始化*/
	camera_init(imgbuff);					//初始化摄像头
    LCD_init();                             //LCD初始化要放到按键消息处理前面，因为按键消息要在小液晶里显示信息
	key_event_init();						//按键消息初始化   按键消息要尽早初始化，里面涉及到很多flash变量的读取问题
	led_init(LED_MAX); 							//LCD初始化
	/*中断优先级配置*/
	NVIC_SetPriorityGrouping(4);			//设置优先级分组,4bit 抢占优先级,没有亚优先级
	NVIC_SetPriority(PORTA_IRQn,0); 		//配置优先级
	NVIC_SetPriority(DMA0_IRQn,1);			//配置优先级
	NVIC_SetPriority(PIT2_IRQn,2);			//配置优先级
	NVIC_SetPriority(PIT0_IRQn,3);			//配置优先级
//	NVIC_SetPriority(PORTE_IRQn,4);  //无线调试 	   //经常无线调试是关闭的
	pit_init_ms(PIT2,1);					//pit1 定时中断 1ms中断，主中断
	pit_init_ms(PIT0,10);					//pit0 定时中断(用于按键定时扫描)
	//配置中断服务函数
	set_vector_handler(PORTA_VECTORn , PORTA_IRQHandler);	//设置 PORTA 的中断服务函数为 PORTA_IRQHandler
	set_vector_handler(DMA0_VECTORn , DMA0_IRQHandler); 	//设置 DMA0 的中断服务函数为 PORTA_IRQHandler
	set_vector_handler(PIT0_VECTORn , PIT0_IRQHandler); 	//设置 PIT0 的中断复位函数为 PIT0_IRQHandler
	set_vector_handler(PIT2_VECTORn , PIT2_IRQHandler); 	//设置 PIT2 的中断复位函数为 PIT1_IRQHandler
//	set_vector_handler(PORTE_VECTORn ,PORTE_IRQHandler);				//设置 PORTE 的中断服务函数为 PORTE_VECTORn
//	enable_irq(PORTE_IRQn); 		  //仅仅调试时使用
	enable_irq(PIT2_IRQn);
	enable_irq(PIT0_IRQn);
#endif
}
/*
**拨码开关程序
*/
void BM_key()
{
  if(gpio_get (PTE8)==0)
  {
    BM1=1;
  }
  if(gpio_get (PTE11)==0)
  {
    BM2=1;
  }
  if(gpio_get (PTE10)==0)
  {
    BM3=1;
  }
  if(gpio_get (PTE12)==0)
  {
    BM4=1;
  }
}
void init_flash_write1()              //上电的时候写入flash 写这个是因为flash数据一老掉 自动批量写入比较好
{
    //数据  168.89 8.00 10.00 50.00 8.00 2.00 170.00 8.00
    //      85 24 0
    int i;
	uint8 flash_offset;
	DisableInterrupts;
	for(i=0;i<fv_MAX;i++)
    {
       flash_offset=FLASH_SECTOR_NUM-1-(uint8)fv_MAX*(data_screen)+i;
	   if(i<8)
	   {
		   flash_erase_sector(flash_offset);					 //擦除扇区
		   flash_write(flash_offset, 0,(uint16)key_num_init1[i]*10 );   //写入数据到扇区，偏移地址为0，必须一次写入4字节
	   }
       else
       {
		   flash_erase_sector(flash_offset);					 //擦除扇区
		   flash_write(flash_offset, 0, (uint16) (key_num_init1[i]) );   //写入数据到扇区
	   }
    }
    EnableInterrupts;
}
void init_flash_write2()              //上电的时候写入flash 写这个是因为flash数据一老掉 自动批量写入比较好
{
    //数据  168.89 8.00 10.00 50.00 8.00 2.00 170.00 8.00
    //      85 24 0
    int i;
	uint8 flash_offset;
	DisableInterrupts;
	for(i=0;i<fv_MAX;i++)
    {
       flash_offset=FLASH_SECTOR_NUM-1-(uint8)fv_MAX*(data_screen)+i;
	   if(i<8)
	   {
		   flash_erase_sector(flash_offset);					 //擦除扇区
		   flash_write(flash_offset, 0,(uint16)key_num_init2[i]*10 );   //写入数据到扇区，偏移地址为0，必须一次写入4字节
	   }
       else
       {
		   flash_erase_sector(flash_offset);					 //擦除扇区
		   flash_write(flash_offset, 0, (uint16) (key_num_init2[i]) );   //写入数据到扇区
	   }
    }
    EnableInterrupts;
}
void init_flash_write3()              //上电的时候写入flash 写这个是因为flash数据一老掉 自动批量写入比较好
{
    //数据  168.89 8.00 10.00 50.00 8.00 2.00 170.00 8.00
    //      85 24 0
    int i;
	uint8 flash_offset;
	DisableInterrupts;
	for(i=0;i<fv_MAX;i++)
    {
       flash_offset=FLASH_SECTOR_NUM-1-(uint8)fv_MAX*(data_screen)+i;
	   if(i<8)
	   {
		   flash_erase_sector(flash_offset);					 //擦除扇区
		   flash_write(flash_offset, 0,(uint16)key_num_init3[i]*10 );   //写入数据到扇区，偏移地址为0，必须一次写入4字节
	   }
       else
       {
		   flash_erase_sector(flash_offset);					 //擦除扇区
		   flash_write(flash_offset, 0, (uint16) (key_num_init3[i]) );   //写入数据到扇区
	   }
    }
    EnableInterrupts;
}

void init_flash_write4()              //上电的时候写入flash 写这个是因为flash数据一老掉 自动批量写入比较好
{
    //数据  168.89 8.00 10.00 50.00 8.00 2.00 170.00 8.00
    //      85 24 0
    int i;
	uint8 flash_offset;
	DisableInterrupts;
	for(i=0;i<fv_MAX;i++)
    {
       flash_offset=FLASH_SECTOR_NUM-1-(uint8)fv_MAX*(data_screen)+i;
	   if(i<8)
	   {
		   flash_erase_sector(flash_offset);					 //擦除扇区
		   flash_write(flash_offset, 0,(uint16)key_num_init4[i]*10 );   //写入数据到扇区，偏移地址为0，必须一次写入4字节
	   }
       else
       {
		   flash_erase_sector(flash_offset);					 //擦除扇区
		   flash_write(flash_offset, 0, (uint16) (key_num_init4[i]) );   //写入数据到扇区
	   }
    }
    EnableInterrupts;
}





