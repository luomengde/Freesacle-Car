#include "include.h"
#include "common.h"
#include "VCAN_key_event.h"
uint32 KEY_text;
/*�мǲ������ͼ��ƫ����һ��Ҫ�ٳ��޸ģ�һ����Ҫ����!!!!!!!!!!!!��Ȼͼ���������ô��*/
float key_num_init1[16]={126.00,12.00,520.0,250.00,25.00,8.00,120.00,9.50,128,14,100,50,0,0,0,0};      //���뿪�ؿ��������ٶ� ����1 2 3
float key_num_init2[16]={125.00,11.50,520.0,250.00,25.00,8.00,108.00,8.00,125,14,90,45,0,0,0,0};
float key_num_init3[16]={250.00,18.00,600.0,250.00,25.00,8.00,170.00,8.00,90,17,90,100,0,0,0,0};
float key_num_init4[16]={130.00,12.50,560.0,250.00,22.00,8.00,170.00,8.00,130,14,105,50,0,0,0,0};
//                                                                        |      |  |-->�Ӽ���ϵ��
//                                                                        |      |-->����ٶ�
//                                                                        |-->�����ٶ�
/*����ʱ�ĸ����뿪�ص�λ����˵��
**BM1** ���ٵ�-->��һ�� �ܹ���ʵ�����ܵ�2.8~2.9���ٶ�  ����ٶȹ�������S����ȥ
**BM2** ���ٵ�-->��һ�� �ڹ�����������������ܵ�2.76���ٶ� �����ٶȱȽ�С�ͼӼ���ϵ���ر��  �����һ�㣬�Ƚ�ѭ����Dֵ�Ƚ�С
**BM3** ���ٵ�-->��һ�� �����ھ�����ʱ���ܹ�������������������Ҫ����  �ۺ��ٶȲ�2.6m����    //���ֻ��������ˣ������ٵ���,�ٵ�һЩ���޼�ֵ��
**BM4** ������̵� �ٶȱȽϸ�
//���뿪��˵��
���뿪�ز�����ʱÿ�ο�������д��Ԥ���������������뿪�ز���ȥ��ʱ����ͱ������ͨ��
�ԣ����������޸ģ�����д��flash�洢
�����⼸�죬һ��Ҫ��Ӧ���ֲ��뿪�ط�ʽ  ��Ҫ�в��Ҳ�����!!!!!!!!!!!!!!!!!!!!!!!
*/
//1.һ�� ���������ٶȾ��Բ��ܵ���120!!!!!!!!!!!!!!!!   ��Ȼ�ͻ������һ������
//2.ÿһ����Kp2һ���ǵü�0.5!!!!!!!!!!!!!!������!!!!!!!!!!
//3.ͼ��ƫ�����������޸�
//4.ÿ��һ����λ��Ҫ�����Σ�ǧ�������!!!!!!
/******************************************************************************
* Function Name: init();
* Input Param  : ��
* Output Param : ��
* Description  : ϵͳ��ʼ��
* Author Data  : 2015.11.28
******************************************************************************/
void Init_system()
{
#if defined(SHANWAI)
	/*FTMģ���ʼ��*/		 //�ŵ�ǰ�棬��ֹ��ת
	ftm_pwm_init(FTM0, FTM_CH0,10000,0);	//�����     5000HZ�Ӽ��ٱ�2000Ч���õö�
	ftm_pwm_init(FTM0, FTM_CH1,10000,0);	//�����
	ftm_pwm_init(FTM1, FTM_CH1,300,49500); 	//��ʼ���������ʼʱ������м�//49100     //�ɳ�48600
	ftm_quad_init(FTM2);					//FTM2 ���������ʼ��
	/*Flash��ʼ��*/
	flash_init();
	DELAY_MS(100);
	/*nrf���ߵ���*/
//	nrf_init();
//    nrf_msg_init();                      //����ģ����Ϣ��ʼ��
	/*I/O��ʼ��*/
	gpio_init (PTE8, GPI,1);	//��ʼ�� ���뿪��1
	gpio_init (PTE10, GPI,1);	//��ʼ�� ���뿪��2
	gpio_init (PTE11, GPI,1);	//��ʼ�� ���뿪��3
	gpio_init (PTE12, GPI,1);	//��ʼ�� ���뿪��4
	gpio_init (PTD4, GPO,0);	//������   //�ߵ�ƽ��Ч ��Ӳ������
	/*��ģ�鹦��ʼ��*/
	camera_init(imgbuff);					//��ʼ������ͷ
    LCD_init();                             //LCD��ʼ��Ҫ�ŵ�������Ϣ����ǰ�棬��Ϊ������ϢҪ��СҺ������ʾ��Ϣ
	key_event_init();						//������Ϣ��ʼ��   ������ϢҪ�����ʼ���������漰���ܶ�flash�����Ķ�ȡ����
	led_init(LED_MAX); 					    //LCD��ʼ��
	BM_key();                              //���뿪�س���
	DELAY_MS(10);
	if(BM1==1)
		init_flash_write1();
	if(BM2==1)
		init_flash_write2();
	if(BM3==1)
		init_flash_write3();
	if(BM4==1)
		init_flash_write4();
//	adc_init(ADC1_SE14);              //ADC��ʼ��   ������ص�ѹ
	/*�ж����ȼ�����*/
	NVIC_SetPriorityGrouping(5);			//�������ȼ�����,4bit ��ռ���ȼ�,û�������ȼ�
	NVIC_SetPriority(PORTA_IRQn,0); 		//�������ȼ�
	NVIC_SetPriority(DMA0_IRQn,1);			//�������ȼ�
	NVIC_SetPriority(PIT2_IRQn,2);			//�������ȼ�
	NVIC_SetPriority(PIT0_IRQn,3);			//�������ȼ�
	NVIC_SetPriority(UART3_RX_TX_IRQn,4);			//�������ȼ�
	NVIC_SetPriority(PORTE_IRQn,5);         //���ߵ��� 	   //�������ߵ����ǹرյ�
	pit_init_ms(PIT2,1);					//pit1 ��ʱ�ж� 1ms�жϣ����ж�
	pit_init_ms(PIT0,10);					//pit0 ��ʱ�ж�(���ڰ�����ʱɨ��)
	//�����жϷ�����
	set_vector_handler(PORTA_VECTORn , PORTA_IRQHandler);	//���� PORTA ���жϷ�����Ϊ PORTA_IRQHandler
	set_vector_handler(DMA0_VECTORn , DMA0_IRQHandler); 	//���� DMA0 ���жϷ�����Ϊ PORTA_IRQHandler
	set_vector_handler(PIT0_VECTORn , PIT0_IRQHandler); 	//���� PIT0 ���жϸ�λ����Ϊ PIT0_IRQHandler
	set_vector_handler(PIT2_VECTORn , PIT2_IRQHandler); 	//���� PIT2 ���жϸ�λ����Ϊ PIT1_IRQHandler
	set_vector_handler(PORTE_VECTORn ,PORTE_IRQHandler);	//���� PORTE ���жϷ�����Ϊ PORTE_VECTORn
	set_vector_handler(UART3_RX_TX_VECTORn ,uart3_handler);         //���ô��ڽ����ж�
	/*ʹ���ж�*/
	enable_irq(PORTE_IRQn); 		  //��������ʱʹ��
	enable_irq(PIT0_IRQn); 
	uart_rx_irq_en (UART3);     //4s֮��Ŵ�
//	enable_irq(PIT2_IRQn);



#elif defined(LONGQIU)
	/*FTMģ���ʼ��*/		 //�ŵ�ǰ�棬��ֹ��ת
	ftm_pwm_init(FTM1, FTM_CH0,5000,0);	//�����     5000HZ�Ӽ��ٱ�2000Ч���õö�
	ftm_pwm_init(FTM1, FTM_CH1,5000,0);	//�����
	ftm_pwm_init(FTM0, FTM_CH4,100,14500); 	//��ʼ���������ʼʱ������м�
	ftm_quad_init(FTM2);					//FTM2 ���������ʼ��
	/*Flash��ʼ��*/
	flash_init();
	DELAY_MS(150);
	/*I/O��ʼ��*/
	gpio_init (PTC3, GPI,0);	//��ʼ�� ���뿪��1
	gpio_init (PTC1, GPI,0);	//��ʼ�� ���뿪��2
	gpio_init (PTB23, GPI,0);	//��ʼ�� ���뿪��3
	gpio_init (PTB21, GPI,0);	//��ʼ�� ���뿪��4
	gpio_init (PTE11, GPO,0);	//������   //�͵�ƽ��Ч ��Ӳ������
	/*��ģ�鹦��ʼ��*/
	camera_init(imgbuff);					//��ʼ������ͷ
    LCD_init();                             //LCD��ʼ��Ҫ�ŵ�������Ϣ����ǰ�棬��Ϊ������ϢҪ��СҺ������ʾ��Ϣ
	key_event_init();						//������Ϣ��ʼ��   ������ϢҪ�����ʼ���������漰���ܶ�flash�����Ķ�ȡ����
	led_init(LED_MAX); 							//LCD��ʼ��
	/*�ж����ȼ�����*/
	NVIC_SetPriorityGrouping(4);			//�������ȼ�����,4bit ��ռ���ȼ�,û�������ȼ�
	NVIC_SetPriority(PORTA_IRQn,0); 		//�������ȼ�
	NVIC_SetPriority(DMA0_IRQn,1);			//�������ȼ�
	NVIC_SetPriority(PIT2_IRQn,2);			//�������ȼ�
	NVIC_SetPriority(PIT0_IRQn,3);			//�������ȼ�
//	NVIC_SetPriority(PORTE_IRQn,4);  //���ߵ��� 	   //�������ߵ����ǹرյ�
	pit_init_ms(PIT2,1);					//pit1 ��ʱ�ж� 1ms�жϣ����ж�
	pit_init_ms(PIT0,10);					//pit0 ��ʱ�ж�(���ڰ�����ʱɨ��)
	//�����жϷ�����
	set_vector_handler(PORTA_VECTORn , PORTA_IRQHandler);	//���� PORTA ���жϷ�����Ϊ PORTA_IRQHandler
	set_vector_handler(DMA0_VECTORn , DMA0_IRQHandler); 	//���� DMA0 ���жϷ�����Ϊ PORTA_IRQHandler
	set_vector_handler(PIT0_VECTORn , PIT0_IRQHandler); 	//���� PIT0 ���жϸ�λ����Ϊ PIT0_IRQHandler
	set_vector_handler(PIT2_VECTORn , PIT2_IRQHandler); 	//���� PIT2 ���жϸ�λ����Ϊ PIT1_IRQHandler
//	set_vector_handler(PORTE_VECTORn ,PORTE_IRQHandler);				//���� PORTE ���жϷ�����Ϊ PORTE_VECTORn
//	enable_irq(PORTE_IRQn); 		  //��������ʱʹ��
	enable_irq(PIT2_IRQn);
	enable_irq(PIT0_IRQn);
#endif
}
/*
**���뿪�س���
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
void init_flash_write1()              //�ϵ��ʱ��д��flash д�������Ϊflash����һ�ϵ� �Զ�����д��ȽϺ�
{
    //����  168.89 8.00 10.00 50.00 8.00 2.00 170.00 8.00
    //      85 24 0
    int i;
	uint8 flash_offset;
	DisableInterrupts;
	for(i=0;i<fv_MAX;i++)
    {
       flash_offset=FLASH_SECTOR_NUM-1-(uint8)fv_MAX*(data_screen)+i;
	   if(i<8)
	   {
		   flash_erase_sector(flash_offset);					 //��������
		   flash_write(flash_offset, 0,(uint16)key_num_init1[i]*10 );   //д�����ݵ�������ƫ�Ƶ�ַΪ0������һ��д��4�ֽ�
	   }
       else
       {
		   flash_erase_sector(flash_offset);					 //��������
		   flash_write(flash_offset, 0, (uint16) (key_num_init1[i]) );   //д�����ݵ�����
	   }
    }
    EnableInterrupts;
}
void init_flash_write2()              //�ϵ��ʱ��д��flash д�������Ϊflash����һ�ϵ� �Զ�����д��ȽϺ�
{
    //����  168.89 8.00 10.00 50.00 8.00 2.00 170.00 8.00
    //      85 24 0
    int i;
	uint8 flash_offset;
	DisableInterrupts;
	for(i=0;i<fv_MAX;i++)
    {
       flash_offset=FLASH_SECTOR_NUM-1-(uint8)fv_MAX*(data_screen)+i;
	   if(i<8)
	   {
		   flash_erase_sector(flash_offset);					 //��������
		   flash_write(flash_offset, 0,(uint16)key_num_init2[i]*10 );   //д�����ݵ�������ƫ�Ƶ�ַΪ0������һ��д��4�ֽ�
	   }
       else
       {
		   flash_erase_sector(flash_offset);					 //��������
		   flash_write(flash_offset, 0, (uint16) (key_num_init2[i]) );   //д�����ݵ�����
	   }
    }
    EnableInterrupts;
}
void init_flash_write3()              //�ϵ��ʱ��д��flash д�������Ϊflash����һ�ϵ� �Զ�����д��ȽϺ�
{
    //����  168.89 8.00 10.00 50.00 8.00 2.00 170.00 8.00
    //      85 24 0
    int i;
	uint8 flash_offset;
	DisableInterrupts;
	for(i=0;i<fv_MAX;i++)
    {
       flash_offset=FLASH_SECTOR_NUM-1-(uint8)fv_MAX*(data_screen)+i;
	   if(i<8)
	   {
		   flash_erase_sector(flash_offset);					 //��������
		   flash_write(flash_offset, 0,(uint16)key_num_init3[i]*10 );   //д�����ݵ�������ƫ�Ƶ�ַΪ0������һ��д��4�ֽ�
	   }
       else
       {
		   flash_erase_sector(flash_offset);					 //��������
		   flash_write(flash_offset, 0, (uint16) (key_num_init3[i]) );   //д�����ݵ�����
	   }
    }
    EnableInterrupts;
}

void init_flash_write4()              //�ϵ��ʱ��д��flash д�������Ϊflash����һ�ϵ� �Զ�����д��ȽϺ�
{
    //����  168.89 8.00 10.00 50.00 8.00 2.00 170.00 8.00
    //      85 24 0
    int i;
	uint8 flash_offset;
	DisableInterrupts;
	for(i=0;i<fv_MAX;i++)
    {
       flash_offset=FLASH_SECTOR_NUM-1-(uint8)fv_MAX*(data_screen)+i;
	   if(i<8)
	   {
		   flash_erase_sector(flash_offset);					 //��������
		   flash_write(flash_offset, 0,(uint16)key_num_init4[i]*10 );   //д�����ݵ�������ƫ�Ƶ�ַΪ0������һ��д��4�ֽ�
	   }
       else
       {
		   flash_erase_sector(flash_offset);					 //��������
		   flash_write(flash_offset, 0, (uint16) (key_num_init4[i]) );   //д�����ݵ�����
	   }
    }
    EnableInterrupts;
}





