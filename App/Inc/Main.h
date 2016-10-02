#ifndef __MAIN_H
#define __MAIN_H

extern  uint8  *imgbuff ;                                                   //定义存储接收图像的数组,2400=160*120/8
extern  uint8  nrf_tx_buff[];         
extern  uint8  img[160*120]; 									//解压后的图像
extern  uint32 camera_zhenshu;
extern  uint8  time_chip_second;
extern  float  float_speed;
extern  uint32 set_speed; 
extern  uint32 Moto_speed_out;
extern  float  img_curvature;
extern  uint32  qibu_time_second;
extern  uint8  qibu_speed_flag;
extern  float  var_power;

extern  uint8  auto_car_flag;      
extern  uint8  hand_car_flag;       
extern  uint8  forward_step;        
extern  uint8  back_step;
extern  uint8  left_step;
extern  uint8  right_step;

extern  void PORTA_IRQHandler();
extern  void DMA0_IRQHandler();
extern  void PIT2_IRQHandler();
extern  void PIT0_IRQHandler();
extern  void PORTE_IRQHandler();
extern  void uart3_handler(void);

extern  void WDOG_Init(void);     //看门狗函数
extern  void WDOG_Feed(void);
extern  void WDOG_Close(void);
extern  void WDOG_Open(void);






















#endif //__MAIN_H

