#include "common.h"
#include "include.h"
#include "VCAN_LCD.h"
#include "VCAN_UI_VAR.h"
#include "LCD_BMP.h"

/*  按键参数  */
#define VAR_VALUE(var_tab)      num_info[var_tab].val       //指定标号的变量的值
#define VAR_OLDVALUE(var_tab)   num_info[var_tab].oldval    //指定标号的变量的最后确认值
#define VAR_MAXVALUE(var_tab)   num_info[var_tab].maxval
#define VAR_MINVALUE(var_tab)   num_info[var_tab].minval
#define VAR_SITE(var_tab)       num_info[var_tab].site

/*  flash参数  */
#define FV_VALUE(var_tab)       fv_info[var_tab].val       //指定标号的变量的值
#define FV_OLDVALUE(var_tab)    fv_info[var_tab].oldval    //指定标号的变量的最后确认值
#define FV_MAXVALUE(var_tab)    fv_info[var_tab].maxval
#define FV_MINVALUE(var_tab)    fv_info[var_tab].minval
#define FV_SITE(var_tab)        fv_info[var_tab].site

#define  STR_VALUE(var_tab)      str_info[var_tab].str       //指定标号的值
#define  STR_SITE(var_tab)       str_info[var_tab].site


#define  mode_camera(x)		(x <= var_MAX)
#define  mode_data(x)		(x > var_MAX)
#define  mode_judge(x,y)         x==1?mode_camera(y):mode_data(y)

extern display_screen mode;
extern const uint16 *bmp[];
extern void LCD_Img_RGB565_Z(Site_t site,Size_t size,const uint16 *img,Size_t imgsize);
extern void LCD_float(Site_t site, float num, uint16 Color, uint16 bkColor);


float flash_duoji_Kp1,flash_duoji_Kp2,flash_duoji_Kp1_you,flash_duoji_Kp2_you,flash_duoji_Kd;        //舵机PD参数
float flash_dianji_Kp,flash_dianji_Ki,flash_dianji_Kd;       //电机PID参数
uint32 flash_speed_want;
uint32 flash_speed_change,flash_speed_change_error;   //两种加减速的方式
uint32 flash_camera_boundary;
uint32 flash_yuzhi;
uint32 flash_speed_kvff,flash_speed_kaff;
//测试变量
uint16 i1=1,i2=2,i3=3,i4=4,i5=5,i6=6,i7=7,i8=8,i9=9,i10=10,i11=11,i12=12,i13=13,i14=14,i15=15,i16=16;
float f1=1.1,f2=2.2,f3=3.3,f4=4.4,f5=5.5,f6=6.6,f7=7.7,f8=8.8,f9=9.9,f10=10.10,f11=11.11,f12=12.12,f13=13.13,f14=14.4,f15=15.5,f16=16.6;

float *fv_addr2[fv_MAX] = {(float *)&flash_duoji_Kp1,(float *)&flash_duoji_Kp2, (float *)&flash_duoji_Kd, (float *)&flash_dianji_Kp, (float *)&flash_dianji_Ki, (float *)&flash_dianji_Kd,(float *)&flash_duoji_Kp1_you,(float *)&flash_duoji_Kp2_you,(float *)&flash_speed_want,(float *)&flash_camera_boundary,(float *)&flash_speed_change,(float *)&flash_speed_change_error,(float *)&flash_speed_kvff, (float *)&flash_speed_kaff,(float *)&i15,(float *)&i16};
//解释：对Kp进行取址，得到一个（int型）指针，再把这个指针强制转换为float型指针。
//参数屏幕
float *fv_addr1[fv_MAX] = {(float *)&i1, (float *)&i2, (float *)&i3,(float *)&i4,(float *)&i5, (float *)&i6,(float *)&i7, (float *)&f8,(float *)&f9,(float *)&f10,(float *)&f11, (float *)&f12,(float *)&f13,(float *)&f14,(float *)&f15,(float *)&f16};
//camera_boundary
ui_var_info_t  num_info[fv_MAX] =
{
    //fv_addr1   第二幅图片右边的参数     整数
   {0, 0, 0, 100, {96, 0 }},               //变量  i1
   {0, 0, 0, 100, {96, 0}},               //变量 i2
   {0, 0, 0, 100, {96, 0}},               //变量   i3
   {0, 0, 0, 100, {96, 0}},               //变量  i4
   {0, 0, 0, 100, {96, 0 }},               //变量  i1
   {0, 0, 0, 100, {96, 0}},               //变量 i2
   {0, 0, 0, 100, {96, 0}},               //变量   i3
   {0, 0, 0, 100, {96, 0}},               //变量  i4

   //fv_addr2   第三幅图片右边的参数
   {0, 0, 0, 200 , {96, 0 }},               //变量  speed
   {0, 0, 0, 100, {96, 16}},               //变量 speed_max
   {0, 0, 0, 200, {96, 32}},               //变量   speed_min
   {0, 0, 0, 200, {96, 48}},               //变量  Kp_Kp
   {0, 0, 0, 300, {96, 64 }},               //变量  Kp_Kp2
   {0, 0, 0, 1000, {96, 80}},               //变量 Kd
   {0, 0, 0, 1000, {96, 96}},               //变量   Kd
   {0, 0, 0, 1000, {96, 112}},               //变量  i16
};

ui_fv_info_t  fv_info[fv_MAX] =
{

    //fv_addr2     第三幅图片左边的参数            浮点数
     {0, 0, 0, 500, {0, 0 }},              //变量 flash_Kp_duoji
     {0, 0, 0, 100, {0, 16}},              //变量 flash_Kd_duoji
     {0, 0, 0, 2000, {0, 32}},              //变量 flash_Kp_dianji
     {0, 0, 0, 300, {0, 48}},              //变量 flash_Ki_dianji
     {0, 0, 0, 100, {0, 64 }},             //变量 flash_Kd_dianji
     {0, 0, 0, 100, {0, 80}},               //变量 MOTOR_Kd_r
     {0, 0, 0, 200, {0, 96}},               //变量  f7
     {0, 0, 0, 100, {0, 112}},               //变量 f8
    //fv_addr1     第二幅图左边的参数       浮点数
     {0, 0, 0, 100, {0, 0 }},              //变量 camera_boundary
     {0, 0, 0, 100, {0, 0}},              //变量 Kp2
     {0, 0, 0, 100, {0, 0}},               //变量  i1
     {0, 0, 0, 100, {0, 0}},               //变量 i2
     {0, 0, 0, 100, {0, 0}},               //变量   i3
     {0, 0, 0, 100, {0, 0}},               //变量  i4
     {0, 0, 0, 100, {0, 0}},               //变量   i3
     {0, 0, 0, 100, {0, 0}},               //变量  i4

};


//常量数组
ui_str_info_t  str_info[max_screen] =
{
    {qidong,{0,  112}},
    {tuxiang,{45,112}},
    {dianji,{90, 112}},
};



uint8    new_tab = 0;                                //当前选择的变量编号
uint32   last_tab;                                   //最后接收到的变量编号

uint8   str_new_tab = 0;                            //当前选择的常量编号
uint32  str_last_tab;                               //最后接收到的常量编号
/*******************************************************************************
 * Function Name: bmp_display();
 * Input Param  : 无
 * Output Param : 无
 * Description  : bpm图片显示，根据str_new_tab进行切换
 * Author Data  : 2014/3/22 星期四, by Li
 ******************************************************************************/
void bmp_display()
{
  Site_t bmp_site     = {0, 0};                     //显示图像左上角位置
  Size_t bmp_imgsize  = {128, 128};                 //图像大小
  Size_t bmp_size     = {128, 108};                   //显示区域图像大小

  LCD_Img_RGB565_Z(bmp_site,bmp_size,bmp[0],bmp_imgsize);
}
/*******************************************************************************
 * Function Name: flash_flaot_init();
 * Input Param  : flash_offset
 * Output Param : 无
 * Description  : 浮点数剥离
 * Author Data  : 2014/5/24 星期六, by Liu
 ******************************************************************************/
float flash_flaot_init(uint8 flash_offset)
{
   uint16 temp_all;
   uint16 temp_integer;
   float  temp_point;
   float  temp_finial;
   temp_all=flash_read(flash_offset, 0, uint16);    //读取4字节
   temp_integer=temp_all/10;
   temp_point=temp_all%10;
   temp_finial=temp_integer+temp_point/10;
   return temp_finial;
}
/*******************************************************************************
 * Function Name: LCD_read_flash();
 * Input Param  : fv_tal      所有读取的数据
 * Input Param  : mode        显示数据的模式
 * Output Param : 无
 * Description  : 读取flash
 * Author Data  : 2014/5/22 星期四, by Liu
 ******************************************************************************/
void LCD_read_flash(fv_tab_e fv_tal,display_screen mode)
{
  uint8 flash_offset;
  switch(mode)
  {
   case camera_screen:
   flash_offset=FLASH_SECTOR_NUM-1-(uint8)fv_MAX*(camera_screen)+fv_tal;
   if(mode_camera(fv_tal))                  //(fv_tal<= var_MAX)
   {
     *((uint16 *)(fv_addr1[fv_tal])) = flash_read(flash_offset, 0, uint16);     //读取4字节
   }
   else
   {
     *((float *)(fv_addr1[fv_tal])) = flash_flaot_init(flash_offset);  //变量是浮点型
   }
   break;

   case data_screen:
   flash_offset=FLASH_SECTOR_NUM-1-(uint8)fv_MAX*(data_screen)+fv_tal;
   if(mode_data(fv_tal))                     //(fv_tal> var_MAX)
   {
      *((uint16 *)(fv_addr2[fv_tal])) = flash_read(flash_offset, 0, uint16);  //读取4字节
   }

   else
   {
     *((float *)(fv_addr2[fv_tal])) = flash_flaot_init(flash_offset);
   }
   break;

  }
}
/*******************************************************************************
 * Function Name: LCD_write_flash();
 * Input Param  : 无
 * Output Param : 无
 * Description  : flash写入
 * Author Data  : 2014/5/22 星期四, by Liu
 ******************************************************************************/
void LCD_write_flash()
{
  int i;
  uint8 flash_offset;

  DisableInterrupts;                                                            //关总中断
  switch(mode)
  {

   case camera_screen:
   for(i=0;i<fv_MAX;i++)
   {
   flash_offset=FLASH_SECTOR_NUM-1-(uint8)fv_MAX*(camera_screen)+i;
   if(mode_camera(i))
   {
   if( VAR_OLDVALUE(i)== VAR_VALUE(i))
   {
     continue;
   }
   else
   {
     *((uint16 *)(fv_addr1[i])) = VAR_VALUE(i);          //给摄像界面整型常量赋值
     flash_erase_sector(flash_offset);                     //擦除扇区
     flash_write(flash_offset, 0, (uint16)VAR_VALUE(i) );   //写入数据到扇区，
     VAR_OLDVALUE(i)= VAR_VALUE(i);
   }

   }
   else
   {
    if( FV_OLDVALUE(i)== FV_VALUE(i) )
   {
     continue;
   }
   else
   {
     *((float *)(fv_addr1[i])) = FV_VALUE(i);              //给摄像界面浮点常量赋值
     flash_erase_sector(flash_offset);                     //擦除扇区
     flash_write(flash_offset, 0, (uint16) (FV_VALUE(i)*10) );   //写入数据到扇区，保存小数点后一位
     FV_OLDVALUE(i)= FV_VALUE(i);
   }
   }
   }
   break;
   case data_screen:
   for(i=0;i<fv_MAX;i++)
   {
   flash_offset=FLASH_SECTOR_NUM-1-(uint8)fv_MAX*(data_screen)+i;
   if(mode_data(i))
   {
   if( VAR_OLDVALUE(i)== VAR_VALUE(i) )
   {
     continue;
   }
   else
   {
      *((uint16 *)(fv_addr2[i])) = (uint16)VAR_VALUE(i);     //给参数界面整型常量赋值
     flash_erase_sector(flash_offset);                     //擦除扇区
     flash_write(flash_offset, 0, VAR_VALUE(i) );   //写入数据到扇区，偏移地址为0，必须一次写入4字节
     VAR_OLDVALUE(i)= VAR_VALUE(i);
   }
   }
   else
   {
    if( FV_OLDVALUE(i)== FV_VALUE(i) )
   {
     continue;
   }
   else
   {
     *((float *)(fv_addr2[i])) = FV_VALUE(i);                ////给参数界面浮点常量赋值
     flash_erase_sector(flash_offset);                     //擦除扇区
     flash_write(flash_offset, 0, (uint16) (FV_VALUE(i)*10) );   //写入数据到扇区
     FV_OLDVALUE(i)= FV_VALUE(i);
   }
   }
   }
   break;

   default:                        //无效选择，不需要切换
   break;
  }
  EnableInterrupts;                                                             //开总中断
}
//对初始化变量进行地址转化
void get_fv(fv_tab_e fv_tal, float *fv_data,uint16 *var_data,display_screen mode)
{
  switch(mode)
 {
  case camera_screen:
  if(mode_camera(fv_tal))
  *var_data = *(uint16 *)(fv_addr1[fv_tal]);
  else
  *fv_data = (float) * ((float *)(fv_addr1[fv_tal]));    //浮点数
  break;
  case data_screen:
  if(mode_data(fv_tal))
  *var_data =   *(uint16 *)(fv_addr2[fv_tal]);
  else
  *fv_data = (float) * ((float *)(fv_addr2[fv_tal]));
  break;

 }
}
/*******************************************************************************
 * Function Name: var_init();
 * Input Param  : 无
 * Output Param : 无
 * Description  : 变量初始化
 * Author Data  : 2014/5/22 星期四, by Liu
 ******************************************************************************/
void var_init()
{
    uint8    fv_num;
    float   fvtemp;
    uint16   iartemp;

    for(fv_num = 0; fv_num < fv_MAX; fv_num++)
    {
        LCD_read_flash((fv_tab_e)fv_num,camera_screen);
        LCD_read_flash((fv_tab_e)fv_num,data_screen);

        get_fv((fv_tab_e)fv_num, &fvtemp,&iartemp,camera_screen);
        get_fv((fv_tab_e)fv_num, &fvtemp,&iartemp,data_screen);

        num_info[fv_num].val       = iartemp;      //目前的值
        num_info[fv_num].oldval    = iartemp;      //即最后发送的值

        fv_info [fv_num].val       = fvtemp;
        fv_info [fv_num].oldval    = fvtemp;
    }
}

//对变量进行选择
void var_select(ui_var_event_e  ctrl)
{
    ASSERT(new_tab < fv_MAX);

    uint8 old_tab = new_tab;       //先备份当前变量标号

    //切换到下一个变量
    switch(ctrl)
    {
    case VAR_NEXT:                      //下一个
        new_tab++;
        if(new_tab >= (fv_MAX) )
        {
            new_tab = 0;               //从头开始
        }
        break;

    case VAR_PREV:                      //上一个
        new_tab--;
        if(new_tab >= (fv_MAX) )
        {
            new_tab = fv_MAX - 1;     //从尾开始
        }
        break;

    case VAR_NEXT_HOLD:                 //快下
        new_tab += VAR_SELECT_HOLD_OFFSET;
        if(new_tab >= (fv_MAX) )
        {
            new_tab = 0;               //从头开始
        }
        break;

    case VAR_PREV_HOLD:                 //快上
        new_tab -= VAR_SELECT_HOLD_OFFSET;
        if(new_tab >= (fv_MAX) )
        {
          new_tab = fv_MAX - 1;     //从尾开始
        }
        break;

    default:                        //无效选择，不需要切换
        return;
    }

    var_display(old_tab);               //处理上一个变量

    var_display(new_tab);              //处理当前变量：

}
//对常量进行选择
void str_select(ui_var_event_e  ctrl)
{
    uint8 str_old_tab = str_new_tab;       //先备份当前变量标号

    //切换到下一个变量
    switch(ctrl)
    {
    case VAR_NEXT:                      //下一个
        str_new_tab++;
        if(str_new_tab >= (max_screen) )
        {
           str_new_tab = 0;               //从头开始
        }
        break;

    case VAR_PREV:                      //上一个
        str_new_tab--;
        if(str_new_tab >= (max_screen) )
        {
            str_new_tab = max_screen - 1;     //从尾开始
        }
        break;

    default:                        //无效选择，不需要切换
        return;
    }

    str_display(str_old_tab);               //处理上一个变量

    str_display(str_new_tab);              //处理当前变量：

//    bmp_display();

}

//对变量的加减进行处理
void var_value(ui_var_event_e ctrl)
{
    //修改当前变量的值
    switch(ctrl)
    {
    case VAR_ADD:
      if(mode_judge(mode,new_tab))
      {

        if(VAR_VALUE(new_tab) < VAR_MAXVALUE(new_tab))
        {
            VAR_VALUE(new_tab)++;
        }
        else
        {
            VAR_VALUE(new_tab) = VAR_MAXVALUE(new_tab);
        }
      }
      else
      {
         if(FV_VALUE(new_tab) < FV_MAXVALUE(new_tab))
        {
            FV_VALUE(new_tab)+=0.1;
        }
        else
        {
            FV_VALUE(new_tab) = FV_MAXVALUE(new_tab);
        }
      }
        break;

    case VAR_SUB:
      if(mode_judge(mode,new_tab))
      {
        if(VAR_VALUE(new_tab) > VAR_MINVALUE(new_tab))
        {
            VAR_VALUE(new_tab)--;
        }
        else
        {
            VAR_VALUE(new_tab) = VAR_MINVALUE(new_tab) ;//最小值减一为最大值
        }
      }
      else
      {
        if(FV_VALUE(new_tab) > FV_MINVALUE(new_tab))
        {
            FV_VALUE(new_tab)-=0.1;
        }
        else
        {
            FV_VALUE(new_tab) = FV_MINVALUE(new_tab) ;//最小值减一为最大值
        }
      }
        break;

    case VAR_ADD_HOLD:
      if(mode_judge(mode,new_tab))
      {
        if(   VAR_MAXVALUE(new_tab) - (VAR_VALUE(new_tab)  >  10) )
        {
            VAR_VALUE(new_tab) += 10;
        }
        else
        {
            VAR_VALUE(new_tab) = VAR_MINVALUE(new_tab);
        }
      }
      else
      {
         if(  FV_MAXVALUE(new_tab) - (FV_VALUE(new_tab)  >  10) )
        {
            FV_VALUE(new_tab) += 10;
        }
        else
        {
            FV_VALUE(new_tab) = FV_MAXVALUE(new_tab);
        }
      }
        break;

    case VAR_SUB_HOLD:
       if(mode_judge(mode,new_tab))
       {
        if( ( VAR_VALUE(new_tab) - VAR_MINVALUE(new_tab)) > 10  )
        {
            VAR_VALUE(new_tab) -= 10;
        }
        else
        {
            VAR_VALUE(new_tab) = VAR_MINVALUE(new_tab) ;//最小值减一为最大值
        }
       }
       else
       {
        if( ( FV_VALUE(new_tab) - FV_MINVALUE(new_tab)) > 10  )
        {
            FV_VALUE(new_tab) -= 10;
        }
        else
        {
            FV_VALUE(new_tab) = FV_MINVALUE(new_tab) ;//最小值减一为最大值
        }
       }
        break;

    default:                        //无效选择，不需要切换
        break;
    }

    var_display(new_tab);
}



uint16 eagle_yuzhi;
//变量确认当前选择
void var_ok()
{
  LCD_write_flash();
  //gpio_set (PTC3,HIGH);//蜂鸣器
  eagle_yuzhi=flash_yuzhi;
//  SCCB_WriteByte(OV7725_CNST ,eagle_yuzhi);             //按键调试摄像头阈值
//  DELAY_MS(50);
//gpio_set (PTC3,LOW);//蜂鸣器

}

/*******************************************************************************
 * Function Name: str_ok();
 * Input Param  : 无
 * Output Param : 无
 * Description  : 进入子屏幕显示
 * Author Data  : 2015/3/28 星期四, by Li
 ******************************************************************************/
//常量确认当前选择
void str_ok()
{
  Site_t site = {0, 0};
 // Site_t site_a={0,112};
  Size_t size =  {LCD_W,LCD_H};
  mode=(display_screen)str_new_tab;
  switch(mode)
  {
  case init_screen:
    {
      LCD_clear(BCOLOUR);
      LCD_rectangle(site,size,RED);        //初始化背景       //现在的问题是进入了这个界面后，无法返回，除了复位
      DELAY_MS(500);
	  //这个问题好好想想，待解决
//	  WDOG_Open();
//      gpio_set (PTD4,1);
//	  DELAY_MS(200);
//	  gpio_set (PTD4,0);
//	  DELAY_MS(200);
//	  gpio_set (PTD4,1);
//	  DELAY_MS(200);
//	  gpio_set (PTD4,0);
//	  DELAY_MS(200);
//	  gpio_set (PTD4,1);
//	  DELAY_MS(200);
//	  gpio_set (PTD4,0);
      DELAY_MS(500);
      enable_irq(PIT2_IRQn);
//	  enable_irq(PORTE_IRQn);
     break;
    }

  case camera_screen:
    {
      LCD_clear(BCOLOUR);
      LCD_rectangle(site,size,RED);     //初始化背景
      var_display(fv_MAX);
     break;
    }

  case data_screen:
    {
      LCD_clear(BCOLOUR);
      LCD_rectangle(site,size,RED);     //初始化背景
      var_display(fv_MAX);
     break;
    }

  }
}
//复位改动数据
void val_cancel()
{
    ASSERT(new_tab < fv_MAX);

    //直接还原当前值
    VAR_VALUE(new_tab) = VAR_OLDVALUE(new_tab);
     FV_VALUE(new_tab) = FV_OLDVALUE(new_tab);

    var_display(new_tab);
}
/*******************************************************************************
 * Function Name: void_return();
 * Input Param  : 无
 * Output Param : 无
 * Description  : //返回常量界面
 * Author Data  : 2015/3/28星期四, by Li
 ******************************************************************************/
void void_return()
{
  Site_t site = {0, 0};
  Size_t size =  {LCD_W,LCD_H};
  mode=init_screen;
  LCD_rectangle(site,size,RED);     //初始化背景
  str_display(max_screen);
  bmp_display();
//  bmp_display();
}

//显示指定的值。tab 为 FV_MAX 时表示全部显示，小于则显示对应的
/*******************************************************************************
 * Function Name: var_display();
 * Input Param  : 无
 * Output Param : 无
 * Description  : //主菜单显示
 * Author Data  : 2015/3/28星期四, by Li
 ******************************************************************************/
void var_display(uint8 tab)
{

#if UI_VAR_USE_LCD

    //定义背景延时
#define SELECT_NO_CHANGE_BG         WHITE   //当前选中，而且没有改变
#define SELECT_CHANGE_BG            WHITE   //当前选中，而且改变了
#define NO_SELECT_NO_CHANGE_BG      RED     //没有选中，而且没有改变（普通的就是这样）
#define NO_SELECT_CHANGE_BG         RED     //没有选中，而且改变了

    //定义文字颜色
#define SELECT_NO_CHANGE            BLUE    //当前选中，而且没有改变
#define SELECT_CHANGE               GREEN   //当前选中，而且改变了
#define NO_SELECT_NO_CHANGE         BLUE    //没有选中，而且没有改变（普通的就是这样）
#define NO_SELECT_CHANGE            GREEN   //没有选中，而且改变了

    uint8  i = 0;
    uint16 bkColor;
    uint16 Color;


    ASSERT((new_tab < fv_MAX) && (tab <= fv_MAX));

    if(tab == fv_MAX)      //显示全部
    {
        i = fv_MAX - 1;    //循环的次数
        tab = 0;
    }

    do
  {   if(mode_judge(mode,tab))
    {
        if(tab == new_tab)
        {
            //显示当前的值：判断值是否改变
            if(VAR_VALUE(tab) == VAR_OLDVALUE(tab)) //值没改变，不需要处理
            {
                Color   =  SELECT_NO_CHANGE;
                bkColor =  SELECT_NO_CHANGE_BG;
            }
            else
            {
                Color   =  SELECT_CHANGE;
                bkColor =  SELECT_CHANGE_BG;
            }
        }
        else
        {
            //显示非当前的值
            if(VAR_VALUE(tab) == VAR_OLDVALUE(tab)) //值没改变，不需要处理
            {
                Color   =  NO_SELECT_NO_CHANGE;
                bkColor =  NO_SELECT_NO_CHANGE_BG;

            }
            else
            {
                Color   =  NO_SELECT_CHANGE;
                bkColor =  NO_SELECT_CHANGE_BG;
            }
        }

        LCD_num_C(VAR_SITE(tab), VAR_VALUE(tab), Color, bkColor);


    }
    else
    {
      if(tab == new_tab)
        {
            //显示当前的值：判断值是否改变
            if(FV_VALUE(tab) == FV_OLDVALUE(tab)) //值没改变，不需要处理
            {
                Color   =  SELECT_NO_CHANGE;
                bkColor =  SELECT_NO_CHANGE_BG;
            }
            else
            {
                Color   =  SELECT_CHANGE;
                bkColor =  SELECT_CHANGE_BG;
            }
        }
        else
        {
            //显示非当前的值
            if(FV_VALUE(tab) == FV_OLDVALUE(tab)) //值没改变，不需要处理
            {
                Color   =  NO_SELECT_NO_CHANGE;
                bkColor =  NO_SELECT_NO_CHANGE_BG;

            }
            else
            {
                Color   =  NO_SELECT_CHANGE;
                bkColor =  NO_SELECT_CHANGE_BG;
            }
        }

      LCD_float(FV_SITE(tab), FV_VALUE(tab), Color, bkColor);


//        tab++;
    }
	tab++;
    }
    while(i--);         //tab != VAR_MAX 的时候，执行一次就跳出

#else
    tab = tab;           //消除编译警告
#endif
}


void str_display(uint8 tab)
{
  #if UI_VAR_USE_LCD

    //定义背景延时
#define SELECT_NO_CHANGE_BG         WHITE   //当前选中，而且没有改变
#define SELECT_CHANGE_BG            WHITE   //当前选中，而且改变了
#define NO_SELECT_NO_CHANGE_BG      RED     //没有选中，而且没有改变（普通的就是这样）
#define NO_SELECT_CHANGE_BG         RED     //没有选中，而且改变了

    //定义文字颜色
#define SELECT_NO_CHANGE            BLUE    //当前选中，而且没有改变
#define SELECT_CHANGE               GREEN   //当前选中，而且改变了
#define NO_SELECT_NO_CHANGE         BLUE    //没有选中，而且没有改变（普通的就是这样）
#define NO_SELECT_CHANGE            GREEN   //没有选中，而且改变了

    uint8  i = 0;
    uint16 bkColor;
    uint16 Color;

    ASSERT((str_new_tab < max_screen) && (tab <= max_screen));

    if(tab == max_screen)      //显示全部
    {
        i = max_screen - 1;    //循环的次数

        tab = 0;
    }

    do
    {
        if(tab == str_new_tab)
        {
            //显示当前的值：判断值是否改变
            if(VAR_VALUE(tab) == VAR_OLDVALUE(tab)) //值没改变，不需要处理
            {
                Color   =  SELECT_NO_CHANGE;
                bkColor =  SELECT_NO_CHANGE_BG;
            }
            else
            {
                Color   =  SELECT_CHANGE;
                bkColor =  SELECT_CHANGE_BG;
            }
        }
        else
        {
            //显示非当前的值
            if(VAR_VALUE(tab) == VAR_OLDVALUE(tab)) //值没改变，不需要处理
            {
                Color   =  NO_SELECT_NO_CHANGE;
                bkColor =  NO_SELECT_NO_CHANGE_BG;

            }
            else
            {
                Color   =  NO_SELECT_CHANGE;
                bkColor =  NO_SELECT_CHANGE_BG;
            }
        }

        /*
         * 显示文字
         * 修改此处可以改变显示字宽
         */
        LCD_FStr_CH (STR_SITE(tab) ,STR_VALUE(tab),2,Color,bkColor);

       // LCD_FSTR_CH(site1,vcan_donglinjichi,BLUE,RED);

        tab++;
    }
    while(i--);         //tab != VAR_MAX 的时候，执行一次就跳出


#else
    tab = tab;          //消除编译警告
#endif
}
