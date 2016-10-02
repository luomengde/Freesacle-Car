#include "common.h"
#include "include.h"
#include "VCAN_LCD.h"
#include "VCAN_UI_VAR.h"
#include "LCD_BMP.h"

/*  ��������  */
#define VAR_VALUE(var_tab)      num_info[var_tab].val       //ָ����ŵı�����ֵ
#define VAR_OLDVALUE(var_tab)   num_info[var_tab].oldval    //ָ����ŵı��������ȷ��ֵ
#define VAR_MAXVALUE(var_tab)   num_info[var_tab].maxval
#define VAR_MINVALUE(var_tab)   num_info[var_tab].minval
#define VAR_SITE(var_tab)       num_info[var_tab].site

/*  flash����  */
#define FV_VALUE(var_tab)       fv_info[var_tab].val       //ָ����ŵı�����ֵ
#define FV_OLDVALUE(var_tab)    fv_info[var_tab].oldval    //ָ����ŵı��������ȷ��ֵ
#define FV_MAXVALUE(var_tab)    fv_info[var_tab].maxval
#define FV_MINVALUE(var_tab)    fv_info[var_tab].minval
#define FV_SITE(var_tab)        fv_info[var_tab].site

#define  STR_VALUE(var_tab)      str_info[var_tab].str       //ָ����ŵ�ֵ
#define  STR_SITE(var_tab)       str_info[var_tab].site


#define  mode_camera(x)		(x <= var_MAX)
#define  mode_data(x)		(x > var_MAX)
#define  mode_judge(x,y)         x==1?mode_camera(y):mode_data(y)

extern display_screen mode;
extern const uint16 *bmp[];
extern void LCD_Img_RGB565_Z(Site_t site,Size_t size,const uint16 *img,Size_t imgsize);
extern void LCD_float(Site_t site, float num, uint16 Color, uint16 bkColor);


float flash_duoji_Kp1,flash_duoji_Kp2,flash_duoji_Kp1_you,flash_duoji_Kp2_you,flash_duoji_Kd;        //���PD����
float flash_dianji_Kp,flash_dianji_Ki,flash_dianji_Kd;       //���PID����
uint32 flash_speed_want;
uint32 flash_speed_change,flash_speed_change_error;   //���ּӼ��ٵķ�ʽ
uint32 flash_camera_boundary;
uint32 flash_yuzhi;
uint32 flash_speed_kvff,flash_speed_kaff;
//���Ա���
uint16 i1=1,i2=2,i3=3,i4=4,i5=5,i6=6,i7=7,i8=8,i9=9,i10=10,i11=11,i12=12,i13=13,i14=14,i15=15,i16=16;
float f1=1.1,f2=2.2,f3=3.3,f4=4.4,f5=5.5,f6=6.6,f7=7.7,f8=8.8,f9=9.9,f10=10.10,f11=11.11,f12=12.12,f13=13.13,f14=14.4,f15=15.5,f16=16.6;

float *fv_addr2[fv_MAX] = {(float *)&flash_duoji_Kp1,(float *)&flash_duoji_Kp2, (float *)&flash_duoji_Kd, (float *)&flash_dianji_Kp, (float *)&flash_dianji_Ki, (float *)&flash_dianji_Kd,(float *)&flash_duoji_Kp1_you,(float *)&flash_duoji_Kp2_you,(float *)&flash_speed_want,(float *)&flash_camera_boundary,(float *)&flash_speed_change,(float *)&flash_speed_change_error,(float *)&flash_speed_kvff, (float *)&flash_speed_kaff,(float *)&i15,(float *)&i16};
//���ͣ���Kp����ȡַ���õ�һ����int�ͣ�ָ�룬�ٰ����ָ��ǿ��ת��Ϊfloat��ָ�롣
//������Ļ
float *fv_addr1[fv_MAX] = {(float *)&i1, (float *)&i2, (float *)&i3,(float *)&i4,(float *)&i5, (float *)&i6,(float *)&i7, (float *)&f8,(float *)&f9,(float *)&f10,(float *)&f11, (float *)&f12,(float *)&f13,(float *)&f14,(float *)&f15,(float *)&f16};
//camera_boundary
ui_var_info_t  num_info[fv_MAX] =
{
    //fv_addr1   �ڶ���ͼƬ�ұߵĲ���     ����
   {0, 0, 0, 100, {96, 0 }},               //����  i1
   {0, 0, 0, 100, {96, 0}},               //���� i2
   {0, 0, 0, 100, {96, 0}},               //����   i3
   {0, 0, 0, 100, {96, 0}},               //����  i4
   {0, 0, 0, 100, {96, 0 }},               //����  i1
   {0, 0, 0, 100, {96, 0}},               //���� i2
   {0, 0, 0, 100, {96, 0}},               //����   i3
   {0, 0, 0, 100, {96, 0}},               //����  i4

   //fv_addr2   ������ͼƬ�ұߵĲ���
   {0, 0, 0, 200 , {96, 0 }},               //����  speed
   {0, 0, 0, 100, {96, 16}},               //���� speed_max
   {0, 0, 0, 200, {96, 32}},               //����   speed_min
   {0, 0, 0, 200, {96, 48}},               //����  Kp_Kp
   {0, 0, 0, 300, {96, 64 }},               //����  Kp_Kp2
   {0, 0, 0, 1000, {96, 80}},               //���� Kd
   {0, 0, 0, 1000, {96, 96}},               //����   Kd
   {0, 0, 0, 1000, {96, 112}},               //����  i16
};

ui_fv_info_t  fv_info[fv_MAX] =
{

    //fv_addr2     ������ͼƬ��ߵĲ���            ������
     {0, 0, 0, 500, {0, 0 }},              //���� flash_Kp_duoji
     {0, 0, 0, 100, {0, 16}},              //���� flash_Kd_duoji
     {0, 0, 0, 2000, {0, 32}},              //���� flash_Kp_dianji
     {0, 0, 0, 300, {0, 48}},              //���� flash_Ki_dianji
     {0, 0, 0, 100, {0, 64 }},             //���� flash_Kd_dianji
     {0, 0, 0, 100, {0, 80}},               //���� MOTOR_Kd_r
     {0, 0, 0, 200, {0, 96}},               //����  f7
     {0, 0, 0, 100, {0, 112}},               //���� f8
    //fv_addr1     �ڶ���ͼ��ߵĲ���       ������
     {0, 0, 0, 100, {0, 0 }},              //���� camera_boundary
     {0, 0, 0, 100, {0, 0}},              //���� Kp2
     {0, 0, 0, 100, {0, 0}},               //����  i1
     {0, 0, 0, 100, {0, 0}},               //���� i2
     {0, 0, 0, 100, {0, 0}},               //����   i3
     {0, 0, 0, 100, {0, 0}},               //����  i4
     {0, 0, 0, 100, {0, 0}},               //����   i3
     {0, 0, 0, 100, {0, 0}},               //����  i4

};


//��������
ui_str_info_t  str_info[max_screen] =
{
    {qidong,{0,  112}},
    {tuxiang,{45,112}},
    {dianji,{90, 112}},
};



uint8    new_tab = 0;                                //��ǰѡ��ı������
uint32   last_tab;                                   //�����յ��ı������

uint8   str_new_tab = 0;                            //��ǰѡ��ĳ������
uint32  str_last_tab;                               //�����յ��ĳ������
/*******************************************************************************
 * Function Name: bmp_display();
 * Input Param  : ��
 * Output Param : ��
 * Description  : bpmͼƬ��ʾ������str_new_tab�����л�
 * Author Data  : 2014/3/22 ������, by Li
 ******************************************************************************/
void bmp_display()
{
  Site_t bmp_site     = {0, 0};                     //��ʾͼ�����Ͻ�λ��
  Size_t bmp_imgsize  = {128, 128};                 //ͼ���С
  Size_t bmp_size     = {128, 108};                   //��ʾ����ͼ���С

  LCD_Img_RGB565_Z(bmp_site,bmp_size,bmp[0],bmp_imgsize);
}
/*******************************************************************************
 * Function Name: flash_flaot_init();
 * Input Param  : flash_offset
 * Output Param : ��
 * Description  : ����������
 * Author Data  : 2014/5/24 ������, by Liu
 ******************************************************************************/
float flash_flaot_init(uint8 flash_offset)
{
   uint16 temp_all;
   uint16 temp_integer;
   float  temp_point;
   float  temp_finial;
   temp_all=flash_read(flash_offset, 0, uint16);    //��ȡ4�ֽ�
   temp_integer=temp_all/10;
   temp_point=temp_all%10;
   temp_finial=temp_integer+temp_point/10;
   return temp_finial;
}
/*******************************************************************************
 * Function Name: LCD_read_flash();
 * Input Param  : fv_tal      ���ж�ȡ������
 * Input Param  : mode        ��ʾ���ݵ�ģʽ
 * Output Param : ��
 * Description  : ��ȡflash
 * Author Data  : 2014/5/22 ������, by Liu
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
     *((uint16 *)(fv_addr1[fv_tal])) = flash_read(flash_offset, 0, uint16);     //��ȡ4�ֽ�
   }
   else
   {
     *((float *)(fv_addr1[fv_tal])) = flash_flaot_init(flash_offset);  //�����Ǹ�����
   }
   break;

   case data_screen:
   flash_offset=FLASH_SECTOR_NUM-1-(uint8)fv_MAX*(data_screen)+fv_tal;
   if(mode_data(fv_tal))                     //(fv_tal> var_MAX)
   {
      *((uint16 *)(fv_addr2[fv_tal])) = flash_read(flash_offset, 0, uint16);  //��ȡ4�ֽ�
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
 * Input Param  : ��
 * Output Param : ��
 * Description  : flashд��
 * Author Data  : 2014/5/22 ������, by Liu
 ******************************************************************************/
void LCD_write_flash()
{
  int i;
  uint8 flash_offset;

  DisableInterrupts;                                                            //�����ж�
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
     *((uint16 *)(fv_addr1[i])) = VAR_VALUE(i);          //������������ͳ�����ֵ
     flash_erase_sector(flash_offset);                     //��������
     flash_write(flash_offset, 0, (uint16)VAR_VALUE(i) );   //д�����ݵ�������
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
     *((float *)(fv_addr1[i])) = FV_VALUE(i);              //��������渡�㳣����ֵ
     flash_erase_sector(flash_offset);                     //��������
     flash_write(flash_offset, 0, (uint16) (FV_VALUE(i)*10) );   //д�����ݵ�����������С�����һλ
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
      *((uint16 *)(fv_addr2[i])) = (uint16)VAR_VALUE(i);     //�������������ͳ�����ֵ
     flash_erase_sector(flash_offset);                     //��������
     flash_write(flash_offset, 0, VAR_VALUE(i) );   //д�����ݵ�������ƫ�Ƶ�ַΪ0������һ��д��4�ֽ�
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
     *((float *)(fv_addr2[i])) = FV_VALUE(i);                ////���������渡�㳣����ֵ
     flash_erase_sector(flash_offset);                     //��������
     flash_write(flash_offset, 0, (uint16) (FV_VALUE(i)*10) );   //д�����ݵ�����
     FV_OLDVALUE(i)= FV_VALUE(i);
   }
   }
   }
   break;

   default:                        //��Чѡ�񣬲���Ҫ�л�
   break;
  }
  EnableInterrupts;                                                             //�����ж�
}
//�Գ�ʼ���������е�ַת��
void get_fv(fv_tab_e fv_tal, float *fv_data,uint16 *var_data,display_screen mode)
{
  switch(mode)
 {
  case camera_screen:
  if(mode_camera(fv_tal))
  *var_data = *(uint16 *)(fv_addr1[fv_tal]);
  else
  *fv_data = (float) * ((float *)(fv_addr1[fv_tal]));    //������
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
 * Input Param  : ��
 * Output Param : ��
 * Description  : ������ʼ��
 * Author Data  : 2014/5/22 ������, by Liu
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

        num_info[fv_num].val       = iartemp;      //Ŀǰ��ֵ
        num_info[fv_num].oldval    = iartemp;      //������͵�ֵ

        fv_info [fv_num].val       = fvtemp;
        fv_info [fv_num].oldval    = fvtemp;
    }
}

//�Ա�������ѡ��
void var_select(ui_var_event_e  ctrl)
{
    ASSERT(new_tab < fv_MAX);

    uint8 old_tab = new_tab;       //�ȱ��ݵ�ǰ�������

    //�л�����һ������
    switch(ctrl)
    {
    case VAR_NEXT:                      //��һ��
        new_tab++;
        if(new_tab >= (fv_MAX) )
        {
            new_tab = 0;               //��ͷ��ʼ
        }
        break;

    case VAR_PREV:                      //��һ��
        new_tab--;
        if(new_tab >= (fv_MAX) )
        {
            new_tab = fv_MAX - 1;     //��β��ʼ
        }
        break;

    case VAR_NEXT_HOLD:                 //����
        new_tab += VAR_SELECT_HOLD_OFFSET;
        if(new_tab >= (fv_MAX) )
        {
            new_tab = 0;               //��ͷ��ʼ
        }
        break;

    case VAR_PREV_HOLD:                 //����
        new_tab -= VAR_SELECT_HOLD_OFFSET;
        if(new_tab >= (fv_MAX) )
        {
          new_tab = fv_MAX - 1;     //��β��ʼ
        }
        break;

    default:                        //��Чѡ�񣬲���Ҫ�л�
        return;
    }

    var_display(old_tab);               //������һ������

    var_display(new_tab);              //����ǰ������

}
//�Գ�������ѡ��
void str_select(ui_var_event_e  ctrl)
{
    uint8 str_old_tab = str_new_tab;       //�ȱ��ݵ�ǰ�������

    //�л�����һ������
    switch(ctrl)
    {
    case VAR_NEXT:                      //��һ��
        str_new_tab++;
        if(str_new_tab >= (max_screen) )
        {
           str_new_tab = 0;               //��ͷ��ʼ
        }
        break;

    case VAR_PREV:                      //��һ��
        str_new_tab--;
        if(str_new_tab >= (max_screen) )
        {
            str_new_tab = max_screen - 1;     //��β��ʼ
        }
        break;

    default:                        //��Чѡ�񣬲���Ҫ�л�
        return;
    }

    str_display(str_old_tab);               //������һ������

    str_display(str_new_tab);              //����ǰ������

//    bmp_display();

}

//�Ա����ļӼ����д���
void var_value(ui_var_event_e ctrl)
{
    //�޸ĵ�ǰ������ֵ
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
            VAR_VALUE(new_tab) = VAR_MINVALUE(new_tab) ;//��Сֵ��һΪ���ֵ
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
            FV_VALUE(new_tab) = FV_MINVALUE(new_tab) ;//��Сֵ��һΪ���ֵ
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
            VAR_VALUE(new_tab) = VAR_MINVALUE(new_tab) ;//��Сֵ��һΪ���ֵ
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
            FV_VALUE(new_tab) = FV_MINVALUE(new_tab) ;//��Сֵ��һΪ���ֵ
        }
       }
        break;

    default:                        //��Чѡ�񣬲���Ҫ�л�
        break;
    }

    var_display(new_tab);
}



uint16 eagle_yuzhi;
//����ȷ�ϵ�ǰѡ��
void var_ok()
{
  LCD_write_flash();
  //gpio_set (PTC3,HIGH);//������
  eagle_yuzhi=flash_yuzhi;
//  SCCB_WriteByte(OV7725_CNST ,eagle_yuzhi);             //������������ͷ��ֵ
//  DELAY_MS(50);
//gpio_set (PTC3,LOW);//������

}

/*******************************************************************************
 * Function Name: str_ok();
 * Input Param  : ��
 * Output Param : ��
 * Description  : ��������Ļ��ʾ
 * Author Data  : 2015/3/28 ������, by Li
 ******************************************************************************/
//����ȷ�ϵ�ǰѡ��
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
      LCD_rectangle(site,size,RED);        //��ʼ������       //���ڵ������ǽ��������������޷����أ����˸�λ
      DELAY_MS(500);
	  //�������ú����룬�����
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
      LCD_rectangle(site,size,RED);     //��ʼ������
      var_display(fv_MAX);
     break;
    }

  case data_screen:
    {
      LCD_clear(BCOLOUR);
      LCD_rectangle(site,size,RED);     //��ʼ������
      var_display(fv_MAX);
     break;
    }

  }
}
//��λ�Ķ�����
void val_cancel()
{
    ASSERT(new_tab < fv_MAX);

    //ֱ�ӻ�ԭ��ǰֵ
    VAR_VALUE(new_tab) = VAR_OLDVALUE(new_tab);
     FV_VALUE(new_tab) = FV_OLDVALUE(new_tab);

    var_display(new_tab);
}
/*******************************************************************************
 * Function Name: void_return();
 * Input Param  : ��
 * Output Param : ��
 * Description  : //���س�������
 * Author Data  : 2015/3/28������, by Li
 ******************************************************************************/
void void_return()
{
  Site_t site = {0, 0};
  Size_t size =  {LCD_W,LCD_H};
  mode=init_screen;
  LCD_rectangle(site,size,RED);     //��ʼ������
  str_display(max_screen);
  bmp_display();
//  bmp_display();
}

//��ʾָ����ֵ��tab Ϊ FV_MAX ʱ��ʾȫ����ʾ��С������ʾ��Ӧ��
/*******************************************************************************
 * Function Name: var_display();
 * Input Param  : ��
 * Output Param : ��
 * Description  : //���˵���ʾ
 * Author Data  : 2015/3/28������, by Li
 ******************************************************************************/
void var_display(uint8 tab)
{

#if UI_VAR_USE_LCD

    //���屳����ʱ
#define SELECT_NO_CHANGE_BG         WHITE   //��ǰѡ�У�����û�иı�
#define SELECT_CHANGE_BG            WHITE   //��ǰѡ�У����Ҹı���
#define NO_SELECT_NO_CHANGE_BG      RED     //û��ѡ�У�����û�иı䣨��ͨ�ľ���������
#define NO_SELECT_CHANGE_BG         RED     //û��ѡ�У����Ҹı���

    //����������ɫ
#define SELECT_NO_CHANGE            BLUE    //��ǰѡ�У�����û�иı�
#define SELECT_CHANGE               GREEN   //��ǰѡ�У����Ҹı���
#define NO_SELECT_NO_CHANGE         BLUE    //û��ѡ�У�����û�иı䣨��ͨ�ľ���������
#define NO_SELECT_CHANGE            GREEN   //û��ѡ�У����Ҹı���

    uint8  i = 0;
    uint16 bkColor;
    uint16 Color;


    ASSERT((new_tab < fv_MAX) && (tab <= fv_MAX));

    if(tab == fv_MAX)      //��ʾȫ��
    {
        i = fv_MAX - 1;    //ѭ���Ĵ���
        tab = 0;
    }

    do
  {   if(mode_judge(mode,tab))
    {
        if(tab == new_tab)
        {
            //��ʾ��ǰ��ֵ���ж�ֵ�Ƿ�ı�
            if(VAR_VALUE(tab) == VAR_OLDVALUE(tab)) //ֵû�ı䣬����Ҫ����
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
            //��ʾ�ǵ�ǰ��ֵ
            if(VAR_VALUE(tab) == VAR_OLDVALUE(tab)) //ֵû�ı䣬����Ҫ����
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
            //��ʾ��ǰ��ֵ���ж�ֵ�Ƿ�ı�
            if(FV_VALUE(tab) == FV_OLDVALUE(tab)) //ֵû�ı䣬����Ҫ����
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
            //��ʾ�ǵ�ǰ��ֵ
            if(FV_VALUE(tab) == FV_OLDVALUE(tab)) //ֵû�ı䣬����Ҫ����
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
    while(i--);         //tab != VAR_MAX ��ʱ��ִ��һ�ξ�����

#else
    tab = tab;           //�������뾯��
#endif
}


void str_display(uint8 tab)
{
  #if UI_VAR_USE_LCD

    //���屳����ʱ
#define SELECT_NO_CHANGE_BG         WHITE   //��ǰѡ�У�����û�иı�
#define SELECT_CHANGE_BG            WHITE   //��ǰѡ�У����Ҹı���
#define NO_SELECT_NO_CHANGE_BG      RED     //û��ѡ�У�����û�иı䣨��ͨ�ľ���������
#define NO_SELECT_CHANGE_BG         RED     //û��ѡ�У����Ҹı���

    //����������ɫ
#define SELECT_NO_CHANGE            BLUE    //��ǰѡ�У�����û�иı�
#define SELECT_CHANGE               GREEN   //��ǰѡ�У����Ҹı���
#define NO_SELECT_NO_CHANGE         BLUE    //û��ѡ�У�����û�иı䣨��ͨ�ľ���������
#define NO_SELECT_CHANGE            GREEN   //û��ѡ�У����Ҹı���

    uint8  i = 0;
    uint16 bkColor;
    uint16 Color;

    ASSERT((str_new_tab < max_screen) && (tab <= max_screen));

    if(tab == max_screen)      //��ʾȫ��
    {
        i = max_screen - 1;    //ѭ���Ĵ���

        tab = 0;
    }

    do
    {
        if(tab == str_new_tab)
        {
            //��ʾ��ǰ��ֵ���ж�ֵ�Ƿ�ı�
            if(VAR_VALUE(tab) == VAR_OLDVALUE(tab)) //ֵû�ı䣬����Ҫ����
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
            //��ʾ�ǵ�ǰ��ֵ
            if(VAR_VALUE(tab) == VAR_OLDVALUE(tab)) //ֵû�ı䣬����Ҫ����
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
         * ��ʾ����
         * �޸Ĵ˴����Ըı���ʾ�ֿ�
         */
        LCD_FStr_CH (STR_SITE(tab) ,STR_VALUE(tab),2,Color,bkColor);

       // LCD_FSTR_CH(site1,vcan_donglinjichi,BLUE,RED);

        tab++;
    }
    while(i--);         //tab != VAR_MAX ��ʱ��ִ��һ�ξ�����


#else
    tab = tab;          //�������뾯��
#endif
}
