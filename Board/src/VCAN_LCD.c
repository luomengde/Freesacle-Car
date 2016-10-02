/*!
 *     COPYRIGHT NOTICE
 *     Copyright (c) 2013,ɽ��Ƽ�
 *     All rights reserved.
 *     �������ۣ�ɽ����̳ http://www.vcan123.com
 *
 *     ��ע�������⣬�����������ݰ�Ȩ����ɽ��Ƽ����У�δ����������������ҵ��;��
 *     �޸�����ʱ���뱣��ɽ��Ƽ��İ�Ȩ������
 *
 * @file       VCAN_LCD.c
 * @brief      LCD ������
 * @author     ɽ��Ƽ�
 * @version    v5.0
 * @date       2013-06-26
 */


#include "common.h"
#include "VCAN_LCD.h"



/*!
 *  @brief      LCD��ʼ��
 *  @since      v5.0
 */
void LCD_init(void)
{
    Site_t site = {0, 0};
    Size_t size ;

    LCD_INIT();                             //��ʼ��LCD

    //LCD_SET_DIR(LCD_DIR+1);

    //���ڳ�ʼ����ʱ����� ��Ļ���� ѡ�������ʼ����ɺ󣬲Ż�ȡ���
    size.W = LCD_W;
    size.H = LCD_H;

    LCD_rectangle(site, size, BCOLOUR);     //��ʼ������
}
/*******************************************************************************
 * Function Name: LCD_Img_Binary_My(site, size, img);
 * Input Param  : site          ���Ͻ�����
 * Input Param  : size          ��ʾͼ���С
 * Input Param  : img           ͼ���ַ
 * Output Param : ��
 * Description  : ��ʾ����ͷ�ɼ�����ͼ��
 * Author Data  : 2014/6/11 ������, by Liu
 ******************************************************************************/
void LCD_Img_Binary_My(Site_t site, Size_t size, uint8 *img)
{
    uint32    total = (size.H * size.W) ;
    uint8     imgtemp;
    uint8     *pimg = (uint8 *)img;

    LCD_PTLON(site, size);                      //����

    LCD_RAMWR();                                //д�ڴ�

    while(total--)
    {
        imgtemp = *(pimg++);
        if(imgtemp==0  )
        {
            LCD_WR_DATA( BINARY_COLOR );      //дͼ������
        }
        else
        {
            LCD_WR_DATA(  BINARY_BGCOLOR  );      //дͼ������
        }
    }
}


void LCD_line(line_info  line[],uint16 Color,uint16 bkColor)
{
  static Site_t site_new;
 if( zhijiao_forward1==1)
  {
    for(int i=35;i<used_length;i++)
    {
      if( (line[i].mid_line_new)!=255 )
      {
	site_new.x=  line[i].mid_line_new;
	site_new.y=line_num-1-i;
	LCD_point(site_new, Color);

	site_new.x=  line[i].left_line+2;
	LCD_point(site_new, BLUE);

	site_new.x=  line[i].right_line-2;
	LCD_point(site_new, GREEN);
      }
    }
   zhijiao_forward1=0;
  }
  else
  {
    for(int i=0;i<used_length;i++)
    {
      if( (line[i].mid_line_new)!=255 )
      {
	site_new.x=  line[i].mid_line_new;
	site_new.y=line_num-1-i;
	LCD_point(site_new, Color);

	site_new.x=  line[i].left_line+2;
	LCD_point(site_new, BLUE);

	site_new.x=  line[i].right_line-2;
	LCD_point(site_new, GREEN);
      }
    }
  }
}
/*******************************************************************************
 * Function Name: LCD_Img_RGB565_Z()
 * Input Param  : ͼ��λ�ã���ʾͼ���С��*img��ͼ���С
 * Output Param : ��
 * Description  : ��ʾRGB565ͼ��
 * Author Data  : 2015/1/5 ����һ  ��֪������
 ******************************************************************************/
void LCD_Img_RGB565_Z(Site_t site,Size_t size,const uint16 *img,Size_t imgsize)
{
    uint16 x,y;
    uint16 X,Y;
    uint32 temp, tempY;
    LCD_PTLON(site,size); 	 //����
    LCD_RAMWR();	 //д�ڴ�
    for( y=0; y < (size.H) ; y++  )	 //�ڿ������������ɨ��
    {
      Y=( (y * imgsize.H + (size.H>>1)-1)/size.H);	//�����img��ĸ߶ȣ�+ size.H>>1����������
      tempY = Y * imgsize.W ;
      for( x=0; x< size.W ; x++  )
      {
        X =( (x * imgsize.W + (size.W>>1)-1)/size.W);
        temp = tempY + X;
        LCD_WR_DATA( img[temp] );	 //дͼ������
      }
    }
}
//��LCD����ʾ������     //ע:��������ĵײ㻹û�����ȵ���ʱ���ˣ��úÿ�һ������̫���ű��˵ĺ���
void LCD_float(Site_t site, float num, uint16 Color, uint16 bkColor)
{
    uint8 t = 0;
    uint8 negative_flag=0;
    Site_t sitetemp;
    Size_t size;
    sitetemp.y = site.y;
    if(num<0)
    negative_flag=1;
    int res = (int)num;
    int zs = abs((int)num);
    int xs=abs((int)((num-zs)*100));
    while( res )
    {
        res /= 10;
        t++;
    }
    t+=3;
     if(negative_flag==1)
     {
     sitetemp.x = site.x -8;
     LCD_char(sitetemp,'-', Color, bkColor);
     }
     else
     {
      sitetemp.x = site.x -8;
      LCD_char(sitetemp,'+', Color, bkColor);
     }
    for(int i=1;i<=t;i++)
    {
      if(i<=2)
      {
      sitetemp.x = site.x + (8 * t) - 8*i;
      LCD_char(sitetemp,(xs%10)+48, Color, bkColor);
      xs /= 10 ;
      }
      else if(i==3)
      {
      sitetemp.x = site.x + (8 * t) - 8*i;
      LCD_char(sitetemp,'.', Color, bkColor);
      }
      else
      {
        sitetemp.x = site.x + (8 * t) - 8*i;
        LCD_char(sitetemp, (zs % 10) + '0', Color, bkColor);
        zs /= 10 ;
      }

    }
    res = t;
    if(res != 8 )
    {
        size.W = 8 * (8 - res);
        sitetemp.x += (8 * res);
        size.H  = 16;
        LCD_rectangle(sitetemp, size, bkColor);
    }
}

/*!
 *  @brief      ��ʾʵ�ľ���
 *  @param      site    ���Ͻ�����
 *  @param      size    ���δ�С
 *  @param      rgb565  ��ɫ
 *  @since      v5.0
*  Sample usage:        Site_t site = {10,20};   //x = 10 ,y = 20
                        Size_t size = {50,60};  // W = 50 ,H = 60
                        LCD_rectangle(site, size, RED);
 */
void LCD_rectangle(Site_t site, Size_t size, uint16 rgb565)
{
    uint32 n, temp;

    LCD_PTLON(site, size);              //����

    temp = (uint32)size.W * size.H;
    LCD_RAMWR();                        //д�ڴ�
    for(n = 0; n < temp; n++)
    {
        LCD_WR_DATA( rgb565 );          //д����
    }
}

void LCD_clear(uint16 rgb565)     //����
{
    Site_t site = {0,0};
    Size_t size;
    size.H = LCD_H;
    size.W = LCD_W;

    LCD_rectangle(site, size, rgb565);

}

//��ʮ����
void LCD_cross(Site_t site,uint16 len,uint16 Color)
{
    //������ֱ��
    Site_t sitetmp;
    Size_t size;
    int16  stmp,etmp;
    uint16 w = LCD_W,h = LCD_H;

    ASSERT((site.x<LCD_W) && (site.y<LCD_H));       //ʮ���ߵ����Ĳ��ܳ���Һ����Χ

    stmp = site.x - len/2;                          //����ˮƽ�ߵ�x�����ͽ����㣬���Ʋ�����Һ����Χ
    if(stmp < 0)stmp = 0;
    etmp = site.x + len/2;
    if(etmp >= w)etmp= w-1;

    sitetmp.x = stmp;
    sitetmp.y = site.y;
    size.W = etmp - stmp +1;
    size.H = 1;
    LCD_rectangle(sitetmp,size, Color);

    stmp = site.y - len/2;                          //����ˮƽ�ߵ�x�����ͽ����㣬���Ʋ�����Һ����Χ
    if(stmp < 0)stmp = 0;
    etmp = site.y + len/2;
    if(etmp >= h)etmp= h-1;

    sitetmp.x = site.x;
    sitetmp.y = stmp;
    size.W = 1;
    size.H = etmp - stmp +1;
    LCD_rectangle(sitetmp,size, Color);



}

/*!
 *  @brief      ����
 *  @param      site    ���Ͻ�����
 *  @param      rgb565  ��ɫ
 *  @since      v5.0
*  Sample usage:        Site_t site = {10,20};   //x = 10 ,y = 20
                        LCD_point(site, RED);
 */
void LCD_point(Site_t site, uint16 rgb565)
{
    Size_t size = {1, 1};
    LCD_PTLON(site, size);
    LCD_RAMWR();                        //д�ڴ�
    LCD_WR_DATA(rgb565);                //д����
}

/*!
 *  @brief      ��һ�ѵ�
 *  @param      site        ����������
 *  @param      point_num   �������
 *  @param      rgb565      ��ɫ
 *  @since      v5.0
*  Sample usage:        Site_t site[3] = {{10,20},{11,21},{12,22}};   //3���㣬����ֱ���  {10,20},{11,21},{12,22}
                        LCD_points(site,3, RED);
 */
void LCD_points          (Site_t *site,uint32 point_num, uint16 rgb565)                 //��һ�ѵ�
{
    while(point_num--)
    {
        LCD_point(site[point_num],rgb565);                 //����
    }
}

/*!
 *  @brief      ��ʾ�ַ�
 *  @param      site    ���Ͻ�����
 *  @param      ascii   �ַ�
 *  @param      Color   ������ɫ
 *  @param      bkColor ������ɫ
 *  @since      v5.0
*  Sample usage:        Site_t site = {10,20};   //x = 10 ,y = 20
                        LCD_char(site,'0', BLUE,RED);
 */
void LCD_char(Site_t site, uint8 ascii, uint16 Color, uint16 bkColor)
{
#define MAX_CHAR_POSX (LCD_W-8)
#define MAX_CHAR_POSY (LCD_H-16)

    uint8 temp, t, pos;
    Size_t size = {8, 16};

    if(site.x > MAX_CHAR_POSX || site.y > MAX_CHAR_POSY)
    {
        return;
    }

    LCD_PTLON(site, size);

    LCD_RAMWR();                    //д�ڴ�

    for (pos = 0; pos < 16; pos++)
    {
        temp = ascii_8x16[((ascii-0x20)*16)+pos];

        for(t = 0; t < 8; t++)
        {

            if(temp & 0x80)
            {
                LCD_WR_DATA(Color);
            }
            else
            {
                LCD_WR_DATA(bkColor);
            }
            temp <<= 1;
        }
    }
    return;
#undef MAX_CHAR_POSX
#undef MAX_CHAR_POSY
}

/*!
 *  @brief      ��ʾ�ַ���
 *  @param      site    ���Ͻ�����
 *  @param      Str     �ַ�����ַ
 *  @param      Color   ������ɫ
 *  @param      bkColor ������ɫ
 *  @since      v5.0
*  Sample usage:        Site_t site = {10,20};   //x = 10 ,y = 20
                        LCD_str(site,"www.vcan123.com", BLUE,RED);
 */
void LCD_str(Site_t site, uint8 *Str, uint16 Color, uint16 bkColor)
{
#define MAX_CHAR_POSX (LCD_W-8)
#define MAX_CHAR_POSY (LCD_H-16)
    while(*Str != '\0')
    {
        if(site.x > MAX_CHAR_POSX )
        {
            //����
            site.x = 0;
            site.y += 16;
        }
        if(site.y > MAX_CHAR_POSY )
        {
            //һ��
            site.y = 0;
            site.x = 0;
        }

        LCD_char(site, *Str, Color, bkColor);
        site.x += 8;
        Str ++ ;
    }
#undef MAX_CHAR_POSX
#undef MAX_CHAR_POSY
}

/*!
 *  @brief      ��ʾ����
 *  @param      site    ���Ͻ�����
 *  @param      num     ����
 *  @param      Color   ������ɫ
 *  @param      bkColor ������ɫ
 *  @since      v5.0
*  Sample usage:        Site_t site = {10,20};   //x = 10 ,y = 20
                        LCD_num(site,123, BLUE,RED);
 */
void LCD_num(Site_t site, uint32 num, uint16 Color, uint16 bkColor)
{
    uint32 res = num;
    uint8 t = 0;
    Site_t sitetemp;
    sitetemp.y = site.y;

    if( num == 0 )
    {
        LCD_char(site, '0', Color, bkColor);
        return;
    }
    while( res )  /*�õ����ֳ���t*/
    {
        res /= 10;
        t++;
    }

    while(num)
    {
        sitetemp.x = site.x + (8 * (t--) - 8);
        LCD_char(sitetemp, (num % 10) + '0', Color, bkColor);
        num /= 10 ;
    }
}

/*!
 *  @brief      ��ʾ���֣���ն����λ��
 *  @param      site            ���Ͻ�����
 *  @param      num             ����
 *  @param      max_num_bit     ����λ��
 *  @param      Color           ������ɫ
 *  @param      bkColor         ������ɫ
 *  @since      v5.0
*  Sample usage:        Site_t site = {10,20};   //x = 10 ,y = 20
                        LCD_num_BC(site,123,5, BLUE,RED);
 */
void LCD_num_BC(Site_t site, uint32 num, uint8 max_num_bit, uint16 Color, uint16 bkColor)
{
    uint32 res = num;
    uint8 t = 0;
    Site_t sitetemp;
    Size_t size;

    sitetemp.y = site.y;

    if( num == 0 )
    {
        LCD_char(site, '0', Color, bkColor);

        site.x += 8;
        size.H  = 16;
        size.W  = 8 * (max_num_bit - 1);
        LCD_rectangle(site, size, bkColor);

        return;
    }
    while( res )            /*�õ����ֳ���t*/
    {
        res /= 10;
        t++;
    }
    if(t >= max_num_bit )    //������󳤶�
    {
        t = max_num_bit;
    }

    res = t;

    while( t != 0 )
    {
        sitetemp.x = site.x + (8 * (t--) - 8);
        LCD_char(sitetemp, (num % 10) + '0', Color, bkColor);
        num /= 10 ;
    }

    if(res != max_num_bit )
    {
        size.W = 8 * (max_num_bit - res);
        site.x += (8 * res);
        size.H  = 16;
        LCD_rectangle(site, size, bkColor);
    }
}

/*!
 *  @brief      �Ҷ�ͼ����ʾ
 *  @param      site            ���Ͻ�����
 *  @param      size            ��ʾͼ���С
 *  @param      img             ͼ���ַ
 *  @since      v5.0
 *  Sample usage:       Site_t site = {10,20};      //x = 10 ,y = 20
                        Size_t size = {320,240};    //W = 320,H = 240
                        LCD_Img_gray(site, size, img);
 */
void LCD_Img_gray(Site_t site, Size_t size, uint8 *img)
{
    uint32     total = (size.H * size.W);
    uint16     imgtemp;
    uint8     *pimg = (uint8 *)img;

    LCD_PTLON(site, size);                      //����
    LCD_RAMWR();                                //д�ڴ�

    while(total--)
    {
        imgtemp     = (uint16) * (pimg++);
        imgtemp = GRAY_2_RGB565(imgtemp);
        LCD_WR_DATA( imgtemp );               //дͼ������
    }
}

/*!
 *  @brief      ���ŻҶ�ͼ����ʾ
 *  @param      site            ���Ͻ�����
 *  @param      size            ��ʾͼ���С
 *  @param      img             ͼ���ַ
 *  @param      imgsize         ͼ���С
 *  @since      v5.0
 *  Sample usage:       Site_t site = {10,20};          //x = 10 ,y = 20
                        Size_t size = {80,60};          //W = 80,H = 60
                        Size_t imgsize = {320,240};     //W = 320,H = 240
                        LCD_Img_gray_Z(site, size, img,imgsize);
 */
void LCD_Img_gray_Z(Site_t site, Size_t size, uint8 *img, Size_t imgsize)
{

    uint32 temp, tempY;
    uint16 x, y;
    uint16 X, Y;
    uint8 *pimg = (uint8 *)img;
    uint16 rgb;

    LCD_PTLON(site, size);                      //����

    LCD_RAMWR();                                //д�ڴ�

    for(y = 0; y < size.H; y++)
    {
        Y = ( (  y * imgsize.H   ) / size.H) ;
        tempY = Y * imgsize.W ;

        for(x = 0; x < size.W; x++)
        {
            X = ( x * imgsize.W  ) / size.W ;
            temp = tempY + X;
            rgb = GRAY_2_RGB565(pimg[temp]);    //
            LCD_WR_DATA(rgb);
        }
    }
}



void LCD_Img_Binary(Site_t site, Size_t size, uint8 *img)
{
    uint32     total = (size.H * size.W) / 8;
    uint8     imgtemp;
    uint8       bitindex;
    uint8     *pimg = (uint8 *)img;

    LCD_PTLON(site, size);                      //����

    LCD_RAMWR();                                //д�ڴ�

    while(total--)
    {
        imgtemp     = *(pimg++);
        bitindex    = 8;
        while(bitindex--)
        {
            if( imgtemp & (0x01 << bitindex) )
            {
                LCD_WR_DATA( BINARY_COLOR );      //дͼ������
            }
            else
            {
                LCD_WR_DATA(  BINARY_BGCOLOR  );      //дͼ������
            }
        }
    }
}

void LCD_Img_Binary_Z(Site_t site, Size_t size, uint8 *img, Size_t imgsize)
{

    uint32 temp, tempY;
    uint16 x, y;
    uint16 X, Y;
    uint8 *pimg = (uint8 *)img;

    LCD_PTLON(site, size);                      //����

    LCD_RAMWR();                                //д�ڴ�

    for(y = 0; y < size.H; y++)
    {
        Y = ( (  y * imgsize.H  ) / size.H) ;
        tempY = Y * imgsize.W ;

        for(x = 0; x < size.W; x++)
        {
            X = (( x * imgsize.W  ) / size.W) ;
            temp = tempY + X;
            if( (pimg[temp>>3] & (1 << (7 - (temp & 0x07))) ) == 0  )
            {
                LCD_WR_DATA( BINARY_BGCOLOR );        //дͼ������
            }
            else
            {
                LCD_WR_DATA( BINARY_COLOR );      //дͼ������
            }
        }
    }
}

void LCD_wave(Site_t site,Size_t size,uint8 *img,uint8 maxval,uint16 Color ,uint16 bkColor)
{

    uint8 h;
    uint16 y = site.y + size.H - 1;

    LCD_rectangle(site, size,bkColor);
    site.x += size.W;
    img  += (size.W - 1);
    while(size.W --)
    {
        if(*img >= maxval )
        {
            h = size.H - 1;
        }
        else
        {
            h = ((*img )* size.H )/maxval;
        }
        site.y = y - h ;
        site.x--;
        img --;
        LCD_point(site, Color);
    }
}

void LCD_wave_display(Site_t site,Size_t size,uint8 *img,uint8 maxval,uint16 Color)
{

    uint8 h;
    uint16 y = site.y + size.H - 1;

    site.x += size.W;
    img  += (size.W - 1);
    while(size.W --)
    {
        if(*img >= maxval )
        {
            h = size.H - 1;
        }
        else
        {
            h = ((*img )* size.H )/maxval;
        }
        site.y = y - h ;
        site.x--;
        img --;
        LCD_point(site, Color);
    }
}