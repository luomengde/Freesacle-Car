#include "common.h"
#include "include.h"
#include "VCAN_camera.h"
#include "main.h"
#include "picture_deal.h"
#include "VCAN_LCD.h"
#include  "VCAN_LED.H"
#include "MK60_FTM.h"
#include "VCAN_UI_VAR.h"
uint8 left_line_first;
uint8 right_line_first;
uint8 left_line_first_flag;
uint8 right_line_first_flag;
#define l_judge_first_line(flag,y) flag==0?(left_line_first=y):(left_line_first=left_line_first)
#define r_judge_first_line(flag,y) flag==0?(right_line_first=y):(right_line_first=right_line_first)


/*  �ϰ�  */
#define Traffic(x)  line[x-3].line_case_mode==all_normal && line[x-2].line_case_mode==all_normal && line[x-1].line_case_mode==all_normal &&  line[x].line_case_mode==all_normal &&  line[x+1].line_case_mode==all_normal && line[x+2].line_case_mode==all_normal && line[x+3].line_case_mode==all_normal
#define Obstacle_Remeber obstacle_remeber[0]==0 && obstacle_remeber[1]==1 && obstacle_remeber[2]==1 //&& obstacle_remeber[3]==1 && obstacle_remeber[4]==1 && obstacle_remeber[5]==1 && obstacle_remeber[6]==1 && obstacle_remeber[7]==1
//�ж�����¼���Ϊ��ֵ����
#define Normal_mid_line(x)   ABS(line[x].mid_line_new-64)<=5&&ABS(line[x+1].mid_line_new-64)<=5&&ABS(line[x+2].mid_line_new-64)<=5&&ABS(line[x+3].mid_line_new-64)<=5&&ABS(line[x+4].mid_line_new-64)<=5
uint8  used_length;
line_info  line[line_num];
//��һ�м��
int16 last_mid;
float last_mid_line;//�ϴ�����
uint8 lost_count;
uint8 left_lost_count;
uint8 right_lost_count;
uint8 all_normal_count;

uint8 s_in,s_out;




//�������
uint8 distance[70]=
 {

	103   ,102	 ,100   , 100	 ,98  ,  98	, 95  ,  95	, 94  ,  92,
	 92   , 90	, 90   , 88	, 88   , 86	, 86   , 84	, 84   , 82,
	 82  ,  80	, 80 ,   78	, 77  ,  76	, 74   , 74	, 72   , 72,
	 70   , 69	, 68  ,  68	 ,66   , 66	, 64   , 64	, 62   , 62,
	 60   , 59	, 58  , 57	 ,56   , 55	, 54   , 53	, 52   , 52,
	 50   , 49	, 48  ,  47	 ,46   , 46	, 45   , 43 , 42   , 40,
	 40   , 38	, 38  ,  37	 ,36   , 35	, 34   , 32	, 32   , 30,
 };

/*����*/
uint16  mid_line_state[70];
uint8 black_startline;
uint8 black_endline;
uint8 black_start_flag;
uint8 black_end_flag;
uint8 lianxu_l;
uint8 lianxu_r;
uint8 l_lianxu_flag;
uint8 r_lianxu_flag;
uint8 black_first_line_qing;
uint8 zhijiao_forward;
uint8 zhijiao_forward1;
/*ֱ��*/
uint8 right_turn_zhijiao_flag;
uint8 left_turn_zhijiao_flag;
uint8 right_zhijiao_flag;
uint8 left_zhijiao_flag;
uint8 right_zhijiao;
uint8 left_zhijiao;
uint8 tubian_line_flag;
uint8 road_lianxu_tumble_flag;
uint8 road_lianxu_right_lose_flag;
uint8 road_lianxu_left_lose_flag;
uint8 left_first_lose_line_flag;
uint8 left_first_lose;
uint8 right_first_lose_line_flag;
uint8 right_first_lose;

//����
uint8 close_black_flag;
uint8 left_single_case;
uint8 right_single_case;
uint8 single_first_flag;
uint8 single_count;
uint8 single_first;
uint8 last_mid_right;
uint8 last_mid_left;
uint8 single_end_flag;
uint8 black;
uint8 single_end;
uint8 single_out;
extern uint8 single_out_flag;
uint8 single;
uint8 danxian_flag;
uint8 single_in;
uint8 alone;       //ÿһ��ͼ���ߵ�������������ֹ��һ�н���Ӱ��������

extern uint8 s_out; //���߼ǲ���־����
extern uint8 s_in;  //����
uint8 l_jibu_flag;
//�µ�
uint16 road_wide_add;
uint8  road_wide_use;
uint16 road_wide_normal;
extern uint8  podao_flag;
int16 podao_flag_time;
//////////////////////////ʮ��
uint8 last_lost_count;
uint8 shi_zi_flag;
uint8 jia_single;

//�ϰ�
int8 left_tubian;
int8 right_tubian;
//������
uint8 stop;
uint16 stop_count;
//�Լ�д��
uint8 Speed_protect=0;
uint8 Speed_protect_count=0;
uint32 shizi_new;     //ʮ���µ��㷨��ͳ�ư׵�������ֵ��Ϊ����ͼ����ֵ
//�Ӽ���·��ʶ��
uint16 the_far_road;    //Զ��ʮ��ͼ��
uint16 the_near_road;   //����ʮ��ͼ��
uint8  the_far_road_flag;  //��־λ
uint8  the_near_road_flag;


/*����б���е�*/
uint8 X_point[70];
uint8 Y_point[70];

int16 hang_mid_point[70];      //���д���õ��׵ĵ���дӶ������ֵ��Ϊ�ο�
uint8 shi_zi_count;
uint8 never_shi_zi_count=0;
uint16 left_add_point_count,right_add_point_count;

/*�ϰ��´���*/
uint8 never_obstacle_flag;
uint8 left_obstancle_flag;
uint8 right_obstancle_flag;

uint8 single_count_new;                 //���ж������е���
uint8 single_lost_count_new;            //���ж����ж���

uint8 into_circle_flag;               //����Բ
uint8 out_circle_flag;                //��Բ
uint8 out_circle_time_flag;
uint16 out_circle_count;
uint8 begin_the_frost_point;     //��ʼ������Ϸ��ĵ�
uint8 black_point_num[30];      //ÿһ�еĺڵ���
/*******************************************************************************
* Function Name: median_line_extract();
* Input Param  : src       img���飬160*120
* Output Param : ��
* Description  : ͼ����������ȡ������
* Author Data  : 2015/6/2 ������, by Li
*******************************************************************************/
void median_line_extract(uint8 *src)
 {
    static int16 last_mid_first;
	static int16 temp_dd_mid_line_add=0;
    uint8  *pimg;
    uint8 i,j;
    uint8 tmpsrc;
    uint8 single_case;
    last_mid=64;
    last_lost_count=lost_count;
    lost_count=0;//���㶼��ʧ����
    left_lost_count=0;
    right_lost_count=0;
    all_normal_count=0;
	left_add_point_count=0;
	right_add_point_count=0;
    left_line_first_flag=0;
    right_line_first_flag=0;
    single_in=0;              //����ı�Ϊ1��
    /*�µ�����*/         //ÿһ�����ж�Ҫ����
    road_wide_add=0;
    road_wide_use=0;
    road_wide_normal=0;


    single_count_new=0;
	single_lost_count_new=0;

    static int16 each_line_num,each_line_sum;
	static uint8 num_of_point,num_of_point_count;
    /*ѭ���߳���*/
	for(i=0;i<30;i++)           //Ŀǰ��ɨ��30�е���ת��ֻ����20��
	{
        pimg=src+(line_num-1-i)*128;             //Ҳ�Ǵӵ�������ɨ
        num_of_point_count=0;
        //ͳ������ʮ��ÿһ�еĺڵ㲢ƽ����
        num_of_point=127;      //ÿ�γ�ʼ��Ϊ127
		for(j=0;j<=num_of_point;j++)					 //ɨ����	//ÿһ�ζ����м俪ʼ
		{
		   tmpsrc=*(pimg+j);
		   if(tmpsrc==0)
			{
			   num_of_point_count++;
			   if(num_of_point_count==1)      //ֻ�е�һ���ڵ��ִ��
			   {
                   num_of_point=j+10;          //ֻɨ��һ���ڵ����ҵ�ʮ���㣬����Ҳû��
			   }
			   each_line_num++;
			   each_line_sum+=j;
			}
		}
		black_point_num[i]=each_line_num;     //ͳ��ûһ�кڵ���
		if(i==0&&each_line_num==0)
		{
            line[i].mid_line_new=64;
			single_lost_count_new++;
			line[i].road_wide=255;             //������ж�ʧ
		}
		else if(i>0&&each_line_num==0)
		{
            line[i].mid_line_new=line[i-1].mid_line_new;
			single_lost_count_new++;
			line[i].road_wide=255;             //������ж�ʧ
		}
		else
		{
            line[i].mid_line_new=each_line_sum/each_line_num;
			single_count_new++;
			line[i].road_wide=2;
		}
		each_line_sum=0;
		each_line_num=0;
		if(i>=1)
		{
            if(line[i-1].mid_line_new<=1||line[i-1].mid_line_new>=126)
            {
                line[i].mid_line_new=line[i-1].mid_line_new;
			}
		}
	}
	//���˵�����ѭ��
	
	each_line_sum=0;
	each_line_num=0;
	for(i=69;i>=60;i--)
		 line[i].mid_line_new=0;
	/*Ѱ���Ϸ�ʮ����*/
	if(begin_the_frost_point==1)
	{
		for(i=69;i>=60;i--)
		{
		    line[i].mid_line_new=0;
	        pimg=src+(line_num-1-i)*128;             //Ҳ�Ǵӵ�������ɨ
	        //ͳ������ʮ��ÿһ�еĺڵ㲢ƽ����
			for(j=0;j<=127;j++)					 //ɨ����	//ÿһ�ζ����м俪ʼ
			{
			   tmpsrc=*(pimg+j);
			   if(tmpsrc==0)
				{
				   each_line_num++;
				   each_line_sum+=j;
				}
			}
			if(each_line_num==0)
			{
				line[i].road_wide=255;             //������ж�ʧ
			}
			else
			{
	            line[i].mid_line_new=each_line_sum/each_line_num;
			}
			each_line_sum=0;
			each_line_num=0;
		}
	}

    /*���ͳ�������Ҫ��!!!!!������*/
//	out_circle_count=0;
	used_length=single_count_new;             //�������Ϊ��Ч��

	///////!!!!!����  ϸ������ս��ᵼ�½���Բ����   �������Ľ���Բ���  ����Բ�Ķ������м�������ϲ�
	if(Normal_mid_line(0)&&single_lost_count_new>=3&&line[28].mid_line_new<=5&&line[29].mid_line_new<=5&&black_point_num[28]>=1&&black_point_num[29]>=1)
	{
        if(line[29].mid_line_new-line[28].mid_line_new>=0&&line[28].mid_line_new-line[27].mid_line_new>=0)
        {
            into_circle_flag=1;
		}
	}
	if(into_circle_flag==1)
	{
        if(single_lost_count_new<=1&&ABS(before_cal_mid_line-64)<=25&&ABS(line[0].mid_line_new-64)<=20&&ABS(line[1].mid_line_new-64)<=20&&ABS(line[2].mid_line_new-64)<=20&&ABS(line[3].mid_line_new-64)<=20)
        {
            into_circle_flag=0;          //��Ϊ����Բ��
			out_circle_time_flag=1;
		}
	} 
	if(begin_the_frost_point==1&&ABS(line[68].mid_line_new-64)<=20&&ABS(line[67].mid_line_new-64)<=20&&ABS(line[66].mid_line_new-64)<=20&&ABS(line[65].mid_line_new-64)<=20&&line[68].mid_line_new-line[66].mid_line_new<=2&&line[67].mid_line_new-line[65].mid_line_new<=2)
	{
        out_circle_flag=1;
	}
	if(out_circle_flag==1)
	{
        if((line[0].road_wide==255&&line[1].road_wide==255)||(ABS(before_cal_mid_line-64)<=15&&single_count_new>=29))
        {
            out_circle_flag=0;
			begin_the_frost_point=0;
		}
	}

}
//�����������
void RoadIdentify()
{
    uint32 far_road_add,near_road_add;
	uint8  i,j;
	for(i=5;i<15;i++)
	{
        near_road_add+=line[i].mid_line_new;
	}
	the_near_road=near_road_add/10;
	for(i=used_length-15;i<used_length-5;i++)
	{
        far_road_add+=line[i].mid_line_new;
	}
	the_far_road=far_road_add/10;
	if(ABS(64-the_near_road)<=10)
	{
        the_near_road_flag=1;    //�������ӽ�ֱ�ߵı�־λ��1
	}
	else
	{
        the_near_road_flag=0;
	}
	if(ABS(64-the_far_road)>=20)
	{
        the_far_road_flag=1;    //��Զ����ת��ı�־λ��1
	}
	else
	{
        the_far_road_flag=0;
	}
	far_road_add=0;        //ÿһ�μ����궼�ǵ���0
	near_road_add=0;
	the_near_road=0;
	the_far_road=0;
}
/*���������*/
void start_line(uint8 *src)
{
   uint8 i,j;
   uint8 *pimg;                    //ˮƽɨ��
   uint8 *pimg_vertical;           //��ֱɨ��
   uint8 scan_change;

   uint8 left_tempsrc;
   uint8 right_tempsrc;

   uint8 tempsrc_ply;
   uint8 tempsrc_ply_last;

   uint8 left_scan_point;
   uint8 right_scan_point;

   uint8 left_scan_flag=0;
   uint8 right_scan_flag=0;
   uint8 mid_scan_flag=0;

   static uint8 white_to_black_num;
   static uint8 black_to_white_num;

   static uint8 white_to_black_num_left;
   static uint8 black_to_white_num_left;
   static uint8 white_to_black_num_right;
   static uint8 black_to_white_num_right;

   white_to_black_num_left=0;
   black_to_white_num_left=0;
   white_to_black_num_right=0;
   black_to_white_num_right=0;

   static uint8 left_start_line_ply;
   static uint8 right_start_line_ply;

   left_start_line_ply=0;
   right_start_line_ply=0;

   int16 left_Sabc_sl;
   int16 right_Sabc_sl;

//   Site_t site_end = {0,0};
   if(lost_count<=10&& last_mid_line>10 && last_mid_line<118&&stop==0 && podao_flag==0)
   {
     if(used_length>=50)          //���ɨ�������ʮ������
      scan_change=50;
     else
      scan_change= used_length;

    for(i=0;i<scan_change;i++)
     {
	     //�������ڲ���������
	     if(i>=2)
	     {
	       left_Sabc_sl=(line[i-1].left_line_unrepiar-line[i-2].left_line_unrepiar)*2-(line[i].left_line_unrepiar-line[i-2].left_line_unrepiar);
	       right_Sabc_sl=(line[i-1].right_line_unrepiar-line[i-2].right_line_unrepiar)*2-(line[i].right_line_unrepiar-line[i-2].right_line_unrepiar);
	       if(left_Sabc_sl<-7)
	       {
	        line[i].left_line_unrepiar=line[i-1].left_line_unrepiar+line[i-2].left_line_unrepiar-line[i-3].left_line_unrepiar;
	       }
	       if(right_Sabc_sl>7)
	       {
	        line[i].right_line_unrepiar=line[i-1].right_line_unrepiar+line[i-2].right_line_unrepiar-line[i-3].right_line_unrepiar;
	       }
	       //Ϊ�˱���
	       if(line[i].left_line_unrepiar<=0) line[i].left_line_unrepiar=0;
	       if(line[i].left_line_unrepiar>=127) line[i].left_line_unrepiar=127;
	       if(line[i].right_line_unrepiar>=127) line[i].right_line_unrepiar=127;
	       if(line[i].right_line_unrepiar<=0) line[i].right_line_unrepiar=0;
	       if(line[i].left_line_unrepiar>line[i].right_line_unrepiar)  line[i].left_line_unrepiar=line[i].right_line_unrepiar;

	       line[i].road_wide=line[i].right_line_unrepiar-line[i].left_line_unrepiar;

	     }

     left_scan_point=line[i].left_line_unrepiar+line[i].road_wide*7/25;
     right_scan_point=line[i].right_line_unrepiar-line[i].road_wide*7/25;

     //�����ֹBUG�ж�
     if(left_scan_point<=0) left_scan_point=0;
     if(left_scan_point>=127) left_scan_point=127;
     if(right_scan_point>=127) right_scan_point=127;
     if(right_scan_point<=0) right_scan_point=0;
     if(left_scan_point>right_scan_point)   left_scan_point= right_scan_point;

//      //����
//     site_end.y=line_num-1-i;
//     site_end.x=left_scan_point;
//     LCD_point(site_end, BLUE);
//     site_end.x=right_scan_point;
//     LCD_point(site_end, BLUE);


     pimg=src+(line_num-1-i)*128;

     left_tempsrc=*(pimg+left_scan_point);
     right_tempsrc=*(pimg+right_scan_point);
     //ִ��

     if(left_tempsrc==0)
     {
	     for(j=left_scan_point;j>line[i].left_line_unrepiar;j--)
	     {
	       //����ڱ��
	       if( (*(pimg+j)-*(pimg+j+1))!=0 && *(pimg+j)==255)
	       black_to_white_num++;
	       //����ױ��
	       if( (*(pimg+j)-*(pimg+j+1))!=0 && *(pimg+j)==0)
	       white_to_black_num++;
	     }
	     if(white_to_black_num<=1 && black_to_white_num==1)
	     	left_scan_flag=1;
	     black_to_white_num=white_to_black_num=0;
     }
       //�ж���·��ֱ���
     pimg_vertical=src+left_scan_point;
     tempsrc_ply=*(pimg_vertical+(line_num-1-i)*128);     //�ж�����
     tempsrc_ply_last=*(pimg_vertical+(line_num-i)*128);
     if(i>=1)
     {
	       //����ڱ��   ��ֱ���
	       if( (tempsrc_ply-tempsrc_ply_last)!=0 && tempsrc_ply==255)
	       {
	         black_to_white_num_left++;
	       }
	       //����ױ��   ��ֱ���
	       if( (tempsrc_ply-tempsrc_ply_last)!=0 && tempsrc_ply==0)
	       {
	         white_to_black_num_left++;
	       }
	       //�ж������߿��
	       if(white_to_black_num_left==1 && black_to_white_num_left==0)
	         left_start_line_ply++;

     }


     if(right_tempsrc==0)
     {
	     for(j=right_scan_point;j<=line[i].right_line_unrepiar;j++)
	     {
	       if( (*(pimg+j)-*(pimg+j-1))!=0 && *(pimg+j)==255)
	       black_to_white_num++;
	       if( (*(pimg+j)-*(pimg+j-1))!=0 && *(pimg+j)==0)
	        white_to_black_num++;
	     }
	     if(black_to_white_num==1 && white_to_black_num<=1)
	     right_scan_flag=1;
	     black_to_white_num=white_to_black_num=0;
     }

     //�ж���·��ֱ���
     pimg_vertical=src+right_scan_point;
     tempsrc_ply=*(pimg_vertical+(line_num-1-i)*128);
     tempsrc_ply_last=*(pimg_vertical+(line_num-i)*128);
     if(i>=1)
     {
	       //����ڱ��     ��ֱ���
	       if( (tempsrc_ply-tempsrc_ply_last)!=0 && tempsrc_ply==255)
	       {
	         black_to_white_num_right++;
	       }
	       //����ױ��    ��ֱ���
	       if( (tempsrc_ply-tempsrc_ply_last)!=0 && tempsrc_ply==0)
	       {
	         white_to_black_num_right++;
	       }
	       //�ж������߿��
	       if(white_to_black_num_right==1 && black_to_white_num_right==0)
	       	right_start_line_ply++;
     }

  }             //���ϴ����forѭ����ִ��
    if(black_to_white_num_right==1&&black_to_white_num_left==1&&white_to_black_num_right==1&&white_to_black_num_left==1)
    {
      mid_scan_flag=1;
    }
      if(left_scan_flag&&right_scan_flag&&mid_scan_flag&&left_start_line_ply<=10&&right_start_line_ply<=10)
     {
		led(LED0,LED_ON);
		led(LED1,LED_ON);
		led(LED2,LED_ON);
		led(LED3,LED_ON);
        stop=1;
        stop_count=0;
     }
   }

}


void zhijiao_wan()
 {
    uint8 i;
    uint8 tubian_line=0;
    int8 err_l=0;
    int8 err_r=0;
    uint8 road_l_lianxu_tumble=0;
    uint8 road_r_lianxu_tumble=0;
    uint8 road_l_lianxu_tumble_flag=0;
    uint8 road_r_lianxu_tumble_flag=0;

    uint8 l_lianxu_stop_flag=1 ;
    uint8 r_lianxu_stop_flag=1 ;
    uint8 road_lianxu_right_lose=0;
    uint8 road_lianxu_left_lose=0;
    road_lianxu_right_lose_flag=0;
    road_lianxu_left_lose_flag=0;
    road_lianxu_tumble_flag=0;

    tubian_line_flag=0;
    //��ֱ�����ж�
    if(used_length<70&&line[used_length-3].road_wide>60&&lost_count<10)     //    &&right_lost_count>5
     {
            for( i=5;i<used_length-1;i++)
             {
                if(left_first_lose<5&&line[i].left_line_unrepiar-line[i-1].left_line_unrepiar>-1&&left_lost_count<35)//&&line[i].road_wide>=(distance[i]-5)&&line[i].road_wide<=(distance[i]+5))
                 {
                    road_l_lianxu_tumble++;
                    if(road_l_lianxu_tumble>used_length-10)
                     {
                        road_l_lianxu_tumble_flag=1;
                     }
                 }
                if(tubian_line_flag==0&&line[i].right_line_unrepiar-line[i-1].right_line_unrepiar<1&&right_lost_count<35)
                 {
                    road_r_lianxu_tumble++;
                    if(road_r_lianxu_tumble>30)
                     {
                        road_r_lianxu_tumble_flag=1;
                     }
                 }
                if(road_lianxu_tumble_flag==0&&road_r_lianxu_tumble_flag&&road_l_lianxu_tumble_flag&&all_normal_count>30)
                 {
                    road_lianxu_tumble_flag=1;
                 }

                if(tubian_line_flag==0&&line[i].road_wide>=(distance[i]+10)&&line[i].left_line_unrepiar-line[i-1].left_line_unrepiar>-1&&line[i].line_case_mode==left_normal_right_lose&&line[i-2].line_case_mode==all_normal&&line[i-5].road_wide>(distance[i]-8) )//&&line[i-5].road_wide<(distance[i]+8)
                 {
                    tubian_line_flag=1;
                    tubian_line=i;
                 }
                if( i>=tubian_line&&tubian_line_flag==1&&line[i].left_line_unrepiar-line[i-1].left_line_unrepiar>-1&&line[i].line_case_mode==left_normal_right_lose&&((line[i].right_line_unrepiar+line[i].left_line_unrepiar)/2>line[0].mid_line_new))
                 {
                    road_lianxu_right_lose++;
                    if(road_lianxu_right_lose>4&&road_lianxu_left_lose<40)
                     {
                        road_lianxu_right_lose_flag=1;
                     }
                 }

                if(road_lianxu_tumble_flag&&road_lianxu_right_lose_flag&&tubian_line_flag&&line[used_length-3].left_line_unrepiar-line[0].left_line_unrepiar<60)
                 {
                    right_zhijiao_flag=1;

                   //   right_turn_zhijiao_flag=1;
                 }
               if( right_zhijiao_flag==1&&tubian_line<35&&tubian_line_flag==1)
                 {
                    right_turn_zhijiao_flag=1;
                    return;
                 }
         }
     }

    //��ֱ�����ж�
    if(used_length<70&&line[used_length-3].road_wide>60&&lost_count<10)  //&&left_lost_count>5
     {
            for( i=5;i<used_length-1;i++)
             {
                if(tubian_line_flag==0&&line[i].left_line_unrepiar-line[i-1].left_line_unrepiar>-1&&left_lost_count<35)//right_first_lose<5&//&&line[i].road_wide>=(distance[i]-5)&&line[i].road_wide<=(distance[i]+5))
                 {
                    road_l_lianxu_tumble++;
                    if(road_l_lianxu_tumble>30)
                     {
                        road_l_lianxu_tumble_flag=1;
                     }
                 }
                if(line[i].right_line_unrepiar-line[i-1].right_line_unrepiar<1&&right_lost_count<35)
                 {
                    road_r_lianxu_tumble++;
                    if(road_r_lianxu_tumble>used_length-10)
                     {
                        road_r_lianxu_tumble_flag=1;
                     }
                 }
                if(road_lianxu_tumble_flag==0&&road_r_lianxu_tumble_flag&&road_l_lianxu_tumble_flag&&all_normal_count>30)
                 {
                    road_lianxu_tumble_flag=1;
                 }
                if( tubian_line_flag==0&&line[i].road_wide>=(distance[i]+10)&&line[i].right_line_unrepiar-line[i-1].right_line_unrepiar<1&&line[i].line_case_mode==left_lose_right_normal&&line[i-2].line_case_mode==all_normal&&line[i-5].road_wide>(distance[i]-8)&&line[i-5].road_wide<(distance[i]+8) )
                 {
                    tubian_line_flag=1;
                    tubian_line=i;
                 }
                if( i>=tubian_line&&tubian_line_flag==1&&line[i].right_line_unrepiar-line[i-1].right_line_unrepiar<1&&line[i].line_case_mode==left_lose_right_normal&&((line[i].right_line_unrepiar+line[i].left_line_unrepiar)/2<line[0].mid_line_new))
                 {
                    road_lianxu_left_lose++;
                    if(road_lianxu_left_lose>4&&road_lianxu_left_lose<40)
                     {
                        road_lianxu_left_lose_flag=1;
                     }
                 }
                if(road_lianxu_tumble_flag&&road_lianxu_left_lose_flag&&tubian_line_flag&&line[used_length-3].right_line_unrepiar-line[0].right_line_unrepiar>-60)
                 {
                    left_zhijiao_flag=1;
               //    left_turn_zhijiao_flag=1;
                 }
                if(left_zhijiao_flag==1&&tubian_line<35&&tubian_line_flag==1)
                 {
                    left_turn_zhijiao_flag=1;
                    return;
                 }
             }

     }
 }




/*  ����  */
void avoid_obstacle(uint8 *src)
 {
    uint8 *pimg;
    uint8 scan[4][70];
    uint8 first_point;
    static uint8 obstacle_remeber[3]={0};
    uint8 i,j;
    uint8 obstacle_length,flag_obstacle;
    uint8 flag_obstacle_l,flag_obstacle_r,flag_obstacle_m,flag_obstacle_l_pre,flag_obstacle_r_pre,flag_obstacle_m_pre;
    static uint16 obstacle_sum=0,obstacle_delay_cnt=50;
    float offset;
    int8 err1,err2,err0;
    Site_t site_obstacle = {0,0};
    uint8 jia_zhangai=0;
    flag_obstacle=0;
    flag_obstacle_l=0;
    flag_obstacle_r=0;
    obstacle_length=0;


    /*  ����ɨ����,����ɨ��㣬���Թ̶�Ϊ����  */
    for(i=4;i<line_num;i++)
     {
        offset=0.5;//0.45;
        site_obstacle.y=line_num-i;

        site_obstacle.x=line[0].right_line-10-(uint8)(offset*i);//104
        scan[1][i]=site_obstacle.x;//left1
//        LCD_point(site_obstacle,BLUE);



        site_obstacle.x=line[0].left_line+10+(uint8)(offset*i);
        scan[0][i]=site_obstacle.x;//right1
//        LCD_point(site_obstacle,BLUE);

     }

    for(i=4;i<(used_length>66?66:used_length);i++)//70-->used_length������ǰ����,���������
     {

        err0=line[i-3].mid_line_new-line[i-1].mid_line_new;//����ǰƫ��
        err1=line[i-1].mid_line_new-line[i].mid_line_new;//����ʱƫ��
        if(ABS(err0)<5 && ABS(err1)>5 && Traffic(i)&&jia_zhangai==0)
         {
            first_point=i;
            /*  ȷ���ϰ�λ���������ı� */
            if(err1<0)
             {
                left_tubian=(line[i].left_line-line[i-1].left_line)/2-line[i].mid_line_new+line[i-1].mid_line_new;
                flag_obstacle=1;//left
                flag_obstacle_l_pre=1;
                flag_obstacle_m_pre=0;
                flag_obstacle_r_pre=0;
             }
            else
             {
                right_tubian=(line[i-1].right_line-line[i].right_line)/2-line[i-1].mid_line_new+line[i].mid_line_new;;
                flag_obstacle=2;//right
                flag_obstacle_l_pre=0;
                flag_obstacle_m_pre=0;
                flag_obstacle_r_pre=1;
             }
            /*  ������㿪ʼȷ���ϰ��������ϰ�����  */
            switch(flag_obstacle)
             {
              case 1://left
                {
                   for(j=i;j<used_length-4;j++)//10?
                    {
                       pimg=src+(line_num-1-j)*128;



                       /*  ɨ�ұ��Ƿ�Ϊ��ɫ����������  */
                       if(*(pimg+scan[1][j])==255&&ABS(left_tubian)<5 && flag_obstacle_r_pre==0)
                        {
                           flag_obstacle_r=0;
                        }
                       else
                        {
                           break;
                        }
                       /*  ɨ�м��Ƿ�Ϊ��ɫ����������  */
                       if((*(pimg+64)==255||*(pimg+74)==255) && flag_obstacle_m_pre==0)
                        {
                           flag_obstacle_m=0;
                        }
                       else
                        {
                           break;
                        }

                       /*  ɨ����Ƿ�Ϊ��ɫ���ǣ����㳤�ȣ���������  */
                      if((*(pimg+scan[0][j])==0)&&flag_obstacle_l_pre==1)   //   //&&line[j-1].road_wide-line[j-2].road_wide<=0&&line[j].road_wide-line[j-1].road_wide>0
                        {
                           flag_obstacle_l=1;
                           obstacle_length=j-i;
                        }
                       else
                        {
                           break;
                        }
                    }
                   break;
                }
              case 2://right
                {
                   for(j=i;j<used_length-4;j++)//10?
                    {
                       pimg=src+(line_num-1-j)*128;



                       /*  ɨ����Ƿ�Ϊ��ɫ����������  */
                       if( *(pimg+scan[0][j])==255&&ABS(right_tubian)<5 && flag_obstacle_l_pre==0)
                        {
                           flag_obstacle_l=0;
                        }
                       else
                        {
                           break;
                        }
                       /*  ɨ�м��Ƿ�Ϊ��ɫ����������  */
                       if((*(pimg+54)==255||*(pimg+64)==255) && flag_obstacle_m_pre==0)
                        {
                           flag_obstacle_m=0;
                        }
                       else
                        {
                           break;
                        }

                       /*  ɨ�ұ��Ƿ�Ϊ��ɫ���ǣ����㳤�ȣ���������  */
                       if((*(pimg+scan[1][j])==0)&& flag_obstacle_r_pre==1)   //       //line[j-1].road_wide-line[j-2].road_wide<=0&&line[j].road_wide-line[j-1].road_wide>0
                        {
                           flag_obstacle_r=1;
                           obstacle_length=j-i;
                        }
                       else
                        {
                           break;
                        }
                    }
                   break;
                }
              default:
                break;
             }
            /*  ������ϰ������д���  */
			static uint8 obstancle_recognize_count;
            if(obstacle_length>5)//8?
             {
                obstacle_sum++;
                obstacle_remeber[0]=1;
                for(j=2;j>0;j--)
                 {
                    obstacle_remeber[j]=obstacle_remeber[j-1];
                 }
                /*  �ϰ�����  */
                switch(flag_obstacle)
                 {
                  case 1://��+
                    obstancle_recognize_count++;
					if(obstancle_recognize_count>=1)
					{
						left_obstancle_flag=1;
						never_obstacle_flag=1;
						obstancle_recognize_count=0;
					}
                    for(j=0;j<used_length;j++)
                     {

                        line[j].mid_line_new=line[i].mid_line_new+8;
                     }
                    break;
                  case 2://��-
                    obstancle_recognize_count++;
                  	if(obstancle_recognize_count>=1)
					{
						right_obstancle_flag=1;
						never_obstacle_flag=1;
						obstancle_recognize_count=0;
					}
                    for(j=0;j<used_length;j++)
                     {

                        line[j].mid_line_new=line[i].mid_line_new-8;
                     }
                    break;
                  default:
                    break;
                 }
             }
            else
             {
                obstacle_remeber[0]=0;
             }
            break;
         }
     }
 }
