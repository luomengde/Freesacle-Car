#include "include.h"
#include "common.h"

extern void Regression_cal(uint8 *X_buff,uint8 *Y_buff,uint8 AvaliableLines);
float DJ_fuzzy_kp(float e,float ec);
extern float DJ_fuzzy_kd(float e,float ec);
unsigned int sqrt_16(unsigned long M);
#define Normal_mid_line(x)   ABS(line[x].mid_line_new-64)<=5&&ABS(line[x+1].mid_line_new-64)<=5&&ABS(line[x+2].mid_line_new-64)<=5&&ABS(line[x+3].mid_line_new-64)<=5&&ABS(line[x+4].mid_line_new-64)<=5



//**�������**//
uint32 dir_duty;    //���ڴ��ݵĶ��ռ�ձȲ���
float mid_line;		//���������һ֡ͼ�����ֵӦ���� float
float mid_line_new;
float Kp_duoji;    //���Kp
float Kd_duoji;    //���Kd

//б��
float index_A,index_B;    //index_AΪб��  index_BΪ�ؾ�   б��һ���Ϊ����
//�µ�
uint8  podao_flag=0;
extern uint16 road_wide_normal;
uint8 never_podao_flag=0;

extern line_info  line[line_num];
extern uint8 zhijiao_forward;

extern uint8  lost_count;      //��֡ͼ�����Ҷ���ʧ������
/*�������*/
int32 SE_duty;                  //�����PWMռ�ձ�
int32 se_du;
int16 ML_duty=0;       //���PWMռ�ձ�
int16 MR_duty=0;
float Ki;                      //���PID
float Kp;
float Kp1=15;
float Kp2=0.36;
float Kp1_you;
float Kp2_you;
float Kd=4.38;
float error,error_d,error_pre; //ƫ��
float dealt_error;    //ƫ��仯��
int8 mid_line_last[5]={0};   //�����ȥ��������ֵ     ����Ϊ��   ���ܶ���Ϊuint8
uint8 test_count=0;
uint8 podao_count=0;            //���ڼ��ν����µ�����μ��٣�ż�β�����
uint16 sqrt_16_kp2;      //����(line_num-used_length)��1.5�η�
int32  obstancle_time_pluse;
float  before_cal_mid_line;     //���⴦��֮ǰ������
uint8 quan2[70]={
   1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
   1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
   1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
   1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
   1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
   1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
   1,  1,  1,  1,  1,  1,  1,  1,  1,  1
};             //Ȩ��ȫΪ1��Ҳ���ǽ������ۼ�Ȼ��ȡƽ��ֵ
uint8 quan1[70]={
   1,  1,  2,  2,  2,  5,  6,  7,  8,  9,//zuichu
   42,  44,  46,  48,  50,  66,  67,  68,  69,  70,
   191,192,193,194,195,195,196,197,198,199,
   190, 191, 192, 193, 194, 194, 193, 192, 191, 190,
   129, 128, 127, 126, 125, 100, 100, 100, 100, 100,
   100, 100, 100, 100, 100, 100, 100, 100, 100, 100,
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};
uint8 quan1_gongda_beifen[70]={
   1,  1,  2,  2,  2,  5,  6,  7,  8,  9,//zuichu
   12,  14,  16,  18,  20,  26,  27,  28,  29,  30,
   131,142,153,154,165,195,196,197,198,199,
   190, 191, 192, 193, 194, 164, 163, 162, 161, 160,
   129, 128, 127, 126, 125, 124, 123, 122, 121, 120,
   119, 118, 117, 116, 115, 114, 113, 112, 111, 110,
   0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

uint8 quan5[70]={
   1,  1,  2,  2,  2,  5,  6,  7,  8,  9,//zuichu
   12,  14,  16,  18,  20,  26,  27,  28,  29,  30,
   131,142,153,154,165,195,196,197,198,199,
   190, 191, 192, 193, 194, 164, 163, 162, 161, 160,
   129, 128, 127, 126, 125, 1,  1,  1,  1,  1 ,
   1,  1,  1,  1,  1,  1,  1,  1,  1,  1 ,
   1,  1,  1,  1,  1,  1,  1,  1,  1,  1
};

uint8 quan3[70]={                   //��quan1���,��������Ȩ�ؼӴ�һЩ
   102,  104,  106,  108,  110,  116,  117,  118,  129,  130,
   102,  104,  106,  108,  110,  116,  117,  118,  129,  130,
   131,142,153,154,165,195,196,197,198,199,
   190, 191, 192, 193, 194, 164, 163, 162, 161, 160,
   129, 128, 127, 126, 125, 124, 123, 122, 121, 120,
   119, 118, 117, 116, 115, 114, 113, 112, 111, 110,
   109, 108, 107, 106, 105, 104, 103, 102, 101, 100
};
uint8 quan_liangge[70]={        //��������Ȩ
   2,  2,  2,  2,  2, 2,  2,  2,  3,  3,
   3, 3, 3, 3, 3, 3, 4, 4,  4,  4,
   7,  7,  7,  7,  7,  7,  8,  8, 8,  8,
    8,  8,  8, 8,  8,  8,  8,  8,  8,  8,
   8,  8,  8, 8,  8,  9,  9,  9,  9,  9,
     8,  8,  8,  8,  8, 9,  9,  9,  9,  9,
   0,  0, 0, 0,  10, 10,  10,  0,  0, 0,
};

uint8 quan[70]={        //��������Ȩ
   2,  2,  2,  2,  2, 2,  2,  2,  3,  3,
   3, 3, 3, 3, 3, 3, 4, 4,  4,  4,
   7,  7,  7,  7,  7,  7,  8,  8, 8,  8,
    7,  7, 7, 7,  7,  7,  7,  7,  7, 7,
   6,  6,  6, 6,  6,  6,  6,  6,  6,  6,
      8,  8, 0, 0,  0, 0,  0,  0,  0, 0,
   0,  0, 0, 0,  0, 0,  0,  0,  0, 0,
};
uint8 quan10[70]={
   2,  2,  2,  2,  2, 2,  2,  2,  3,  3,
   3, 3, 3, 3, 3, 3, 4, 4,  4,  4,
   7,  7,  7,  7,  7,  7,  8,  8, 8,  8,
    8,  8,  8, 8,  8,  8,  8,  8,  8,  8,
   8,  8,  8, 8,  8,  9,  9,  9,  9,  9,
     8,  8,  8,  8,  8, 9,  9,  9,  9,  9,
   0,  0, 0, 0,  0, 0,  0,  0,  0, 0,
};

uint8 quan_change[70]={                            //Ŀǰʮ�ֹ��У�ע����
	10,	10,	10,	10,	10, 10,  10,  10,  10,  10,
	10, 10, 10, 10, 10, 10, 10, 10,  10,  10,
	8,	8,	8,	8,	8,	8,	8,	8,  8,  8,
	8,	8,	8,	8,	8,	8, 8, 8, 8,  8,            //
	8,	8,	8, 8,  8,  8,	8,	8, 8,  8,
	  8,  8,  8,  8,  8, 8,  8,  8,  8,  8,
	0,	0, 0, 0,  0,  0,  0,	0,	0, 0,
};
uint8 quan_change_beifen[70]={
	3,	3,	3,	3,	3, 3,  3,  3,  3,  3,
	6, 6, 6, 6, 6, 6, 6, 6,  6,  6,
	8,	8,	8, 8,  8,  8,  8,  8,  8,  8,
	8,	8,	8,	8,	8,	8,     8,	8, 8,  8,
	8,	8,	8, 8,  8,  8,  8,  8,  8,  8,
	  8,  8,  8,  8,  8, 8,  8,  8,  8,  8,
	0,	0, 0, 0,  0,  0,  0,	0,	0, 0,
};


uint8 quan8[70]={
	2,	2,	2,	2,	2, 2,  2,  2,  3,  3,    //������¿�ʼ���
	4, 4, 4, 4, 4, 5, 5, 5,  5,  5,
	8,	8,	8,	8,	8,	8,	8,	8, 8,  8,
	 9,  9,  9, 9,	9,	9,	9,	9,	9,	9,
	 9,  9,  9, 9,	9,	9,	9,	9,	9,	9,
     8,  8,  8, 8,	8,	8,	8,	8,	8,	8,
	0,	0, 0, 0,  10, 10,  10,	0,	0, 0,
};

uint8 quan_change_zanshi[70]={                            //Ŀǰʮ�ֹ��У�ע����
	3,	3,	3,	3,	3, 3,  3,  3,  3,  3,
	6, 6, 6, 6, 6, 7, 7, 7,  7,  7,
	8,	8,	8,	8,	8,	9,	9,	9,  9,  9,
	9,	9,	9,	9,	9,	9, 9, 9, 9,  9,            //
	8,	8,	8, 8,  8,  8,	8,	8, 8,  8,
	  8,  8,  8,  8,  8, 7,  7,  7,  7,  7,
	0,	0, 0, 0,  0,  0,  0,	0,	0, 0,
};




/* Function Name: dir_duty_new();
* Description  : ���߼�Ȩ�����Ͷ���������
* Author Data  : 2015/4/4 , by Li*/
void quan_dir_duty_new()
 {
    uint8 i;                                        //����������ȼ���µ�
    static int32 num,add;
	static uint8 temp_mid_line_into_circle=64;

    last_mid_line=mid_line;               //������һ�ε�ͼ����ֵ

    for(i=0;i<20;i++)                  //����������Ȩ���������ڵ�
     {
         if(line[i].mid_line_new!=200)
         {
			 num=(line[i].mid_line_new)*quan1[i]+num;		//��Ȩÿһ�е�mid_line_new����
			 add=add+quan1[i];
		 }
     }
    if(add>0)
    {
        mid_line=(float)num/(float)add;
    }
	else
	{
        mid_line=last_mid_line;             //���ȫ��ʧ�򱣳���һ�ε�ͼ��
	}
	before_cal_mid_line=mid_line;
	/*���⴦��*/
	if(into_circle_flag==1)         //����Բ��һС�β�Ҫ���
	{
        mid_line=temp_mid_line_into_circle;
	}
	if(out_circle_flag==1)
	{
        mid_line=temp_mid_line_into_circle;
	}
	mid_line_last[4]=mid_line_last[3];          //��������ͼ����ֵ
	mid_line_last[3]=mid_line_last[2];
	mid_line_last[2]=mid_line_last[1];
	mid_line_last[1]=mid_line_last[0];
	mid_line_last[0]=mid_line;
    num=0;                         //����
    add=0;
    if(mid_line>128)               //�Լ�Ȩ������ֵ�����޷�
     {
        mid_line=128;
     }
    else if(mid_line<0)
     {
        mid_line=0;
     }

	error_pre=error;
	error=64-mid_line;    //��ת error>0   ��ת error<0
    error_d=error-error_pre;       //�������յõ���ʵ����    last_mid_line-mid_line !(֮ǰд���ˣ��ֲ��ö��D��������!!!!!!!!!!!!!)
    Regression_cal(X_point,Y_point,used_length);                      //�������ǰ����б��
//	dealt_error=error_d/(0.035);
 }



#define MID_dir_duty (49500)     //���������ֵ           //Ӳ�����³���ֵ48600
#define MAX_dir_duty (61000)     //�����  61050     11900
#define MIN_dir_duty (38100)     //�Ҵ���  38100     11050


/*���PD���ƺ���*/    //���Գ��Ա���㷨 ����㷨�����ľ�����
#define nWidth  128
#define nHeight 70
void DJ_PID(void)
{
    //���Լ�ǰ������PID
    Kp1=flash_duoji_Kp1;
	Kp2=flash_duoji_Kp2;
	Kp1_you=flash_duoji_Kp1_you;
	Kp2_you=flash_duoji_Kp2_you;
//	Kp1_you=Kp1;                                            //���һ���һ��
//	Kp2_you=Kp2;
	Kd= 600;
//    if(ABS(error)<=30)        //С���Dֵ��С��Ǿͺ�        ����СDֵ  ��Ǳ�̫��
//		Kd=800;
	sqrt_16_kp2=sqrt_16((30-used_length)*(30-used_length)*(30-used_length));
//    Kp=Kp1+(nHeight-used_length)*(nHeight-used_length)*Kp2/100;
    Kp=Kp1+sqrt_16_kp2*Kp2/50.0;
    /*�ϰ���ʱ����ʹ�����*/
    if(Normal_mid_line(0)&&Normal_mid_line(5)&&ABS(error)<=15)       //��ֱ����������ô��
    {
        Kd=400;
//		Kp=150;
    }
	if(podao_flag==1)
	{
        Kd=200;
	}
	if(Speed_protect==1)
	{
        Kp=125;
	}

    se_du=(int32)(Kp*error+Kd*error_d);   //���������ߣ���errorС���㣬se_du<0 ���Ӹ���ת
                                          //���ռ�ձ�Ӧ�ô�����ֵ
//    if(ABS(error)<=10&&used_length>=70)
//		se_du=0;

    SE_duty=(uint32)(MID_dir_duty-se_du);   //��ʾ����ת��


	/*�Զ���ʻ�����ֶ���ʻ*/
    static int32 the_last_duty;
    if(auto_car_flag==1)
    {
        the_last_duty=SE_duty;
	}
	else    //�ֶ���ʻ
	{
        if(left_step==1)
        {
            the_last_duty-=2000;
			left_step=0;
		}
        if(right_step==1)
        {
            the_last_duty+=2000; 
			right_step=0;
		}		
		SE_duty=the_last_duty;
	}
	
		
    if(SE_duty>MAX_dir_duty)
        SE_duty=MAX_dir_duty;
    else if(SE_duty<MIN_dir_duty)
        SE_duty=MIN_dir_duty;
#if defined(SHANWAI)
    ftm_pwm_duty(FTM1,FTM_CH1,SE_duty);         //�ж�ʲôʱ��������Ҽ���
//    if(SE_duty>=MAX_dir_duty-1000)
////		gpio_set(PTD4,1);
//	else
//		gpio_set(PTD4,0);
//	if(SE_duty<=MIN_dir_duty+1000)
////		gpio_set(PTD4,1);
//	else
//		gpio_set(PTD4,0);
#elif defined(LONGQIU)
    ftm_pwm_duty(FTM0,FTM_CH4,SE_duty);
#endif
}

void DJ_PID_New(void)
{
    Kp=DJ_fuzzy_kp(error/20,error_d/10);
	if(ABS(error)<=25&&used_length>=60)
	{
        Kp=120;
	}
	Kd= flash_duoji_Kd;

    se_du=(int32)(Kp*error+Kd*error_d);   //���������ߣ���errorС���㣬se_du<0 ���Ӹ���ת
                                          //���ռ�ձ�Ӧ�ô�����ֵ
//    if(se_du>9500)
//		se_du=9500;
//	if(se_du<-9500)
//		se_du=-9500;
    last_mid_line=mid_line;               //������һ�ε�ͼ����ֵ
//    if(podao_flag==1)       //��⵽�µ��򲻴��
//		se_du=0;

    SE_duty=(uint32)(MID_dir_duty-se_du);   //��ʾ����ת��
    if(SE_duty>MAX_dir_duty)
        SE_duty=MAX_dir_duty;
    else if(SE_duty<MIN_dir_duty)
        SE_duty=MIN_dir_duty;
#if defined(SHANWAI)
    ftm_pwm_duty(FTM1,FTM_CH1,SE_duty);         //�ж�ʲôʱ��������Ҽ���
//    if(SE_duty>=MAX_dir_duty-1000)
////		gpio_set(PTD4,1);
//	else
//		gpio_set(PTD4,0);
//	if(SE_duty<=MIN_dir_duty+1000)
////		gpio_set(PTD4,1);
//	else
//		gpio_set(PTD4,0);
#elif defined(LONGQIU)
    ftm_pwm_duty(FTM0,FTM_CH4,SE_duty);
#endif
}


/****************************************/
/*Function: �����Ŵ���                  */
/*��ڲ���������������������            */
/*���ڲ������������������              */
/****************************************/
unsigned int sqrt_16(unsigned long M)
{
    unsigned int N, i;
    unsigned long tmp, ttp;   // �����ѭ������
    if (M == 0)               // �����������������ҲΪ0
        return 0;
    N = 0;
    tmp = (M >> 30);          // ��ȡ���λ��B[m-1]
    M <<= 2;
    if (tmp > 1)              // ���λΪ1
    {
        N ++;                 // �����ǰλΪ1������ΪĬ�ϵ�0
        tmp -= N;
    }
    for (i=15; i>0; i--)      // ��ʣ���15λ
    {
        N <<= 1;              // ����һλ
        tmp <<= 2;
        tmp += (M >> 30);     // ����
        ttp = N;
        ttp = (ttp<<1)+1;
        M <<= 2;
        if (tmp >= ttp)       // �������
        {
            tmp -= ttp;
            N ++;
        }
    }
    return N;
}


/*! @brief ��С���˷�
 *  @note  Y=A+BX
 *  @data 2015/3/15
 */
//���ô˺������ݲ���������Ϊ��Ч��
void Regression_cal(uint8 *X_buff,uint8 *Y_buff,uint8 AvaliableLines)
{
    int16 cnt_R;
    int16 SumX,SumY;
    float AverageX,AverageY,SumUp,SumDown;
    SumX=0;
    SumY=0;
    for(cnt_R=0;cnt_R<AvaliableLines;cnt_R++)
    {
        SumX+=X_buff[cnt_R];
        SumY+=Y_buff[cnt_R];
    }
    AverageX=(float)SumX/(float)AvaliableLines;
    AverageY=(float)SumY/(float)AvaliableLines;
    SumUp=0;
    SumDown=0;
    for(cnt_R=0;cnt_R<AvaliableLines;cnt_R++)
    {
        SumUp+=(Y_buff[cnt_R]-AverageY)*(X_buff[cnt_R]-AverageX);
        SumDown+=(X_buff[cnt_R]-AverageX)*(X_buff[cnt_R]-AverageX);
    }
    if(SumDown==0) index_B=0;
    else index_B=(SumUp/SumDown);
    index_A=(SumY-index_B*SumX)/AvaliableLines;
}


/*! @brief ���������
 *  @note
 *  @data
 */
float Cal_AngleArea(int16 x1,int16 y1,int16 x2,int16 y2,int16 x3,int16 y3)
{
    float temp;
    temp=((x2-x1)*(y3-y1)-(x3-x1)*(y2-y1))/2;
    return temp;
}
/*! @brief ����
 *  @note
 *  @data
 */
uint8 m_sqrt(uint8 x)
{
    uint8 ans=0,p=0x80;
    while(p!=0)
    {
        ans+=p;
        if(ans*ans>x)
        {
            ans-=p;
        }
        p=(uint8)(p/2);
    }
    return(ans);
}

/*! @brief ���
 *  @note
 *  @data
 */
float Cal_Distance(int16 X1,int16 Y1,int16 X2,int16 Y2)
{
    float temp;
    temp=m_sqrt((X1-X2)*(X1-X2)-(Y1-Y2)*(Y1-Y2));
    return temp;
}

/*! @brief ����
 *  @note
 *  @data
 */
float cal_curvature(int16 x1,int16 y1,int16 x2,int16 y2,int16 x3,int16 y3)
{
//    float temp,temp1,d1,d2,d3;
//    y1=Y_change[y1];
//    y2=Y_change[y2];
//    y3=Y_change[y3];
//    x1=(int16)((x1-64)*X_change[y1]);//�е��Ϊ0 ����ϵ����
//    x2=(int16)((x2-64)*X_change[y2]);
//    x3=(int16)((x3-64)*X_change[y3]);
//    temp=Cal_AngleArea(x1,y1,x2,y2,x3,y3);
//    d1=Cal_Distance(x1,y1,x2,y2);
//    d2=Cal_Distance(x3,y3,x2,y2);
//    d3=Cal_Distance(x3,y3,x1,y1);
//    if(d1==0) d1=1;
//    if(d2==0) d2=1;
//    if(d3==0) d3=1;
//    temp1=temp*4/d1/d2/d3;
//    return temp1;
}

/*ģ������Kp*/
float DJ_fuzzy_kp(float e,float ec)
{
    float Kp_calcu;
	uint8 num,pe,pec;
	float eRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0};        //��������
	float ecRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0};       //���仯�ʵ�����
	float eFuzzy[2]={0.0,0.0};                              //���������̶�
	float ecFuzzy[2]={0.0,0.0};                             //���仯�ʵ������̶�
	float kpRule[4]={0.0,80.0,160.0,240.0};                    //Kp��ģ���Ӽ�
	float KpFuzzy[4]={0.0,0.0,0.0,0.0};                     //������Kp�������̶�
	int   KpRule[7][7]=                    //Kp��ģ�����Ʊ�
	{   /*��Ϊpe*/
        3,3,3,3,3,3,3,    /*��Ϊpec*/
		2,2,2,2,2,2,2,
		1,1,1,1,1,1,1,
		1,1,0,1,0,1,1,
		0,0,1,0,0,1,0,
		0,1,0,1,0,0,2,
		3,3,3,3,3,3,3,
	};
	/*���e�������̶�����*/
	if(e<eRule[0])
	{
	    eFuzzy[0]=1.0;
		pe=0;
	}
	else if(eRule[0]<=e&&e<eRule[1])
	{
        eFuzzy[0]=(eRule[1]-e)/(eRule[1]-eRule[0]);
		pe=0;
	}
	else if(eRule[1]<=e&&e<eRule[2])
	{
        eFuzzy[0]=(eRule[2]-e)/(eRule[2]-eRule[1]);
		pe=1;
	}
	else if(eRule[2]<=e&&e<eRule[3])
	{
        eFuzzy[0]=(eRule[3]-e)/(eRule[3]-eRule[2]);
		pe=2;
	}
	else if(eRule[3]<=e&&e<eRule[4])
	{
        eFuzzy[0]=(eRule[4]-e)/(eRule[4]-eRule[3]);
		pe=3;
	}
	else if(eRule[4]<=e&&e<eRule[5])
	{
        eFuzzy[0]=(eRule[5]-e)/(eRule[5]-eRule[4]);
		pe=4;
	}
	else if(eRule[5]<=e&&e<eRule[6])
	{
        eFuzzy[0]=(eRule[6]-e)/(eRule[6]-eRule[5]);
		pe=5;
	}
	else
	{
        eFuzzy[0]=0.0;
		pe=5;
	}
	eFuzzy[1]=1.0-eFuzzy[0];
	/*���仯��ec�ĵ�������������*/
	if(ec<ecRule[0])
	{
        ecFuzzy[0]=1.0;
		pec=0;
	}
	else if(ecRule[0]<=ec&&ec<ecRule[1])
	{
        ecFuzzy[0]=(ecRule[1]-ec)/(ecRule[1]-ecRule[0]);
		pec=0;
	}
	else if(ecRule[1]<=ec&&ec<ecRule[2])
	{
        ecFuzzy[0]=(ecRule[2]-ec)/(ecRule[2]-ecRule[1]);
		pec=1;
	}
	else if(ecRule[2]<=ec&&ec<ecRule[3])
	{
        ecFuzzy[0]=(ecRule[3]-ec)/(ecRule[3]-ecRule[2]);
		pec=2;
	}
	else if(ecRule[3]<=ec&&ec<ecRule[4])
	{
        ecFuzzy[0]=(ecRule[4]-ec)/(ecRule[4]-ecRule[3]);
		pec=3;
	}
	else if(ecRule[4]<=ec&&ec<ecRule[5])
	{
        ecFuzzy[0]=(ecRule[5]-ec)/(ecRule[5]-ecRule[4]);
		pec=4;
	}
	else if(ecRule[5]<=ec&&ec<ecRule[6])
	{
        ecFuzzy[0]=(ecRule[6]-ec)/(ecRule[6]-ecRule[5]);
		pec=5;
	}
	else
	{
        ecFuzzy[0]=0.0;
		pec=5;
	}
	ecFuzzy[1]=1.0-ecFuzzy[0];
	/*��ѯģ�������*/
	num=KpRule[pe][pec];
	KpFuzzy[num]+=eFuzzy[0]*ecFuzzy[0];

	num=KpRule[pe][pec+1];
	KpFuzzy[num]+=eFuzzy[0]*ecFuzzy[1];

	num=KpRule[pe+1][pec];
	KpFuzzy[num]+=eFuzzy[1]*ecFuzzy[0];

	num=KpRule[pe+1][pec+1];
	KpFuzzy[num]+=eFuzzy[1]*ecFuzzy[1];
	/*��Ȩƽ���㷨��ģ��*/
	Kp_calcu=KpFuzzy[0]*kpRule[0]+KpFuzzy[1]*kpRule[1]+KpFuzzy[2]*kpRule[2]+KpFuzzy[3]*kpRule[3];
	return(Kp_calcu);
}

/*ģ������Kp*/
float DJ_fuzzy_kd(float e,float ec)
{
    float Kp_calcu;
	uint8 num,pe,pec;
	float eRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0};        //��������
	float ecRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0};       //���仯�ʵ�����
	float eFuzzy[2]={0.0,0.0};                              //���������̶�
	float ecFuzzy[2]={0.0,0.0};                             //���仯�ʵ������̶�
	float kpRule[4]={0.0,200.0,400.0,600.0};                    //Kp��ģ���Ӽ�
	float KpFuzzy[4]={0.0,0.0,0.0,0.0};                     //������Kp�������̶�
	int   KpRule[7][7]=                    //Kp��ģ�����Ʊ�
	{   /*��Ϊpe*/
        3,3,3,2,2,2,2,
		2,2,2,1,1,1,1,
		1,1,2,1,1,2,1,
		1,1,0,1,0,1,1,
		1,1,0,0,0,1,1,
		2,2,1,0,1,1,1,
		3,3,3,3,2,3,2,
	};
	/*���e�������̶�����*/
	if(e<eRule[0])
	{
	    eFuzzy[0]=1.0;
		pe=0;
	}
	else if(eRule[0]<=e&&e<eRule[1])
	{
        eFuzzy[0]=(eRule[1]-e)/(eRule[1]-eRule[0]);
		pe=0;
	}
	else if(eRule[1]<=e&&e<eRule[2])
	{
        eFuzzy[0]=(eRule[2]-e)/(eRule[2]-eRule[1]);
		pe=1;
	}
	else if(eRule[2]<=e&&e<eRule[3])
	{
        eFuzzy[0]=(eRule[3]-e)/(eRule[3]-eRule[2]);
		pe=2;
	}
	else if(eRule[3]<=e&&e<eRule[4])
	{
        eFuzzy[0]=(eRule[4]-e)/(eRule[4]-eRule[3]);
		pe=3;
	}
	else if(eRule[4]<=e&&e<eRule[5])
	{
        eFuzzy[0]=(eRule[5]-e)/(eRule[5]-eRule[4]);
		pe=4;
	}
	else if(eRule[5]<=e&&e<eRule[6])
	{
        eFuzzy[0]=(eRule[6]-e)/(eRule[6]-eRule[5]);
		pe=5;
	}
	else
	{
        eFuzzy[0]=0.0;
		pe=5;
	}
	eFuzzy[1]=1.0-eFuzzy[0];
	/*���仯��ec�ĵ�������������*/
	if(ec<ecRule[0])
	{
        ecFuzzy[0]=1.0;
		pec=0;
	}
	else if(ecRule[0]<=ec&&ec<ecRule[1])
	{
        ecFuzzy[0]=(ecRule[1]-ec)/(ecRule[1]-ecRule[0]);
		pec=0;
	}
	else if(ecRule[1]<=ec&&ec<ecRule[2])
	{
        ecFuzzy[0]=(ecRule[2]-ec)/(ecRule[2]-ecRule[1]);
		pec=1;
	}
	else if(ecRule[2]<=ec&&ec<ecRule[3])
	{
        ecFuzzy[0]=(ecRule[3]-ec)/(ecRule[3]-ecRule[2]);
		pec=2;
	}
	else if(ecRule[3]<=ec&&ec<ecRule[4])
	{
        ecFuzzy[0]=(ecRule[4]-ec)/(ecRule[4]-ecRule[3]);
		pec=3;
	}
	else if(ecRule[4]<=ec&&ec<ecRule[5])
	{
        ecFuzzy[0]=(ecRule[5]-ec)/(ecRule[5]-ecRule[4]);
		pec=4;
	}
	else if(ecRule[5]<=ec&&ec<ecRule[6])
	{
        ecFuzzy[0]=(ecRule[6]-ec)/(ecRule[6]-ecRule[5]);
		pec=5;
	}
	else
	{
        ecFuzzy[0]=0.0;
		pec=5;
	}
	ecFuzzy[1]=1.0-ecFuzzy[0];
	/*��ѯģ�������*/
	num=KpRule[pe][pec];
	KpFuzzy[num]+=eFuzzy[0]*ecFuzzy[0];

	num=KpRule[pe][pec+1];
	KpFuzzy[num]+=eFuzzy[0]*ecFuzzy[1];

	num=KpRule[pe+1][pec];
	KpFuzzy[num]+=eFuzzy[1]*ecFuzzy[0];

	num=KpRule[pe+1][pec+1];
	KpFuzzy[num]+=eFuzzy[1]*ecFuzzy[1];
	/*��Ȩƽ���㷨��ģ��*/
	Kp_calcu=KpFuzzy[0]*kpRule[0]+KpFuzzy[1]*kpRule[1]+KpFuzzy[2]*kpRule[2]+KpFuzzy[3]*kpRule[3];
	return(Kp_calcu);
}











