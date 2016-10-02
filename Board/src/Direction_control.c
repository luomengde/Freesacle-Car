#include "include.h"
#include "common.h"

extern void Regression_cal(uint8 *X_buff,uint8 *Y_buff,uint8 AvaliableLines);
float DJ_fuzzy_kp(float e,float ec);
extern float DJ_fuzzy_kd(float e,float ec);
unsigned int sqrt_16(unsigned long M);
#define Normal_mid_line(x)   ABS(line[x].mid_line_new-64)<=5&&ABS(line[x+1].mid_line_new-64)<=5&&ABS(line[x+2].mid_line_new-64)<=5&&ABS(line[x+3].mid_line_new-64)<=5&&ABS(line[x+4].mid_line_new-64)<=5



//**舵机参数**//
uint32 dir_duty;    //用于传递的舵机占空比参数
float mid_line;		//经过计算后，一帧图像的中值应该是 float
float mid_line_new;
float Kp_duoji;    //舵机Kp
float Kd_duoji;    //舵机Kd

//斜率
float index_A,index_B;    //index_A为斜率  index_B为截距   斜率一般更为常用
//坡道
uint8  podao_flag=0;
extern uint16 road_wide_normal;
uint8 never_podao_flag=0;

extern line_info  line[line_num];
extern uint8 zhijiao_forward;

extern uint8  lost_count;      //本帧图像左右都丢失的行数
/*舵机变量*/
int32 SE_duty;                  //舵机的PWM占空比
int32 se_du;
int16 ML_duty=0;       //电机PWM占空比
int16 MR_duty=0;
float Ki;                      //舵机PID
float Kp;
float Kp1=15;
float Kp2=0.36;
float Kp1_you;
float Kp2_you;
float Kd=4.38;
float error,error_d,error_pre; //偏差
float dealt_error;    //偏差变化率
int8 mid_line_last[5]={0};   //保存过去的五组中值     可以为负   不能定义为uint8
uint8 test_count=0;
uint8 podao_count=0;            //看第几次进入坡道，奇次减速，偶次不减速
uint16 sqrt_16_kp2;      //计算(line_num-used_length)的1.5次方
int32  obstancle_time_pluse;
float  before_cal_mid_line;     //特殊处理之前的中线
uint8 quan2[70]={
   1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
   1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
   1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
   1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
   1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
   1,  1,  1,  1,  1,  1,  1,  1,  1,  1,
   1,  1,  1,  1,  1,  1,  1,  1,  1,  1
};             //权重全为1，也就是将中线累加然后取平均值
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

uint8 quan3[70]={                   //与quan1相比,将近处的权重加大一些
   102,  104,  106,  108,  110,  116,  117,  118,  129,  130,
   102,  104,  106,  108,  110,  116,  117,  118,  129,  130,
   131,142,153,154,165,195,196,197,198,199,
   190, 191, 192, 193, 194, 164, 163, 162, 161, 160,
   129, 128, 127, 126, 125, 124, 123, 122, 121, 120,
   119, 118, 117, 116, 115, 114, 113, 112, 111, 110,
   109, 108, 107, 106, 105, 104, 103, 102, 101, 100
};
uint8 quan_liangge[70]={        //梁玉亮加权
   2,  2,  2,  2,  2, 2,  2,  2,  3,  3,
   3, 3, 3, 3, 3, 3, 4, 4,  4,  4,
   7,  7,  7,  7,  7,  7,  8,  8, 8,  8,
    8,  8,  8, 8,  8,  8,  8,  8,  8,  8,
   8,  8,  8, 8,  8,  9,  9,  9,  9,  9,
     8,  8,  8,  8,  8, 9,  9,  9,  9,  9,
   0,  0, 0, 0,  10, 10,  10,  0,  0, 0,
};

uint8 quan[70]={        //梁玉亮加权
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

uint8 quan_change[70]={                            //目前十字过切，注意解决
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
	2,	2,	2,	2,	2, 2,  2,  2,  3,  3,    //从最底下开始算的
	4, 4, 4, 4, 4, 5, 5, 5,  5,  5,
	8,	8,	8,	8,	8,	8,	8,	8, 8,  8,
	 9,  9,  9, 9,	9,	9,	9,	9,	9,	9,
	 9,  9,  9, 9,	9,	9,	9,	9,	9,	9,
     8,  8,  8, 8,	8,	8,	8,	8,	8,	8,
	0,	0, 0, 0,  10, 10,  10,	0,	0, 0,
};

uint8 quan_change_zanshi[70]={                            //目前十字过切，注意解决
	3,	3,	3,	3,	3, 3,  3,  3,  3,  3,
	6, 6, 6, 6, 6, 7, 7, 7,  7,  7,
	8,	8,	8,	8,	8,	9,	9,	9,  9,  9,
	9,	9,	9,	9,	9,	9, 9, 9, 9,  9,            //
	8,	8,	8, 8,  8,  8,	8,	8, 8,  8,
	  8,  8,  8,  8,  8, 7,  7,  7,  7,  7,
	0,	0, 0, 0,  0,  0,  0,	0,	0, 0,
};




/* Function Name: dir_duty_new();
* Description  : 中线加权函数和舵机方向控制
* Author Data  : 2015/4/4 , by Li*/
void quan_dir_duty_new()
 {
    uint8 i;                                        //利用赛道宽度检测坡道
    static int32 num,add;
	static uint8 temp_mid_line_into_circle=64;

    last_mid_line=mid_line;               //保存上一次的图像中值

    for(i=0;i<20;i++)                  //正常赛道加权有用行以内的
     {
         if(line[i].mid_line_new!=200)
         {
			 num=(line[i].mid_line_new)*quan1[i]+num;		//加权每一行的mid_line_new坐标
			 add=add+quan1[i];
		 }
     }
    if(add>0)
    {
        mid_line=(float)num/(float)add;
    }
	else
	{
        mid_line=last_mid_line;             //如果全丢失则保持上一次的图像
	}
	before_cal_mid_line=mid_line;
	/*特殊处理*/
	if(into_circle_flag==1)         //进入圆的一小段不要打角
	{
        mid_line=temp_mid_line_into_circle;
	}
	if(out_circle_flag==1)
	{
        mid_line=temp_mid_line_into_circle;
	}
	mid_line_last[4]=mid_line_last[3];          //保存五组图像中值
	mid_line_last[3]=mid_line_last[2];
	mid_line_last[2]=mid_line_last[1];
	mid_line_last[1]=mid_line_last[0];
	mid_line_last[0]=mid_line;
    num=0;                         //清零
    add=0;
    if(mid_line>128)               //对加权后中线值进行限幅
     {
        mid_line=128;
     }
    else if(mid_line<0)
     {
        mid_line=0;
     }

	error_pre=error;
	error=64-mid_line;    //左转 error>0   右转 error<0
    error_d=error-error_pre;       //这里最终得到了实际是    last_mid_line-mid_line !(之前写反了，怪不得舵机D不起作用!!!!!!!!!!!!!)
    Regression_cal(X_point,Y_point,used_length);                      //计算出当前中线斜率
//	dealt_error=error_d/(0.035);
 }



#define MID_dir_duty (49500)     //矫正舵机中值           //硬底盘新车中值48600
#define MAX_dir_duty (61000)     //左打死  61050     11900
#define MIN_dir_duty (38100)     //右打死  38100     11050


/*舵机PD控制函数*/    //可以尝试别的算法 这个算法有它的局限性
#define nWidth  128
#define nHeight 70
void DJ_PID(void)
{
    //可以加前馈补偿PID
    Kp1=flash_duoji_Kp1;
	Kp2=flash_duoji_Kp2;
	Kp1_you=flash_duoji_Kp1_you;
	Kp2_you=flash_duoji_Kp2_you;
//	Kp1_you=Kp1;                                            //左右还是一样
//	Kp2_you=Kp2;
	Kd= 600;
//    if(ABS(error)<=30)        //小弯大D值，小打角就好        大弯小D值  打角别太猛
//		Kd=800;
	sqrt_16_kp2=sqrt_16((30-used_length)*(30-used_length)*(30-used_length));
//    Kp=Kp1+(nHeight-used_length)*(nHeight-used_length)*Kp2/100;
    Kp=Kp1+sqrt_16_kp2*Kp2/50.0;
    /*障碍的时候不能使用这个*/
    if(Normal_mid_line(0)&&Normal_mid_line(5)&&ABS(error)<=15)       //让直到抖动不那么大
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

    se_du=(int32)(Kp*error+Kd*error_d);   //如果车在左边，即error小于零，se_du<0 车子该右转
                                          //舵机占空比应该大于中值
//    if(ABS(error)<=10&&used_length>=70)
//		se_du=0;

    SE_duty=(uint32)(MID_dir_duty-se_du);   //显示类型转换


	/*自动驾驶还是手动驾驶*/
    static int32 the_last_duty;
    if(auto_car_flag==1)
    {
        the_last_duty=SE_duty;
	}
	else    //手动驾驶
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
    ftm_pwm_duty(FTM1,FTM_CH1,SE_duty);         //判断什么时候打到了左右极限
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

    se_du=(int32)(Kp*error+Kd*error_d);   //如果车在左边，即error小于零，se_du<0 车子该右转
                                          //舵机占空比应该大于中值
//    if(se_du>9500)
//		se_du=9500;
//	if(se_du<-9500)
//		se_du=-9500;
    last_mid_line=mid_line;               //保存上一次的图像中值
//    if(podao_flag==1)       //检测到坡道则不打角
//		se_du=0;

    SE_duty=(uint32)(MID_dir_duty-se_du);   //显示类型转换
    if(SE_duty>MAX_dir_duty)
        SE_duty=MAX_dir_duty;
    else if(SE_duty<MIN_dir_duty)
        SE_duty=MIN_dir_duty;
#if defined(SHANWAI)
    ftm_pwm_duty(FTM1,FTM_CH1,SE_duty);         //判断什么时候打到了左右极限
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
/*Function: 开根号处理                  */
/*入口参数：被开方数，长整型            */
/*出口参数：开方结果，整型              */
/****************************************/
unsigned int sqrt_16(unsigned long M)
{
    unsigned int N, i;
    unsigned long tmp, ttp;   // 结果、循环计数
    if (M == 0)               // 被开方数，开方结果也为0
        return 0;
    N = 0;
    tmp = (M >> 30);          // 获取最高位：B[m-1]
    M <<= 2;
    if (tmp > 1)              // 最高位为1
    {
        N ++;                 // 结果当前位为1，否则为默认的0
        tmp -= N;
    }
    for (i=15; i>0; i--)      // 求剩余的15位
    {
        N <<= 1;              // 左移一位
        tmp <<= 2;
        tmp += (M >> 30);     // 假设
        ttp = N;
        ttp = (ttp<<1)+1;
        M <<= 2;
        if (tmp >= ttp)       // 假设成立
        {
            tmp -= ttp;
            N ++;
        }
    }
    return N;
}


/*! @brief 最小二乘法
 *  @note  Y=A+BX
 *  @data 2015/3/15
 */
//调用此函数传递参数的数量为有效行
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


/*! @brief 三角形面积
 *  @note
 *  @data
 */
float Cal_AngleArea(int16 x1,int16 y1,int16 x2,int16 y2,int16 x3,int16 y3)
{
    float temp;
    temp=((x2-x1)*(y3-y1)-(x3-x1)*(y2-y1))/2;
    return temp;
}
/*! @brief 开方
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

/*! @brief 点距
 *  @note
 *  @data
 */
float Cal_Distance(int16 X1,int16 Y1,int16 X2,int16 Y2)
{
    float temp;
    temp=m_sqrt((X1-X2)*(X1-X2)-(Y1-Y2)*(Y1-Y2));
    return temp;
}

/*! @brief 曲率
 *  @note
 *  @data
 */
float cal_curvature(int16 x1,int16 y1,int16 x2,int16 y2,int16 x3,int16 y3)
{
//    float temp,temp1,d1,d2,d3;
//    y1=Y_change[y1];
//    y2=Y_change[y2];
//    y3=Y_change[y3];
//    x1=(int16)((x1-64)*X_change[y1]);//中点变为0 坐标系变了
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

/*模糊计算Kp*/
float DJ_fuzzy_kp(float e,float ec)
{
    float Kp_calcu;
	uint8 num,pe,pec;
	float eRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0};        //误差的论域
	float ecRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0};       //误差变化率的论域
	float eFuzzy[2]={0.0,0.0};                              //误差的隶属程度
	float ecFuzzy[2]={0.0,0.0};                             //误差变化率的隶属程度
	float kpRule[4]={0.0,80.0,160.0,240.0};                    //Kp的模糊子集
	float KpFuzzy[4]={0.0,0.0,0.0,0.0};                     //隶属于Kp的隶属程度
	int   KpRule[7][7]=                    //Kp的模糊控制表
	{   /*行为pe*/
        3,3,3,3,3,3,3,    /*列为pec*/
		2,2,2,2,2,2,2,
		1,1,1,1,1,1,1,
		1,1,0,1,0,1,1,
		0,0,1,0,0,1,0,
		0,1,0,1,0,0,2,
		3,3,3,3,3,3,3,
	};
	/*误差e的隶属程度描述*/
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
	/*误差变化率ec的的隶属函数描述*/
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
	/*查询模糊规则表*/
	num=KpRule[pe][pec];
	KpFuzzy[num]+=eFuzzy[0]*ecFuzzy[0];

	num=KpRule[pe][pec+1];
	KpFuzzy[num]+=eFuzzy[0]*ecFuzzy[1];

	num=KpRule[pe+1][pec];
	KpFuzzy[num]+=eFuzzy[1]*ecFuzzy[0];

	num=KpRule[pe+1][pec+1];
	KpFuzzy[num]+=eFuzzy[1]*ecFuzzy[1];
	/*加权平均算法解模糊*/
	Kp_calcu=KpFuzzy[0]*kpRule[0]+KpFuzzy[1]*kpRule[1]+KpFuzzy[2]*kpRule[2]+KpFuzzy[3]*kpRule[3];
	return(Kp_calcu);
}

/*模糊计算Kp*/
float DJ_fuzzy_kd(float e,float ec)
{
    float Kp_calcu;
	uint8 num,pe,pec;
	float eRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0};        //误差的论域
	float ecRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0};       //误差变化率的论域
	float eFuzzy[2]={0.0,0.0};                              //误差的隶属程度
	float ecFuzzy[2]={0.0,0.0};                             //误差变化率的隶属程度
	float kpRule[4]={0.0,200.0,400.0,600.0};                    //Kp的模糊子集
	float KpFuzzy[4]={0.0,0.0,0.0,0.0};                     //隶属于Kp的隶属程度
	int   KpRule[7][7]=                    //Kp的模糊控制表
	{   /*行为pe*/
        3,3,3,2,2,2,2,
		2,2,2,1,1,1,1,
		1,1,2,1,1,2,1,
		1,1,0,1,0,1,1,
		1,1,0,0,0,1,1,
		2,2,1,0,1,1,1,
		3,3,3,3,2,3,2,
	};
	/*误差e的隶属程度描述*/
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
	/*误差变化率ec的的隶属函数描述*/
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
	/*查询模糊规则表*/
	num=KpRule[pe][pec];
	KpFuzzy[num]+=eFuzzy[0]*ecFuzzy[0];

	num=KpRule[pe][pec+1];
	KpFuzzy[num]+=eFuzzy[0]*ecFuzzy[1];

	num=KpRule[pe+1][pec];
	KpFuzzy[num]+=eFuzzy[1]*ecFuzzy[0];

	num=KpRule[pe+1][pec+1];
	KpFuzzy[num]+=eFuzzy[1]*ecFuzzy[1];
	/*加权平均算法解模糊*/
	Kp_calcu=KpFuzzy[0]*kpRule[0]+KpFuzzy[1]*kpRule[1]+KpFuzzy[2]*kpRule[2]+KpFuzzy[3]*kpRule[3];
	return(Kp_calcu);
}











