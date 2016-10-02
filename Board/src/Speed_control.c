#include "include.h"
#include "common.h"


extern float fuzzy_kp(float e,float ec);
extern float fuzzy_ki(float e,float ec);
extern float fuzzy_kd(float e,float ec);
extern float fuzzy_speed(float e,float ec);

#define Normal_mid_line(x)   ABS(line[x].mid_line_new-64)<=5&&ABS(line[x+1].mid_line_new-64)<=5&&ABS(line[x+2].mid_line_new-64)<=5&&ABS(line[x+3].mid_line_new-64)<=5&&ABS(line[x+4].mid_line_new-64)<=5


/*速度闭环控制变量*/
int16  Moto_pluse;
int16 speed_current;
uint16 speed_max;
uint16 speed_min;
uint16 speed_want;
float  speed_P;
float  speed_I;
float  speed_D;
float  speed_err;
float  speed_err_pre;
float  speed_err_pre_pre;
double  speed_control_output;
double  speed_control_preoutput;
double  speed_control_curoptput;
float  speed_duty;
float  speed_err_ec;           //速度偏差变化率
float  last_speed_current;     //记录上一次的速度编码器返回值
float  last_speed_want;        //记录上一次的期望速度
float  speed_racc;             //计算小车加速度  即速度微分
float  speed_kvff;             //速度前馈系数
float  speed_kvff_param;       //速度前馈系数放大倍数
float  speed_kaff;             //加速度前馈系数
int16  Moto_pluse_time;        //剧烈减速时的记步
uint32 pluse_time_count;
uint8  first_enter_obstancle;
/*end*/

PID_speed_weizhi pid;            //这个pid名字得改  改成pid_speed_weizhi
#define Motrol_PortA PTC1
uint16 normal_car_started_count;
uint8  normal_car_flag;
uint16 stop_car_count;
/////////////这里是一个简单的速度位置式PID控制算法，非常简单，仅作为测试之用//////////////
void PID_init() {                      //位置式PID   简单做一下测试
	printf("PID_init begin \n");
	pid.SetSpeed=0.0;
	pid.ActualSpeed=0.0;
	pid.err=0.0;
	pid.err_last=0.0;
	pid.voltage=0.0;
	pid.integral=0.0;
//id.Kp=flash_Kp_dianji;
//id.Ki=flash_Ki_dianji;
//id.Kd=flash_Kd_dianji;
	printf("PID_init end \n");
}



float PID_realize(float speed){
	pid.SetSpeed=speed;
	pid.err=pid.SetSpeed-pid.ActualSpeed;
	pid.integral+=pid.err;
	pid.voltage=pid.Kp*pid.err+pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last);
	pid.err_last=pid.err;
	pid.ActualSpeed=pid.voltage*1.0;
	return pid.ActualSpeed;
}
/**************************************END******************************************/

/************************简单的增量式PID控制算法，测试用****************************/





/****************************************END****************************************/

/*编码器得到当前车速*/
//Time:2016.1.16
void GetMotoPulse(void)
{
    static int16 speed_pre,speed_pre_pre;
	speed_pre_pre=speed_pre;
	speed_pre=speed_current;
	last_speed_current=speed_current;    //保存上一次小车速度
	Moto_pluse= -ftm_quad_get(FTM2);		  //获取FTM 正交解码 的脉冲数(负数表示反方向)
	if(Moto_pluse<0)
		led(LED0,LED_ON);
	else
		led(LED0,LED_OFF);
//	Moto_pluse=abs(Moto_pluse);           //这里是硬件是AB相弄反了，所以软件上也反过来就好了，要记得有这么回事
 	speed_current=(Moto_pluse+speed_pre_pre+speed_pre)/3;    //用三次采集的速度值滤波
 	pluse_time_count+=Moto_pluse;
	if(pluse_time_count>=50000)
		pluse_time_count=0;
	ftm_quad_clean(FTM2);
	if(speed_current>=80)
	{
        normal_car_started_count++;
		if(normal_car_started_count>=100)      //0.5s小车速度正常即代表发车正常
		{
			normal_car_started_count=100;
			normal_car_flag=1;           //小车正常发车标志位
		}
	}
}

/*速度控制*/
void SpeedControl(void)
{
//    speed_want=flash_speed_want-(70-used_length)*speed_change/50;  //82192753  外屋wifi    //如果速度快就启用这个
    uint8 index;
    uint32 umax=5000;
	uint32 umin=1000;
	float intenger;    //积分项
	int16 duoji_error;
	static uint16 high_speed_count=0,exit_high_speed_count=0;       //进入狂冲模式计数与退出狂冲模式计数
	static uint8  speed_type=0;             //speed_type 为0时为正常状态，为1时为狂冲状态
	static uint16 speed_obstancle;
//  speed_P=2500/10;
//	speed_I=220/10;
//	speed_D=80/10;
    /*记录上一次的期望速度*/
    last_speed_want=speed_want;
/************************************************计算期望速度****************************************************************/
	if(Speed_protect==0)         //正常开车程序
	{
		speed_want=flash_speed_want;	    //期望速度赋值
		/*连续加减速如何控制期望速度不要忽加忽减，这样不仅能极大的提高速度的稳定性，车的稳定性也提高了，未解决,待完成*/
        if(flash_speed_want>=120)
        {
		    duoji_error=error;      //舵机偏差赋值
		    if(speed_type==0)       //正常模式
		    {
				speed_want=flash_speed_want-(uint32)((ABS(error)*flash_speed_change_error)/50.0+error_d);
				if(speed_want<=0)
					speed_want=flash_speed_change;
				if(used_length>=70&&ABS(error)<=20&&speed_current>=flash_speed_want-5)            //防止在直道晃动而频繁加减速
					speed_want=flash_speed_want;
			}
			if(used_length>=70&&ABS(error)<=20&&ABS(se_du)<=2000&&speed_current>=flash_speed_want-5)
			{
			    high_speed_count++;
			    if(high_speed_count>=10&&Normal_mid_line(0)&&Normal_mid_line(1))       //连续50ms都是速度几乎加到期望速度  这样才进入高速模式
			    {
					speed_type=1;	                      //进入狂飙模式
					high_speed_count=0;
				    speed_kvff_param=100;             //将速度前馈系数放大100倍
				}
			}
			if(speed_type==0&&(used_length<=50||ABS(error)>=25||ABS(se_du)>=2500))     //在正常模式时时时清零，防止累计出问题
			{
	            high_speed_count=0;
			}
			if(speed_type==1)
			{
	            speed_want=flash_speed_want+20;                                //加减速模式下期望速度已经很快了，加20就够
				if(used_length<=62||ABS(error)>=12||ABS(se_du)>=1500)
				{
				    exit_high_speed_count++;
					if(exit_high_speed_count>=2)
					{
	                    speed_type=0;     //只要前方转弯就进入正常模式
	                    speed_want=flash_speed_want-(uint32)((ABS(error)*flash_speed_change_error)/50.0+error_d);               //退出狂飙模式直接给最低速度
	                    if(speed_want<=0)
					        speed_want=flash_speed_change;
	                    high_speed_count=0;
						exit_high_speed_count=0;
					}
				}
			}
			/*给障碍固定一个速度*/
			if((left_obstancle_flag==1||right_obstancle_flag==1)&&flash_speed_want>=120)             //检测到障碍时，速度不能改变
			{
			    if(first_enter_obstancle==0)          //就第一次固定障碍的速度，直到障碍过去为止
			    {
                    speed_obstancle=speed_current-5;
				}
				first_enter_obstancle=1;
	            speed_want=speed_obstancle;
			}
			if(speed_want<=flash_speed_change)     //限制最小幅度得放到期望速度给定的最后    //限制最小速度
			{
	            speed_want=flash_speed_change;     //给定一个最小速度,使之速度有一个下限值,以免加减速减得过于严重
	            high_speed_count=0;                //多次清零，防止累计然后速度混乱
			}

        }
		else     //BM开关第三档 也就是低速挡位    现在第三档是可以控制速度的，平常期望速度由flash_speed_want确定
		{
		    if(Normal_mid_line(0)&&ABS(error)<=15&&ABS(se_du)<=2000&&used_length>=65)
//				speed_want=flash_speed_want+20;
			    speed_want=flash_speed_want;
			else
                speed_want=flash_speed_want;
			if(speed_current>=100&&ABS(error)<=25&&ABS(se_du)<=2500)
//				speed_want=flash_speed_want+20;
			    speed_want=flash_speed_want;
		}
	}
	/*停车部分*/
    else              //Speed_protect==1     //停车程序，一般是检测到起跑线了
    {                                     //停车之后，舵机打脚不能停，以防出弯
		speed_want=0;                     //直接停车   期望速度直接给零
	}
/*************************************************************END计算期望速度****************************************************************/

/**********************************************以下为开始计算速度占空比输出量***********************************/
    speed_kvff=flash_speed_kvff;      //获取前馈系数
    speed_kaff=flash_speed_kaff;
    /*计算出近三次的速度偏差*/
    speed_err_pre_pre=speed_err_pre;
    speed_err_pre=speed_err;
	speed_err=speed_want-speed_current;
	/*计算小车加速度  即速度微分*/
	speed_racc=0;
	speed_racc=speed_current-last_speed_current;
	/*计算出速度偏差变化率*/
    speed_err_ec=speed_err-speed_err_pre;
	/*求得前馈系数放大倍数*/
	if(podao_flag==1)       //坡道减速，将前馈系数放大100倍
	{
	    speed_kvff_param=100.0;
	}
	if(speed_err<=-20)
	{
        speed_kvff_param=100.0;
	}
    if(ABS(speed_err)<=10)         //只要速度偏差不大 即恢复前馈系数
		speed_kvff_param=1.0;
	/*获得PID参数*/
    speed_P=2500/10;
	speed_I=220/10;
	speed_D=80/10;
	////////////*以下是前馈-微分先行增量式PID计算*/
    //保存速度控制值
	speed_control_preoutput=speed_control_curoptput;
	//计算当前速度控制输出值
//	speed_control_curoptput=speed_P*(speed_err-speed_err_pre)+speed_I*speed_err+speed_D*(speed_err-2*speed_err_pre+speed_err_pre_pre);
	speed_control_curoptput=speed_kvff*speed_kvff_param*(speed_want-last_speed_want)
		                   +speed_P*(speed_err-speed_err_pre)
		                   +speed_I*speed_err
		                   +speed_D*(speed_err-2*speed_err_pre+speed_err_pre_pre)
		                   -speed_kaff*speed_racc;
}

/*速度控制输出*/
void SpeedControlOutput(void)
{
    static  uint16 step_count;
    speed_control_output+=speed_control_curoptput;         //如何解决长时间不发车PID控制器将速度的输出值调节过大还没有很好的解决，这样限幅

	if(speed_control_output>10000)     //限位
	{
        speed_control_output=10000;
	}
	if(speed_control_output<-10000)     //可以有负值
	{
        speed_control_output=-10000;
	}

	speed_duty=speed_control_output;

    if(auto_car_flag==1)             //自动模式
    {
		if(qibu_speed_flag==1)
		{
			Motor_control(2500);
			if(speed_current>=35)
			{
				qibu_speed_flag=0;
			}
		}
		else
		{
			Motor_control(speed_duty);
		}
	}
	if(hand_car_flag==1)            //手动模式
	{
        if(forward_step==1)            //向前进一小段
        {
            step_count++;
			if(step_count<=50)
			{
                Motor_control(2500);
			}
			else
			{
                Motor_control(0);
				step_count=0;
				forward_step=0;
			}

		}
		else if(back_step==1)              //向后退一小段
        {
            step_count++;
			if(step_count<=50)
			{
                Motor_control(-2500);
			}
			else
			{
                Motor_control(0);
				step_count=0;
				back_step=0;
			}
		}
		else
		{
            Motor_control(0);
		}
	}

}


void start_line_stop(uint16 count)
{
	if(speed_current>=120)            //速度比较大，延时20ms
	{
        if(stop_count>=20)
	    {
			if(stop==1)                //停车标志位
			    Speed_protect=1;
	    }
	}
    else                             //速度比较小，延时50ms
	{
        if(stop_count>=30)
	    {
			if(stop==1)                //停车标志位
			    Speed_protect=1;
	    }
	}
}

/*电机控制函数*/
void Motor_control(int32 motor_num)
{
    int32 temp;
    if(motor_num>=0)         //电机正反转的实现
    {
        temp=ABS(motor_num);
		ftm_pwm_duty(FTM0, FTM_CH0,0);
		ftm_pwm_duty(FTM0, FTM_CH1,temp);
    }
    else                  //就是输出值小于零的情况
    {
		temp=ABS(motor_num);
//		temp=0;
		ftm_pwm_duty(FTM0, FTM_CH0,temp);
		ftm_pwm_duty(FTM0, FTM_CH1,0);
    }
}
/*模糊计算Kp*/
float fuzzy_kp(float e,float ec)
{
    float Kp_calcu;
	uint8 num,pe,pec;
	float eRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0};        //误差的论域
	float ecRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0};       //误差变化率的论域
	float eFuzzy[2]={0.0,0.0};                              //误差的隶属程度
	float ecFuzzy[2]={0.0,0.0};                             //误差变化率的隶属程度
	float kpRule[4]={0.0,280.0,280.0*2.0,280.0*3.0};                    //Kp的模糊子集
	float KpFuzzy[4]={0.0,0.0,0.0,0.0};                     //隶属于Kp的隶属程度
	/*
	KpFuzzy[0]  kpRule[0]
	KpFuzzy[1]  kpRule[1]
	KpFuzzy[2]  kpRule[2]
	KpFuzzy[4]  kpRule[3]
	*/
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
	    eFuzzy[0]=1.0;               //eFuzzy[0] 表示不隶属的意思
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
        eFuzzy[0]=0.0;                          //超出范围隶属为零
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
        ecFuzzy[0]=0.0;                                //偏差超出范围隶属为零
		pec=5;
	}
	ecFuzzy[1]=1.0-ecFuzzy[0];
	//注意下面是  += 是一个累计的过程
	/*num代表将当前隶属程度即概率归属与哪一个模糊子集范围内，3代表最大 2次之 1再次之 0就是0*/
	/*查询模糊规则表*/
	num=KpRule[pe][pec];
	KpFuzzy[num]+=eFuzzy[0]*ecFuzzy[0];       //偏差隶属*偏差变化率隶属

	num=KpRule[pe][pec+1];
	KpFuzzy[num]+=eFuzzy[0]*ecFuzzy[1];      //偏差隶属*偏差变化率不隶属

	num=KpRule[pe+1][pec];
	KpFuzzy[num]+=eFuzzy[1]*ecFuzzy[0];      //偏差不隶属*偏差变化率隶属

	num=KpRule[pe+1][pec+1];
	KpFuzzy[num]+=eFuzzy[1]*ecFuzzy[1];      //偏差不隶属*偏差变化率不隶属
	/*加权平均算法解模糊*/                                //模糊子集与偏差隶属程度的乘积
	Kp_calcu=KpFuzzy[0]*kpRule[0]+KpFuzzy[1]*kpRule[1]+KpFuzzy[2]*kpRule[2]+KpFuzzy[3]*kpRule[3];
	/* KpFuzzy[0] KpFuzzy[1] KpFuzzy[2] KpFuzzy[3] 各自分别代表一种比例或者说权重
    num=KpRule[pe][pec]; 这一系列的运算算的都是num归属于哪一个 KpFuzzy
    那么模糊规则表中，0 1 2 3 则分别与模糊子集相对应
	*/
	return(Kp_calcu);
}

/*模糊计算速度*/
/*模糊计算速度跟模糊计算PID参数不一样
*模糊计算速度时，传进来的是舵机偏差和偏差变化率
*而计算参数时传进来的速度偏差和偏差变化率
*速度加减速要求偏差大时速度小，偏差小时速度大
*跟参数的计算刚好相反
*/
float fuzzy_speed(float e,float ec)
{
    float Kp_calcu;
	uint8 num,pe,pec;
	float eRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0};        //误差的论域
	float ecRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0};       //误差变化率的论域
	float eFuzzy[2]={0.0,0.0};                              //误差的隶属程度
	float ecFuzzy[2]={0.0,0.0};                             //误差变化率的隶属程度
	float kpRule[4]={120.0,80.0,50.0,0.0};                    //Kp的模糊子集                  //跟参数的推理反向
	float KpFuzzy[4]={0.0,0.0,0.0,0.0};                     //隶属于Kp的隶属程度
	/*
	KpFuzzy[0]  kpRule[0]
	KpFuzzy[1]  kpRule[1]
	KpFuzzy[2]  kpRule[2]
	KpFuzzy[4]  kpRule[3]
	*/
 	//error=64-mid_line    error>0   左拐
 	//                     error<0   右拐
	int   KpRule[7][7]=                    //speed的模糊控制表
	{   /*行为pe*/
        3,3,3,3,3,3,3,    /*列为pec*/
		2,2,2,2,2,2,2,
		1,1,1,1,1,1,1,
		0,0,0,0,0,0,0,
		1,1,1,1,1,1,1,
		2,2,2,2,2,2,2,
		3,3,3,3,3,3,3,
	};
//      Kp的模糊参数
//	    3,3,3,3,3,3,3,    /*列为pec*/
//		2,2,2,2,2,2,2,
//		1,1,1,1,1,1,1,
//		1,1,0,1,0,1,1,
//		0,0,1,0,0,1,0,
//		0,1,0,1,0,0,2,
//		3,3,3,3,3,3,3,
	/*误差e的隶属程度描述*/
	if(e<eRule[0])
	{
	    eFuzzy[0]=1.0;               //eFuzzy[0] 表示不隶属的意思
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
        eFuzzy[0]=0.0;                          //超出范围隶属为零
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
        ecFuzzy[0]=0.0;                                //偏差超出范围隶属为零
		pec=5;
	}
	ecFuzzy[1]=1.0-ecFuzzy[0];
	//注意下面是  += 是一个累计的过程
	/*num代表将当前隶属程度即概率归属与哪一个模糊子集范围内，3代表最大 2次之 1再次之 0就是0*/
	/*查询模糊规则表*/
	num=KpRule[pe][pec];
	KpFuzzy[num]+=eFuzzy[0]*ecFuzzy[0];       //偏差隶属*偏差变化率隶属

	num=KpRule[pe][pec+1];
	KpFuzzy[num]+=eFuzzy[0]*ecFuzzy[1];      //偏差隶属*偏差变化率不隶属

	num=KpRule[pe+1][pec];
	KpFuzzy[num]+=eFuzzy[1]*ecFuzzy[0];      //偏差不隶属*偏差变化率隶属

	num=KpRule[pe+1][pec+1];
	KpFuzzy[num]+=eFuzzy[1]*ecFuzzy[1];      //偏差不隶属*偏差变化率不隶属
	/*加权平均算法解模糊*/                                //模糊子集与偏差隶属程度的乘积
	Kp_calcu=KpFuzzy[0]*kpRule[0]+KpFuzzy[1]*kpRule[1]+KpFuzzy[2]*kpRule[2]+KpFuzzy[3]*kpRule[3];
	/* KpFuzzy[0] KpFuzzy[1] KpFuzzy[2] KpFuzzy[3] 各自分别代表一种比例或者说权重
    num=KpRule[pe][pec]; 这一系列的运算算的都是num归属于哪一个 KpFuzzy
    那么模糊规则表中，0 1 2 3 则分别与模糊子集相对应
	*/
	return(Kp_calcu);
}


/*模糊计算Ki*/
float fuzzy_ki(float e,float ec)
{
    float Kp_calcu;
	uint8 num,pe,pec;
	float eRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0};        //误差的论域
	float ecRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0};       //误差变化率的论域
	float eFuzzy[2]={0.0,0.0};                              //误差的隶属程度
	float ecFuzzy[2]={0.0,0.0};                             //误差变化率的隶属程度
	float kpRule[4]={0.0,22.0,44.0,66.0};                    //Kp的模糊子集
	float KpFuzzy[4]={0.0,0.0,0.0,0.0};                     //隶属于Kp的隶属程度
	int   KpRule[7][7]=                    //Kp的模糊控制表
	{   /*行为pe*/
        0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,
		2,0,0,0,0,0,1,
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

/*模糊计算Kd*/
float fuzzy_kd(float e,float ec)
{
    float Kp_calcu;
	uint8 num,pe,pec;
	float eRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0};        //误差的论域
	float ecRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0};       //误差变化率的论域
	float eFuzzy[2]={0.0,0.0};                              //误差的隶属程度
	float ecFuzzy[2]={0.0,0.0};                             //误差变化率的隶属程度
	float kpRule[4]={0.0,8.0,16.0,24.0};                    //Kp的模糊子集
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

/*简单的速度增量式控制*/
/*
void SpeedControl(void)
{
    speed_real=Moto_pluse;
    err =speed_want-speed_real;
    speed_control_out_old = speed_control_out_new;
    speed_control_out_new =err *speed_P;
    if(speed_control_out_new>10)
        speed_control_out_new=10;
    speed_duty=(uint16)(speed_duty+speed_control_out_new);
    if(speed_duty>2600)
        speed_duty=1800;
    ftm_pwm_duty(FTM0, FTM_CH3,speed_duty);
}
*/

