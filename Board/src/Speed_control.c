#include "include.h"
#include "common.h"


extern float fuzzy_kp(float e,float ec);
extern float fuzzy_ki(float e,float ec);
extern float fuzzy_kd(float e,float ec);
extern float fuzzy_speed(float e,float ec);

#define Normal_mid_line(x)   ABS(line[x].mid_line_new-64)<=5&&ABS(line[x+1].mid_line_new-64)<=5&&ABS(line[x+2].mid_line_new-64)<=5&&ABS(line[x+3].mid_line_new-64)<=5&&ABS(line[x+4].mid_line_new-64)<=5


/*�ٶȱջ����Ʊ���*/
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
float  speed_err_ec;           //�ٶ�ƫ��仯��
float  last_speed_current;     //��¼��һ�ε��ٶȱ���������ֵ
float  last_speed_want;        //��¼��һ�ε������ٶ�
float  speed_racc;             //����С�����ٶ�  ���ٶ�΢��
float  speed_kvff;             //�ٶ�ǰ��ϵ��
float  speed_kvff_param;       //�ٶ�ǰ��ϵ���Ŵ���
float  speed_kaff;             //���ٶ�ǰ��ϵ��
int16  Moto_pluse_time;        //���Ҽ���ʱ�ļǲ�
uint32 pluse_time_count;
uint8  first_enter_obstancle;
/*end*/

PID_speed_weizhi pid;            //���pid���ֵø�  �ĳ�pid_speed_weizhi
#define Motrol_PortA PTC1
uint16 normal_car_started_count;
uint8  normal_car_flag;
uint16 stop_car_count;
/////////////������һ���򵥵��ٶ�λ��ʽPID�����㷨���ǳ��򵥣�����Ϊ����֮��//////////////
void PID_init() {                      //λ��ʽPID   ����һ�²���
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

/************************�򵥵�����ʽPID�����㷨��������****************************/





/****************************************END****************************************/

/*�������õ���ǰ����*/
//Time:2016.1.16
void GetMotoPulse(void)
{
    static int16 speed_pre,speed_pre_pre;
	speed_pre_pre=speed_pre;
	speed_pre=speed_current;
	last_speed_current=speed_current;    //������һ��С���ٶ�
	Moto_pluse= -ftm_quad_get(FTM2);		  //��ȡFTM �������� ��������(������ʾ������)
	if(Moto_pluse<0)
		led(LED0,LED_ON);
	else
		led(LED0,LED_OFF);
//	Moto_pluse=abs(Moto_pluse);           //������Ӳ����AB��Ū���ˣ����������Ҳ�������ͺ��ˣ�Ҫ�ǵ�����ô����
 	speed_current=(Moto_pluse+speed_pre_pre+speed_pre)/3;    //�����βɼ����ٶ�ֵ�˲�
 	pluse_time_count+=Moto_pluse;
	if(pluse_time_count>=50000)
		pluse_time_count=0;
	ftm_quad_clean(FTM2);
	if(speed_current>=80)
	{
        normal_car_started_count++;
		if(normal_car_started_count>=100)      //0.5sС���ٶ�����������������
		{
			normal_car_started_count=100;
			normal_car_flag=1;           //С������������־λ
		}
	}
}

/*�ٶȿ���*/
void SpeedControl(void)
{
//    speed_want=flash_speed_want-(70-used_length)*speed_change/50;  //82192753  ����wifi    //����ٶȿ���������
    uint8 index;
    uint32 umax=5000;
	uint32 umin=1000;
	float intenger;    //������
	int16 duoji_error;
	static uint16 high_speed_count=0,exit_high_speed_count=0;       //������ģʽ�������˳����ģʽ����
	static uint8  speed_type=0;             //speed_type Ϊ0ʱΪ����״̬��Ϊ1ʱΪ���״̬
	static uint16 speed_obstancle;
//  speed_P=2500/10;
//	speed_I=220/10;
//	speed_D=80/10;
    /*��¼��һ�ε������ٶ�*/
    last_speed_want=speed_want;
/************************************************���������ٶ�****************************************************************/
	if(Speed_protect==0)         //������������
	{
		speed_want=flash_speed_want;	    //�����ٶȸ�ֵ
		/*�����Ӽ�����ο��������ٶȲ�Ҫ���Ӻ��������������ܼ��������ٶȵ��ȶ��ԣ������ȶ���Ҳ����ˣ�δ���,�����*/
        if(flash_speed_want>=120)
        {
		    duoji_error=error;      //���ƫ�ֵ
		    if(speed_type==0)       //����ģʽ
		    {
				speed_want=flash_speed_want-(uint32)((ABS(error)*flash_speed_change_error)/50.0+error_d);
				if(speed_want<=0)
					speed_want=flash_speed_change;
				if(used_length>=70&&ABS(error)<=20&&speed_current>=flash_speed_want-5)            //��ֹ��ֱ���ζ���Ƶ���Ӽ���
					speed_want=flash_speed_want;
			}
			if(used_length>=70&&ABS(error)<=20&&ABS(se_du)<=2000&&speed_current>=flash_speed_want-5)
			{
			    high_speed_count++;
			    if(high_speed_count>=10&&Normal_mid_line(0)&&Normal_mid_line(1))       //����50ms�����ٶȼ����ӵ������ٶ�  �����Ž������ģʽ
			    {
					speed_type=1;	                      //������ģʽ
					high_speed_count=0;
				    speed_kvff_param=100;             //���ٶ�ǰ��ϵ���Ŵ�100��
				}
			}
			if(speed_type==0&&(used_length<=50||ABS(error)>=25||ABS(se_du)>=2500))     //������ģʽʱʱʱ���㣬��ֹ�ۼƳ�����
			{
	            high_speed_count=0;
			}
			if(speed_type==1)
			{
	            speed_want=flash_speed_want+20;                                //�Ӽ���ģʽ�������ٶ��Ѿ��ܿ��ˣ���20�͹�
				if(used_length<=62||ABS(error)>=12||ABS(se_du)>=1500)
				{
				    exit_high_speed_count++;
					if(exit_high_speed_count>=2)
					{
	                    speed_type=0;     //ֻҪǰ��ת��ͽ�������ģʽ
	                    speed_want=flash_speed_want-(uint32)((ABS(error)*flash_speed_change_error)/50.0+error_d);               //�˳����ģʽֱ�Ӹ�����ٶ�
	                    if(speed_want<=0)
					        speed_want=flash_speed_change;
	                    high_speed_count=0;
						exit_high_speed_count=0;
					}
				}
			}
			/*���ϰ��̶�һ���ٶ�*/
			if((left_obstancle_flag==1||right_obstancle_flag==1)&&flash_speed_want>=120)             //��⵽�ϰ�ʱ���ٶȲ��ܸı�
			{
			    if(first_enter_obstancle==0)          //�͵�һ�ι̶��ϰ����ٶȣ�ֱ���ϰ���ȥΪֹ
			    {
                    speed_obstancle=speed_current-5;
				}
				first_enter_obstancle=1;
	            speed_want=speed_obstancle;
			}
			if(speed_want<=flash_speed_change)     //������С���ȵ÷ŵ������ٶȸ��������    //������С�ٶ�
			{
	            speed_want=flash_speed_change;     //����һ����С�ٶ�,ʹ֮�ٶ���һ������ֵ,����Ӽ��ټ��ù�������
	            high_speed_count=0;                //������㣬��ֹ�ۼ�Ȼ���ٶȻ���
			}

        }
		else     //BM���ص����� Ҳ���ǵ��ٵ�λ    ���ڵ������ǿ��Կ����ٶȵģ�ƽ�������ٶ���flash_speed_wantȷ��
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
	/*ͣ������*/
    else              //Speed_protect==1     //ͣ������һ���Ǽ�⵽��������
    {                                     //ͣ��֮�󣬶����Ų���ͣ���Է�����
		speed_want=0;                     //ֱ��ͣ��   �����ٶ�ֱ�Ӹ���
	}
/*************************************************************END���������ٶ�****************************************************************/

/**********************************************����Ϊ��ʼ�����ٶ�ռ�ձ������***********************************/
    speed_kvff=flash_speed_kvff;      //��ȡǰ��ϵ��
    speed_kaff=flash_speed_kaff;
    /*����������ε��ٶ�ƫ��*/
    speed_err_pre_pre=speed_err_pre;
    speed_err_pre=speed_err;
	speed_err=speed_want-speed_current;
	/*����С�����ٶ�  ���ٶ�΢��*/
	speed_racc=0;
	speed_racc=speed_current-last_speed_current;
	/*������ٶ�ƫ��仯��*/
    speed_err_ec=speed_err-speed_err_pre;
	/*���ǰ��ϵ���Ŵ���*/
	if(podao_flag==1)       //�µ����٣���ǰ��ϵ���Ŵ�100��
	{
	    speed_kvff_param=100.0;
	}
	if(speed_err<=-20)
	{
        speed_kvff_param=100.0;
	}
    if(ABS(speed_err)<=10)         //ֻҪ�ٶ�ƫ��� ���ָ�ǰ��ϵ��
		speed_kvff_param=1.0;
	/*���PID����*/
    speed_P=2500/10;
	speed_I=220/10;
	speed_D=80/10;
	////////////*������ǰ��-΢����������ʽPID����*/
    //�����ٶȿ���ֵ
	speed_control_preoutput=speed_control_curoptput;
	//���㵱ǰ�ٶȿ������ֵ
//	speed_control_curoptput=speed_P*(speed_err-speed_err_pre)+speed_I*speed_err+speed_D*(speed_err-2*speed_err_pre+speed_err_pre_pre);
	speed_control_curoptput=speed_kvff*speed_kvff_param*(speed_want-last_speed_want)
		                   +speed_P*(speed_err-speed_err_pre)
		                   +speed_I*speed_err
		                   +speed_D*(speed_err-2*speed_err_pre+speed_err_pre_pre)
		                   -speed_kaff*speed_racc;
}

/*�ٶȿ������*/
void SpeedControlOutput(void)
{
    static  uint16 step_count;
    speed_control_output+=speed_control_curoptput;         //��ν����ʱ�䲻����PID���������ٶȵ����ֵ���ڹ���û�кܺõĽ���������޷�

	if(speed_control_output>10000)     //��λ
	{
        speed_control_output=10000;
	}
	if(speed_control_output<-10000)     //�����и�ֵ
	{
        speed_control_output=-10000;
	}

	speed_duty=speed_control_output;

    if(auto_car_flag==1)             //�Զ�ģʽ
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
	if(hand_car_flag==1)            //�ֶ�ģʽ
	{
        if(forward_step==1)            //��ǰ��һС��
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
		else if(back_step==1)              //�����һС��
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
	if(speed_current>=120)            //�ٶȱȽϴ���ʱ20ms
	{
        if(stop_count>=20)
	    {
			if(stop==1)                //ͣ����־λ
			    Speed_protect=1;
	    }
	}
    else                             //�ٶȱȽ�С����ʱ50ms
	{
        if(stop_count>=30)
	    {
			if(stop==1)                //ͣ����־λ
			    Speed_protect=1;
	    }
	}
}

/*������ƺ���*/
void Motor_control(int32 motor_num)
{
    int32 temp;
    if(motor_num>=0)         //�������ת��ʵ��
    {
        temp=ABS(motor_num);
		ftm_pwm_duty(FTM0, FTM_CH0,0);
		ftm_pwm_duty(FTM0, FTM_CH1,temp);
    }
    else                  //�������ֵС��������
    {
		temp=ABS(motor_num);
//		temp=0;
		ftm_pwm_duty(FTM0, FTM_CH0,temp);
		ftm_pwm_duty(FTM0, FTM_CH1,0);
    }
}
/*ģ������Kp*/
float fuzzy_kp(float e,float ec)
{
    float Kp_calcu;
	uint8 num,pe,pec;
	float eRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0};        //��������
	float ecRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0};       //���仯�ʵ�����
	float eFuzzy[2]={0.0,0.0};                              //���������̶�
	float ecFuzzy[2]={0.0,0.0};                             //���仯�ʵ������̶�
	float kpRule[4]={0.0,280.0,280.0*2.0,280.0*3.0};                    //Kp��ģ���Ӽ�
	float KpFuzzy[4]={0.0,0.0,0.0,0.0};                     //������Kp�������̶�
	/*
	KpFuzzy[0]  kpRule[0]
	KpFuzzy[1]  kpRule[1]
	KpFuzzy[2]  kpRule[2]
	KpFuzzy[4]  kpRule[3]
	*/
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
	    eFuzzy[0]=1.0;               //eFuzzy[0] ��ʾ����������˼
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
        eFuzzy[0]=0.0;                          //������Χ����Ϊ��
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
        ecFuzzy[0]=0.0;                                //ƫ�����Χ����Ϊ��
		pec=5;
	}
	ecFuzzy[1]=1.0-ecFuzzy[0];
	//ע��������  += ��һ���ۼƵĹ���
	/*num������ǰ�����̶ȼ����ʹ�������һ��ģ���Ӽ���Χ�ڣ�3������� 2��֮ 1�ٴ�֮ 0����0*/
	/*��ѯģ�������*/
	num=KpRule[pe][pec];
	KpFuzzy[num]+=eFuzzy[0]*ecFuzzy[0];       //ƫ������*ƫ��仯������

	num=KpRule[pe][pec+1];
	KpFuzzy[num]+=eFuzzy[0]*ecFuzzy[1];      //ƫ������*ƫ��仯�ʲ�����

	num=KpRule[pe+1][pec];
	KpFuzzy[num]+=eFuzzy[1]*ecFuzzy[0];      //ƫ�����*ƫ��仯������

	num=KpRule[pe+1][pec+1];
	KpFuzzy[num]+=eFuzzy[1]*ecFuzzy[1];      //ƫ�����*ƫ��仯�ʲ�����
	/*��Ȩƽ���㷨��ģ��*/                                //ģ���Ӽ���ƫ�������̶ȵĳ˻�
	Kp_calcu=KpFuzzy[0]*kpRule[0]+KpFuzzy[1]*kpRule[1]+KpFuzzy[2]*kpRule[2]+KpFuzzy[3]*kpRule[3];
	/* KpFuzzy[0] KpFuzzy[1] KpFuzzy[2] KpFuzzy[3] ���Էֱ����һ�ֱ�������˵Ȩ��
    num=KpRule[pe][pec]; ��һϵ�е�������Ķ���num��������һ�� KpFuzzy
    ��ôģ��������У�0 1 2 3 ��ֱ���ģ���Ӽ����Ӧ
	*/
	return(Kp_calcu);
}

/*ģ�������ٶ�*/
/*ģ�������ٶȸ�ģ������PID������һ��
*ģ�������ٶ�ʱ�����������Ƕ��ƫ���ƫ��仯��
*���������ʱ���������ٶ�ƫ���ƫ��仯��
*�ٶȼӼ���Ҫ��ƫ���ʱ�ٶ�С��ƫ��Сʱ�ٶȴ�
*�������ļ���պ��෴
*/
float fuzzy_speed(float e,float ec)
{
    float Kp_calcu;
	uint8 num,pe,pec;
	float eRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0};        //��������
	float ecRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0};       //���仯�ʵ�����
	float eFuzzy[2]={0.0,0.0};                              //���������̶�
	float ecFuzzy[2]={0.0,0.0};                             //���仯�ʵ������̶�
	float kpRule[4]={120.0,80.0,50.0,0.0};                    //Kp��ģ���Ӽ�                  //��������������
	float KpFuzzy[4]={0.0,0.0,0.0,0.0};                     //������Kp�������̶�
	/*
	KpFuzzy[0]  kpRule[0]
	KpFuzzy[1]  kpRule[1]
	KpFuzzy[2]  kpRule[2]
	KpFuzzy[4]  kpRule[3]
	*/
 	//error=64-mid_line    error>0   ���
 	//                     error<0   �ҹ�
	int   KpRule[7][7]=                    //speed��ģ�����Ʊ�
	{   /*��Ϊpe*/
        3,3,3,3,3,3,3,    /*��Ϊpec*/
		2,2,2,2,2,2,2,
		1,1,1,1,1,1,1,
		0,0,0,0,0,0,0,
		1,1,1,1,1,1,1,
		2,2,2,2,2,2,2,
		3,3,3,3,3,3,3,
	};
//      Kp��ģ������
//	    3,3,3,3,3,3,3,    /*��Ϊpec*/
//		2,2,2,2,2,2,2,
//		1,1,1,1,1,1,1,
//		1,1,0,1,0,1,1,
//		0,0,1,0,0,1,0,
//		0,1,0,1,0,0,2,
//		3,3,3,3,3,3,3,
	/*���e�������̶�����*/
	if(e<eRule[0])
	{
	    eFuzzy[0]=1.0;               //eFuzzy[0] ��ʾ����������˼
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
        eFuzzy[0]=0.0;                          //������Χ����Ϊ��
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
        ecFuzzy[0]=0.0;                                //ƫ�����Χ����Ϊ��
		pec=5;
	}
	ecFuzzy[1]=1.0-ecFuzzy[0];
	//ע��������  += ��һ���ۼƵĹ���
	/*num������ǰ�����̶ȼ����ʹ�������һ��ģ���Ӽ���Χ�ڣ�3������� 2��֮ 1�ٴ�֮ 0����0*/
	/*��ѯģ�������*/
	num=KpRule[pe][pec];
	KpFuzzy[num]+=eFuzzy[0]*ecFuzzy[0];       //ƫ������*ƫ��仯������

	num=KpRule[pe][pec+1];
	KpFuzzy[num]+=eFuzzy[0]*ecFuzzy[1];      //ƫ������*ƫ��仯�ʲ�����

	num=KpRule[pe+1][pec];
	KpFuzzy[num]+=eFuzzy[1]*ecFuzzy[0];      //ƫ�����*ƫ��仯������

	num=KpRule[pe+1][pec+1];
	KpFuzzy[num]+=eFuzzy[1]*ecFuzzy[1];      //ƫ�����*ƫ��仯�ʲ�����
	/*��Ȩƽ���㷨��ģ��*/                                //ģ���Ӽ���ƫ�������̶ȵĳ˻�
	Kp_calcu=KpFuzzy[0]*kpRule[0]+KpFuzzy[1]*kpRule[1]+KpFuzzy[2]*kpRule[2]+KpFuzzy[3]*kpRule[3];
	/* KpFuzzy[0] KpFuzzy[1] KpFuzzy[2] KpFuzzy[3] ���Էֱ����һ�ֱ�������˵Ȩ��
    num=KpRule[pe][pec]; ��һϵ�е�������Ķ���num��������һ�� KpFuzzy
    ��ôģ��������У�0 1 2 3 ��ֱ���ģ���Ӽ����Ӧ
	*/
	return(Kp_calcu);
}


/*ģ������Ki*/
float fuzzy_ki(float e,float ec)
{
    float Kp_calcu;
	uint8 num,pe,pec;
	float eRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0};        //��������
	float ecRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0};       //���仯�ʵ�����
	float eFuzzy[2]={0.0,0.0};                              //���������̶�
	float ecFuzzy[2]={0.0,0.0};                             //���仯�ʵ������̶�
	float kpRule[4]={0.0,22.0,44.0,66.0};                    //Kp��ģ���Ӽ�
	float KpFuzzy[4]={0.0,0.0,0.0,0.0};                     //������Kp�������̶�
	int   KpRule[7][7]=                    //Kp��ģ�����Ʊ�
	{   /*��Ϊpe*/
        0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,
		0,0,0,0,0,0,0,
		2,0,0,0,0,0,1,
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

/*ģ������Kd*/
float fuzzy_kd(float e,float ec)
{
    float Kp_calcu;
	uint8 num,pe,pec;
	float eRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0};        //��������
	float ecRule[7]={-3.0,-2.0,-1.0,0.0,1.0,2.0,3.0};       //���仯�ʵ�����
	float eFuzzy[2]={0.0,0.0};                              //���������̶�
	float ecFuzzy[2]={0.0,0.0};                             //���仯�ʵ������̶�
	float kpRule[4]={0.0,8.0,16.0,24.0};                    //Kp��ģ���Ӽ�
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

/*�򵥵��ٶ�����ʽ����*/
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

