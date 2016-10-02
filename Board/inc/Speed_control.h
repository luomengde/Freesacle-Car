#ifndef _SPEED_CONTROL_H
#define _SPEED_CONTROL_H




extern void  PID_init();                        //�ٶ�pid��ʼ��
extern float PID_realize(float speed);
typedef struct {                                   //λ��ʽPID   ����һ�²���
float SetSpeed; //�����趨ֵ
float ActualSpeed; //����ʵ��ֵ
float err; //����ƫ��ֵ
float err_last; //������һ��ƫ��ֵ
float Kp,Ki,Kd; //������������֡�΢��ϵ��
float voltage; //�����ѹֵ������ִ�����ı�����
float integral; //�������ֵ
}PID_speed_weizhi;


typedef struct {
float SetSpeed; //�����趨ֵ
float ActualSpeed; //����ʵ��ֵ
float err; //����ƫ��ֵ
float err_next; //������һ��ƫ��ֵ
float err_last; //��������ǰ��ƫ��ֵ
float Kp,Ki,Kd; //������������֡�΢��ϵ��
}PID_speed_zengliang;


extern PID_speed_weizhi pid;
extern int16 Moto_pluse; 
extern float speed_duty;
extern int16 speed_current;
extern double  speed_control_output;
extern uint16 speed_want;
extern float  speed_err;
extern float  speed_racc;
extern uint8  first_enter_obstancle;

extern void GetMotoPulse(void);
extern void SpeedControl(void);
extern void SpeedControlOutput(void);
extern void start_line_stop(uint16 count);
extern void Motor_control(int32 motor_num);



#endif   //end of  _SPEED_CONTROL_H

