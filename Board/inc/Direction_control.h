#ifndef _DIRECTION_CONTROL_H
#define _DIRECTION_CONTROL_H


extern uint32 dir_duty;;    //���ڴ��ݵĶ��ռ�ձȲ���
extern float mid_line;		//���������һ֡ͼ�����ֵӦ���� float
extern float mid_line_new;
extern float Kp_duoji;    //���Kp
extern float Kd_duoji;    //���Kd
extern int32 SE_duty;
extern float dealt_error;
extern float error;        //���ƫ��
extern uint8 podao_flag;
extern uint8 never_podao_flag;
extern int8 mid_line_last[5];
extern int32 se_du;
extern uint8 podao_count;
extern float index_A;
extern float index_B;
extern float error_d;
extern float  before_cal_mid_line;

extern void quan_dir_duty_new();
extern void DJ_PID(void);
extern float cal_curvature(int16 x1,int16 y1,int16 x2,int16 y2,int16 x3,int16 y3);
extern void DJ_PID_New(void);













#endif    //end of _DIRECTION_CONTROL_H
