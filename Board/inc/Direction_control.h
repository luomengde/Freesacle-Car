#ifndef _DIRECTION_CONTROL_H
#define _DIRECTION_CONTROL_H


extern uint32 dir_duty;;    //用于传递的舵机占空比参数
extern float mid_line;		//经过计算后，一帧图像的中值应该是 float
extern float mid_line_new;
extern float Kp_duoji;    //舵机Kp
extern float Kd_duoji;    //舵机Kd
extern int32 SE_duty;
extern float dealt_error;
extern float error;        //舵机偏差
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
