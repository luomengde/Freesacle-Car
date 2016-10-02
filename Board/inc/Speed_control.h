#ifndef _SPEED_CONTROL_H
#define _SPEED_CONTROL_H




extern void  PID_init();                        //速度pid初始化
extern float PID_realize(float speed);
typedef struct {                                   //位置式PID   简单做一下测试
float SetSpeed; //定义设定值
float ActualSpeed; //定义实际值
float err; //定义偏差值
float err_last; //定义上一个偏差值
float Kp,Ki,Kd; //定义比例、积分、微分系数
float voltage; //定义电压值（控制执行器的变量）
float integral; //定义积分值
}PID_speed_weizhi;


typedef struct {
float SetSpeed; //定义设定值
float ActualSpeed; //定义实际值
float err; //定义偏差值
float err_next; //定义上一个偏差值
float err_last; //定义最上前的偏差值
float Kp,Ki,Kd; //定义比例、积分、微分系数
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

