#ifndef __PICTURE_DEAL_H
#define __PICTURE_DEAL_H

#define line_num 70

extern void    median_line_extract(uint8 *src);
extern void avoid_obstacle(uint8 *src);
extern void start_line(uint8 *src);
extern void RoadIdentify();
extern uint8  used_length;
extern float last_mid_line;
extern  float mid_line;
extern  float mid_line_new;
extern uint8 left_line_first_flag;
extern uint8 right_line_first_flag;
/*黑线*/
extern uint8 black_startline;
extern uint8 black_endline;
extern  uint8 distance[70];
extern uint8 zhijiao_forward1;
/*直角*/
extern uint8 right_turn_zhijiao_flag;
extern uint8 left_turn_zhijiao_flag;
extern uint8 left_zhijiao_flag;
extern uint8 right_zhijiao_flag;
extern uint8 right_zhijiao;
extern uint8 left_zhijiao;
extern uint8 tubian_line_flag;
extern uint8 road_lianxu_tumble_flag;
extern uint8 road_lianxu_right_lose_flag;
extern uint8 road_lianxu_left_lose_flag;
/*中线*/
extern uint8 close_black_flag;
extern uint16 road_wide_add;      //坡道宽度累加
extern uint16 road_wide_normal;
extern uint8  road_wide_use;
extern uint8 single_in;
//变量信息
extern Site_t site_end;
//起跑线
extern uint8 stop;
extern uint16 stop_count;

extern uint8 Speed_protect;    //保护
extern uint32 shizi_new;       //按列处理法   //多试一试
extern uint8 shi_zi_flag;
extern int16 podao_flag_time;

extern uint8  the_far_road_flag;  //赛道类型标志位
extern uint8  the_near_road_flag;
extern int16 DD_mid_line_add;

extern uint8  X_point[70];
extern uint8  Y_point[70];

extern uint8 lost_count;
extern uint16 left_add_point_count;
extern uint16 right_add_point_count;

extern uint8 left_lost_count;
extern uint8 right_lost_count;

extern uint8 shi_zi_count;
extern uint8 never_shi_zi_count;

extern uint8 never_obstacle_flag;
extern uint8 left_obstancle_flag;
extern uint8 right_obstancle_flag;

extern uint8 single_count_new;                
extern uint8 single_lost_count_new;    
extern uint8 into_circle_flag;             
extern uint8 out_circle_flag;    
extern uint8 out_circle_time_flag;
extern uint16 out_circle_count;
extern uint8 begin_the_frost_point;
typedef enum
{
  left_normal_right_lose,
  left_lose_right_normal,
   all_normal,
   all_lose,
   all_back_line
}line_case;

typedef struct
{
  int16 left_line;
  int16 right_line;
  int16 left_line_unrepiar;
  int16  right_line_unrepiar;
  int16 left_single;
  int16 right_single;
  int16 mid_line_new;
  uint16 road_wide;
  line_case line_case_mode;
  uint8 single_road_wide;

} line_info;
extern line_info  line[line_num];
#endif //___PICTURE_DEAL_H