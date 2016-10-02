
#ifndef _VCAN_UI_VAR_H_
#define _VCAN_UI_VAR_H_


#define VAR_SELECT_HOLD_OFFSET      ((fv_MAX+4-1)/4)       //快上快下时，每次切换偏移量。目前设置为变量总数的1/4.可修改成其他数
#define VAR_VALUE_HOLE_OFFSET       10

#define UI_VAR_USE_LCD              1                       //同步信号时需要定义是否使用LCD显示数据（1为显示，0为不显示）

typedef struct
{
    uint32  val;            //目前的值
    uint32  oldval;         //通常情况下，两者是相同，修改后，没按确认键，则不相同。按确认键发送后，则相同
                            //即最后发送的值
    uint32  minval;         //最小值
    uint32  maxval;         //最大值
    Site_t  site;           //LCD 显示的坐标位置
}  ui_var_info_t;            //变量信息

typedef struct
{
    float  val;            //目前的值
    float  oldval;         //通常情况下，两者是相同，修改后，没按确认键，则不相同。按确认键发送后，则相同
                            //即最后发送的值
    float  minval;         //最小值
    float  maxval;         //最大值

    Site_t  site;           //LCD 显示的坐标位置
} ui_fv_info_t;            //变量信息

typedef struct
{
    const uint8 *str;            //目前的值

    Site_t  site;           //LCD 显示的坐标位置
} ui_str_info_t;            //常量信息


typedef enum
{
  init_screen,

  camera_screen,

  data_screen,

  max_screen,
} display_screen;


typedef enum
{
    //变量的编号

    /* 8位变量 */
    CAR_CTRL,
    VAR1,
    VAR2,
    VAR_8BIT = VAR2, //8位变量的结束编号

    /* 16位变量 */
    VAR3,
    VAR4,
    VAR_16BIT  = VAR4,          //16位变量的结束编号

    /* 32位变量 */
    VAR5,
    VAR6,
    VAR_32BIT  = VAR6,          //32位变量的结束编号

    VAR_MAX,        //变量数目
} var_tab_e;

typedef enum
{   FV1,
    FV2,
    FV3,
    FV4,
    FV5,
    FV6,
    FV7,
    FV8,
    var_MAX=FV8,
    FV9,
    FV10,
    FV11,
    FV12,
    FV13,
    FV14,
    FV15,
    FV16,
    fv_MAX,        //变量数目
}fv_tab_e;

typedef enum
{
    VAR_NEXT,           //下一个
    VAR_PREV,           //上一个
    VAR_NEXT_HOLD,      //快下，偏移为：VAR_SELECT_HOLD_OFFSET
    VAR_PREV_HOLD,      //快上，偏移为：VAR_SELECT_HOLD_OFFSET

    VAR_ADD,            //加1
    VAR_SUB,            //减1
    VAR_ADD_HOLD,       //快加，偏移为：VAR_VALUE_HOLE_OFFSET
    VAR_SUB_HOLD,       //快减，偏移为：VAR_VALUE_HOLE_OFFSET

    VAR_OK,             //确定
    VAR_CANCEL,         //取消

    VAR_EVENT_MAX,
} ui_var_event_e;

extern uint8    new_tab;                                //当前选择的变量编号
extern uint32   last_tab;                                   //最后接收到的变量编号

extern uint8   str_new_tab;                            //当前选择的常量编号
extern uint32  str_last_tab;                               //最后接收到的常量编号

extern float   flash_duoji_Kp1;
extern float   flash_duoji_Kp2;
extern float   flash_duoji_Kp1_you;
extern float   flash_duoji_Kp2_you;
extern float   flash_duoji_Kd;

extern float   flash_dianji_Kp;
extern float   flash_dianji_Ki;
extern float   flash_dianji_Kd; 
extern uint32   flash_speed_change;
extern uint32  flash_speed_change_error;
extern uint32 flash_speed_want;
extern uint32 flash_camera_boundary;
extern uint32 flash_yuzhi;
extern uint32 flash_speed_kvff;
extern uint32 flash_speed_kaff;

extern void     var_init();
extern void     var_display(uint8 tab);                             //显示编号变量的值


//变量 的 选择 与 加减 控制
extern void     var_select(ui_var_event_e  ctrl);   //选择切换变量
extern void     var_value(ui_var_event_e ctrl);     //改变变量的值
extern void     var_ok();                           //确认当前选择的
extern void     val_cancel();                       //取消当前选择的值  OK

extern void     str_display(uint8 tab);
extern void     str_select(ui_var_event_e  ctrl);
extern void     str_ok();
extern void     void_return();
extern void     bmp_display();

#endif  //_FIRE_UI_VAR_H_


