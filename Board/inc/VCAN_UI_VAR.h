
#ifndef _VCAN_UI_VAR_H_
#define _VCAN_UI_VAR_H_


#define VAR_SELECT_HOLD_OFFSET      ((fv_MAX+4-1)/4)       //���Ͽ���ʱ��ÿ���л�ƫ������Ŀǰ����Ϊ����������1/4.���޸ĳ�������
#define VAR_VALUE_HOLE_OFFSET       10

#define UI_VAR_USE_LCD              1                       //ͬ���ź�ʱ��Ҫ�����Ƿ�ʹ��LCD��ʾ���ݣ�1Ϊ��ʾ��0Ϊ����ʾ��

typedef struct
{
    uint32  val;            //Ŀǰ��ֵ
    uint32  oldval;         //ͨ������£���������ͬ���޸ĺ�û��ȷ�ϼ�������ͬ����ȷ�ϼ����ͺ�����ͬ
                            //������͵�ֵ
    uint32  minval;         //��Сֵ
    uint32  maxval;         //���ֵ
    Site_t  site;           //LCD ��ʾ������λ��
}  ui_var_info_t;            //������Ϣ

typedef struct
{
    float  val;            //Ŀǰ��ֵ
    float  oldval;         //ͨ������£���������ͬ���޸ĺ�û��ȷ�ϼ�������ͬ����ȷ�ϼ����ͺ�����ͬ
                            //������͵�ֵ
    float  minval;         //��Сֵ
    float  maxval;         //���ֵ

    Site_t  site;           //LCD ��ʾ������λ��
} ui_fv_info_t;            //������Ϣ

typedef struct
{
    const uint8 *str;            //Ŀǰ��ֵ

    Site_t  site;           //LCD ��ʾ������λ��
} ui_str_info_t;            //������Ϣ


typedef enum
{
  init_screen,

  camera_screen,

  data_screen,

  max_screen,
} display_screen;


typedef enum
{
    //�����ı��

    /* 8λ���� */
    CAR_CTRL,
    VAR1,
    VAR2,
    VAR_8BIT = VAR2, //8λ�����Ľ������

    /* 16λ���� */
    VAR3,
    VAR4,
    VAR_16BIT  = VAR4,          //16λ�����Ľ������

    /* 32λ���� */
    VAR5,
    VAR6,
    VAR_32BIT  = VAR6,          //32λ�����Ľ������

    VAR_MAX,        //������Ŀ
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
    fv_MAX,        //������Ŀ
}fv_tab_e;

typedef enum
{
    VAR_NEXT,           //��һ��
    VAR_PREV,           //��һ��
    VAR_NEXT_HOLD,      //���£�ƫ��Ϊ��VAR_SELECT_HOLD_OFFSET
    VAR_PREV_HOLD,      //���ϣ�ƫ��Ϊ��VAR_SELECT_HOLD_OFFSET

    VAR_ADD,            //��1
    VAR_SUB,            //��1
    VAR_ADD_HOLD,       //��ӣ�ƫ��Ϊ��VAR_VALUE_HOLE_OFFSET
    VAR_SUB_HOLD,       //�����ƫ��Ϊ��VAR_VALUE_HOLE_OFFSET

    VAR_OK,             //ȷ��
    VAR_CANCEL,         //ȡ��

    VAR_EVENT_MAX,
} ui_var_event_e;

extern uint8    new_tab;                                //��ǰѡ��ı������
extern uint32   last_tab;                                   //�����յ��ı������

extern uint8   str_new_tab;                            //��ǰѡ��ĳ������
extern uint32  str_last_tab;                               //�����յ��ĳ������

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
extern void     var_display(uint8 tab);                             //��ʾ��ű�����ֵ


//���� �� ѡ�� �� �Ӽ� ����
extern void     var_select(ui_var_event_e  ctrl);   //ѡ���л�����
extern void     var_value(ui_var_event_e ctrl);     //�ı������ֵ
extern void     var_ok();                           //ȷ�ϵ�ǰѡ���
extern void     val_cancel();                       //ȡ����ǰѡ���ֵ  OK

extern void     str_display(uint8 tab);
extern void     str_select(ui_var_event_e  ctrl);
extern void     str_ok();
extern void     void_return();
extern void     bmp_display();

#endif  //_FIRE_UI_VAR_H_


