#ifndef _VCAN_KEY_EVNET_H_
#define _VCAN_KEY_EVNET_H_
#include "common.h"
#include "VCAN_key.h"
#include "include.h"
extern void  key_event_init();                              //��ʼ��������Ϣ
extern void  deal_key_event(void);                          //��������Ϣ���Զ��л�ѡ�е����֣��Զ������ֽ��мӼ���
extern void  bmp_init();
extern const uint16    *bmp[3];
extern uint8  BM1,BM2,BM3,BM4;
#endif  