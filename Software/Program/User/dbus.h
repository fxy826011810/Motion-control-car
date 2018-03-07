#ifndef __DBUS_H
#define __DBUS_H
#include "stm32f4xx.h"
#include "monitor.h"
 typedef struct
{
	 struct
	 {
		int16_t ch0; 
		int16_t ch1; 
		int16_t ch2; 
		int16_t ch3; 
		uint8_t s1; 
		uint8_t s2;
	 }rc;
	 struct
	 {
		 int16_t x; 
		 int16_t y; 
		 int16_t z; 
		 uint8_t press_l; 
		 uint8_t press_r;
	 }mouse;

	 struct
	 {
		 uint16_t v;
	 }key;
}DBUS_TypeDef;//ң�ؽ����Ľṹ��
 
 typedef struct
{
	System_Monitor_t *mon;//������
	DBUS_TypeDef dbusRec;
}remote_t;

typedef struct ChassisSend_t
{
	float X;
	float LAST_X;
	float XMax;
	float Y;
	float LAST_Y;
	float YMax;
	float rotate;
	float LAST_rotate;
}ChassisSend_t;//�����ٶ�
extern ChassisSend_t ChassisSend;


//extern DBUS_TypeDef DBUS;
void dbus_reset(DBUS_TypeDef *dbus);
void dbus_getdata(DBUS_TypeDef *dbus);
void status_check(DBUS_TypeDef *dbus);
void remote_s2_check(DBUS_TypeDef *dbus);
void remote_s1_check(DBUS_TypeDef *dbus);
void remote_control(DBUS_TypeDef *dbus);//ң��ģʽ
void keyboard_control(DBUS_TypeDef *dbus);//����ģʽ
void keyboard_k_check(DBUS_TypeDef *dbus);
#endif
