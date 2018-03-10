#ifndef __DBUS_H
#define __DBUS_H
#include "stm32f4xx.h"
#include "monitor.h"
#include "control.h"
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
}DBUS_TypeDef;//遥控解算后的结构体
 
 typedef struct
{
	System_Monitor_t *mon;//监视器
	dataStatus_t dataStatus;
	DBUS_TypeDef dbusRec;
}remote_t;

void dbus_reset(remote_t *dbus);
void dbus_getdata(remote_t *dbus);

#endif
