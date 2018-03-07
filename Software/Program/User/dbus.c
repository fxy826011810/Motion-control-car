#include "dbus.h"
#include "tim.h"
#include "gpio.h"
#include "control.h"
#include "stm32f4xx.h"
#include "pid.h"
/* ----------------------- RC Channel Definition---------------------------- */
#define RC_CH_VALUE_MIN	((uint16_t)364 )
#define RC_CH_VALUE_OFFSET	((uint16_t)1024)
#define RC_CH_VALUE_MAX	((uint16_t)1684)

/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP	((uint16_t)1)
#define RC_SW_MID	((uint16_t)3)
#define RC_SW_DOWN	((uint16_t)2)

/* ----------------------- PC Key Definition-------------------------------- */
#define KEY_W	((uint16_t)0x01<<0)
#define KEY_S	((uint16_t)0x01<<1)
#define KEY_A	((uint16_t)0x01<<2)
#define KEY_D	((uint16_t)0x01<<3)
#define KEY_Q	((uint16_t)0x01<<4)
#define KEY_E	((uint16_t)0x01<<5)
#define KEY_SHIFT	((uint16_t)0x01<<6)
#define KEY_CTRL	((uint16_t)0x01<<7)

#define KEY_V		0x4000
#define KEY_C		0x2000
#define KEY_X		0x1000
#define KEY_Z		0x0800
#define KEY_G		0x0400
#define KEY_F		0x0200
#define KEY_R		0x0100
/* ----------------------- Data Struct ------------------------------------- */
uint8_t DBUS_BUFFER[19];


void dbus_reset(DBUS_TypeDef *dbus)
{
	dbus->key.v=0;
	dbus->mouse.press_l=0;
	dbus->mouse.press_r=0;
	dbus->mouse.x=0;
	dbus->mouse.y=0;
	dbus->mouse.z=0;
	dbus->rc.ch0=0;
	dbus->rc.ch1=0;
	dbus->rc.ch2=0;
	dbus->rc.ch3=0;
}

void dbus_getdata(DBUS_TypeDef *dbus)//遥控数据解算
{				

		dbus->rc.ch0=((DBUS_BUFFER[0]|(int16_t)DBUS_BUFFER[1]<<8)&0x07FF)-1024;
		dbus->rc.ch1=((DBUS_BUFFER[1]>>3|(int16_t)DBUS_BUFFER[2]<<5)&0x07FF)-1024;
		dbus->rc.ch2 =((DBUS_BUFFER[2]>>6|(int16_t)DBUS_BUFFER[3] << 2| DBUS_BUFFER[4]<<10) & 0x07FF)-1024;
		dbus->rc.ch3 =((DBUS_BUFFER[4]>>1|(int16_t)DBUS_BUFFER[5] << 7) & 0x07FF)-1024;
		dbus->rc.s1 = (DBUS_BUFFER[5] >> 4) & 0x03;
		dbus->rc.s2 = (DBUS_BUFFER[5] >> 6) & 0x03;
		dbus->mouse.x = DBUS_BUFFER[6] | (int16_t)DBUS_BUFFER[7] << 8;
		dbus->mouse.y = DBUS_BUFFER[8] | (int16_t)DBUS_BUFFER[9] << 8;
		dbus->mouse.z = DBUS_BUFFER[10] | (int16_t)DBUS_BUFFER[11] << 8;
		dbus->mouse.press_l = DBUS_BUFFER[12];
		dbus->mouse.press_r = DBUS_BUFFER[13];
		dbus->key.v = DBUS_BUFFER[14] | (int16_t)DBUS_BUFFER[15] << 8;
		if(dbus->rc.s1==1)
		{
//			control_mode=remote;
		}
		else if(dbus->rc.s1==2)
		{
//			control_mode=stop;
		}
		else if(dbus->rc.s1==3)
		{
//			control_mode=keyboard;
		}		
}


void status_check(DBUS_TypeDef *dbus)//检测控制模式
{	
//	switch (control_mode)
//	{
//		case prepare:
//		break;
//		case remote:
////		remote_control(&DBUS);
////		remote_s2_check(&DBUS);//摩擦轮的启动和关闭
//		break;
//		case keyboard:
////		keyboard_control(&DBUS);
////		keyboard_k_check(&DBUS);
//		break;
//		case stop:
//		break;
//	}
	
}


void keyboard_k_check(DBUS_TypeDef *dbus)
{

}


void remote_s2_check(DBUS_TypeDef *dbus)
{
	
}


void keyboard_control(DBUS_TypeDef *dbus)//键盘控制
{

}

void remote_control(DBUS_TypeDef *dbus)//遥控控制
{

}
