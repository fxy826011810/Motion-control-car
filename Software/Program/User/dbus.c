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


void dbus_reset(remote_t *dbus)
{
	dbus->dbusRec.key.v=0;
	dbus->dbusRec.mouse.press_l=0;
	dbus->dbusRec.mouse.press_r=0;
	dbus->dbusRec.mouse.x=0;
	dbus->dbusRec.mouse.y=0;
	dbus->dbusRec.mouse.z=0;
	dbus->dbusRec.rc.ch0=0;
	dbus->dbusRec.rc.ch1=0;
	dbus->dbusRec.rc.ch2=0;
	dbus->dbusRec.rc.ch3=0;
	dbus->dbusRec.rc.s1=0;
	dbus->dbusRec.rc.s2=0;
	dbus->dataStatus=wait;
}

void dbus_getdata(remote_t *dbus)//Ò£¿ØÊý¾Ý½âËã
{
		dbus->dbusRec.rc.ch0=((DBUS_BUFFER[0]|(int16_t)DBUS_BUFFER[1]<<8)&0x07FF)-1024;
		dbus->dbusRec.rc.ch1=((DBUS_BUFFER[1]>>3|(int16_t)DBUS_BUFFER[2]<<5)&0x07FF)-1024;
		dbus->dbusRec.rc.ch2 =((DBUS_BUFFER[2]>>6|(int16_t)DBUS_BUFFER[3] << 2| DBUS_BUFFER[4]<<10) & 0x07FF)-1024;
		dbus->dbusRec.rc.ch3 =((DBUS_BUFFER[4]>>1|(int16_t)DBUS_BUFFER[5] << 7) & 0x07FF)-1024;
		dbus->dbusRec.rc.s1 = (DBUS_BUFFER[5] >> 4) & 0x03;
		dbus->dbusRec.rc.s2 = (DBUS_BUFFER[5] >> 6) & 0x03;
		dbus->dbusRec.mouse.x = DBUS_BUFFER[6] | (int16_t)DBUS_BUFFER[7] << 8;
		dbus->dbusRec.mouse.y = DBUS_BUFFER[8] | (int16_t)DBUS_BUFFER[9] << 8;
		dbus->dbusRec.mouse.z = DBUS_BUFFER[10] | (int16_t)DBUS_BUFFER[11] << 8;
		dbus->dbusRec.mouse.press_l = DBUS_BUFFER[12];
		dbus->dbusRec.mouse.press_r = DBUS_BUFFER[13];
		dbus->dbusRec.key.v = DBUS_BUFFER[14] | (int16_t)DBUS_BUFFER[15] << 8;
		dbus->dataStatus=ready;	
}

//void keyboard_k_check(DBUS_TypeDef *dbus)
//{

//}


//void remote_s2_check(DBUS_TypeDef *dbus)
//{
//	
//}


//void keyboard_control(DBUS_TypeDef *dbus)//¼üÅÌ¿ØÖÆ
//{

//}

//void remote_control(DBUS_TypeDef *dbus)//Ò£¿Ø¿ØÖÆ
//{

//}
