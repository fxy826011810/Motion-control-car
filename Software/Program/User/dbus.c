#include "dbus.h"
#include "tim.h"
#include "gpio.h"
#include "control.h"
#include "stm32f4xx.h"
#include "pid.h"

uint8_t DBUS_BUFFER[19];
DBUS_TypeDef DBUS={0};
control_mode_t control_mode=prepare;

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

void dbus_getdata(DBUS_TypeDef *dbus)//ң�����ݽ���
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
			control_mode=remote;
		}
		else if(dbus->rc.s1==2)
		{
			control_mode=stop;
		}
		else if(dbus->rc.s1==3)
		{
			control_mode=keyboard;
		}		
}


void status_check(DBUS_TypeDef *dbus)//������ģʽ
{	
	switch (control_mode)
	{
		case prepare:
		break;
		case remote:
		remote_control(&DBUS);
		remote_s2_check(&DBUS);//Ħ���ֵ������͹ر�
		break;
		case keyboard:
		keyboard_control(&DBUS);
		keyboard_k_check(&DBUS);
		break;
		case stop:
		break;
	}
	
}


ChassisSend_t ChassisSend={0};

void keyboard_k_check(DBUS_TypeDef *dbus)
{

}


void remote_s2_check(DBUS_TypeDef *dbus)
{
	
}


void keyboard_control(DBUS_TypeDef *dbus)//���̿���
{

}


extern float Cm1realpangle,Cm2realpangle;
void remote_control(DBUS_TypeDef *dbus)//ң�ؿ���
{

}
