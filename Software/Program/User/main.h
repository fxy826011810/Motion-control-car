#ifndef __MAIN_H__
#define __MAIN_H__

#include "stm32f4xx.h"
#include "monitor.h"

#include "config.h"
#include "debug.h"
#include "pid.h"
#include "usart.h"
#include "dbus.h"
#include "control.h"

#define	LED_HEAT() GPIOC->ODR^=GPIO_Pin_1
#define LED1(x)	x ? GPIO_SetBits(GPIOC,GPIO_Pin_1):GPIO_ResetBits(GPIOC,GPIO_Pin_1)
#define LED2(x)	x ? GPIO_SetBits(GPIOC,GPIO_Pin_2):GPIO_ResetBits(GPIOC,GPIO_Pin_2)

typedef struct
{
	motion_t motion;
	remote_t remote;
}Rec_t;


typedef struct
{
	uint32_t heart;
	mecArm_t *mecArm;//»úÐµ±Û
	chassis_t *chassis;//µ×ÅÌ
	Rec_t *rec;
	debug_t *debug;//µ÷ÊÔ
}cmd_t;

extern chassis_t Chassis;
extern mecArm_t MecArm;
extern Rec_t Rec;
extern cmd_t cmd;


#endif

