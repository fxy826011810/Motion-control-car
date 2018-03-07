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
#include "debug.h"
#define	LED_HEAT() GPIOC->ODR^=GPIO_Pin_1|GPIO_Pin_2
#define LED1(x)	x ? GPIO_SetBits(GPIOC,GPIO_Pin_1):GPIO_ResetBits(GPIOC,GPIO_Pin_1)
#define LED2(x)	x ? GPIO_SetBits(GPIOC,GPIO_Pin_2):GPIO_ResetBits(GPIOC,GPIO_Pin_2)

typedef struct
{
	motion_t motion;		//手势
	remote_t remote;		//遥控
}Rec_t;

typedef enum
{
	stop=0,
	normal=1,
	prepare=2,
}systemStatus_t;

typedef enum
{
	motion=1,
	remote=2,
	allUse=3,
	noUse=4,
}operateStatus_t;//操作
typedef struct
{
	uint32_t heart;			//心跳
	mecArm_t *mecArm;		//机械臂
	chassis_t *chassis;	//底盘
	Rec_t *rec;					//接收
	debug_t *debug;			//调试
	systemStatus_t systemStatus;//系统状态
	operateStatus_t operateStatus;//操作状态
}cmd_t;

extern chassis_t Chassis;
extern mecArm_t MecArm;
extern Rec_t Rec;
extern cmd_t cmd;


#endif

