#ifndef __MAIN_H__
#define __MAIN_H__

#include "stm32f4xx.h"
#include "common.h"
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
}Rec_t;//接收

typedef enum
{
	stop=0,//停止
	normal=1,//正常
	prepare=2,//准备
}systemStatus_t;//系统状态

typedef enum
{
	motion=1,//手势
	remote=2,//遥控
	allUse=3,//全部使用
	noUse=4,//全不使用
}operateStatus_t;//操作状态
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

