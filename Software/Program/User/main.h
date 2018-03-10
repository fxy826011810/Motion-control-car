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
	motion_t motion;		//����
	remote_t remote;		//ң��
}Rec_t;//����

typedef enum
{
	stop=0,//ֹͣ
	normal=1,//����
	prepare=2,//׼��
}systemStatus_t;//ϵͳ״̬

typedef enum
{
	motion=1,//����
	remote=2,//ң��
	allUse=3,//ȫ��ʹ��
	noUse=4,//ȫ��ʹ��
}operateStatus_t;//����״̬
typedef struct
{
	uint32_t heart;			//����
	mecArm_t *mecArm;		//��е��
	chassis_t *chassis;	//����
	Rec_t *rec;					//����
	debug_t *debug;			//����
	systemStatus_t systemStatus;//ϵͳ״̬
	operateStatus_t operateStatus;//����״̬
}cmd_t;

extern chassis_t Chassis;
extern mecArm_t MecArm;
extern Rec_t Rec;
extern cmd_t cmd;


#endif

