#ifndef __MONITOR_H__
#define __MONITOR_H__
#include "stm32f4xx.h"


typedef enum 
{
	online=1,
	offline=2
}moduleStatus_t;

typedef struct System_Monitor_t 
{
	int16_t count;
	int16_t time;
	moduleStatus_t status;
	void (*cailback)(void);
}System_Monitor_t;

void Monitor_Init(void);
void Monitor_Update(void);
void Monitor_Set(System_Monitor_t *mon);

extern System_Monitor_t ForeArm_Monitor;//大臂接收帧率
extern System_Monitor_t MainArm_Monitor;//小臂接收帧率

extern System_Monitor_t CanCm1_Monitor;//底盘1接收帧率
extern System_Monitor_t CanCm2_Monitor;//底盘2接收帧率
extern System_Monitor_t CanCm3_Monitor;//底盘3接收帧率
extern System_Monitor_t CanCm4_Monitor;//底盘4接收帧率

extern System_Monitor_t IMURec_Monitor;
extern System_Monitor_t ChassisGyro_Monitor;

extern System_Monitor_t DbusRec_Monitor;
#endif
