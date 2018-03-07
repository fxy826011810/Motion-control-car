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

extern System_Monitor_t ForeArm_Monitor;//��۽���֡��
extern System_Monitor_t MainArm_Monitor;//С�۽���֡��

extern System_Monitor_t CanCm1_Monitor;//����1����֡��
extern System_Monitor_t CanCm2_Monitor;//����2����֡��
extern System_Monitor_t CanCm3_Monitor;//����3����֡��
extern System_Monitor_t CanCm4_Monitor;//����4����֡��

extern System_Monitor_t IMURec_Monitor;
extern System_Monitor_t ChassisGyro_Monitor;

extern System_Monitor_t DbusRec_Monitor;
#endif
