#ifndef __CONTROL_H
#define __CONTROL_H
#include "stm32f4xx.h"
#include "monitor.h"
#include "pid.h"
#include "can.h"
typedef struct
{
	System_Monitor_t *mon;//¼àÊÓÆ÷
	PID_TypeDef *positionPid;			//pid
	PID_TypeDef *speedPid;			//pid
	volatile ArmEncoder *encoder;	//±àÂëÆ÷
}moter_t;
typedef struct
{
	moter_t moter1;
	moter_t moter2;
	moter_t moter3;
	moter_t moter4;
	float chassisAngle[3];
}chassis_t;

typedef struct
{
	moter_t forearm;
	moter_t mainArm;
}mecArm_t;


void GMPitch_ControlLoop(void);
void GMYaw_ControlLoop(void);
void CMControlLoop(void);
void MecanumCalculate(float Vx, float Vy, float Omega, int16_t *Speed);
void shooting_ControlLoop(void);
void controlLoop(void);
#endif
