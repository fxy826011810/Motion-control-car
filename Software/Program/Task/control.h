#ifndef __CONTROL_H
#define __CONTROL_H
#include "stm32f4xx.h"
#include "monitor.h"
#include "pid.h"
#include "can.h"
#include "common.h"
typedef enum
{
	ready=1,
	wait=1,
}dataStatus_t;//Êý¾Ý×´Ì¬

typedef struct
{
	uint8_t id;
	System_Monitor_t *mon;				//¼àÊÓÆ÷
	PID_TypeDef *positionPid;			//pid
	PID_TypeDef *speedPid;				//pid
	ArmEncoder *encoder;	//±àÂëÆ÷
	dataStatus_t dataStatus;
}moter_t;

typedef struct
{
	System_Monitor_t *mon;//¼àÊÓÆ÷
	PID_TypeDef *pid;				//pid
	YawCalculate_t yawCalc;
	float chassisAngle[3];
	float setAngle;
	float remoteRotateAngle;
	float motionRotateAngle;
	dataStatus_t dataStatus;
}gyro_t;

typedef struct
{
	int16_t Vx;
	int16_t Vy;
	int16_t Omega;
	int16_t moterSpeed[4];
}speed_t;

typedef struct
{
	moter_t moter1;
	moter_t moter2;
	moter_t moter3;
	moter_t moter4;
	gyro_t	gyro;
	speed_t speed;
}chassis_t;

typedef struct
{
	moter_t forearm;//Ç°±Û
	moter_t mainArm;//´ó±Û
}mecArm_t;

void controlLoop(void);
#endif
