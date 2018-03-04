#ifndef __CONTROL_H
#define __CONTROL_H
#include "stm32f4xx.h"
#include "monitor.h"
#include "pid.h"
#include "can.h"
#include "common.h"
typedef struct
{
	System_Monitor_t *mon;				//������
	PID_TypeDef *positionPid;			//pid
	PID_TypeDef *speedPid;				//pid
	volatile ArmEncoder *encoder;	//������
}moter_t;

typedef struct
{
	System_Monitor_t *mon;//������
	PID_TypeDef *pid;				//pid
	YawCalculate_t yawCalc;
	float chassisAngle[3];
}gyro_t;

typedef struct
{
	moter_t moter1;
	moter_t moter2;
	moter_t moter3;
	moter_t moter4;
	gyro_t	gyro;
}chassis_t;

typedef struct
{
	moter_t forearm;//ǰ��
	moter_t mainArm;//���
}mecArm_t;

void CMControlLoop(void);
//void MecanumCalculate(float Vx, float Vy, float Omega, int16_t *Speed);
void controlLoop(void);
#endif
