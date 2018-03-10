#ifndef __CONTROL_H
#define __CONTROL_H
#include "stm32f4xx.h"
#include "monitor.h"
#include "pid.h"
#include "common.h"
#include "can.h"



typedef struct
{
	uint8_t id;
	System_Monitor_t *mon;				//������
	PID_TypeDef *positionPid;			//pid
	PID_TypeDef *speedPid;				//pid
	ArmEncoder *encoder;	//������
}moter_t;

typedef struct
{
	System_Monitor_t *mon;//������
	PID_TypeDef *pid;				//pid
	YawCalculate_t yawCalc;
	float chassisAngle[3];
	float chassisBiasAngle[3];
	uint32_t count;
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
	moter_t foreArm;//ǰ��
	moter_t mainArm;//���
	float forearmSetAngle;//ǰ�����ýǶ�
	float mainArmSetAngle;//������ýǶ�
}mecArm_t;

void controlLoop(void);
#endif
