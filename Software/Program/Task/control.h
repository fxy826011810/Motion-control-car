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
	ArmEncoder *encoder;					//������
}moter_t;

typedef struct
{
	System_Monitor_t *mon;				//������
	PID_TypeDef *pid;							//pid
	LineCalculate_t yawCalc;			//���ԽǶ�
	float chassisAngle[3];				//���̽Ƕ�
	float chassisBiasAngle[3];		//���̳�ʼ�Ƕ�
	uint32_t count;								//����֡��
	float setAngle;								//�����趨�Ƕ�
	float remoteRotateAngle;			//����ң�ؽǶ�
	float motionRotateAngle;			//������̬�Ƕ�
	dataStatus_t dataStatus;			//����״̬
}gyro_t;

typedef struct
{
	int16_t Vx;										//�����ٶ�
	int16_t Vy;										//�����ٶ�
	int16_t Omega;								//��ת�ٶ�
	int16_t moterSpeed[4];				//��Ӧ���̵���ٶ�
}speed_t;

typedef struct
{
	
	moter_t moter1;							//���1
	moter_t moter2;							//���2
	moter_t moter3;							//���3
	moter_t moter4;							//���4
	gyro_t	gyro;								//������
	speed_t speed;							//�ٶ�
}chassis_t;

typedef struct
{
	moter_t foreArm;						//ǰ��
	moter_t mainArm;						//���
	float forearmSetAngle;			//ǰ�����ýǶ�
	float mainArmSetAngle;			//������ýǶ�
}mecArm_t;

void controlLoop(void);
#endif
