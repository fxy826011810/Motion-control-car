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
	System_Monitor_t *mon;				//监视器
	PID_TypeDef *positionPid;			//pid
	PID_TypeDef *speedPid;				//pid
	ArmEncoder *encoder;					//编码器
}moter_t;

typedef struct
{
	System_Monitor_t *mon;				//监视器
	PID_TypeDef *pid;							//pid
	LineCalculate_t yawCalc;			//线性角度
	float chassisAngle[3];				//底盘角度
	float chassisBiasAngle[3];		//底盘初始角度
	uint32_t count;								//数据帧数
	float setAngle;								//跟随设定角度
	float remoteRotateAngle;			//跟随遥控角度
	float motionRotateAngle;			//跟随姿态角度
	dataStatus_t dataStatus;			//数据状态
}gyro_t;

typedef struct
{
	int16_t Vx;										//横向速度
	int16_t Vy;										//纵向速度
	int16_t Omega;								//旋转速度
	int16_t moterSpeed[4];				//对应地盘电机速度
}speed_t;

typedef struct
{
	
	moter_t moter1;							//电机1
	moter_t moter2;							//电机2
	moter_t moter3;							//电机3
	moter_t moter4;							//电机4
	gyro_t	gyro;								//陀螺仪
	speed_t speed;							//速度
}chassis_t;

typedef struct
{
	moter_t foreArm;						//前臂
	moter_t mainArm;						//大臂
	float forearmSetAngle;			//前臂设置角度
	float mainArmSetAngle;			//大臂设置角度
}mecArm_t;

void controlLoop(void);
#endif
