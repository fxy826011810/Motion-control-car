#ifndef __COMMON_H__
#define __COMMON_H__

#include "stm32f4xx.h"

typedef union
{
    uint8_t U[4];
    float F;
    int I;
}FormatTrans;
extern FormatTrans FT;

typedef enum
{
	ready=1,
	wait=1,
}dataStatus_t;//数据状态

typedef struct
{
	float *in;			//输入
	float temp;			//临时数值
	float lastTemp;	//上次临时数值
	float out;			//输出值
	int count;			//圈数
	const int limit;//最小跳变值
}LineCalculate_t;
#define YawLineCalculate LineCalculate
float LineCalculate(LineCalculate_t *yaw);

#endif

