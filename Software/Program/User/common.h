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
}dataStatus_t;//Êý¾Ý×´Ì¬

typedef struct
{
	float *in,temp,lastTemp,out;
	int count;
}YawCalculate_t;
float YawLineCalculate(YawCalculate_t *yaw);

#endif

