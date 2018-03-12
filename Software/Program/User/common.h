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
}dataStatus_t;//����״̬

typedef struct
{
	float *in;			//����
	float temp;			//��ʱ��ֵ
	float lastTemp;	//�ϴ���ʱ��ֵ
	float out;			//���ֵ
	int count;			//Ȧ��
	const int limit;//��С����ֵ
}LineCalculate_t;
#define YawLineCalculate LineCalculate
float LineCalculate(LineCalculate_t *yaw);

#endif

