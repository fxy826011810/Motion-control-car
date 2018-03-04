#ifndef __USART_H
#define __USART_H
#include "monitor.h" 
#include "common.h" 
typedef struct
{
	System_Monitor_t *mon;//¼àÊÓÆ÷
	YawCalculate_t yawCalc;
	float recAngle[3];
	
}motion_t;

void Bsp_Usart_Init(void);
#endif

