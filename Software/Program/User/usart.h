#ifndef __USART_H
#define __USART_H
#include "monitor.h" 
typedef struct
{
	System_Monitor_t *mon;//¼àÊÓÆ÷
	float recAngle[3];
}motion_t;

void Bsp_Usart_Init(void);
#endif

