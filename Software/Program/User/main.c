#include "stm32f4xx.h"
#include "usart.h"
#include "delay.h"
#include "gpio.h"
#include "nvic.h"
#include "tim.h"
#include "dma.h"
#include "main.h"


cmd_t cmd={0,&MecArm,&Chassis,&Rec,&Debug};

void system_init(void)
{
	Bsp_Pid_Init();
  Bsp_NVIC_Init();
	Bsp_GPIO_Init();
	Bsp_Can_Init();
	Bsp_Tim_Init();
	Bsp_DMA_Init();
	Bsp_Usart_Init();
	SystemIRQ_Enable();
}


int main(void)
{
  system_init();//系统初始化
	while (1)
	{	

  }
  
}
