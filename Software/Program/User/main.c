#include "stm32f4xx.h"
#include "usart.h"
#include "delay.h"
#include "gpio.h"
#include "nvic.h"
#include "tim.h"
#include "dma.h"

void system_init(void)
{
  Bsp_NVIC_Init();
	Bsp_GPIO_Init();
	Bsp_Tim_Init();
	Bsp_DMA_Init();
	Bsp_Usart_Init();

}


int main(void)
{
  system_init();//系统初始化
	while (1)
	{	

  }
  
}
