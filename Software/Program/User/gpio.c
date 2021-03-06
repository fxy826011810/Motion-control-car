#include "stm32f4xx.h"
#include "gpio.h"


void Bsp_GPIO_Init(void)
{
	GPIO_InitTypeDef						gpio;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOC| RCC_AHB1Periph_GPIOB |RCC_AHB1Periph_GPIOA, ENABLE);

	//led1&led2
	gpio.GPIO_Mode							= GPIO_Mode_OUT;
	gpio.GPIO_OType							= GPIO_OType_PP;
	gpio.GPIO_Pin							= GPIO_Pin_1| GPIO_Pin_2;
	gpio.GPIO_PuPd							= GPIO_PuPd_UP;
	gpio.GPIO_Speed							= GPIO_Speed_100MHz;
	GPIO_Init(GPIOC, &gpio);

	//usart1(DBusң��)
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Pin = GPIO_Pin_7;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	gpio.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &gpio);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);

	//usart3
	gpio.GPIO_Mode = GPIO_Mode_AF;
	gpio.GPIO_OType = GPIO_OType_PP;
	gpio.GPIO_Pin = GPIO_Pin_10| GPIO_Pin_11;
	gpio.GPIO_PuPd = GPIO_PuPd_UP;
	gpio.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &gpio);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
	//can1
	gpio.GPIO_Mode							= GPIO_Mode_AF;
	gpio.GPIO_OType							= GPIO_OType_PP;
	gpio.GPIO_Pin							= GPIO_Pin_11 | GPIO_Pin_12;
	gpio.GPIO_PuPd							= GPIO_PuPd_UP;
	gpio.GPIO_Speed							= GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &gpio);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);

	//can2
	gpio.GPIO_Mode							= GPIO_Mode_AF;
	gpio.GPIO_OType							= GPIO_OType_PP;
	gpio.GPIO_Pin							= GPIO_Pin_12 | GPIO_Pin_13;
	gpio.GPIO_PuPd							= GPIO_PuPd_UP;
	gpio.GPIO_Speed							= GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &gpio);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2);
}





