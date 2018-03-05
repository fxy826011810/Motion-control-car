#include "stm32f4xx.h"
#include "delay.h"
#include "usart.h"
#include "dma.h"
#include <stdio.h>
#include "main.h"
#include "common.h"
void Bsp_Usart_Init(void)
{
	
  USART_InitTypeDef								usart;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	//usart1(遥控)
	USART_DeInit(USART1);
	USART_StructInit(&usart);
	usart.USART_BaudRate							= 100000;
	usart.USART_HardwareFlowControl		= USART_HardwareFlowControl_None;
	usart.USART_Mode									= USART_Mode_Rx;
	usart.USART_Parity								= USART_Parity_Even;
	usart.USART_StopBits							= USART_StopBits_1;
	usart.USART_WordLength						= USART_WordLength_8b;
	USART_Init(USART1, &usart);
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
  USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);
	USART_Cmd(USART1, ENABLE);

	//usart3
	USART_DeInit(USART3);
	USART_StructInit(&usart);
	usart.USART_BaudRate							= 115200;
	usart.USART_HardwareFlowControl		= USART_HardwareFlowControl_None;
	usart.USART_Mode									= USART_Mode_Rx|USART_Mode_Tx;
	usart.USART_Parity								= USART_Parity_No;
	usart.USART_StopBits							= USART_StopBits_1;
	usart.USART_WordLength						= USART_WordLength_8b;
	USART_Init(USART3, &usart);
	USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
	USART_Cmd(USART3, ENABLE);

	USART_ClearFlag(USART1,USART_FLAG_IDLE);//防止接受不到第一个字符
	USART_ClearFlag(USART3,USART_FLAG_RXNE);//防止接受不到第一个字符


}



void usart3_dma_upgrade(uint8_t* data,uint8_t len)
{
		if(DMA1_Stream3->NDTR)
    {
        return;
    }
    DMA_Cmd(DMA1_Stream3, DISABLE);                                     //关闭 DMA 传输
    while (DMA_GetCmdStatus(DMA1_Stream3) != DISABLE){}                 //确保 DMA 可以被设置
    DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3 | DMA_FLAG_HTIF3);       //清空标志位
    DMA1_Stream3->M0AR = (uint32_t)data;              //设置数据
    DMA_SetCurrDataCounter(DMA1_Stream3, len);                          //数据传输量
    DMA_Cmd(DMA1_Stream3, ENABLE);
}

void usart1_dma_upgrade(uint8_t* data,uint8_t len)
{
		if(DMA2_Stream7->NDTR)
    {
        return;
    }
    DMA_Cmd(DMA2_Stream7, DISABLE);                                     //关闭 DMA 传输
    while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE){}                 //确保 DMA 可以被设置
    DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7 | DMA_FLAG_HTIF7);       //清空标志位
    DMA2_Stream7->M0AR = (uint32_t)data;              //设置数据
    DMA_SetCurrDataCounter(DMA2_Stream7, len);                          //数据传输量
    DMA_Cmd(DMA2_Stream7, ENABLE);
}

void Usart_DMASend(USART_TypeDef* USARTx,uint8_t *send,uint8_t len)//各种数据
{
	if(USARTx==USART3)
	{
		usart3_dma_upgrade(send, len);
	}
	else if(USARTx==USART1)
	{
		usart1_dma_upgrade(send, len);
	}
}



void USART1_IRQHandler(void)//接收遥控器值
{
		static uint8_t Len;
    if (USART_GetITStatus(USART1, USART_IT_IDLE) == 1)
    {
			USART_ClearITPendingBit(USART1, USART_IT_IDLE);
			USART_ReceiveData(USART1);
			DMA_Cmd(DMA2_Stream2, DISABLE);
			Len=30-DMA2_Stream2->NDTR;
			if(Len==18)
			{
				__disable_irq();
				dbus_getdata(&Rec.remote.dbusRec);
				__enable_irq();
				Monitor_Set(Rec.remote.mon);
				
      }
       //重启DMA
       DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
       while(DMA_GetCmdStatus(DMA2_Stream2) != DISABLE);
       DMA_SetCurrDataCounter(DMA2_Stream2, 30);
       DMA_Cmd(DMA2_Stream2, ENABLE);        
     }
}

typedef struct
{
	const uint8_t head;
	const uint8_t end;
	uint8_t lock;
	uint8_t tmpLen;
	uint8_t len;
	uint8_t buf[30];
}FrameRec_t;
FrameRec_t AngleRec={0xAA,0xBB,0,0.0};


void angleRecCalculate(uint16_t temp)
{
		__disable_irq();
		if(temp==AngleRec.head)
		{
			AngleRec.lock=1;
		}
		if(AngleRec.lock==1)
		{
			AngleRec.buf[AngleRec.tmpLen++]=temp;
		}
		if(temp==AngleRec.end)
		{
			AngleRec.lock=0;
			AngleRec.len=AngleRec.tmpLen;
			AngleRec.tmpLen=0;
			if(AngleRec.buf[0]==AngleRec.head&&AngleRec.buf[7]==AngleRec.end)
			{
				int16_t angleint[3];
				float tempYaw;
				angleint[2]=(AngleRec.buf[1]<<8|AngleRec.buf[2]);
				angleint[1]=(AngleRec.buf[3]<<8|AngleRec.buf[4]);
				angleint[0]=(AngleRec.buf[5]<<8|AngleRec.buf[6]);
							
				Rec.motion.recAngle[2]=(float)angleint[2]/100;
				Rec.motion.recAngle[1]=(float)angleint[1]/100;
				tempYaw=(float)angleint[0]/100;
				Rec.motion.yawCalc.in=&tempYaw;
				Rec.motion.recAngle[0]=YawLineCalculate(&Rec.motion.yawCalc);
				Monitor_Set(Rec.motion.mon);
			}
		}
		__enable_irq();
}	


void USART3_IRQHandler(void)
{

    if (USART_GetITStatus(USART3, USART_IT_RXNE) == 1)
      {
				uint16_t temp;
        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
				temp=USART_ReceiveData(USART3);
				angleRecCalculate(temp);
      }
}





