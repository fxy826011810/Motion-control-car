#include "stm32f4xx.h"
#include "delay.h"
#include "usart.h"
#include "dma.h"
#include <stdio.h>
//int fputc(int ch,FILE*f)//printf�����ض���
//{
//	USART_SendData(USART3,(unsigned char)ch);
//	while(!USART_GetFlagStatus(USART3,USART_FLAG_TC));
//	return ch;
//}

void Bsp_Usart_Init(void)
{
	
  USART_InitTypeDef								usart;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	//usart1(ң��)
	USART_DeInit(USART1);
	USART_StructInit(&usart);
	usart.USART_BaudRate							= 115200;
	usart.USART_HardwareFlowControl		= USART_HardwareFlowControl_None;
	usart.USART_Mode									= USART_Mode_Rx;
	usart.USART_Parity								= USART_Parity_Even;
	usart.USART_StopBits							= USART_StopBits_1;
	usart.USART_WordLength						= USART_WordLength_8b;
	USART_Init(USART1, &usart);
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
  USART_ITConfig(USART1,USART_IT_IDLE,ENABLE);
	USART_Cmd(USART1, ENABLE);

	USART_ClearFlag(USART1,USART_FLAG_IDLE);//��ֹ���ܲ�����һ���ַ�


}



void usart3_dma_upgrade(uint8_t* data,uint8_t len)
{
		if(DMA1_Stream3->NDTR)
    {
        return;
    }
    DMA_Cmd(DMA1_Stream3, DISABLE);                                     //�ر� DMA ����
    while (DMA_GetCmdStatus(DMA1_Stream3) != DISABLE){}                 //ȷ�� DMA ���Ա�����
    DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3 | DMA_FLAG_HTIF3);       //��ձ�־λ
    DMA1_Stream3->M0AR = (uint32_t)data;              //��������
    DMA_SetCurrDataCounter(DMA1_Stream3, len);                          //���ݴ�����
    DMA_Cmd(DMA1_Stream3, ENABLE);
}

void usart1_dma_upgrade(uint8_t* data,uint8_t len)
{
		if(DMA2_Stream7->NDTR)
    {
        return;
    }
    DMA_Cmd(DMA2_Stream7, DISABLE);                                     //�ر� DMA ����
    while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE){}                 //ȷ�� DMA ���Ա�����
    DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7 | DMA_FLAG_HTIF7);       //��ձ�־λ
    DMA2_Stream7->M0AR = (uint32_t)data;              //��������
    DMA_SetCurrDataCounter(DMA2_Stream7, len);                          //���ݴ�����
    DMA_Cmd(DMA2_Stream7, ENABLE);
}

void Usart_DMASend(USART_TypeDef* USARTx,uint8_t *send,uint8_t len)//��������
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



void USART1_IRQHandler(void)//����ң����ֵ
{

    if (USART_GetITStatus(USART1, USART_IT_IDLE) == 1)
      {
        USART_ClearITPendingBit(USART1, USART_IT_IDLE);
				USART_ReceiveData(USART1);      
      }
}


void USART3_IRQHandler(void)
{

    if (USART_GetITStatus(USART3, USART_IT_IDLE) == 1)
      {
        USART_ClearITPendingBit(USART3, USART_IT_IDLE);
				USART_ReceiveData(USART3);      
      }
}

	



