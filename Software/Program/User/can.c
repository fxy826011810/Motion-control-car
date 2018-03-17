#include "stm32f4xx.h"
#include "can.h"
#include "dbus.h"
#include "pid.h"
#include "main.h"
#include "monitor.h"

ArmEncoder ForeArmEncoder 	= {0,0,0,0,0,0,-4*8192,0,0,0,0,wait};
ArmEncoder MainArmEncoder 	= {0,0,0,0,0,0,4*8192,0,0,0,0,wait};

CMEncoder CM1Encoder 			= {0,0,0,0,0,0,0,0,0,wait};
CMEncoder CM2Encoder 			= {0,0,0,0,0,0,0,0,0,wait};
CMEncoder CM3Encoder 			= {0,0,0,0,0,0,0,0,0,wait};
CMEncoder CM4Encoder 			= {0,0,0,0,0,0,0,0,0,wait};




void Bsp_Can_Init(void)
{
			CAN_InitTypeDef                       can;
			CAN_FilterInitTypeDef                 can_filter;

			RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1 | RCC_APB1Periph_CAN2, ENABLE);

			//can1
			CAN_DeInit(CAN1);
			CAN_StructInit(&can);
			can.CAN_ABOM									= DISABLE;
			can.CAN_AWUM									= DISABLE;
			can.CAN_NART									= DISABLE;
			can.CAN_RFLM									= DISABLE;
			can.CAN_TTCM									= DISABLE;
			can.CAN_TXFP									= ENABLE;
			can.CAN_SJW										= CAN_SJW_1tq;
			can.CAN_BS1										= CAN_BS1_9tq;
			can.CAN_BS2										= CAN_BS2_4tq;
			can.CAN_Prescaler								= 3;
			can.CAN_Mode									= CAN_Mode_Normal;
			CAN_Init(CAN1, &can);
			
			can_filter.CAN_FilterActivation					= ENABLE;
			can_filter.CAN_FilterFIFOAssignment				= 0;
			can_filter.CAN_FilterIdHigh						= 0x0000;
			can_filter.CAN_FilterIdLow						= 0x0000;
			can_filter.CAN_FilterMaskIdHigh					= 0x0000;
			can_filter.CAN_FilterMaskIdLow					= 0x0000;
			can_filter.CAN_FilterMode						= CAN_FilterMode_IdMask;
			can_filter.CAN_FilterNumber						= 0;
			can_filter.CAN_FilterScale						= CAN_FilterScale_32bit;
			can_filter.CAN_FilterNumber						= 0;
			CAN_FilterInit(&can_filter);

			//can2
			CAN_DeInit(CAN2);
			CAN_StructInit(&can);
			can.CAN_ABOM									= DISABLE;
			can.CAN_AWUM									= DISABLE;
			can.CAN_NART									= DISABLE;
			can.CAN_RFLM									= DISABLE;
			can.CAN_TTCM									= DISABLE;
			can.CAN_TXFP									= ENABLE;
			can.CAN_SJW										= CAN_SJW_1tq;
			can.CAN_BS1										= CAN_BS1_9tq;
			can.CAN_BS2										= CAN_BS2_4tq;
			can.CAN_Prescaler								= 3;
			can.CAN_Mode									= CAN_Mode_Normal;
			CAN_Init(CAN2, &can);

			can_filter.CAN_FilterActivation					= ENABLE;
			can_filter.CAN_FilterFIFOAssignment				= 0;
			can_filter.CAN_FilterIdHigh						= 0x0000;
			can_filter.CAN_FilterIdLow						= 0x0000;
			can_filter.CAN_FilterMaskIdHigh					= 0x0000;
			can_filter.CAN_FilterMaskIdLow					= 0x0000;
			can_filter.CAN_FilterMode						= CAN_FilterMode_IdMask;
			can_filter.CAN_FilterNumber						= 0;
			can_filter.CAN_FilterScale						= CAN_FilterScale_32bit;
			can_filter.CAN_FilterNumber						= 14;
			CAN_FilterInit(&can_filter);

			CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
			CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);
}


void cm_senddata(CAN_TypeDef* CANx, int num1, int num2, int num3, int num4)//底盘can发送
{
		CanTxMsg  sendmessage;
		sendmessage.DLC									= 0x08;
		sendmessage.IDE									= CAN_ID_STD;
		sendmessage.RTR									= CAN_RTR_DATA;
		sendmessage.StdId								= 0x200;
		sendmessage.Data[0]								= ((num1) >> 8);
		sendmessage.Data[1]								= (num1);
		sendmessage.Data[2]								= ((num2) >> 8);
		sendmessage.Data[3]								= (num2);
		sendmessage.Data[4]								= ((num3) >> 8);
		sendmessage.Data[5]								= (num3);
		sendmessage.Data[6]								= ((num4) >> 8);
		sendmessage.Data[7]								= (num4);

		CAN_Transmit(CANx, &sendmessage);
}
void gm_senddata(CAN_TypeDef* CANx, int num1, int num2)//云台can发送
 {
		 CanTxMsg  sendmessage;
		 sendmessage.DLC = 0x08;
		 sendmessage.IDE = CAN_ID_STD;
		 sendmessage.RTR = CAN_RTR_DATA;
		 sendmessage.StdId = 0x1FF;
		 sendmessage.Data[0] = ((num1) >> 8);
		 sendmessage.Data[1] = (num1);
		 sendmessage.Data[2] = ((num2) >> 8);
		 sendmessage.Data[3] = (num2);
		 sendmessage.Data[4] = 0;
		 sendmessage.Data[5] = 0;
		 sendmessage.Data[6] = 0;
		 sendmessage.Data[7] = 0;

		 CAN_Transmit(CANx, &sendmessage);
 }
 
static void EncoderProcess(Encoder *v, CanRxMsg * msg)//云台值解算函数
{ 
		v->last_raw_value = v->raw_value;
		v->raw_value = (msg->Data[0]<<8)|msg->Data[1];
		v->diff = v->raw_value - v->last_raw_value;
		if(v->diff < -7000)    //两次编码器的反馈值差别太大，表示圈数发生了改变
		{
			v->round_cnt++;
		}
		else if(v->diff>7000)
		{
			v->round_cnt--;
		}
		//计算得到连续的编码器输出值
		v->ecd_value = v->raw_value-v->zero_bias-(int32_t)v->ecd_bias + v->round_cnt * 8192;
		//计算得到角度值，范围正负无穷大
		v->ecd_angle =v->ecd_value*360/8192;
		v->filter_rate =(msg->Data[2]<<8)|msg->Data[3];
		v->count++;
		v->dataStatus=ready;
}
void GetEncoderBias(Encoder *v, CanRxMsg * msg)
{
	v->zero_bias = (msg->Data[0]<<8)|msg->Data[1];  //保存初始编码器值作为偏差  
	v->ecd_value = v->zero_bias;
	v->last_raw_value = v->zero_bias;
	v->count++;
}
static void canGyroCalculate(chassis_t *chassis,CanRxMsg * rec)
{
	int16_t angleint[3];
	float tempYaw;
	angleint[2]=(rec->Data[0]<<8|rec->Data[1]);
	angleint[1]=(rec->Data[2]<<8|rec->Data[3]);
	angleint[0]=(rec->Data[4]<<8|rec->Data[5]);
	
	if(chassis->gyro.count<50)
	{
		chassis->gyro.chassisBiasAngle[2]=(float)angleint[2]/100;
		chassis->gyro.chassisBiasAngle[1]=(float)angleint[1]/100;
		tempYaw=(float)angleint[0]/100;
		chassis->gyro.yawCalc.in=&tempYaw;
		chassis->gyro.chassisBiasAngle[0]=YawLineCalculate(&chassis->gyro.yawCalc);
		chassis->gyro.dataStatus=wait;
	}
	else
	{
		chassis->gyro.chassisAngle[2]=(float)angleint[2]/100;
		chassis->gyro.chassisAngle[1]=(float)angleint[1]/100;
		tempYaw=(float)angleint[0]/100;
		chassis->gyro.yawCalc.in=&tempYaw;
		chassis->gyro.chassisAngle[0]=YawLineCalculate(&chassis->gyro.yawCalc)-chassis->gyro.chassisBiasAngle[0];
		chassis->gyro.dataStatus=ready;
	}
	chassis->gyro.count++;
}
static void moterDataProcess(moter_t *moter,CanRxMsg * rec)
{
	if(moter->encoder->count<50)
	{
		GetEncoderBias(moter->encoder,rec);
	}
	else
	{
		EncoderProcess(moter->encoder,rec);
		Monitor_Set(moter->mon);
	}
	
}
static void Can2_RecviveData(chassis_t *chassis,CanRxMsg * rec)//返回值解算
{
	
			switch (rec->StdId)
			{
				case CAN_RM3510_1_ID:
				{
					moterDataProcess(&chassis->moter1,rec);
				}
					break;
				case CAN_RM3510_2_ID:
				{
					moterDataProcess(&chassis->moter2,rec);
				}
					break;
				case CAN_RM3510_3_ID:
				{
					moterDataProcess(&chassis->moter3,rec);
				}
					break;
				case CAN_RM3510_4_ID:
				{
					moterDataProcess(&chassis->moter4,rec);
				}
					break;
//------------------------------------------------------------//				
				case CAN_GYRO_ID:
				{
					if(rec->Data[6]==0xAA&&rec->Data[7]==0xBB)
					{
						canGyroCalculate(chassis,rec);
						Monitor_Set(chassis->gyro.mon);
					}
				}
					break;				
			}
}
static void Can1_RecviveData(chassis_t *chassis,CanRxMsg * rec)//返回值解算
{
	
			switch (rec->StdId)
			{
//------------------------------------------------------------//
				case CAN_RM3510_6_ID:
				{
					moterDataProcess(&MecArm.foreArm,rec);
				}
					break;				
				case CAN_RM3510_5_ID:
				{
					moterDataProcess(&MecArm.mainArm,rec);
				}
					break;
//------------------------------------------------------------//
				case CAN_GYRO_ID:
				{
					if(rec->Data[6]==0xAA&&rec->Data[7]==0xBB)
					{
						canGyroCalculate(chassis,rec);
						Monitor_Set(chassis->gyro.mon);
					}
				}
					break;
			}
}


void CAN2_RX0_IRQHandler(void)//底盘云台值解算中断
{
	CanRxMsg		receivemessage;
	if(CAN_GetITStatus(CAN2,CAN_IT_FMP0)!=0)
	{
		CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
		CAN_Receive(CAN2, CAN_FIFO0, &receivemessage);
		Can2_RecviveData(cmd.chassis,&receivemessage);
	}
}
void CAN1_RX0_IRQHandler(void)//机械臂值解算中断
{
	CanRxMsg		receivemessage;
	if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != 0)
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
		CAN_Receive(CAN1, CAN_FIFO0, &receivemessage);
		Can1_RecviveData(cmd.chassis,&receivemessage);
	}
}





