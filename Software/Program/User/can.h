#ifndef __CAN_H
#define __CAN_H
#include "stm32f4xx.h"
#define __CAN_EXT extern
#define RATE_BUF_SIZE 6

typedef enum
{
 CAN_6623_ID					=0x1FF,
 
 CAN_RM3510_ID				=0x200,
 CAN_RM3510_1_ID			=0x201,
 CAN_RM3510_2_ID			=0x202,
 CAN_RM3510_3_ID			=0x203,
 CAN_RM3510_4_ID			=0x204,
 CAN_RM3510_5_ID			=0x205,
 CAN_RM3510_6_ID			=0x206,
 CAN_GYRO_ID					=0x40,
}CAN_ID;

typedef struct{
	int32_t raw_value;   									//编码器不经处理的原始值
	int32_t last_raw_value;								//上一次的编码器原始值
	int32_t ecd_value;                    //经过处理后连续的编码器值
	int32_t diff;													//两次编码器之间的差值
	int32_t ecd_bias;											//初始编码器值	
	int32_t cnt_bias;
	int32_t round_cnt;										//圈数
	int16_t filter_rate;									//速度
	float ecd_angle;											//角度
}ArmEncoder;
#define CMEncoder ArmEncoder
extern ArmEncoder CM1ArmEncoder;
extern ArmEncoder CM2ArmEncoder;

extern CMEncoder CM1Encoder;
extern CMEncoder CM2Encoder;
extern CMEncoder CM3Encoder;
extern CMEncoder CM4Encoder;

void Bsp_Can_Init(void);
void cm_senddata(CAN_TypeDef* CANx, int num1, int num2, int num3, int num4);
void gm_senddata(CAN_TypeDef* CANx, int num1, int num2);
void Can2_RecviveData(CanRxMsg * rec);


#define Arm_senddata gm_senddata
#define Yaw_senddata gm_senddata
#define CM_senddata 	cm_senddata
#define YawCan 	CAN2
#define CMCan 	CAN2
#define ArmCan 	CAN1
#endif

