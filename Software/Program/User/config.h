#ifndef __CONFIG_H__
#define __CONFIG_H__

#include "stm32f4xx.h" 
#define init_time_ms 6000//��ʼ��б��
			//��̨��ʼ���Ƕ�
			#define CM1ArmEncoder_Default	0x1BCF
			#define CM2ArmEncoder_Default		0x1559
			#define GMYawAngle_Default 0xC26
			#define MaxWheelSpeed 5610
			#define Cm1_ratio 27.0f 
			#define Cm2_ratio 19.0f
			#define Speedrate 5.0f//0-100����ΪС��

#endif
