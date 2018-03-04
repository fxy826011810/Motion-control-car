#include "stm32f4xx.h"
#include "monitor.h" 

System_Monitor_t CanCm1Arm_Monitor;//��۽���֡��
System_Monitor_t CanCm2Arm_Monitor;//С�۽���֡��

System_Monitor_t CanCm1_Monitor;//����1����֡��
System_Monitor_t CanCm2_Monitor;//����2����֡��
System_Monitor_t CanCm3_Monitor;//����3����֡��
System_Monitor_t CanCm4_Monitor;//����4����֡��
System_Monitor_t IMURec_Monitor;
System_Monitor_t ChassisGyro_Monitor;
System_Monitor_t DbusRec_Monitor;
//����������
void Monitor_Reset(System_Monitor_t *mon)
{
	mon->count=0;
	mon->time=0;
}
//��������ʼ��
void Monitor_Init(void)
{
	Monitor_Reset(&CanCm1Arm_Monitor);
	Monitor_Reset(&CanCm2Arm_Monitor);
	Monitor_Reset(&CanCm1_Monitor);
	Monitor_Reset(&CanCm2_Monitor);
	Monitor_Reset(&CanCm3_Monitor);
	Monitor_Reset(&CanCm4_Monitor);
	Monitor_Reset(&IMURec_Monitor);
	Monitor_Reset(&DbusRec_Monitor);
	Monitor_Reset(&ChassisGyro_Monitor);
}


//����������
void Monitor_Calc(System_Monitor_t *mon)
{
	mon->count=mon->time;
	mon->time=0;
}
//���ü�����
void Monitor_Set(System_Monitor_t *mon)
{
	mon->time++;
}
//����������
void Monitor_Update(void)
{
	Monitor_Calc(&CanCm1Arm_Monitor);
	Monitor_Calc(&CanCm2Arm_Monitor);
	Monitor_Calc(&CanCm1_Monitor);
	Monitor_Calc(&CanCm2_Monitor);
	Monitor_Calc(&CanCm3_Monitor);
	Monitor_Calc(&CanCm4_Monitor);
	Monitor_Calc(&IMURec_Monitor);
	Monitor_Calc(&DbusRec_Monitor);
	Monitor_Calc(&ChassisGyro_Monitor);
}



