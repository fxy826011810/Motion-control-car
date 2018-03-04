#include "stm32f4xx.h"
#include "monitor.h" 

System_Monitor_t CanCm1Arm_Monitor;//大臂接收帧率
System_Monitor_t CanCm2Arm_Monitor;//小臂接收帧率

System_Monitor_t CanCm1_Monitor;//底盘1接收帧率
System_Monitor_t CanCm2_Monitor;//底盘2接收帧率
System_Monitor_t CanCm3_Monitor;//底盘3接收帧率
System_Monitor_t CanCm4_Monitor;//底盘4接收帧率
System_Monitor_t IMURec_Monitor;
System_Monitor_t ChassisGyro_Monitor;
System_Monitor_t DbusRec_Monitor;
//监视器重启
void Monitor_Reset(System_Monitor_t *mon)
{
	mon->count=0;
	mon->time=0;
}
//监视器初始化
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


//监视器计算
void Monitor_Calc(System_Monitor_t *mon)
{
	mon->count=mon->time;
	mon->time=0;
}
//设置监视器
void Monitor_Set(System_Monitor_t *mon)
{
	mon->time++;
}
//监视器更新
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



