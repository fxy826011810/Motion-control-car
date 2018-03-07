#include "stm32f4xx.h"
#include "monitor.h" 
#include "stdio.h" 
#include "cailback.h"
System_Monitor_t ForeArm_Monitor;//大臂接收帧率
System_Monitor_t MainArm_Monitor;//小臂接收帧率

System_Monitor_t CanCm1_Monitor;//底盘1接收帧率
System_Monitor_t CanCm2_Monitor;//底盘2接收帧率
System_Monitor_t CanCm3_Monitor;//底盘3接收帧率
System_Monitor_t CanCm4_Monitor;//底盘4接收帧率
System_Monitor_t IMURec_Monitor;
System_Monitor_t ChassisGyro_Monitor;
System_Monitor_t DbusRec_Monitor;

void Monitor_Cailback(void)
{
	
}
//监视器计算
void Monitor_Calc(System_Monitor_t *mon)
{
	mon->count=mon->time;
	mon->time=0;
	if(mon->count==0)
	{
		mon->status=offline;
		mon->cailback();
	}
	else
	{
		mon->status=online;
	}
}

//监视器重启
void Monitor_Reset(System_Monitor_t *mon,void (*cailback)(void))
{
	mon->count=0;
	mon->time=0;
	mon->status=offline;
	mon->cailback=cailback;
}
//监视器初始化
void Monitor_Init(void)
{
	Monitor_Reset(&ForeArm_Monitor,&ForeArm_offlineCailback);
	Monitor_Reset(&MainArm_Monitor,&MainArm_offlineCailback);
	Monitor_Reset(&CanCm1_Monitor, &CanCm_offlineCailback);
	Monitor_Reset(&CanCm2_Monitor, &CanCm_offlineCailback);
	Monitor_Reset(&CanCm3_Monitor, &CanCm_offlineCailback);
	Monitor_Reset(&CanCm4_Monitor, &CanCm_offlineCailback);
	Monitor_Reset(&IMURec_Monitor, &IMURec_offlineCailback);
	Monitor_Reset(&DbusRec_Monitor,&DbusRec_offlineCailback);
	Monitor_Reset(&ChassisGyro_Monitor,&ChassisGyro_offlineCailback);
}

//设置监视器
void Monitor_Set(System_Monitor_t *mon)
{
	mon->time++;
}

//监视器更新
void Monitor_Update(void)
{
	Monitor_Calc(&ForeArm_Monitor);
	Monitor_Calc(&MainArm_Monitor);
	Monitor_Calc(&CanCm1_Monitor);
	Monitor_Calc(&CanCm2_Monitor);
	Monitor_Calc(&CanCm3_Monitor);
	Monitor_Calc(&CanCm4_Monitor);
	Monitor_Calc(&IMURec_Monitor);
	Monitor_Calc(&DbusRec_Monitor);
	Monitor_Calc(&ChassisGyro_Monitor);
}



