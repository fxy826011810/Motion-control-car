#include "cailback.h"
#include "main.h"
	
	
	
void ForeArm_offlineCailback(void)//前臂电机丢失回调
{
	
}

void MainArm_offlineCailback(void)//大臂电机丢失回调
{
	
}

void CanCm_offlineCailback(void)//底盘电机丢失回调
{
	
}

void IMURec_offlineCailback(void)//陀螺仪接收模块丢失回调
{
	cmd.chassis->gyro.remoteRotateAngle+=cmd.chassis->gyro.motionRotateAngle;
	cmd.chassis->gyro.motionRotateAngle=0;
}

void DbusRec_offlineCailback(void)//遥控丢失回调
{
	dbus_reset(&cmd.rec->remote);
}

void ChassisGyro_offlineCailback(void)//底盘陀螺仪丢失回调
{
	cmd.operateStatus=remote;
}