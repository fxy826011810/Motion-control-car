#include "cailback.h"
#include "main.h"
	
	
	
void ForeArm_offlineCailback(void)//ǰ�۵����ʧ�ص�
{
	
}

void MainArm_offlineCailback(void)//��۵����ʧ�ص�
{
	
}

void CanCm_offlineCailback(void)//���̵����ʧ�ص�
{
	
}

void IMURec_offlineCailback(void)//�����ǽ���ģ�鶪ʧ�ص�
{
	cmd.chassis->gyro.remoteRotateAngle+=cmd.chassis->gyro.motionRotateAngle;
	cmd.chassis->gyro.motionRotateAngle=0;
}

void DbusRec_offlineCailback(void)//ң�ض�ʧ�ص�
{
	dbus_reset(&cmd.rec->remote);
}

void ChassisGyro_offlineCailback(void)//���������Ƕ�ʧ�ص�
{
	cmd.operateStatus=remote;
}