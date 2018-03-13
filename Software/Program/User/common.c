#include "common.h"


FormatTrans FT;//����float����ת��Ϊchar[4]����

//�������Ի������Ա�������ֵ���е��䣩
float LineCalculate(LineCalculate_t *yaw)
{
	yaw->lastTemp = yaw->temp;
	yaw->temp = *yaw->in;
	if (yaw->temp - yaw->lastTemp >= yaw->limit)  //yaw��ǶȾ����������������
		yaw->count--;
	else if (yaw->temp - yaw->lastTemp <= -yaw->limit)
		yaw->count++;
	yaw->out = yaw->temp + yaw->count * 360;  //yaw��Ƕ�
	return yaw->out;
}


void abs_int16_t_limit(int16_t *a, int16_t ABS_MAX)
{
	if (*a > ABS_MAX)
		*a = ABS_MAX;
	if (*a < -ABS_MAX)
		*a = -ABS_MAX;
}
void abs_float_limit(float *a, float ABS_MAX)
{
	if (*a > ABS_MAX)
		*a = ABS_MAX;
	if (*a < -ABS_MAX)
		*a = -ABS_MAX;
}