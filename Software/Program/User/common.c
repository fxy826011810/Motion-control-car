#include "common.h"


FormatTrans FT;//����float����ת��Ϊchar[4]����

float YawLineCalculate(YawCalculate_t *yaw)
{
	yaw->lastTemp = yaw->temp;
	yaw->temp = *yaw->in;
	if (yaw->temp - yaw->lastTemp >= 330)  //yaw��ǶȾ����������������
		yaw->count--;
	else if (yaw->temp - yaw->lastTemp <= -330)
		yaw->count++;
	yaw->out = yaw->temp + yaw->count * 360;  //yaw��Ƕ�
	return yaw->out;
}
