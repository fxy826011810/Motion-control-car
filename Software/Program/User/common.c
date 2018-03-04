#include "common.h"


FormatTrans FT;//用于float类型转化为char[4]类型

float YawLineCalculate(YawCalculate_t *yaw)
{
	yaw->lastTemp = yaw->temp;
	yaw->temp = *yaw->in;
	if (yaw->temp - yaw->lastTemp >= 330)  //yaw轴角度经过处理后变成连续的
		yaw->count--;
	else if (yaw->temp - yaw->lastTemp <= -330)
		yaw->count++;
	yaw->out = yaw->temp + yaw->count * 360;  //yaw轴角度
	return yaw->out;
}
