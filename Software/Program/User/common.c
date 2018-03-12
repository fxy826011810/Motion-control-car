#include "common.h"


FormatTrans FT;//用于float类型转化为char[4]类型

//用于线性化非线性编码器的值（有调变）
float LineCalculate(LineCalculate_t *yaw)
{
	yaw->lastTemp = yaw->temp;
	yaw->temp = *yaw->in;
	if (yaw->temp - yaw->lastTemp >= yaw->limit)  //yaw轴角度经过处理后变成连续的
		yaw->count--;
	else if (yaw->temp - yaw->lastTemp <= -yaw->limit)
		yaw->count++;
	yaw->out = yaw->temp + yaw->count * 360;  //yaw轴角度
	return yaw->out;
}
