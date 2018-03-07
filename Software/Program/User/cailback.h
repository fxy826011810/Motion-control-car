#ifndef __CAILBACK_H__
#define __CAILBACK_H__

#include "stm32f4xx.h"

void ForeArm_offlineCailback(void);

void MainArm_offlineCailback(void);

void CanCm_offlineCailback(void);

void IMURec_offlineCailback(void);

void DbusRec_offlineCailback(void);

void ChassisGyro_offlineCailback(void);

#endif

