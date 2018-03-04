#include "control.h"
#include "stm32f4xx.h"
#include "usart.h"
#include "gpio.h"
#include "pid.h"
#include "dbus.h"
#include "can.h"
#include "tim.h"
#include "gpio.h"
#include "main.h"
#include "stdio.h"



chassis_t Chassis={{&CanCm1_Monitor,NULL,&CM1SpeedPID,&CM1Encoder},{&CanCm2_Monitor,NULL,&CM2SpeedPID,&CM2Encoder},{&CanCm3_Monitor,NULL,&CM3SpeedPID,&CM3Encoder},{&CanCm4_Monitor,NULL,&CM4SpeedPID,&CM1Encoder},{0.0f,0.0f,0.0f}};
mecArm_t MecArm={{&CanCm1_Monitor,&CM1ArmPositionPID,&CM1SpeedPID,&CM1ArmEncoder},{&CanCm2_Monitor,&CM1ArmPositionPID,&CM2SpeedPID,&CM2ArmEncoder}};
Rec_t Rec={&DbusRec_Monitor,0.0f,0.0f,0.0f,&IMURec_Monitor,0};



void CM1Arm_ControlLoop(void)
{

}

void CM2Arm_ControlLoop(void)
{
	
}
void Arm_ControlLoop(void)
{
	CM1Arm_ControlLoop();
	CM2Arm_ControlLoop();
	Arm_senddata(ArmCan,CM2ArmSpeedPID.output[0],CM1ArmSpeedPID.output[0]);
}	


void smoothFilter(float *i,float *o,float rate)
{
	*o=(*o)*(1-rate)+(*i)*rate;
}

void systemStatus(void)
{

}


void controlLoop(void)
{
	cmd.heart++;
	if(cmd.heart%1000==0)
	{
		Monitor_Update();
	}
	if(cmd.heart==10000)
	{
		cmd.heart=0;
	}
}


void MecanumCalculate(float Vx, float Vy, float Omega, int16_t *Speed)
{

}

void CMControlLoop(void)
{

}



      