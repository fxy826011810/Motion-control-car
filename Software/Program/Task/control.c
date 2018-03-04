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


//���̸���ֵ
chassis_t Chassis={\
{&CanCm1_Monitor,NULL,&CM1SpeedPID,&CM1Encoder},\
{&CanCm2_Monitor,NULL,&CM2SpeedPID,&CM2Encoder},\
{&CanCm3_Monitor,NULL,&CM3SpeedPID,&CM3Encoder},\
{&CanCm4_Monitor,NULL,&CM4SpeedPID,&CM1Encoder},\
{&ChassisGyro_Monitor,&CMRotatePID,\
{NULL,0,0,0,0},0.0f,0.0f,0.0f}\
};
//��е�۸���ֵ
mecArm_t MecArm={\
{&CanCm1_Monitor,&CM1ArmPositionPID,&CM1SpeedPID,&CM1ArmEncoder},\
{&CanCm2_Monitor,&CM1ArmPositionPID,&CM2SpeedPID,&CM2ArmEncoder}};
//���ո���ֵ
Rec_t Rec={\
{&DbusRec_Monitor,{NULL,0,0,0,0},0.0f,0.0f,0.0f},\
{&IMURec_Monitor,0}\
};


void ArmPidControlLoop(moter_t *moter,float SetData)
{
	moter->positionPid->setdata	= SetData;											//λ��PID�趨ֵ
	moter->positionPid->realdata = moter->encoder->ecd_angle;		//λ��PIDʵ��ֵ
	moter->positionPid->test(moter->positionPid);								//λ��PID����
	
	moter->speedPid->setdata = moter->positionPid->output;	//�ٶ�PID�趨ֵ
	moter->speedPid->realdata	= moter->encoder->filter_rate;	//�ٶ�PIDʵ��ֵ
  moter->speedPid->test(moter->speedPid);											//�ٶ�PID����
}
void CM1Arm_ControlLoop(void)
{
	ArmPidControlLoop(&MecArm.forearm,0);
}

void CM2Arm_ControlLoop(void)
{
	ArmPidControlLoop(&MecArm.mainArm,0);
}
void Arm_ControlLoop(void)
{
	CM1Arm_ControlLoop();
	CM2Arm_ControlLoop();
	Arm_senddata(ArmCan,CM2ArmSpeedPID.output,CM1ArmSpeedPID.output);
}	


void smoothFilter(float *i,float *o,float rate)
{
	*o=(*o)*(1-rate)+(*i)*rate;
}

void systemStatus(void)
{

}

void MecanumCalculate(float Vx, float Vy, float Omega, int16_t *Speed)
{
		float Buffer[4], Param, MaxSpeed;
    uint8_t index;
                                         //        |
																				 //	     _____
    Buffer[0] = Vx - Vy + Omega;         //   2||     || 1  ��������ʱ��ת3510
    Buffer[1] = Vx + Vy + Omega;         //     |     |
		Buffer[2] = -Vx + Vy + Omega;        //     |     |
		Buffer[3] = -Vx - Vy + Omega;        //  3 ||     || 4
                                         //      -----

    //�޷�
    for(index = 0, MaxSpeed = 0; index < 4; index++)
    {
      if((Buffer[index] > 0 ? Buffer[index] : -Buffer[index]) > MaxSpeed)
      {
          MaxSpeed = (Buffer[index] > 0 ? Buffer[index] : -Buffer[index]);
      }
    }
    if(MaxWheelSpeed < MaxSpeed)
    {
      Param = (float)MaxWheelSpeed / MaxSpeed;
      Speed[0] = Buffer[0] * Param;
      Speed[1] = Buffer[1] * Param;
      Speed[2] = Buffer[2] * Param;
      Speed[3] = Buffer[3] * Param; 
    }
    else
    {
      Speed[0] = Buffer[0];
      Speed[1] = Buffer[1];
      Speed[2] = Buffer[2];
      Speed[3] = Buffer[3];
    }
}
void Chassis_PidControlLoop(PID_TypeDef *PositionPID , PID_TypeDef *SpeedPID , ArmEncoder *encoder)
{
	PositionPID->setdata	=0;											//λ��PID�趨ֵ
	PositionPID->realdata =encoder->ecd_angle;		//λ��PIDʵ��ֵ
  PositionPID->test(PositionPID);								//λ��PID����
        
  SpeedPID->setdata			= PositionPID->output;	//�ٶ�PID�趨ֵ
  SpeedPID->realdata		=encoder->filter_rate;	//�ٶ�PIDʵ��ֵ
  SpeedPID->test(SpeedPID);											//�ٶ�PID����
}
void CMControlLoop(void)
{
	Chassis.gyro.pid->setdata=0;
	Chassis.gyro.pid->realdata=Chassis.gyro.chassisAngle[0];
	
}

void controlLoop(void)
{
	cmd.heart++;
	if(cmd.heart%1000==0)
	{
		Monitor_Update();//������
	}
	if(cmd.heart==10000)
	{
		cmd.heart=0;
	}
	Arm_ControlLoop();//��е��
	CMControlLoop();//����
}






      