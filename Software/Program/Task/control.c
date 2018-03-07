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
{0,&CanCm1_Monitor,NULL,&CM1SpeedPID,&CM1Encoder},\
{1,&CanCm2_Monitor,NULL,&CM2SpeedPID,&CM2Encoder},\
{2,&CanCm3_Monitor,NULL,&CM3SpeedPID,&CM3Encoder},\
{3,&CanCm4_Monitor,NULL,&CM4SpeedPID,&CM4Encoder},\
{&ChassisGyro_Monitor,&CMRotatePID,\
{NULL,0,0,0,0},0.0f,0.0f,0.0f},\
{0,0,0,{0,0,0,0}}\
};
//��е�۸���ֵ
mecArm_t MecArm={\
{6,&ForeArm_Monitor,&CM1ArmPositionPID,&CM1ArmSpeedPID,&CM1ArmEncoder},\
{5,&MainArm_Monitor,&CM2ArmPositionPID,&CM2ArmSpeedPID,&CM2ArmEncoder}};
//���ո���ֵ
Rec_t Rec={\
{&IMURec_Monitor,{NULL,0,0,0,0},0.0f,0.0f,0.0f},\
{&DbusRec_Monitor,0}\
};


void ArmPidControlLoop(moter_t *moter,float SetData)
{
	moter->positionPid->setdata	= SetData;											//λ��PID�趨ֵ
	moter->positionPid->realdata = moter->encoder->ecd_angle;		//λ��PIDʵ��ֵ
	moter->positionPid->test(moter->positionPid);								//λ��PID����
	
	moter->speedPid->setdata = moter->positionPid->output;			//�ٶ�PID�趨ֵ
	moter->speedPid->realdata	= moter->encoder->filter_rate;		//�ٶ�PIDʵ��ֵ
  moter->speedPid->test(moter->speedPid);											//�ٶ�PID����
}
void ForeArm_ControlLoop(void)//ǰ��
{
	ArmPidControlLoop(&MecArm.forearm,0);
}

void MainArm_ControlLoop(void)
{
	ArmPidControlLoop(&MecArm.mainArm,0);
}
void Arm_ControlLoop(void)
{
	ForeArm_ControlLoop();
	MainArm_ControlLoop();
//	Arm_senddata(ArmCan,MecArm.forearm.speedPid->output,MecArm.mainArm.speedPid->output);
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
float Moter_PidControlLoop(moter_t *moter,int16_t *setdata)
{
  moter->speedPid->setdata		= setdata[moter->id];									//�ٶ�PID�趨ֵ
  moter->speedPid->realdata		=moter->encoder->filter_rate;	//�ٶ�PIDʵ��ֵ
  moter->speedPid->test(moter->speedPid);											//�ٶ�PID����
	return moter->speedPid->output;
}
void CMControlLoop(chassis_t *chassis)
{
	
	MecanumCalculate(chassis->speed.Vx,chassis->speed.Vy,chassis->speed.Omega,chassis->speed.moterSpeed);
	
	CM_senddata(CMCan,
	Moter_PidControlLoop(&chassis->moter1,chassis->speed.moterSpeed),
	Moter_PidControlLoop(&chassis->moter2,chassis->speed.moterSpeed),
	Moter_PidControlLoop(&chassis->moter3,chassis->speed.moterSpeed),
	Moter_PidControlLoop(&chassis->moter4,chassis->speed.moterSpeed));
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
	CMControlLoop(cmd.chassis);//����
}






      