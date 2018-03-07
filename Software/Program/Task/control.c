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


//底盘赋初值
chassis_t Chassis={\
{0,&CanCm1_Monitor,NULL,&CM1SpeedPID,&CM1Encoder,wait},\
{1,&CanCm2_Monitor,NULL,&CM2SpeedPID,&CM2Encoder,wait},\
{2,&CanCm3_Monitor,NULL,&CM3SpeedPID,&CM3Encoder,wait},\
{3,&CanCm4_Monitor,NULL,&CM4SpeedPID,&CM4Encoder,wait},\
{&ChassisGyro_Monitor,&CMRotatePID,{NULL,0,0,0,0},{0.0f,0.0f,0.0f},0.0f,wait},\
{0,0,0,{0,0,0,0}}\
};
//机械臂赋初值
mecArm_t MecArm={\
{6,&ForeArm_Monitor,&CM1ArmPositionPID,&CM1ArmSpeedPID,&CM1ArmEncoder,wait},\
{5,&MainArm_Monitor,&CM2ArmPositionPID,&CM2ArmSpeedPID,&CM2ArmEncoder,wait}};
//接收赋初值
Rec_t Rec={\
{&IMURec_Monitor,{NULL,0,0,0,0},0.0f,0.0f,0.0f},\
{&DbusRec_Monitor,0}\
};


void ArmPidControlLoop(moter_t *moter,float SetData)
{
	moter->positionPid->setdata	= SetData;											//位置PID设定值
	moter->positionPid->realdata = moter->encoder->ecd_angle;		//位置PID实际值
	moter->positionPid->test(moter->positionPid);								//位置PID计算
	
	moter->speedPid->setdata = moter->positionPid->output;			//速度PID设定值
	moter->speedPid->realdata	= moter->encoder->filter_rate;		//速度PID实际值
  moter->speedPid->test(moter->speedPid);											//速度PID计算
}
void ForeArm_ControlLoop(void)//前臂
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

void systemStatusChange(cmd_t *cmd)
{
	if((cmd->chassis->moter1.mon->status==offline)||
		 (cmd->chassis->moter1.mon->status==offline)||
		 (cmd->chassis->moter1.mon->status==offline)||
		 (cmd->chassis->moter1.mon->status==offline)||
		 (cmd->mecArm->forearm.mon->status==offline)||
		 (cmd->mecArm->mainArm.mon->status==offline))
	{
		cmd->systemStatus=stop;
	}
	else{
		cmd->systemStatus=normal;
	}
}
void operateStatusChange(cmd_t *cmd)
{
	if(cmd->systemStatus==normal)
	{
		if((cmd->rec->motion.mon->status!=offline)&&(cmd->rec->remote.mon->status!=offline))
		{
			cmd->operateStatus=allUse;
		}
		else if((cmd->rec->motion.mon->status==offline)&&(cmd->rec->remote.mon->status!=offline))
		{
			cmd->operateStatus=remote;
		}
		else if((cmd->rec->motion.mon->status!=offline)&&(cmd->rec->remote.mon->status==offline))
		{
			cmd->operateStatus=motion;
		}
		else if((cmd->rec->motion.mon->status==offline)&&(cmd->rec->remote.mon->status==offline))
		{
			cmd->operateStatus=noUse;
		}
	}
}

void statusChange(cmd_t *cmd)
{
	systemStatusChange(cmd);
	operateStatusChange(cmd);
}

void operateStatusExecute(cmd_t *cmd)//操作模式执行
{
	if(cmd->operateStatus==remote)
	{
		cmd->chassis->speed.Vy=cmd->rec->remote.dbusRec.rc.ch1;//前后
		cmd->chassis->speed.Vx=cmd->rec->remote.dbusRec.rc.ch0;//左右
//		cmd->chassis->speed.Omega=cmd->rec->remote.dbusRec.rc.ch2;//旋转
		cmd->chassis->gyro.remoteRotateAngle+=cmd->rec->remote.dbusRec.rc.ch2*0.0003f;
	}
	else if(cmd->operateStatus==motion)
	{
//		=cmd->rec->motion.recAngle[0]*10;//yaw
		cmd->chassis->speed.Vy=cmd->rec->motion.recAngle[1]*10;//pitch
		cmd->chassis->speed.Vx=cmd->rec->motion.recAngle[2]*10;//roll
		cmd->chassis->gyro.motionRotateAngle=cmd->rec->motion.recAngle[0];
	}
	else if(cmd->operateStatus==allUse)
	{
		cmd->chassis->speed.Vy=cmd->rec->remote.dbusRec.rc.ch1;//前后
		cmd->chassis->speed.Vx=cmd->rec->remote.dbusRec.rc.ch0;//左右
		cmd->chassis->gyro.motionRotateAngle=cmd->rec->motion.recAngle[0];
	}
	else if(cmd->operateStatus==noUse)
	{
		cmd->chassis->speed.Vy=0;//前后
		cmd->chassis->speed.Vx=0;//左右
		cmd->chassis->speed.Omega=0;//旋转
	}
	cmd->chassis->gyro.setAngle=cmd->chassis->gyro.motionRotateAngle+cmd->chassis->gyro.remoteRotateAngle;
}
void MecanumCalculate(float Vx, float Vy, float Omega, int16_t *Speed)
{
	float Buffer[4], Param, MaxSpeed;
  uint8_t index;
                                       //        |
																			 //	     _____
  Buffer[0] = Vx - Vy + Omega;         //   2||     || 1  电流正逆时针转3510
  Buffer[1] = Vx + Vy + Omega;         //     |     |
	Buffer[2] = -Vx + Vy + Omega;        //     |     |
	Buffer[3] = -Vx - Vy + Omega;        //  3 ||     || 4
                                       //      -----

  //限幅
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
float CmMoter_PidControlLoop(moter_t *moter,int16_t *setdata)
{
  moter->speedPid->setdata		= setdata[moter->id];									//速度PID设定值
  moter->speedPid->realdata		=moter->encoder->filter_rate;	//速度PID实际值
  moter->speedPid->test(moter->speedPid);											//速度PID计算
	return moter->speedPid->output;
}

float ChassisRotate_PidControlLoop(gyro_t *gyro)
{
  gyro->pid->setdata		=gyro->setAngle;									//速度PID设定值
  gyro->pid->realdata		=gyro->chassisAngle[0];	//速度PID实际值
  gyro->pid->test(gyro->pid);											//速度PID计算
	return gyro->pid->output;
}

void CMControlLoop(chassis_t *chassis)
{
	chassis->speed.Omega=ChassisRotate_PidControlLoop(&chassis->gyro);
	MecanumCalculate(chassis->speed.Vx,chassis->speed.Vy,chassis->speed.Omega,chassis->speed.moterSpeed);
	
	CM_senddata(CMCan,
	CmMoter_PidControlLoop(&chassis->moter1,chassis->speed.moterSpeed),
	CmMoter_PidControlLoop(&chassis->moter2,chassis->speed.moterSpeed),
	CmMoter_PidControlLoop(&chassis->moter3,chassis->speed.moterSpeed),
	CmMoter_PidControlLoop(&chassis->moter4,chassis->speed.moterSpeed));
}

void controlLoop(void)
{
	cmd.heart++;
	if(cmd.heart%500==0)
	{
		LED_HEAT();
		Monitor_Update();//监视器
	}
	if(cmd.heart==10000)
	{
		cmd.heart=0;
	}
	statusChange(&cmd);//状态检测
	operateStatusExecute(&cmd);
	Arm_ControlLoop();//机械臂
	CMControlLoop(cmd.chassis);//底盘
}






      