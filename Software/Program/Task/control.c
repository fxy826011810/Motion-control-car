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
{0,&CanCm1_Monitor,NULL,&CM1SpeedPID,&CM1Encoder},\
{1,&CanCm2_Monitor,NULL,&CM2SpeedPID,&CM2Encoder},\
{2,&CanCm3_Monitor,NULL,&CM3SpeedPID,&CM3Encoder},\
{3,&CanCm4_Monitor,NULL,&CM4SpeedPID,&CM4Encoder},\
{&ChassisGyro_Monitor,&CMRotatePID,{NULL,0,0,0,0},{0.0f,0.0f,0.0f},{0.0f,0.0f,0.0f},0,0.0f,wait},\
{0,0,0,{0,0,0,0}}\
};
//机械臂赋初值
mecArm_t MecArm={\
{6,&ForeArm_Monitor,&CM1ArmPositionPID,&CM1ArmSpeedPID,&CM1ArmEncoder},\
{5,&MainArm_Monitor,&CM2ArmPositionPID,&CM2ArmSpeedPID,&CM2ArmEncoder},\
0.0f,0.0f};
//接收赋初值
Rec_t Rec={\
{&IMURec_Monitor,{NULL,0,0,0,0},0.0f,0.0f,0.0f},\
{&DbusRec_Monitor,wait,0}\
};


static void ArmPidControlLoop(moter_t *moter,float SetData)
{
	moter->positionPid->setdata	= SetData;											//位置PID设定值
	moter->positionPid->realdata = moter->encoder->ecd_angle;		//位置PID实际值
	moter->positionPid->test(moter->positionPid);								//位置PID计算
	
	moter->speedPid->setdata = moter->positionPid->output;			//速度PID设定值
	moter->speedPid->realdata	= moter->encoder->filter_rate;		//速度PID实际值
  moter->speedPid->test(moter->speedPid);											//速度PID计算
}
static void ForeArm_ControlLoop(void)//前臂
{
	if(MecArm.foreArm.encoder->dataStatus==ready)
	{
		ArmPidControlLoop(&MecArm.foreArm,MecArm.forearmSetAngle);
		MecArm.foreArm.encoder->dataStatus=wait;
	}
}

static void MainArm_ControlLoop(void)
{
	if(MecArm.mainArm.encoder->dataStatus==ready)
	{
		ArmPidControlLoop(&MecArm.mainArm,MecArm.mainArmSetAngle);
		MecArm.mainArm.encoder->dataStatus=wait;
	}
}
static void Arm_ControlLoop(void)
{
	ForeArm_ControlLoop();
	MainArm_ControlLoop();
	Arm_senddata(ArmCan,MecArm.foreArm.speedPid->output,MecArm.mainArm.speedPid->output);
}	


void smoothFilter(float *i,float *o,float rate)
{
	*o=(*o)*(1-rate)+(*i)*rate;
}
static float ChassisRotate_PidControlLoop(gyro_t *gyro)
{
  gyro->pid->setdata		=gyro->setAngle;				//速度PID设定值
  gyro->pid->realdata		=gyro->chassisAngle[0];	//速度PID实际值
  gyro->pid->test(gyro->pid);										//速度PID计算
	return gyro->pid->output;
}

static void operateStatusChange(cmd_t *cmd)
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


static void operateStatusExecute(cmd_t *cmd)//操作模式执行
{
	FunctionalState status=DISABLE;
	if(cmd->operateStatus==remote)
	{
		if(Rec.remote.dataStatus==ready)
		{
			Rec.remote.dataStatus=wait;
			cmd->chassis->speed.Vy=cmd->rec->remote.dbusRec.rc.ch1*3;//前后
			cmd->chassis->speed.Vx=cmd->rec->remote.dbusRec.rc.ch0*3;//左右
			cmd->chassis->gyro.remoteRotateAngle+=(float)cmd->rec->remote.dbusRec.rc.ch2*5.0f/cmd->rec->remote.mon->count/330;//遥控角度
			status=ENABLE;
		}
	}
	else if(cmd->operateStatus==motion)
	{
		cmd->chassis->speed.Vy=cmd->rec->motion.recAngle[1]*50;//pitch
		cmd->chassis->speed.Vx=cmd->rec->motion.recAngle[2]*50;//roll
		cmd->chassis->gyro.motionRotateAngle=cmd->rec->motion.recAngle[0];//手势角度
		status=ENABLE;
	}
	else if(cmd->operateStatus==allUse)
	{
		if(Rec.remote.dataStatus==ready)
		{
			Rec.remote.dataStatus=wait;
			cmd->chassis->speed.Vy=cmd->rec->remote.dbusRec.rc.ch1*3;//前后
			cmd->chassis->speed.Vx=cmd->rec->remote.dbusRec.rc.ch0*3;//左右
		}
		cmd->chassis->gyro.motionRotateAngle=cmd->rec->motion.recAngle[0];//手势角度
		status=ENABLE;
	}
	else if(cmd->operateStatus==noUse)
	{
		cmd->chassis->speed.Vy=0;//前后
		cmd->chassis->speed.Vx=0;//左右
		cmd->chassis->speed.Omega=0;//旋转
		status=ENABLE;
	}
	cmd->chassis->gyro.setAngle=cmd->chassis->gyro.motionRotateAngle+cmd->chassis->gyro.remoteRotateAngle;//底盘角度
	if(status==ENABLE&&Chassis.gyro.dataStatus==ready)
	{
		cmd->chassis->speed.Omega=ChassisRotate_PidControlLoop(&cmd->chassis->gyro);
		Chassis.gyro.dataStatus=wait;
	}
}
static void MecanumCalculate(float Vx, float Vy, float Omega, int16_t *Speed)
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
	moter->encoder->dataStatus=wait;
	return moter->speedPid->output;
}



static void CMControlLoop(chassis_t *chassis)
{
	MecanumCalculate(chassis->speed.Vx,chassis->speed.Vy,chassis->speed.Omega,chassis->speed.moterSpeed);
	if(chassis->moter1.encoder->dataStatus==ready&&
		 chassis->moter2.encoder->dataStatus==ready&&
		 chassis->moter3.encoder->dataStatus==ready&&
		 chassis->moter4.encoder->dataStatus==ready)
	{
	CM_senddata(CMCan,
	CmMoter_PidControlLoop(&chassis->moter1,chassis->speed.moterSpeed),
	CmMoter_PidControlLoop(&chassis->moter2,chassis->speed.moterSpeed),
	CmMoter_PidControlLoop(&chassis->moter3,chassis->speed.moterSpeed),
	CmMoter_PidControlLoop(&chassis->moter4,chassis->speed.moterSpeed));
	}
}
static void systemStatusChange(cmd_t *cmd)
{
	if((cmd->chassis->moter1.mon->status==offline)||
		 (cmd->chassis->moter1.mon->status==offline)||
		 (cmd->chassis->moter1.mon->status==offline)||
		 (cmd->chassis->moter1.mon->status==offline)||
		 (cmd->mecArm->foreArm.mon->status==offline)||
		 (cmd->mecArm->mainArm.mon->status==offline))
	{
		cmd->systemStatus=stop;
	}
	else{
//------------------此处写准备阶段----------------------//
		if(1)
		
//------------------此处写准备阶段----------------------//
		{
		cmd->systemStatus=normal;
		}
	}
}
static void systemStatusExecute(cmd_t *cmd)//系统模式执行
{
		if(cmd->systemStatus==prepare)
	{
		
	}
	if(cmd->systemStatus==normal)
	{
		operateStatusChange(cmd);//检测操作状态
		operateStatusExecute(cmd);//按操作状态执行
		Arm_ControlLoop();//机械臂
		CMControlLoop(cmd->chassis);//底盘
	}
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
	systemStatusChange(&cmd);//检测系统状态
	systemStatusExecute(&cmd);//按系统状态执行
}






      