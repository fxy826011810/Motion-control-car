#ifndef __PID_H
#define __PID_H



#define __PID_EXT extern

enum //时间状态
{
	NOW = 0,
	LAST = 1,
	LLAST = 2,
};

typedef struct PID_TypeDef
{	
	float Kp;//比例系数
	float Ki;//积分系数
	float Kd;//微分系数
	
	float setdata;//设定值
	float setimax;
  float setmax;
	float setmin;
	
	float Pout;//比例输出
	float Iout;//积分输出
	float Dout;//微分输出
	float error[3];//偏差值
	float realdata[3];//实际速度值
	float output[3];//位置PID输出
	
	void(*test)(struct PID_TypeDef *pid);//pid计算函数
	void(*reset)(struct PID_TypeDef *pid);//pid重置函数
}PID_TypeDef;

void Bsp_Pid_Init(void);
void Pid_Test(PID_TypeDef* pid);
void  Pid_Reset(PID_TypeDef* pid);



__PID_EXT PID_TypeDef CM1ArmSpeedPID ;
__PID_EXT PID_TypeDef CM2ArmSpeedPID ;
__PID_EXT PID_TypeDef CM1ArmPositionPID;
__PID_EXT PID_TypeDef CM2ArmPositionPID;
__PID_EXT PID_TypeDef CMRotatePID ;
__PID_EXT PID_TypeDef CM1SpeedPID ;
__PID_EXT PID_TypeDef CM2SpeedPID ;
__PID_EXT PID_TypeDef CM3SpeedPID ;
__PID_EXT PID_TypeDef CM4SpeedPID ;

			//------底盘电机速度初始值-----//
			
			#define CMSpeedPID_default \
			{\
			4.0f,\
			0.0f,\
			10.0f,\
			0,\
			1000,\
			10000,\
			0,\
			0,\
			0,\
			0,\
			{0,0,0},\
			{0,0,0},\
			{0,0,0},\
			&Pid_Test,\
			&Pid_Reset,\
			}
			#define CMArmSpeedPID_default \
			{\
			6.5f,\
			0.0f,\
			0.0f,\
			0,\
			500,\
			8000 ,\
			0,\
			0,\
			0,\
			0,\
			{0,0,0},\
			{0,0,0},\
			{0,0,0},\
			&Pid_Test,\
			&Pid_Reset,\
			}        
			#define CMArmPositionPID_default \
			{\
			5.0f,\
			0.0f,\
			0.0f,\
			0,\
			500,\
			5000,\
			0,\
			0,\
			0,\
			0,\
			{0,0,0},\
			{0,0,0},\
			{0,0,0},\
			&Pid_Test,\
			&Pid_Reset,\
			}

			#define CMRotatePID_default \
			{\
			4.0f,\
			0.0f,\
			10.0f,\
			0,\
			1000,\
			10000,\
			0,\
			0,\
			0,\
			0,\
			{0,0,0},\
			{0,0,0},\
			{0,0,0},\
			&Pid_Test,\
			&Pid_Reset,\
			}
#endif
