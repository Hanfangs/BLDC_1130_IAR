
#include "PI_Cale.h"
#include "BLDC.h" 
#include "Analog.h"
#include "SysConfig.h"
#include "Timer_ISR.h"
#include "User_Config.h"


 void bldcSpeedControlTime(int32_t Idle_SpeedValue,int32_t Actual_SpeedValue);  //速度环

__IO uint16_t Limit_speed_duty = 1500; //低速启动限制占空比

PIDCONTROL   PID_Speed = {	0.01,                                                 /* p设置为10                    */
							0.005,                                                   /* i设置为3                     */
							0,                                                     /* d设置为0                     */
							1500,                                                  /* 最大输出值9A   */
							300,                                                   /* 最小输出值  300->600     */		
							0,
							0,                                                     /* 误差累计积分为0                  */
							0,                                                   /* 初始化值               */
							INIT_PURPOSE,                                       /* 启动PID时重新计算错误积分    */
							0,
							0,
							1500,												/* 最大占空比*/
							100													/* 最小占空比*/

}; 

/*****************************************************************************
 函 数 名  : CalcAverageSpeedTime
 功能描述  : 对转速时间求平均	没用？
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void CalcAvgSpeedTime(void)
{
	if(!sysflags.flag_SpeedTime)
	{
		Sysvariable.SpeedTime = Sysvariable.SpeedTimeTemp;	
	}
	Sysvariable.SpeedTimeSum += Sysvariable.SpeedTimeTemp;
	
	Sysvariable.SpeedTimeCnt ++;	
	if(Sysvariable.SpeedTimeCnt >= 6)
	{
		if(!sysflags.flag_SpeedTime)
		{
			sysflags.flag_SpeedTime = 1;	
		}
		Sysvariable.SpeedTime = (uint16_t)(Sysvariable.SpeedTimeSum /6);	//一次换相时间
		Sysvariable.SpeedTimeSum = 0;
		Sysvariable.SpeedTimeCnt = 0;		
	}
}
void ErrorCorrection(void)
{

}
/*****************************************************************************
 函 数 名  : CalcSpeedTime
 功能描述  : 计算时间[60度]		还没做补偿
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
 void CalcSpeedTime(void)
{	
	uint16_t theta; 
	theta = atan(DELAYFACTOR * Motor.ActualSpeed);		//延迟角
	// Sysvariable.CorrectionTime = 0.8 * Sysvariable.DelayTime30;		//延迟时间
	TIM3->ARR = Sysvariable.DelayTime30 - Sysvariable.CorrectionTime;		//重新定时了，单位us
	// TIM3->ARR=Sysvariable.DelayTime30;
	TIM3->CR1|=0x0001;       //打开定时器
	TIM3->CNT = 0;
	
	Sysvariable.SpeedTimeTemp=TIM2->CNT; //过零点时间间隔  60°  把TIM2的计数值记录一下  1us一个脉冲
	TIM2->CNT = 0;
	
	Sysvariable.DelayTime30=(uint16_t)(Sysvariable.SpeedTimeTemp/2); 
	Sysvariable.SpeedTime=Sysvariable.SpeedTimeTemp;
	Sysvariable.LastDragTime = Sysvariable.SpeedTimeTemp;
	sysflags.ChangePhase=1; //开始换向		到这只是给了个标志位，还没计时完，等定时中断到了才换相
}
/*****************************************************************************
 函 数 名  : SpeedController
 功能描述  : 占空比控制
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
#if SHF_TEST_SPEED
PID bldc_pid;
void IncPIDInit(void) 
{
	// Motor.motor_speed   = Motor.Duty * 16 / 30 ;			//这个要根据加速完成后的速度来定
	Motor.motor_speed = Motor.ActualSpeed + MOTOR_ACC_DELTA_SPEED;
	bldc_pid.LastError  = 0;                    //Error[-1]
	bldc_pid.PrevError  = 0;                    //Error[-2]
	bldc_pid.Proportion = P_DATA_ACC;              //比例常数 Proportional Const
	bldc_pid.Integral   = I_DATA_ACC;                //积分常数  Integral Const
	bldc_pid.Derivative = D_DATA_ACC;              //微分常数 Derivative Const
	bldc_pid.SetPoint   = Motor.motor_speed;		//设定目标Desired Value     刚才已经设为100了
}

int IncPIDCalc(int NextPoint) 
{
    int iError,iIncpid;                                       //当前误差
    
    iError = bldc_pid.SetPoint - NextPoint;                     //增量计算
    iIncpid = (int)((bldc_pid.Proportion * iError)             /*E[k]项*/\
                -(bldc_pid.Integral * bldc_pid.LastError)     /*E[k-1]项*/\
                +(bldc_pid.Derivative * bldc_pid.PrevError));  /*E[k-2]*/
                
    bldc_pid.PrevError = bldc_pid.LastError;                    //存储误差，用于下次计算
    bldc_pid.LastError = iError;
    
    return(iIncpid);                                    //返回增量值
}

//转速PI控制 + 转速计算
void SpeedController(void)			
{
	uint16_t tmp_Min_Speed_Duty;
	if(mcState == mcRun || mcState == mcDec)
	{
		TuneDutyRatioCnt ++;	
		if(TuneDutyRatioCnt >= SPEEDLOOPCNT)	// 速度环	50ms调整一次
		{
			static uint16_t DutyInc;	//测试用的
			int pid_result;

			// if(sysflags.SwapFlag == 0)
			// {
			// 	Motor.ActualSpeed = Motor.Duty * 16 / 30;	//占空比和速度间的转换
			// 	sysflags.SwapFlag = 1;
			// }
			// else
			// {
			// Motor.ActualSpeed = (Motor.step_counter + Motor.step_counter_prev) * 25;		//与前50ms的值取均值，temp值就是当前速度值
			//M法测速
			// Motor.ActualSpeed = (Motor.step_counter * 60 * 1000)/ (24 * SPEEDLOOPCNT);		//n=(60*m1)/(P*Tg)=60*(cnt/24)*(1/50ms)
			//T法测速
			Motor.ActualSpeed = SPEEDFACTOR / Sysvariable.SpeedTime;	//60°		SpeedTime就是60°的脉冲个数，1个脉冲时间为1/72M，与主频有关
			// }

			bldc_pid.SetPoint = Motor.motor_speed;		//motor_speed是一直在变的，1ms变一次，也就是说每次motor_speed加50
			pid_result = IncPIDCalc(Motor.ActualSpeed);

			pid_result = pid_result * 30 / 16;		//占空比和速度的一个转换，转速（0~1600），占空比（0~3000）
			DutyInc = pid_result;
		
			if (mcState == mcDec)
			{
				tmp_Min_Speed_Duty = MOTOR_DEC_MIN_DUTY_SPEED;		//减速的最小占空比
			} else
			{
				tmp_Min_Speed_Duty = MOTOR_MIN_DUTY_SPEED;			//加速的最小占空比
			}

			if((pid_result + Motor.Duty) < MOTOR_MIN_DUTY_SPEED)
			{
				Motor.Duty = MOTOR_MIN_DUTY_SPEED;        //占空比最小120
			}
			else if((pid_result + Motor.Duty) > MOTOR_MAX_DUTY_SPEED)//950->600
			{
				Motor.Duty = MOTOR_MAX_DUTY_SPEED;      //最大占空比
			}
			else
			{            
				Motor.Duty += pid_result;  
				if(Motor.ActualSpeed <= 100)                 ///在转速比较低的情况下，如果输出的占空比又比较大的话，造成电机受不了的
				{
					if(Motor.Duty >= Limit_speed_duty)         
					{
						Motor.Duty = Limit_speed_duty;
					}
				}
			}

			TuneDutyRatioCnt = 0;
			Motor.step_counter_prev = Motor.step_counter;
			Motor.step_counter = 0;
		}
	}
}

#else
 void SpeedController(void)
{	
	if(mcState == mcRun)
	{
		// TuneDutyRatioCnt ++;	
		if(TuneDutyRatioCnt >= SPEEDLOOPCNT)	// 速度环	10ms调整一次
		{
			TuneDutyRatioCnt = 0;
			Motor.Last_Speed = SPEEDFACTOR / Sysvariable.SpeedTime;   //60°		SpeedTime就是60°的脉冲个数
			// FirstOrder_LPF_Cacl(Motor.Last_Speed,Motor.ActualSpeed,0.06);		//用的硬件滤波
			// bldcSpeedControlTime(UserRequireSpeed,Motor.ActualSpeed);
			bldcSpeedControlTime(UserRequireSpeed,Motor.Last_Speed);
		}
	}

}
/*****************************************************************************
 函 数 名  : bldcSpeedControlTime
 功能描述  : 速度PID
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void bldcSpeedControlTime(int32_t Idle_SpeedValue,int32_t Actual_SpeedValue)
{
    float    SpeedTemp;                                                         /* 临时变量                     */
	if ( PID_Speed.Purpose == INIT_PURPOSE ) 
	{			  	
		PID_Speed.fpAllErr=0;
		PID_Speed.Purpose = RUN_PURPOSE;
	}	
	PID_Speed.Error=Idle_SpeedValue -Actual_SpeedValue;		 
	PID_Speed.fpAllErr+=PID_Speed.Error*PID_Speed.Kp;
	PID_Speed.Ui=PID_Speed.Ki*PID_Speed.fpAllErr; 
	if ( PID_Speed.fpAllErr > PID_Speed.MaxValue/PID_Speed.Ki) 	/* 控制积分上限 输出最大电流是9000MA  */
	{              
		PID_Speed.fpAllErr = PID_Speed.MaxValue/PID_Speed.Ki;
	}
	if ( PID_Speed.fpAllErr < PID_Speed.MinValue/PID_Speed.Ki) /* 控制积分下限 输出最小电流值*/
	{              
		PID_Speed.fpAllErr = PID_Speed.MinValue/PID_Speed.Ki;
	}
	
	SpeedTemp=PID_Speed.Kp*PID_Speed.Error+PID_Speed.Ui;
		 			
	PID_Speed.Out = SpeedTemp;
			
   if (PID_Speed.Out >= PID_Speed.Max) {                               /* 占空比上下限限制             */
        PID_Speed.Out = PID_Speed.Max;
    }
    if (PID_Speed.Out < PID_Speed.Min) {
        PID_Speed.Out = PID_Speed.Min;
    }
		Motor.Duty=PID_Speed.Out; 
}
#endif



