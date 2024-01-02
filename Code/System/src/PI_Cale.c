
#include "PI_Cale.h"
#include "BLDC.h" 
#include "Analog.h"
#include "SysConfig.h"
#include "Timer_ISR.h"
#include "User_Config.h"

 void bldcSpeedControlTime(int32_t Idle_SpeedValue,int32_t Actual_SpeedValue);  //速度环

PIDCONTROL   PID_Speed = {	0.01,                                                 /* p设置为10                    */
							0.005,                                                   /* i设置为3                     */
							0,                                                     /* d设置为0                     */
							1500,                                                  /* 最大输出值9A   */
							300,                                                   /* 最小输出值       */
							0,
							0,                                                     /* 误差累计积分为0                  */
							0,                                                   /* 初始化值               */
							INIT_PURPOSE,                                       /* 启动PID时重新计算错误积分    */
							0,
							0,
							1500,												/* 最大占空比*/
							300													/* 最小占空比*/

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
/*****************************************************************************
 函 数 名  : CalcSpeedTime
 功能描述  : 计算时间[60度]
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
 void CalcSpeedTime(void)
{	 
	
	TIM3->ARR=Sysvariable.DelayTime30;		//重新定时了，单位us
	TIM3->CR1|=0x0001;       //打开定时器
	TIM3->CNT = 0;
	
	Sysvariable.SpeedTimeTemp=TIM2->CNT; //过零点时间间隔  60°  把TIM2的计数值记录一下  1us一个脉冲	//可能还是要改成1s，
	TIM2->CNT = 0;
	
	Sysvariable.DelayTime30=(uint16_t)(Sysvariable.SpeedTimeTemp/2); 
	Sysvariable.SpeedTime=Sysvariable.SpeedTimeTemp;
	sysflags.ChangePhase=1; //开始换向		到这只是给了个标志位，还没计时完，等定时中断到了才换相
}
/*****************************************************************************
 函 数 名  : SpeedController
 功能描述  : 占空比控制
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
 void SpeedController(void)
{	
	if(	Motor.ControlMode ==CLOSED_SPEEDLOOP_Halless) //闭环运行
	{
		if(TuneDutyRatioCnt >= SPEEDLOOPCNT)	// 速度环	3个周期调整一次
		{
			TuneDutyRatioCnt = 0;
			Motor.Last_Speed = SPEEDFACTOR / Sysvariable.SpeedTime;   //60°/60°时间
			//FirstOrder_LPF_Cacl(Motor.Last_Speed,Motor.ActualSpeed,0.06);		//用的硬件滤波
			//bldcSpeedControlTime(UserRequireSpeed,Motor.ActualSpeed);
			bldcSpeedControlTime(UserRequireSpeed,Motor.Last_Speed);
		}
	}

  else//开环运行
  {
		  if(TuneDutyRatioCnt >= SPEEDLOOPCNT)	
			{
				 TuneDutyRatioCnt = 0;
				 Motor.StepNum++;
				 Motor.Last_Speed = SPEEDFACTOR / Sysvariable.SpeedTime;

				 FirstOrder_LPF_Cacl(Motor.Last_Speed,Motor.ActualSpeed,0.06);
		  	 if(Motor.Duty < PWM_MIN_DUTY)
				 {
					 
					  if(Motor.StepNum>LOW_DUTY_COUNT)
						{
							 Motor.StepNum=0;
						   Motor.Duty += ADD_DUTY1;		//小于最小占空比时，占空比每两步+1			 
					  }
				 }
			   else
				 {
					if(Motor.Duty < UserRequireSpeed)		//小于最大占空比 1500
					{
						if(Motor.Duty<=DUTYTHRESHOLD1)		//小于0.5*最大占空比
						{
							if(Motor.StepNum>LOW_DUTY_COUNT)
							{
								 Motor.StepNum=0;
								 Motor.Duty += ADD_DUTY1;	//占空比每两步+1		 
							}
						}
						else if(Motor.Duty<DUTYTHRESHOLD2)		//小于0.7*最大占空比
						{
							if(Motor.StepNum>HIGH_DUTY_COUNT)
					   	{
							 Motor.StepNum=0;
						   Motor.Duty += ADD_DUTY2;			//	占空比每两步+2	 
						  }
						}
						else
						{
							if(Motor.StepNum>HIGH_DUTY_COUNT)
						  {
								 Motor.StepNum=0;
								 Motor.Duty += ADD_DUTY3;		//	占空比每两步+3		 
						  }
						}
					}
					else if(Motor.Duty  > UserRequireSpeed)		//大于最大占空比
					{
						Motor.Duty -= ADD_DUTY2;
						Motor.StepNum=0;

					} 

				}
			}
			if(Motor.Duty <= MIN_DUTY)	// 限制输出占空比的最大最小值
			{
				Motor.Duty = MIN_DUTY;
			}
			else if(Motor.Duty >= MAX_DUTY)
			{
				Motor.Duty = MAX_DUTY;
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



