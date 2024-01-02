#ifndef _BLDC_FILE
#define _BLDC_FILE

#include "Analog.h"
#include "BLDC.h"
#include "SysConfig.h" 
#include "Timer.h"
#include "GPIO_Init.h"
#include "PI_Cale.h"
#include "Timer_ISR.h"
#include "User_Config.h"

//参数初始化
uint16_t  UserSpeedSample=0;
uint8_t  pos_check_stage=0;
uint16_t  ADC_check_buf[6]={0};
uint16_t  ADC_Sum_buf[6]={0};
uint8_t  Initial_stage=0;
uint8_t  PhaseCnt=0;
uint8_t  pos_idx=0;
uint8_t  Flag_adc=0;
uint8_t  Charger_Time=0;
uint8_t  Flag_Charger=0;
uint8_t  Flag_OverCurr=0;
uint16_t  test_idx=0;


BLDC Motor={                                                          /* 电机结构体        */
             OPEN_LOOP_Halless,                                     /* 电机控制方式      */
             BLDCSTOP,                                                /* 电机状态          */
             0,                                                       /* 理论速度          */
             0,                                                       /* 实际速度          */
             0,
             0,                                                     /* 当前占空比        */
	           0,
             0,
             0,
             0
     };
void EnterDragInit(void);
void Change_Voltage(void);
void Startup_Turn(void);
		 
void Sys_Variable_Init(void)
{
	 sysflags.flag_SpeedTime = 0;		//标志位
	 sysflags.Angle_Mask=0;
	 sysflags.ChangePhase=0;
	 sysflags.Motor_Stop=0;
	 sysflags.System_Start=0;
	 Sysvariable.SpeedTimeCnt = 0;		//变量
	 Sysvariable.SpeedTime = 0;
	 Sysvariable.SpeedTimeTemp = 0;
	 Sysvariable.SpeedTimeSum = 0;
	 Sysvariable.Stop_Time=0;
	
}
 /*****************************************************************************
 函 数 名  : MotorInit
 功能描述  : 初始化
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
 void MotorInit(void)
 {
	Motor.PhaseCnt=0;
	Motor.LastPhase=0;
	Motor.NextPhase=0;
	Motor.ActualSpeed=0;
	Motor.Last_Speed=0;
	
	UserRequireSpeed = 0;	//
	PID_Speed.Purpose = INIT_PURPOSE;
	Mask_TIME=Low_DutyMask_Time;
	Filter_Count=Delay_Filter;
	
	Flag_Charger=0;
	Flag_adc=0;
	Flag_OverCurr=0;
	pos_idx=0;
	pos_check_stage=0;
	// TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable); //使能上管
	// TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
	// TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
	 
	// GPIO_SetBits(GPIOB, U_Mos_L_Pin);  //下管截止
	// GPIO_SetBits(GPIOB, W_Mos_L_Pin);
	// GPIO_SetBits(GPIOB, V_Mos_L_Pin); 
	Start_Motor();
	 
	mcState = mcAlignment;
	mcFault=RunNormal;
 }
 
/*****************************************************************************
 函 数 名  : MotorAlignment
 功能描述  : 对齐定位
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void MotorAlignment(void)		//没用到
{
	Motor.Duty =ALIGNMENTDUTY ;  //对齐占空比		占空比100
	Motor.PhaseCnt ++;   //0-1	换相
	Startup_Turn(); //强制换向
	Delay_ms(ALIGNMENTNMS);//对齐时间
	EnterDragInit();  //加速初始化
}	
/*****************************************************************************
 函 数 名  : EnterDragInit
 功能描述  : 进入强拖初始化
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
 void EnterDragInit(void)
{
	Motor.Duty =ALIGNMENTDUTY ; 
	Sysvariable.ADCTimeCnt = 0;
	Sysvariable.DragTime = RAMP_TIM_STA;
	mcState =  mcDrag; //初始化完成   进入加速
	Sysvariable.ChangeTime_Count = 0;
          
	Sysvariable.ChangeCount = 1;
	TIM_Cmd(TIM2, ENABLE);			
}
/*****************************************************************************
 函 数 名  : EnterRunInit
 功能描述  : 进入Run的初始化
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void EnterRunInit(void)
{
	Sysvariable.ADCTimeCnt = 0;
	Sysvariable.Timer3OutCnt = 0;
	Sysvariable.BlankingCnt = 0;
	sysflags.ChangePhase=0;
	sysflags.Angle_Mask=0;
	TuneDutyRatioCnt=0;
	// Motor.PhaseCnt++;
	//Startup_Turn(); //强制换向
	mcState = mcRun;
	
}
/*****************************************************************************
 函 数 名  : StartupDrag
 功能描述  : 启动 加速 
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
#if 1
 void StartupDrag(void)
{	
	//Sysvariable.ADCTimeCnt ++;
	static uint16_t idx;
	static uint16_t DragTimeArray[60];
	static uint16_t DutyArray[60];
	static uint16_t U_Vol[100];
	static uint16_t TimeLeft;
	if(Sysvariable.ADCTimeCnt >= Sysvariable.DragTime)		//DragTime开始步进时间 = 190,采样时间大于步进时间时，开始递减
	{
		Sysvariable.ADCTimeCnt = 0;
		Motor.Duty += RAMP_DUTY_INC;
		Sysvariable.DragTime -= (uint16_t)((Sysvariable.DragTime / RAMP_TIM_STEP) + 1);		//一开始是  190-[(190/9)+1]=168
		if(Sysvariable.DragTime < RAMP_TIM_END)				// 190，168，149，...20；注意此处递减步数应该按照电机参数来设计
		{
			Sysvariable.DragTime = RAMP_TIM_END;
			TimeLeft = Sysvariable.ChangeTime_Count;
			//All_Discharg();		//后面记得删，调试用
			EnterRunInit();		//此时强拖完成了，切换为反电势检测
			return;

	   //}
		}
		DragTimeArray[idx] = Sysvariable.DragTime;
		DutyArray[idx] = Motor.Duty;
		U_Vol[idx] = RegularConvData_Tab[0];
		idx++;
		Motor.PhaseCnt++;		//步进一次，换相一次
		Sysvariable.DelayTime30=TIM2->CNT; //过零点时间间隔，从0到一次换相的时间，就是一个过零点的时间间隔
		Startup_Turn(); //换向
		TIM2->CNT = 0;
		Sysvariable.DelayTime30=Sysvariable.DelayTime30/2;		//过零点间隔是60个电角度
	}
	if(Motor.Duty < RAMP_DUTY_STA)	// 限制输出占空比的最大最小值		100-300
	{
		Motor.Duty = RAMP_DUTY_STA;
	}
	else if(Motor.Duty > RAMP_DUTY_END)
	{
		Motor.Duty = RAMP_DUTY_END;
	}
}
#else
void StartupDrag(void)
{
	static uint16_t idx;
	static uint16_t ChangeTimeArray[100];
	static uint16_t U_Vol[100];
	if(Sysvariable.ChangeTime_Count == 5)
	{
		Startup_Turn();		//打个断点看能不能进来
	}
	if(Sysvariable.ChangeTime_Count == ((uint16_t)ACC_FUN(Sysvariable.ChangeCount, P1, P2, P3, P4)))	//换相条件
	{
		if(Sysvariable.ChangeCount > 46)
		{
			Sysvariable.ChangeCount = 0;
			
			All_Discharg();		//后面记得删，调试用
			EnterRunInit();
			idx = 0;
			return;
		} 
		ChangeTimeArray[idx] = Sysvariable.ChangeTime_Count;
		U_Vol[idx] = RegularConvData_Tab[0];
		idx++;
		Sysvariable.ChangeCount++;		//换相次数：0~47
		Change_Voltage();
		Motor.PhaseCnt++;
		Sysvariable.DelayTime30=TIM2->CNT;
		TIM2->CNT = 0;
		Sysvariable.DelayTime30=Sysvariable.DelayTime30/2;
		Startup_Turn(); //换向
	}

}
void Change_Voltage(void)		//不太对，因为每次换相时才会进入一次这个函数，所以不需要给区间
{
	if(Sysvariable.ChangeTime_Count >= 20 && Sysvariable.ChangeTime_Count < 71)
	{
		Motor.Duty = PWM_ARR * 144 / 1000;		//14.4%
	}
	if(Sysvariable.ChangeTime_Count >= 71 && Sysvariable.ChangeTime_Count < 118)
	{
		Motor.Duty = PWM_ARR * 152 / 1000;		//15.2%
	}
	if(Sysvariable.ChangeTime_Count >= 118 && Sysvariable.ChangeTime_Count < 173)
	{
		Motor.Duty = PWM_ARR * 162 / 1000;		//16.2%
	}
	if(Sysvariable.ChangeTime_Count >= 173 && Sysvariable.ChangeTime_Count < 222)
	{
		Motor.Duty = PWM_ARR * 178 / 1000;		//17.8%
	}
	if(Sysvariable.ChangeTime_Count >= 222 && Sysvariable.ChangeTime_Count < 267)
	{
		Motor.Duty = PWM_ARR * 199 / 1000;		//19.9%
	}
	if(Sysvariable.ChangeTime_Count >= 267 && Sysvariable.ChangeTime_Count < 315)
	{
		Motor.Duty = PWM_ARR * 232 / 1000;		//23.2%
	}
	if(Sysvariable.ChangeTime_Count >= 315 && Sysvariable.ChangeTime_Count < 370)
	{
		Motor.Duty = PWM_ARR * 270 / 1000;		//27.0%
	}
	if(Sysvariable.ChangeTime_Count >= 370 && Sysvariable.ChangeTime_Count < 418)
	{
		Motor.Duty = PWM_ARR * 301 / 1000;		//29.6%
	}
	if(Sysvariable.ChangeTime_Count >= 416 && Sysvariable.ChangeTime_Count < 467)
	{
		Motor.Duty = PWM_ARR * 333 / 1000;		//29.6%
	}
	if(Sysvariable.ChangeTime_Count >= 467 && Sysvariable.ChangeTime_Count < 519)
	{
		Motor.Duty = PWM_ARR * 365 / 1000;		//29.6%
	}
	if(Sysvariable.ChangeTime_Count >= 519 && Sysvariable.ChangeTime_Count < 570)
	{
		Motor.Duty = PWM_ARR * 391 / 1000;		//29.6%
	}
	if(Sysvariable.ChangeTime_Count >= 570 && Sysvariable.ChangeTime_Count < 618)
	{
		Motor.Duty = PWM_ARR * 411 / 1000;		//29.6%
	}

	// if(Sysvariable.ChangeCount >= 17 && Sysvariable.ChangeCount < 70)
	// {
	// 	Motor.Duty = PWM_ARR * 144 / 1000;		//14.4%
	// }
	// if(Sysvariable.ChangeCount >= 70 && Sysvariable.ChangeCount < 118)
	// {
	// 	Motor.Duty = PWM_ARR * 152 / 1000;		//15.2%
	// }
	// if(Sysvariable.ChangeCount >= 118 && Sysvariable.ChangeCount < 174)
	// {
	// 	Motor.Duty = PWM_ARR * 162 / 1000;		//16.2%
	// }
	// if(Sysvariable.ChangeCount >= 174 && Sysvariable.ChangeCount < 226)
	// {
	// 	Motor.Duty = PWM_ARR * 178 / 1000;		//17.8%
	// }
	// if(Sysvariable.ChangeCount >= 226 && Sysvariable.ChangeCount < 271)
	// {
	// 	Motor.Duty = PWM_ARR * 199 / 1000;		//19.9%
	// }
	// if(Sysvariable.ChangeCount >= 271 && Sysvariable.ChangeCount < 317)
	// {
	// 	Motor.Duty = PWM_ARR * 232 / 1000;		//23.2%
	// }
	// if(Sysvariable.ChangeCount >= 317 && Sysvariable.ChangeCount < 370)
	// {
	// 	Motor.Duty = PWM_ARR * 270 / 1000;		//27.0%
	// }
	// if(Sysvariable.ChangeCount >= 370 && Sysvariable.ChangeCount < 416)
	// {
	// 	Motor.Duty = PWM_ARR * 301 / 1000;		//29.6%
	// }
	// if(Sysvariable.ChangeCount >= 416 && Sysvariable.ChangeCount < 467)
	// {
	// 	Motor.Duty = PWM_ARR * 333 / 1000;		//29.6%
	// }
	// if(Sysvariable.ChangeCount >= 467 && Sysvariable.ChangeCount < 519)
	// {
	// 	Motor.Duty = PWM_ARR * 365 / 1000;		//29.6%
	// }
	// if(Sysvariable.ChangeCount >= 519 && Sysvariable.ChangeCount < 570)
	// {
	// 	Motor.Duty = PWM_ARR * 391 / 1000;		//29.6%
	// }
	// if(Sysvariable.ChangeCount >= 570 && Sysvariable.ChangeCount < 618)
	// {
	// 	Motor.Duty = PWM_ARR * 411 / 1000;		//29.6%
	// }
}
#endif
/*****************************************************************************
 函 数 名  : Startup_Turn
 功能描述  : 换相 
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void Startup_Turn(void) 
{
		if(Motor.PhaseCnt > 6) {
			Motor.PhaseCnt = 1;
		}
		if(Motor.PhaseCnt < 1) {
			Motor.PhaseCnt = 6;
		} 
		switch(Motor.PhaseCnt) 
		{
			case  1:		//此处无霍尔信号，所以顺序就是1-6；有霍尔应为651342或546231
			{
				MOS_Q15PWM();//UV	A+B-
			}
			break;
			
			case  2:   
			{
				MOS_Q16PWM();//UW	A+C-
			}
			break;

			case  3:
			{
		  		MOS_Q26PWM();// VW	B+C-
			}
			break;
		
			case  4:     
			{
				MOS_Q24PWM();//VU	B+A-
			}
			break;
			
			case  5:    
			{
				MOS_Q34PWM();//WU	C+A-
			}
			break;
			
			case  6:    
			{
				MOS_Q35PWM();//WV	C+B-
			}
			break;	
			default:
			{
				
				MotorStop();

			}
			break;
		}
}
/*****************************************************************************
 函 数 名  : MotorControl
 功能描述  : 电机控制
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void MotorControl(void)
{
	switch(mcState)
	{
		case mcInit:	// 初始化
			MotorInit();
			Sys_Variable_Init();	
			break;
			
		case mcAlignment:	// 定位
		   	break;
			
		case mcDrag:	// 强拖				
			  break;
			
		case mcRun:	//运行
			 SpeedController();		//占空比控制，可开环，可闭环，源代码是开环，后面可以换一下闭环
			 break;
									
		case mcReset:	// 电机立即重启
			break;
			
		case mcStop:	// 电机停止，重新上电
		  	 MotorStop();
			break;
			
		default:
			MotorStop();
			break;			
	}
}
/*****************************************************************************
 函 数 名  : MotorStop
 功能描述  : 电机停止
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void MotorStop(void)
{

	mcState=mcStop;
	Motor.Duty = 0;
	Motor.PhaseCnt = 0;
	TuneDutyRatioCnt=0;
	TIM_SetCompare1(TIM1, 0);
	TIM_SetCompare2(TIM1, 0);
	TIM_SetCompare3(TIM1, 0);
	TIM1->CCR1=0;
	TIM1->CCR2=0;
	TIM1->CCR3=0;
	
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Disable);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Disable);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Disable);
	
 // TIM_Cmd(TIM14, DISABLE);    //失能定时器	 
	TIM_Cmd(TIM2, DISABLE);    //失能定时器	 
	TIM_SetCounter(TIM2,0);  //重新计数
	TIM_SetCounter(TIM3,0);  //重新计数
	
	//LIN 输入与输出反向  故停止时 应该设置输入电平为高 这样输出为低--刹车
	if(STOPMODE==BRAKEDOWN)
	{
		GPIO_ResetBits(GPIOB, U_Mos_L_Pin);
		GPIO_ResetBits(GPIOB, W_Mos_L_Pin);	
		GPIO_ResetBits(GPIOB, V_Mos_L_Pin); 
		Delay_ms(300);
		sysflags.Motor_Stop=1;
				
	////	//LIN 输入与输出反向  故停止时 应该设置输入电平为高 这样输出为低--自由停
 	}
	if(STOPMODE==FREEDOWN)
	{
		GPIO_SetBits(GPIOB, U_Mos_L_Pin);
		GPIO_SetBits(GPIOB, W_Mos_L_Pin);
		GPIO_SetBits(GPIOB, V_Mos_L_Pin); 
		//Delay_ms(300);
		Sysvariable.Stop_Time=POWER_DELAY;
		sysflags.Motor_Stop=1;
 	}
}

 
/*****************************************************************************
 函 数 名  : UserSpeedControlInit
 功能描述  : 调速控制开始
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void UserSpeedControlInit(void)
{
	static uint8_t RheostatCnt0=0;
	
 if(ADJ_MODE==DIRECT_GIVE)
 {
		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4)==0)		//按键低电平为打开
		{
			RheostatCnt0++;
			if(RheostatCnt0>100)		//检测到100次PF_1为低电平，系统启动，所以PF检测的是什么？？
			{
			RheostatCnt0=0;
			sysflags.System_Start=1;
			}	
		}
		else
		{	
			RheostatCnt0=0;
			sysflags.System_Start=0;

		}
  }
}
/*****************************************************************************
 函 数 名  : UserSpeedControl
 功能描述  : 调速控制
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void UserSpeedControl(void)
{
	static uint8_t RheostatCnt0=0;  //调速局部变量
  static uint8_t RheostatCnt1=0;  //调速局部变量
	if( Motor.ControlMode==CLOSED_SPEEDLOOP_Halless)
	{
			if(ADJ_MODE==DIRECT_GIVE)
			{	 
				if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4)==0)
				{
					RheostatCnt0++;
					if(RheostatCnt0>10)
					{
						RheostatCnt0=0;
						UserRequireSpeed=Motor_UserSpeed;
					}	
				}
				else
				{
					RheostatCnt1++;
					if(RheostatCnt1>10)
					{
						RheostatCnt1=0;
						UserRequireSpeed=0;
						Sysvariable.Stop_Time=Motor_DelayTime;		
						MotorStop();
					}	
				}
			}
    
  }
	else  //开环
	{
			if(ADJ_MODE==DIRECT_GIVE)
			{	 
				if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4)==0)
				{
					RheostatCnt0++;
					if(RheostatCnt0>10)
					{
						RheostatCnt0=0;
						sysflags.System_Start=1;
						UserRequireSpeed=MAX_DUTY;

					}	
				}
				else
				{
					RheostatCnt1++;
					if(RheostatCnt1>10)
					{
						RheostatCnt1=0;
						UserRequireSpeed=0;
						sysflags.System_Start=0;
						Sysvariable.Stop_Time=Motor_DelayTime;		
						MotorStop();
					}	
				}
		
    }
  }
}
#if 0
/*****************************************************************************
 函 数 名  : Charger
 功能描述  : 充电
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void  Charger(void)
{
	TIM1->CCR1 = 0;   //关上桥   
	TIM1->CCR2 = 0;   //   
	TIM1->CCR3 = 0;   //   
	
	GPIOB->BRR = U_Mos_L_Pin|V_Mos_L_Pin;  //UV下桥开
	GPIOB->BRR = W_Mos_L_Pin ;  //w下管开
	Flag_Charger=1;  //充电标志位

}
/*****************************************************************************
 函 数 名  : All_Discharg
 功能描述  : 全部关闭
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void  All_Discharg(void)	
{
	TIM1->CCR1=0;
	TIM1->CCR2=0;
	TIM1->CCR3=0;
	GPIOB->BSRR = U_Mos_L_Pin | V_Mos_L_Pin | W_Mos_L_Pin;  //UVW下桥关
}
/*****************************************************************************
 函 数 名  : UV_W_phase_inject
 功能描述  : UV_W注入
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void  UV_W_phase_inject(void)	//110
{ 

	TIM1->CCR1 = Lock_Duty;	  //	 U上		
	TIM1->CCR2 = Lock_Duty;   //    V上
	GPIOB->BRR = W_Mos_L_Pin ;  //w下管开
	Flag_adc=1;  
	pos_idx=0;
}
/*****************************************************************************
 函 数 名  : W_UV_phase_inject
 功能描述  : W_UV注入
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void  W_UV_phase_inject(void)	//001
{
	
	TIM1->CCR3 = Lock_Duty;   //W上
	GPIOB->BRR = U_Mos_L_Pin|V_Mos_L_Pin;  //下桥开  uv 
	Flag_adc=1;  
	pos_idx=1;	
}
/*****************************************************************************
 函 数 名  : WU_V_phase_inject
 功能描述  : WU  V注入
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void  WU_V_phase_inject(void)	//101
{
	TIM1->CCR1 = Lock_Duty;   //U上
	TIM1->CCR3 = Lock_Duty;	 //	W上	
	GPIOB->BRR = V_Mos_L_Pin;  //下桥开  v  
	Flag_adc=1;  
	pos_idx=2;
}
/*****************************************************************************
 函 数 名  : V_WU_phase_inject
 功能描述  : V_WU 注入
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void  V_WU_phase_inject(void)	//010
{  
	TIM1->CCR2 = Lock_Duty;   //    V上
	GPIOB->BRR = W_Mos_L_Pin ;  //w下管开
	GPIOB->BRR = U_Mos_L_Pin ;//U下管开
	Flag_adc=1;  
	pos_idx=3;
}
/*****************************************************************************
 函 数 名  : VW_U_phase_inject
 功能描述  : VW_U 注入
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void  VW_U_phase_inject(void)	//011
{

	 TIM1->CCR2 = Lock_Duty;   //W上
	 TIM1->CCR3 = Lock_Duty;	 //	V上	
	 GPIOB->BRR = U_Mos_L_Pin ;//U下管开
	 Flag_adc=1;  
	 pos_idx=4;
	
}
/*****************************************************************************
 函 数 名  : U_VW_phase_inject
 功能描述  : U_VW 注入
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void  U_VW_phase_inject(void)	//100
{
	 
	TIM1->CCR1 = Lock_Duty;	 //	U上	
	GPIOB->BRR = W_Mos_L_Pin ;  //w下管开
	GPIOB->BRR = V_Mos_L_Pin ;//V下管开
	Flag_adc=1;  
	pos_idx=5;
}
#else
//------------------------------------------------------------------------------------------------------
void  Charger(void)
{
	TIM1->CCR1 = 0;  
	TIM1->CCR2 = 0;         					  
	TIM1->CCR3 = 0;
	GPIO_SetBits(GPIOB, GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15); 

	Flag_Charger=1;  //充电标志位

}
void  All_Discharg(void)	
{
	TIM1->CCR1 = 0;  
	TIM1->CCR2 = 0;         					  
	TIM1->CCR3 = 0;
	GPIO_ResetBits(GPIOB, GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15); 
}
void  UV_W_phase_inject(void)	//110
{ 

	TIM1->CCR1 = Lock_Duty;  
	TIM1->CCR2 = Lock_Duty;         					  
	TIM1->CCR3 = 0;
	GPIO_ResetBits(GPIOB, GPIO_Pin_13 | GPIO_Pin_14); 
	GPIO_SetBits(GPIOB, GPIO_Pin_15); 

	Flag_adc=1;  
	pos_idx=0;
}
void  W_UV_phase_inject(void)	//001
{
	
	TIM1->CCR1 = 0;  
	TIM1->CCR2 = 0;         					  
	TIM1->CCR3 = Lock_Duty;
	GPIO_ResetBits(GPIOB, GPIO_Pin_15);
	GPIO_SetBits(GPIOB, GPIO_Pin_13 | GPIO_Pin_14); 
	 
	Flag_adc=1;  
	pos_idx=1;	
}
void  WU_V_phase_inject(void)	//101
{
	TIM1->CCR1 = Lock_Duty;  
	TIM1->CCR2 = 0;         					  
	TIM1->CCR3 = Lock_Duty;
	GPIO_ResetBits(GPIOB, GPIO_Pin_13 | GPIO_Pin_15); 
	GPIO_SetBits(GPIOB, GPIO_Pin_14); 

	Flag_adc=1;  
	pos_idx=2;
}
void  V_WU_phase_inject(void)	//010
{  
	TIM1->CCR1 = 0;  
	TIM1->CCR2 = Lock_Duty;         					  
	TIM1->CCR3 = 0;
	GPIO_ResetBits(GPIOB, GPIO_Pin_14);
	GPIO_SetBits(GPIOB, GPIO_Pin_13 | GPIO_Pin_15); 

	Flag_adc=1;  
	pos_idx=3;
}
void  VW_U_phase_inject(void)	//011
{
	TIM1->CCR1 = 0;  
	TIM1->CCR2 = Lock_Duty;         					  
	TIM1->CCR3 = Lock_Duty;
	GPIO_ResetBits(GPIOB, GPIO_Pin_14 | GPIO_Pin_15); 
	GPIO_SetBits(GPIOB, GPIO_Pin_13); 

	 Flag_adc=1;  
	 pos_idx=4;
}
void  U_VW_phase_inject(void)	//100
{
	 
	TIM1->CCR1 = Lock_Duty;  
	TIM1->CCR2 = 0;         					  
	TIM1->CCR3 = 0;
	GPIO_ResetBits(GPIOB, GPIO_Pin_13);
	GPIO_SetBits(GPIOB, GPIO_Pin_14 | GPIO_Pin_15); 

	Flag_adc=1;  
	pos_idx=5;
}
//------------------------------------------------------------------------------------------------------
#endif

/*****************************************************************************
 函 数 名  : Align_pos_check_proc
 功能描述  : 位置检测过程
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
#if PULSE_INJECTION
void Align_pos_check_proc(void)
{
	if((Flag_adc==0)&&(Flag_Charger==0))  //电流采集并且充电完成
	{
		switch(pos_check_stage)//
		{
#if 1
			case 0: //先充电	
				Charger();
				pos_check_stage = 10;
				break;
			case 10://第一个脉冲注入
				UV_W_phase_inject();
				// U_VW_phase_inject();
				pos_check_stage = 1;
				break;
			
			case 1: //充电
				Charger();
				pos_check_stage = 20;
				break;
			case  20://第二个脉冲注入
				W_UV_phase_inject();
				// UV_W_phase_inject();
				pos_check_stage = 3;
				break; 
			
			case 3: //充电	
				Charger();
				pos_check_stage =30;
				break;
			case 30://第三个脉冲注入
				WU_V_phase_inject();
				// V_WU_phase_inject();
				pos_check_stage =4;
				break; 
			case 4://充电
				Charger();
				pos_check_stage =40;
				break;
			case 40://第四个脉冲注入
				V_WU_phase_inject();
				// VW_U_phase_inject();
				pos_check_stage =5;
				break;
			case 5://充电
				Charger();
				pos_check_stage =50;
				break;
			case 50://第五个脉冲注入
				VW_U_phase_inject();
				// W_UV_phase_inject();		   
				pos_check_stage =6;
				break;
			case 6://充电
				Charger();	
				pos_check_stage =60;
				test_idx = 0;
				break;
			case 60://第六个脉冲注入
				U_VW_phase_inject();
				// WU_V_phase_inject();
				pos_check_stage =7;
				break;
			case 7://
				PhaseCnt=0;
			//电流比较获取位置
				if(ADC_check_buf[0]<=ADC_check_buf[1])PhaseCnt|= 0x04;  
				if(ADC_check_buf[2]<=ADC_check_buf[3])PhaseCnt|= 0x02;
				if(ADC_check_buf[4]<=ADC_check_buf[5])PhaseCnt|= 0x01;	
				Initial_stage= PhaseCnt; //测试用的  看位置变量
				Flag_OverCurr=1;         //打完脉冲之后  使能硬件过流保护
#if 1
				All_Discharg();
#endif
#endif
				//Motor.PhaseCnt=PhaseCnt;
				Motor.Duty =ALIGNMENTDUTY ;
				 switch(PhaseCnt)		//此时预定位完成
				{
					case  5:  
					{
						Motor.PhaseCnt=1;
						Startup_Turn(); //强制换向
						Delay_ms(ALIGNMENTNMS);//对齐时间
						EnterDragInit();
					}
					break;
					
					case  1:   
					{
						Motor.PhaseCnt=2;
						Startup_Turn(); //强制换向
						Delay_ms(ALIGNMENTNMS);//对齐时间
						EnterDragInit();

					}
					break;

					case  3:
					{
						Motor.PhaseCnt=3;
						Startup_Turn(); //强制换向
						Delay_ms(ALIGNMENTNMS);//对齐时间
						EnterDragInit();

					}
					break;
				
					case  2:     
					{
						Motor.PhaseCnt=4;
						Startup_Turn(); //强制换向
						Delay_ms(ALIGNMENTNMS);//对齐时间
						EnterDragInit();

					}
					break;
					
					case  6:    
					{
						Motor.PhaseCnt=5;
						Startup_Turn(); //强制换向
						Delay_ms(ALIGNMENTNMS);//对齐时间
						EnterDragInit();

					}
					break;
					
					case  4:    
					{
						Motor.PhaseCnt=6;
						Startup_Turn(); //强制换向
						Delay_ms(ALIGNMENTNMS);//对齐时间
						EnterDragInit();
					}
					break;	

					default:
					{
						Startup_Turn(); //强制换向
						Delay_ms(ALIGNMENTNMS);//对齐时间
						MotorStop();

					}
					break;
				}
				pos_check_stage=70;
				break;
			default:
				break;
		}
	}	
}
#else
void Align_pos_check_proc(void)
{
	Motor.Duty = BeforDragDuty;
	Motor.PhaseCnt=1;
	Startup_Turn(); //强制换向
	Delay_ms(BeforDragTimes);//对齐时间
	//Motor.PhaseCnt = 2;
	// Startup_Turn(); //强制换向
	// Delay_ms(BeforDragTimes);//对齐时间
	Motor.PhaseCnt++;
	// All_Discharg();
	// Delay_ms(200);//对齐时间
	EnterDragInit();
}
#endif

#endif
