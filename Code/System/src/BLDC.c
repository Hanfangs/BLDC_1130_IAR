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
uint8_t  charger_idx=0;
uint8_t  Flag_adc=0;
uint8_t  Charger_Time=0;
uint8_t  Flag_Charger=0;
uint8_t  Flag_OverCurr=0;
uint8_t  Flag_Alignment=0;
uint16_t  test_idx=0;
static uint16_t curr_test[6][60] = {0};
uint16_t i = 0;
uint16_t j = 0;


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
	Motor.Duty = Lock_Duty;
	
	UserRequireSpeed = 0;	//
	PID_Speed.Purpose = INIT_PURPOSE;
	Mask_TIME=Low_DutyMask_Time;
	Filter_Count=Delay_Filter;
	
	Flag_Charger=0;
	Flag_adc=0;
	Flag_OverCurr=0;
	Flag_Alignment=0;
	pos_idx=0;
	pos_check_stage=0;
	// TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable); //使能上管
	// TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
	// TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
	 
	// GPIO_SetBits(GPIOB, U_Mos_L_Pin);  //下管截止
	// GPIO_SetBits(GPIOB, W_Mos_L_Pin);
	// GPIO_SetBits(GPIOB, V_Mos_L_Pin); 
	Start_Motor();

	// Motor.Duty = Lock_Duty ;  //对齐占空比		占空比100 
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
	// Motor.motorRunTime = (u32)(M_TIMER_BASE * Motor_Run_Time + Motor_UserSpeed * 5);
	Motor.motorRunTime = (u32)(M_TIMER_BASE * Motor_Run_Time);	//Motor_Run_Time个10ms

	//位移计算
	Sysvariable.ChangeCount_Dec = (uint32_t)(Motor_UserSpeed * Motor_Run_Time * 24 / 6000 + 0.5);
	Sysvariable.ChangeCount_Stop = (uint32_t)(Sysvariable.ChangeCount_Dec + (Motor_UserSpeed/6) * 0.9 - 18 + 0.5);
	if (Sysvariable.ChangeCount_Stop < Sysvariable.ChangeCount_Dec)
	{
		Sysvariable.ChangeCount_Stop = Sysvariable.ChangeCount_Dec;
	}
	// Motor.PhaseCnt++;
	// if(Motor.PhaseCnt > 6) {
	// 	Motor.PhaseCnt = 1;
	// }
	// if(Motor.PhaseCnt < 1) {
	// 	Motor.PhaseCnt = 6;
	// } 	
	// Startup_Turn();
          
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
	Sysvariable.CorrectionTime = 0;
	sysflags.ChangePhase=0;
	sysflags.Angle_Mask=0;
	sysflags.SwapFlag = 0;
	TuneDutyRatioCnt=0;
	Sysvariable.SpeedTime = Sysvariable.DelayTime30 * 2;

	// Motor.PhaseCnt++;
	// if(Motor.PhaseCnt > 6) {
	// 	Motor.PhaseCnt = 1;
	// }
	// if(Motor.PhaseCnt < 1) {
	// 	Motor.PhaseCnt = 6;
	// } 
	// Startup_Turn(); //强制换向
#if SHF_TEST_SPEED
	IncPIDInit();
#endif
	mcState = mcRun;
	
}
/*****************************************************************************
 函 数 名  : StartupDrag
 功能描述  : 启动 加速 
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
#if 0
 void StartupDrag(void)
{	
	//Sysvariable.ADCTimeCnt ++;
	static uint16_t idx;
	static uint16_t DragTimeArray[60];
	static uint16_t DutyArray[60];
	static uint16_t U_Vol[50];
	static uint16_t V_Vol[50];
	static uint16_t W_Vol[50];
	static uint16_t U_Bus[50];
	static uint16_t TimeLeft;
	if(Sysvariable.ADCTimeCnt >= Sysvariable.DragTime)		//拖动时间到了
	{
		Sysvariable.ADCTimeCnt = 0;
		Motor.Duty += RAMP_DUTY_INC;
		Sysvariable.DragTime -= (uint16_t)((Sysvariable.DragTime / RAMP_TIM_STEP) + 1);		
		if(Sysvariable.DragTime < RAMP_TIM_END)				
		{
			Sysvariable.DragTime = RAMP_TIM_END;
			TimeLeft = Sysvariable.ChangeTime_Count;
			EnterRunInit();		//此时强拖完成了，切换为反电势检测
			return;
		}
#if 1
		DragTimeArray[idx] = Sysvariable.DragTime;
		DutyArray[idx] = Motor.Duty;
		U_Vol[idx] = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
		V_Vol[idx] = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_2);
		W_Vol[idx] = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_3);
		U_Bus[idx] = RegularConvData_Tab[0];
		idx++;
#endif
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
		ChangeTimeArray[idx] = Sysvariable.ChangeTime_Count;
		U_Vol[idx] = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
		idx++;
		Sysvariable.ChangeCount++;		//换相次数
		Change_Voltage();
		Motor.PhaseCnt++;
		Sysvariable.DelayTime30=TIM2->CNT;
		TIM2->CNT = 0;
		Motor.ActualSpeed = SPEEDFACTOR / Sysvariable.DelayTime30;
		Sysvariable.DelayTime30=Sysvariable.DelayTime30/2;
		Startup_Turn(); //换向
#if EMPTY_LOAD
		if(Sysvariable.ChangeCount > 45)
#else
		if(Sysvariable.ChangeCount >= Motor_Start_ChangeCount)
#endif
		{
			// Sysvariable.ChangeCount = 0;
			All_Discharg();		//后面记得删，调试用
			EnterRunInit();
			idx = 0;
			return;
		} 
	}

}
#if EMPTY_LOAD
void Change_Voltage(void)		//不太对，因为每次换相时才会进入一次这个函数，所以不需要给区间
{
	if(Sysvariable.ChangeTime_Count >= 17 && Sysvariable.ChangeTime_Count < 70)
	{
		Motor.Duty = PWM_ARR * 144 / 1000;		//14.4%
	}
	if(Sysvariable.ChangeTime_Count >= 70 && Sysvariable.ChangeTime_Count < 118)
	{
		Motor.Duty = PWM_ARR * 152 / 1000;		//15.2%
	}
	if(Sysvariable.ChangeTime_Count >= 118 && Sysvariable.ChangeTime_Count < 174)
	{
		Motor.Duty = PWM_ARR * 162 / 1000;		//16.2%
	}
	if(Sysvariable.ChangeTime_Count >= 174 && Sysvariable.ChangeTime_Count < 226)
	{
		Motor.Duty = PWM_ARR * 178 / 1000;		//17.8%
	}
	if(Sysvariable.ChangeTime_Count >= 226 && Sysvariable.ChangeTime_Count < 271)
	{
		Motor.Duty = PWM_ARR * 199 / 1000;		//19.9%
	}
	if(Sysvariable.ChangeTime_Count >= 271 && Sysvariable.ChangeTime_Count < 317)
	{
		Motor.Duty = PWM_ARR * 232 / 1000;		//23.2%
	}
	if(Sysvariable.ChangeTime_Count >= 317 && Sysvariable.ChangeTime_Count < 370)
	{
		Motor.Duty = PWM_ARR * 270 / 1000;		//27.0%
	}
	if(Sysvariable.ChangeTime_Count >= 370 && Sysvariable.ChangeTime_Count < 416)
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
}
#else
void Change_Voltage(void)		//不太对，因为每次换相时才会进入一次这个函数，所以不需要给区间
{
	if(Sysvariable.ChangeTime_Count >= 0 && Sysvariable.ChangeTime_Count < 52)
	{
		Motor.Duty = PWM_ARR * 150 / 1000;		//14.4%
	}
	if(Sysvariable.ChangeTime_Count >= 52 && Sysvariable.ChangeTime_Count < 105)
	{
		Motor.Duty = PWM_ARR * 176 / 1000;		//14.4%
	}
	if(Sysvariable.ChangeTime_Count >= 105 && Sysvariable.ChangeTime_Count < 149)
	{
		Motor.Duty = PWM_ARR * 202 / 1000;		//15.2%
	}
	if(Sysvariable.ChangeTime_Count >= 149 && Sysvariable.ChangeTime_Count < 197)
	{
		Motor.Duty = PWM_ARR * 235 / 1000;		//16.2%
	}
	if(Sysvariable.ChangeTime_Count >= 197 && Sysvariable.ChangeTime_Count < 252)
	{
		Motor.Duty = PWM_ARR * 273 / 1000;		//17.8%
	}
	if(Sysvariable.ChangeTime_Count >= 252 && Sysvariable.ChangeTime_Count < 297)
	{
		Motor.Duty = PWM_ARR * 304 / 1000;		//19.9%
	}
	if(Sysvariable.ChangeTime_Count >= 297 && Sysvariable.ChangeTime_Count < 348)
	{
		Motor.Duty = PWM_ARR * 336 / 1000;		//23.2%
	}
	if(Sysvariable.ChangeTime_Count >= 348 && Sysvariable.ChangeTime_Count < 395)
	{
		Motor.Duty = PWM_ARR * 348 / 1000;		//27.0%
	}
}
#endif
#endif

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
			if(Flag_Alignment == 1)
			{
				Flag_Alignment = 0;
				Align_pos_check_proc();
			}
		   	break;
			
		case mcDrag:	// 强拖				
			  break;
			
		case mcRun:	//运行
			AccSpeedControl();
			// SpeedController();		//这里应该放到定时器中断中，主程序中应该是控制目标转速
			break;

		case mcDec:
			DecSpeedControl();
			break;
			
		case mcStop:	// 电机停止
		  	MotorStop();
			break;

		case mcStopNext:	// 采用三下管导通刹车模式
			MotorStopNext();
			break;

		case mcStopNext2:	// 采用三下管导通刹车模式
			MotorStopNext2();
			break;	
			
		default:
			MotorStop();
			break;			
	}
}
/*****************************************************************************
 函 数 名  : AccSpeedControl
 功能描述  : 加速调速控制
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void AccSpeedControl(void)
{
	// if((0 == gs_Motor_Param.dwMotorRunTimer) || (g_Motor_Hall_Count >= g_Motor_Hall_Count_Dec) )//运行时间结束
	// if((Motor.motorRunTime == 0) || Sysvariable.ChangeCount >= Sysvariable.ChangeCount_Dec)	//运行时间结束
	
	if(Motor.motorRunTime == 0)
	{
		static u16 testCount;
		testCount = (u16)Sysvariable.ChangeCount;
		mcState = mcDec;
		Motor.motorRunTime = Motor_Dec_Time;               ////这里是150ms
	} 
	else
	{
		if((Motor.motor_speed + MOTOR_ACC_DELTA_SPEED) < Motor_UserSpeed)
		{
			Motor.motor_speed += MOTOR_ACC_DELTA_SPEED;                       ////每毫秒增加5         这个加的挺快的  1秒钟10转的话，1毫秒就是0.01转
		}
		else
		{
			Motor.motor_speed = Motor_UserSpeed;
		}
		
		if(Motor.motor_speed > Motor_MAX_SPEED)
		{
			Motor.motor_speed = Motor_MAX_SPEED;
		}				
		bldc_pid.SetPoint = Motor.motor_speed;  
	}
}

/*****************************************************************************
 函 数 名  : DecSpeedControl
 功能描述  : 减速调速控制
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void DecSpeedControl(void)
{
	bldc_pid.Proportion = P_DATA_ACC;              //比例常数 Proportional Const
	bldc_pid.Integral   = I_DATA_ACC;                //积分常数  Integral Const
	// if((gs_Motor_Param.dnMotor_NowSpeed <= MOTOR_MIN_SPEED) || (0 == gs_Motor_Param.dwMotorRunTimer) || (g_Motor_Hall_Count >= g_Motor_Hall_Count_Stop))
	// if((Motor.ActualSpeed <= Motor_MIN_SPEED) || (Motor.motorRunTime == 0))	//减速时间到了
	// if((Motor.motorRunTime == 0) || (Motor.ActualSpeed <= Motor_MIN_SPEED) || (Sysvariable.ChangeCount >= Sysvariable.ChangeCount_Stop))
	if(Sysvariable.ChangeCount >= Sysvariable.ChangeCount_Dec)
	{
		Motor.motor_speed = Motor_MIN_SPEED;
		mcState = mcStop;
	}
	else
	{
		if((Motor.motor_speed - MOTOR_DEC_DELTA_SPEED) >= Motor_MIN_SPEED)
		{
			Motor.motor_speed -= MOTOR_DEC_DELTA_SPEED;            
		}
		else
		{
			Motor.motor_speed = Motor_MIN_SPEED;
		}		
		bldc_pid.SetPoint = Motor.motor_speed;
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
	Motor.Duty = 0;
	Motor.PhaseCnt = 0;
	TuneDutyRatioCnt=0;
	Motor_PWM_IDLE();
	Set_Motor_Stop_Delay(5);//设置停机延时5ms
	mcState = mcStopNext;
}
void MotorStopNext(void)
{
	if(Sysvariable.StopDelayTime == 0)
	{
		Motor_PWM_IDLE();
		mcState = mcStopNext2;
		Set_Motor_Stop_Delay(100);//设置停机延时5ms
	}
}
void MotorStopNext2(void)
{
	if(Sysvariable.StopDelayTime == 0)
	{
		// Motor_PWM_IDLE();
		All_Discharg();
	}
}

/**************设置停机延时********************************/
void Set_Motor_Stop_Delay(uint32_t delay_time) 
{
    Sysvariable.StopDelayTime = delay_time;
}
/**************电机停机延时********************************/
void Motor_Stop_Delay(void)
{
    if(Sysvariable.StopDelayTime > 0)     ///在1ms定时器中调用它
    {
        Sysvariable.StopDelayTime--;
    }
}

void MotorTimerDecrease(void)
{
	if(Motor.motorRunTime > 0)
	{
		Motor.motorRunTime--;
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
		TIM_SetCompare4(BLDC_TIMx, Motor.Duty / 2); 
}
//------------------------------------------------------------------------------------------------------
void  Charger(void)			//下桥全开，上桥全关
{
    // /*  Channel1 configuration */
    // TIM_CCxCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCx_Disable);        
    // TIM_SetCompare1(BLDC_TIMx,PWM_ARR);
    // TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCxN_Enable);
	// /*  Channel2 configuration */
	// TIM_CCxCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCx_Disable);        
    // TIM_SetCompare2(BLDC_TIMx,PWM_ARR);
    // TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCxN_Enable);
	// /*  Channel3 configuration */
	// TIM_CCxCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCx_Disable);        
    // TIM_SetCompare3(BLDC_TIMx,PWM_ARR);
    // TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCxN_Enable);
	TIM_CCxCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCx_Disable);        
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCxN_Disable);      
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCx_Disable);
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCxN_Disable);
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCx_Disable);
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCxN_Disable);

	Flag_Charger=1;  //充电标志位

}
void  All_Discharg(void)	
{
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCx_Disable);        
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCxN_Disable);      
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCx_Disable);
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCxN_Disable);
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCx_Disable);
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCxN_Disable);
}
void Motor_PWM_IDLE(void)
{
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCx_Disable);
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCxN_Disable);
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCx_Disable);
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCxN_Disable);
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCx_Disable);
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCxN_Disable);
    TIM_ForcedOC1Config(BLDC_TIMx,TIM_ForcedAction_InActive);        //        强制低
    TIM_ForcedOC2Config(BLDC_TIMx,TIM_ForcedAction_InActive);
    TIM_ForcedOC3Config(BLDC_TIMx,TIM_ForcedAction_InActive);
    TIM_OC1NPolarityConfig(BLDC_TIMx,TIM_OCNPolarity_High);          ////停机时，这个极性配置是高电平有效  也就是 CC1NP = 0;    TIM_OCNPolarity_High=0x0
    TIM_OC2NPolarityConfig(BLDC_TIMx,TIM_OCNPolarity_High);         //// 按照定义：OCxN=CCxNP，OCxN_EN=0  此时，这个N的输出为低电平，而低电平，正好是下管导通
    TIM_OC3NPolarityConfig(BLDC_TIMx,TIM_OCNPolarity_High);

}
void  UV_W_phase_inject(void)	//110
{ 

    /*  Channel1 configuration */
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCxN_Disable);
    TIM_SetCompare1(BLDC_TIMx,Lock_Duty);
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCx_Enable);
	/*  Channel2 configuration */
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCxN_Disable);
    TIM_SetCompare2(BLDC_TIMx,Lock_Duty);
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCx_Enable);
	/*  Channel3 configuration */
	TIM_CCxCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCx_Disable);        
    TIM_SetCompare3(BLDC_TIMx,PWM_ARR);
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCxN_Enable);

	Flag_adc=1;  
	pos_idx=0;
}
void  W_UV_phase_inject(void)	//001
{
	
	/*  Channel3 configuration */
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCxN_Disable);
    TIM_SetCompare3(BLDC_TIMx,Lock_Duty);
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCx_Enable);
    /*  Channel1 configuration */
	TIM_CCxCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCx_Disable);        
    TIM_SetCompare1(BLDC_TIMx,PWM_ARR);
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCxN_Enable);
	/*  Channel2 configuration */
	TIM_CCxCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCx_Disable);        
    TIM_SetCompare2(BLDC_TIMx,PWM_ARR);
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCxN_Enable);
	 
	Flag_adc=1;  
	pos_idx=1;	
}
void  WU_V_phase_inject(void)	//101
{
    /*  Channel1 configuration */
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCxN_Disable);
    TIM_SetCompare1(BLDC_TIMx,Lock_Duty);
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCx_Enable);
	/*  Channel3 configuration */
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCxN_Disable);
    TIM_SetCompare3(BLDC_TIMx,Lock_Duty);
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCx_Enable);
	/*  Channel2 configuration */
	TIM_CCxCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCx_Disable);        
    TIM_SetCompare2(BLDC_TIMx,PWM_ARR);
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCxN_Enable);

	Flag_adc=1;  
	pos_idx=2;
}
void  V_WU_phase_inject(void)	//010
{  
	/*  Channel2 configuration */
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCxN_Disable);
    TIM_SetCompare2(BLDC_TIMx,Lock_Duty);
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCx_Enable);
    /*  Channel1 configuration */
	TIM_CCxCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCx_Disable);        
    TIM_SetCompare1(BLDC_TIMx,PWM_ARR);
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCxN_Enable);
	/*  Channel3 configuration */
	TIM_CCxCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCx_Disable);        
    TIM_SetCompare3(BLDC_TIMx,PWM_ARR);
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCxN_Enable);

	Flag_adc=1;  
	pos_idx=3;
}
void  VW_U_phase_inject(void)	//011
{
    /*  Channel2 configuration */
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCxN_Disable);
    TIM_SetCompare2(BLDC_TIMx,Lock_Duty);
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCx_Enable);
	/*  Channel3 configuration */
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCxN_Disable);
    TIM_SetCompare3(BLDC_TIMx,Lock_Duty);
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCx_Enable);
	/*  Channel1 configuration */
	TIM_CCxCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCx_Disable);        
    TIM_SetCompare1(BLDC_TIMx,PWM_ARR);
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCxN_Enable);

	 Flag_adc=1;  
	 pos_idx=4;
}
void  U_VW_phase_inject(void)	//100
{
	 
	/*  Channel1 configuration */
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCxN_Disable);
    TIM_SetCompare1(BLDC_TIMx,Lock_Duty);
    TIM_CCxCmd(BLDC_TIMx,TIM_Channel_1,TIM_CCx_Enable);
    /*  Channel2 configuration */
	TIM_CCxCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCx_Disable);        
    TIM_SetCompare2(BLDC_TIMx,PWM_ARR);
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_2,TIM_CCxN_Enable);
	/*  Channel3 configuration */
	TIM_CCxCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCx_Disable);        
    TIM_SetCompare3(BLDC_TIMx,PWM_ARR);
    TIM_CCxNCmd(BLDC_TIMx,TIM_Channel_3,TIM_CCxN_Enable);

	Flag_adc=1;  
	pos_idx=5;
}
//------------------------------------------------------------------------------------------------------


/*****************************************************************************
 函 数 名  : Align_pos_check_proc
 功能描述  : 位置检测过程
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
#if PULSE_INJECTION
void Align_pos_check_proc(void)
{
#if SHF_TEST_START
	if((Flag_adc==0)&&(Flag_Charger==0))  //电流采集并且充电完成
	{
		switch(pos_check_stage)//
		{
			case 0: //先充电	
				Charger();
				pos_check_stage = 10;
				break;
			case 10://第一个脉冲注入
				UV_W_phase_inject();	//A+B+C- 0
				pos_check_stage = 1;
				break;
			case 1: //充电
				charger_idx = 1;
				Charger();
				pos_check_stage = 20;
				break;
			case 20://第二个脉冲注入
				W_UV_phase_inject();	//C+A-B- 1
				pos_check_stage = 3;
				break; 
			case 3: //充电
				charger_idx = 2;	
				Charger();
				pos_check_stage =30;
				break;

			case 30://第三个脉冲注入
				if(ADC_check_buf[0] <= ADC_check_buf[1])
				{
					WU_V_phase_inject();	//A+C+B- 2
					pos_check_stage = 4;
				}
				else{
					V_WU_phase_inject();	//B+A-C- 3
					pos_check_stage = 5;
				}
				break;
			case 4://充电
				charger_idx  = 3;
				Charger();
				pos_check_stage =40;
				break;
			case 40://第四个脉冲注入
				VW_U_phase_inject();	 //B+C+A- 4
				pos_check_stage = 6;
				break;

			case 5://充电
				charger_idx = 3;
				Charger();
				pos_check_stage =50;
				break;
			case 50://第四个脉冲注入
				U_VW_phase_inject();	//A+B-C- 5
				pos_check_stage = 6;
				break;
				
			case 6://充电
				charger_idx = 4;
				Charger();
				pos_check_stage =7;
				break;
			case 7://
				PhaseCnt=0;
			//电流比较获取位置
				if(ADC_check_buf[0]<=ADC_check_buf[1])
				{
					if(ADC_check_buf[2]<=ADC_check_buf[4])
					{
						Motor.PhaseCnt = 6;
					}
					else
					{
						Motor.PhaseCnt = 1;
					}
				}
				else
				{
					if(ADC_check_buf[3]<=ADC_check_buf[5])
					{
						Motor.PhaseCnt = 3;
					}
					else
					{
						Motor.PhaseCnt = 4;
					}
				}

				Flag_OverCurr=1;         //打完脉冲之后  使能硬件过流保护
				All_Discharg();
				Motor.Duty = ALIGNMENTDUTY ;

				Startup_Turn(); //强制换向
				Delay_ms(BeforDragTimes);//对齐时间
				// All_Discharg();		//测试用，记得删！！
				Motor.PhaseCnt++;
				EnterDragInit();
				break;
			default:
				break;
		}
	}


#else
	if((Flag_adc==0)&&(Flag_Charger==0))  //电流采集并且充电完成
	{
		switch(pos_check_stage)//
		{
			case 0: //先充电	
				Charger();
				pos_check_stage = 10;
				break;
			case 10://第一个脉冲注入
				UV_W_phase_inject();
				// MOS_Q24PWM();		//B+A-
				pos_check_stage = 1;
				break;
			
			case 1: //充电
				charger_idx ++;
				Charger();
				pos_check_stage = 20;
				break;
			case  20://第二个脉冲注入
				W_UV_phase_inject();
				// MOS_Q15PWM();		//A+B-
				pos_check_stage = 3;
				break; 
			
			case 3: //充电
				charger_idx ++;	
				Charger();
				pos_check_stage =30;
				break;
			case 30://第三个脉冲注入
				WU_V_phase_inject();
				// MOS_Q35PWM();		//C+B-
				pos_check_stage =4;
				break; 
			case 4://充电
				charger_idx ++;
				Charger();
				pos_check_stage =40;
				break;
			case 40://第四个脉冲注入
				V_WU_phase_inject();
				// MOS_Q26PWM();		//B+C-
				pos_check_stage =5;
				break;
			case 5://充电
				charger_idx ++;
				Charger();
				pos_check_stage =50;
				break;
			case 50://第五个脉冲注入
				VW_U_phase_inject();	 
				// MOS_Q16PWM();		//A+C- 
				pos_check_stage =6;
				break;
			case 6://充电
				charger_idx ++;
				Charger();	
				pos_check_stage =60;
				break;
			case 60://第六个脉冲注入
				U_VW_phase_inject();
				// MOS_Q34PWM();		//C+A-
				pos_check_stage =7;
				break;
			case 7://第六个脉冲注入
				charger_idx ++;
				Charger();
				pos_check_stage =70;
				break;
			case 70://
				PhaseCnt=0;
			//电流比较获取位置
				if(ADC_check_buf[0]<=ADC_check_buf[1])PhaseCnt|= 0x04;  		//V4<V1		"1"靠近A+
				if(ADC_check_buf[2]<=ADC_check_buf[3])PhaseCnt|= 0x02;			//V2<V5		"1"靠近B+
				if(ADC_check_buf[4]<=ADC_check_buf[5])PhaseCnt|= 0x01;			//V6<V3		"1"靠近C+
				Initial_stage= PhaseCnt; //测试用的  看位置变量
				Flag_OverCurr=1;         //打完脉冲之后  使能硬件过流保护
				All_Discharg();

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
#endif

	// if((Flag_adc==1)||(Flag_Charger==1))//充电标志位或者电流检测标志位置1
	// {
	// 	Charger_Time++;  //时间计数++
	// }

	if(Flag_adc==1)//电流检测标志位置1
	{
		// if(pos_idx<=5) //ADC电流检测序号
		// {
		// 	// if(ADC_check_buf[pos_idx] < Motor.CheckCurrent)
		// 	// {
		// 		ADC_check_buf[pos_idx] = Motor.CheckCurrent;	//获取峰值电流	//此处有待商酌
		// 		// Motor.CheckCurrent = 0;
		// 	// }			
		// }
		// if((Charger_Time == ShortPulse_Time - 5) && (pos_idx<=5)) //ADC电流检测序号
		// CatchFlag = 1;
		// j = 0;
		// if(pos_idx<=5)
		// {
		ADC_check_buf[pos_idx] = Motor.CheckCurrent;	//获取峰值电流	//此处有待商酌
		// }
		curr_test[pos_idx][j] = Motor.CheckCurrent;
		j++;							
	} 
	
	if(Flag_Charger==1) //充电标志位
	{
		if((charger_idx > 0) && (Charger_Time <= LongPulse_Time / 5))
		{
			ADC_check_buf[pos_idx] = Motor.CheckCurrent;
			curr_test[charger_idx - 1][j] = Motor.CheckCurrent;
			j++;				
		}

		if(Charger_Time>=LongPulse_Time)  //充电时间到
		{
			Charger_Time=0;  //计数清0
			All_Discharg();  //输出全部关闭
			Flag_Charger=0;  //充电标志位清0
			j = 0;
			// test_idx ++;
			// if(test_idx>=6)
			// {
			// 	test_idx = 0;
			// }	
		}
	}
	else if(Flag_adc==1) //电流检测标志位置1
	{
		if(Charger_Time>=ShortPulse_Time) //时间计数到
		{
			All_Discharg();
			Charger_Time=0;
			Flag_adc=0;
			// j = 0;
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
