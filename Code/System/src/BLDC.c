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

//������ʼ��
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


BLDC Motor={                                                          /* ����ṹ��        */
             OPEN_LOOP_Halless,                                     /* ������Ʒ�ʽ      */
             BLDCSTOP,                                                /* ���״̬          */
             0,                                                       /* �����ٶ�          */
             0,                                                       /* ʵ���ٶ�          */
             0,
             0,                                                     /* ��ǰռ�ձ�        */
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
	 sysflags.flag_SpeedTime = 0;		//��־λ
	 sysflags.Angle_Mask=0;
	 sysflags.ChangePhase=0;
	 sysflags.Motor_Stop=0;
	 sysflags.System_Start=0;
	 Sysvariable.SpeedTimeCnt = 0;		//����
	 Sysvariable.SpeedTime = 0;
	 Sysvariable.SpeedTimeTemp = 0;
	 Sysvariable.SpeedTimeSum = 0;
	 Sysvariable.Stop_Time=0;
	
}
 /*****************************************************************************
 �� �� ��  : MotorInit
 ��������  : ��ʼ��
 �������  : ��
 �������  : void
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
	// TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable); //ʹ���Ϲ�
	// TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
	// TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
	 
	// GPIO_SetBits(GPIOB, U_Mos_L_Pin);  //�¹ܽ�ֹ
	// GPIO_SetBits(GPIOB, W_Mos_L_Pin);
	// GPIO_SetBits(GPIOB, V_Mos_L_Pin); 
	Start_Motor();

	// Motor.Duty = Lock_Duty ;  //����ռ�ձ�		ռ�ձ�100 
	mcState = mcAlignment;
	mcFault=RunNormal;
 }
 
/*****************************************************************************
 �� �� ��  : MotorAlignment
 ��������  : ���붨λ
 �������  : ��
 �������  : void
*****************************************************************************/
void MotorAlignment(void)		//û�õ�
{
	Motor.Duty =ALIGNMENTDUTY ;  //����ռ�ձ�		ռ�ձ�100
	Motor.PhaseCnt ++;   //0-1	����
	Startup_Turn(); //ǿ�ƻ���
	Delay_ms(ALIGNMENTNMS);//����ʱ��
	EnterDragInit();  //���ٳ�ʼ��
}	
/*****************************************************************************
 �� �� ��  : EnterDragInit
 ��������  : ����ǿ�ϳ�ʼ��
 �������  : ��
 �������  : void
*****************************************************************************/
 void EnterDragInit(void)
{
	Motor.Duty =ALIGNMENTDUTY ; 
	Sysvariable.ADCTimeCnt = 0;
	Sysvariable.DragTime = RAMP_TIM_STA;
	mcState =  mcDrag; //��ʼ�����   �������
	Sysvariable.ChangeTime_Count = 0;
	// Motor.motorRunTime = (u32)(M_TIMER_BASE * Motor_Run_Time + Motor_UserSpeed * 5);
	Motor.motorRunTime = (u32)(M_TIMER_BASE * Motor_Run_Time);	//Motor_Run_Time��10ms

	//λ�Ƽ���
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
 �� �� ��  : EnterRunInit
 ��������  : ����Run�ĳ�ʼ��
 �������  : ��
 �������  : void
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
	// Startup_Turn(); //ǿ�ƻ���
#if SHF_TEST_SPEED
	IncPIDInit();
#endif
	mcState = mcRun;
	
}
/*****************************************************************************
 �� �� ��  : StartupDrag
 ��������  : ���� ���� 
 �������  : ��
 �������  : void
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
	if(Sysvariable.ADCTimeCnt >= Sysvariable.DragTime)		//�϶�ʱ�䵽��
	{
		Sysvariable.ADCTimeCnt = 0;
		Motor.Duty += RAMP_DUTY_INC;
		Sysvariable.DragTime -= (uint16_t)((Sysvariable.DragTime / RAMP_TIM_STEP) + 1);		
		if(Sysvariable.DragTime < RAMP_TIM_END)				
		{
			Sysvariable.DragTime = RAMP_TIM_END;
			TimeLeft = Sysvariable.ChangeTime_Count;
			EnterRunInit();		//��ʱǿ������ˣ��л�Ϊ�����Ƽ��
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
		Motor.PhaseCnt++;		//����һ�Σ�����һ��
		Sysvariable.DelayTime30=TIM2->CNT; //�����ʱ��������0��һ�λ����ʱ�䣬����һ��������ʱ����
		Startup_Turn(); //����
		TIM2->CNT = 0;
		Sysvariable.DelayTime30=Sysvariable.DelayTime30/2;		//���������60����Ƕ�
	}
	if(Motor.Duty < RAMP_DUTY_STA)	// �������ռ�ձȵ������Сֵ		100-300
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
		Startup_Turn();		//����ϵ㿴�ܲ��ܽ���
	}
	if(Sysvariable.ChangeTime_Count == ((uint16_t)ACC_FUN(Sysvariable.ChangeCount, P1, P2, P3, P4)))	//��������
	{
		ChangeTimeArray[idx] = Sysvariable.ChangeTime_Count;
		U_Vol[idx] = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
		idx++;
		Sysvariable.ChangeCount++;		//�������
		Change_Voltage();
		Motor.PhaseCnt++;
		Sysvariable.DelayTime30=TIM2->CNT;
		TIM2->CNT = 0;
		Motor.ActualSpeed = SPEEDFACTOR / Sysvariable.DelayTime30;
		Sysvariable.DelayTime30=Sysvariable.DelayTime30/2;
		Startup_Turn(); //����
#if EMPTY_LOAD
		if(Sysvariable.ChangeCount > 45)
#else
		if(Sysvariable.ChangeCount >= Motor_Start_ChangeCount)
#endif
		{
			// Sysvariable.ChangeCount = 0;
			All_Discharg();		//����ǵ�ɾ��������
			EnterRunInit();
			idx = 0;
			return;
		} 
	}

}
#if EMPTY_LOAD
void Change_Voltage(void)		//��̫�ԣ���Ϊÿ�λ���ʱ�Ż����һ��������������Բ���Ҫ������
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
void Change_Voltage(void)		//��̫�ԣ���Ϊÿ�λ���ʱ�Ż����һ��������������Բ���Ҫ������
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
 �� �� ��  : MotorControl
 ��������  : �������
 �������  : ��
 �������  : void
*****************************************************************************/
void MotorControl(void)
{
	switch(mcState)
	{
		case mcInit:	// ��ʼ��
			MotorInit();
			Sys_Variable_Init();	
			break;
			
		case mcAlignment:	// ��λ
			if(Flag_Alignment == 1)
			{
				Flag_Alignment = 0;
				Align_pos_check_proc();
			}
		   	break;
			
		case mcDrag:	// ǿ��				
			  break;
			
		case mcRun:	//����
			AccSpeedControl();
			// SpeedController();		//����Ӧ�÷ŵ���ʱ���ж��У���������Ӧ���ǿ���Ŀ��ת��
			break;

		case mcDec:
			DecSpeedControl();
			break;
			
		case mcStop:	// ���ֹͣ
		  	MotorStop();
			break;

		case mcStopNext:	// �������¹ܵ�ͨɲ��ģʽ
			MotorStopNext();
			break;

		case mcStopNext2:	// �������¹ܵ�ͨɲ��ģʽ
			MotorStopNext2();
			break;	
			
		default:
			MotorStop();
			break;			
	}
}
/*****************************************************************************
 �� �� ��  : AccSpeedControl
 ��������  : ���ٵ��ٿ���
 �������  : ��
 �������  : void
*****************************************************************************/
void AccSpeedControl(void)
{
	// if((0 == gs_Motor_Param.dwMotorRunTimer) || (g_Motor_Hall_Count >= g_Motor_Hall_Count_Dec) )//����ʱ�����
	// if((Motor.motorRunTime == 0) || Sysvariable.ChangeCount >= Sysvariable.ChangeCount_Dec)	//����ʱ�����
	
	if(Motor.motorRunTime == 0)
	{
		static u16 testCount;
		testCount = (u16)Sysvariable.ChangeCount;
		mcState = mcDec;
		Motor.motorRunTime = Motor_Dec_Time;               ////������150ms
	} 
	else
	{
		if((Motor.motor_speed + MOTOR_ACC_DELTA_SPEED) < Motor_UserSpeed)
		{
			Motor.motor_speed += MOTOR_ACC_DELTA_SPEED;                       ////ÿ��������5         ����ӵ�ͦ���  1����10ת�Ļ���1�������0.01ת
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
 �� �� ��  : DecSpeedControl
 ��������  : ���ٵ��ٿ���
 �������  : ��
 �������  : void
*****************************************************************************/
void DecSpeedControl(void)
{
	bldc_pid.Proportion = P_DATA_ACC;              //�������� Proportional Const
	bldc_pid.Integral   = I_DATA_ACC;                //���ֳ���  Integral Const
	// if((gs_Motor_Param.dnMotor_NowSpeed <= MOTOR_MIN_SPEED) || (0 == gs_Motor_Param.dwMotorRunTimer) || (g_Motor_Hall_Count >= g_Motor_Hall_Count_Stop))
	// if((Motor.ActualSpeed <= Motor_MIN_SPEED) || (Motor.motorRunTime == 0))	//����ʱ�䵽��
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
 �� �� ��  : MotorStop
 ��������  : ���ֹͣ
 �������  : ��
 �������  : void
*****************************************************************************/
void MotorStop(void)
{
	Motor.Duty = 0;
	Motor.PhaseCnt = 0;
	TuneDutyRatioCnt=0;
	Motor_PWM_IDLE();
	Set_Motor_Stop_Delay(5);//����ͣ����ʱ5ms
	mcState = mcStopNext;
}
void MotorStopNext(void)
{
	if(Sysvariable.StopDelayTime == 0)
	{
		Motor_PWM_IDLE();
		mcState = mcStopNext2;
		Set_Motor_Stop_Delay(100);//����ͣ����ʱ5ms
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

/**************����ͣ����ʱ********************************/
void Set_Motor_Stop_Delay(uint32_t delay_time) 
{
    Sysvariable.StopDelayTime = delay_time;
}
/**************���ͣ����ʱ********************************/
void Motor_Stop_Delay(void)
{
    if(Sysvariable.StopDelayTime > 0)     ///��1ms��ʱ���е�����
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
 �� �� ��  : UserSpeedControlInit
 ��������  : ���ٿ��ƿ�ʼ
 �������  : ��
 �������  : void
*****************************************************************************/
void UserSpeedControlInit(void)
{
	static uint8_t RheostatCnt0=0;
	
 if(ADJ_MODE==DIRECT_GIVE)
 {
		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4)==0)		//�����͵�ƽΪ��
		{
			RheostatCnt0++;
			if(RheostatCnt0>100)		//��⵽100��PF_1Ϊ�͵�ƽ��ϵͳ����������PF������ʲô����
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
 �� �� ��  : Startup_Turn
 ��������  : ���� 
 �������  : ��
 �������  : void
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
			case  1:		//�˴��޻����źţ�����˳�����1-6���л���ӦΪ651342��546231
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
void  Charger(void)			//����ȫ��������ȫ��
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

	Flag_Charger=1;  //����־λ

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
    TIM_ForcedOC1Config(BLDC_TIMx,TIM_ForcedAction_InActive);        //        ǿ�Ƶ�
    TIM_ForcedOC2Config(BLDC_TIMx,TIM_ForcedAction_InActive);
    TIM_ForcedOC3Config(BLDC_TIMx,TIM_ForcedAction_InActive);
    TIM_OC1NPolarityConfig(BLDC_TIMx,TIM_OCNPolarity_High);          ////ͣ��ʱ��������������Ǹߵ�ƽ��Ч  Ҳ���� CC1NP = 0;    TIM_OCNPolarity_High=0x0
    TIM_OC2NPolarityConfig(BLDC_TIMx,TIM_OCNPolarity_High);         //// ���ն��壺OCxN=CCxNP��OCxN_EN=0  ��ʱ�����N�����Ϊ�͵�ƽ�����͵�ƽ���������¹ܵ�ͨ
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
 �� �� ��  : Align_pos_check_proc
 ��������  : λ�ü�����
 �������  : ��
 �������  : void
*****************************************************************************/
#if PULSE_INJECTION
void Align_pos_check_proc(void)
{
#if SHF_TEST_START
	if((Flag_adc==0)&&(Flag_Charger==0))  //�����ɼ����ҳ�����
	{
		switch(pos_check_stage)//
		{
			case 0: //�ȳ��	
				Charger();
				pos_check_stage = 10;
				break;
			case 10://��һ������ע��
				UV_W_phase_inject();	//A+B+C- 0
				pos_check_stage = 1;
				break;
			case 1: //���
				charger_idx = 1;
				Charger();
				pos_check_stage = 20;
				break;
			case 20://�ڶ�������ע��
				W_UV_phase_inject();	//C+A-B- 1
				pos_check_stage = 3;
				break; 
			case 3: //���
				charger_idx = 2;	
				Charger();
				pos_check_stage =30;
				break;

			case 30://����������ע��
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
			case 4://���
				charger_idx  = 3;
				Charger();
				pos_check_stage =40;
				break;
			case 40://���ĸ�����ע��
				VW_U_phase_inject();	 //B+C+A- 4
				pos_check_stage = 6;
				break;

			case 5://���
				charger_idx = 3;
				Charger();
				pos_check_stage =50;
				break;
			case 50://���ĸ�����ע��
				U_VW_phase_inject();	//A+B-C- 5
				pos_check_stage = 6;
				break;
				
			case 6://���
				charger_idx = 4;
				Charger();
				pos_check_stage =7;
				break;
			case 7://
				PhaseCnt=0;
			//�����Ƚϻ�ȡλ��
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

				Flag_OverCurr=1;         //��������֮��  ʹ��Ӳ����������
				All_Discharg();
				Motor.Duty = ALIGNMENTDUTY ;

				Startup_Turn(); //ǿ�ƻ���
				Delay_ms(BeforDragTimes);//����ʱ��
				// All_Discharg();		//�����ã��ǵ�ɾ����
				Motor.PhaseCnt++;
				EnterDragInit();
				break;
			default:
				break;
		}
	}


#else
	if((Flag_adc==0)&&(Flag_Charger==0))  //�����ɼ����ҳ�����
	{
		switch(pos_check_stage)//
		{
			case 0: //�ȳ��	
				Charger();
				pos_check_stage = 10;
				break;
			case 10://��һ������ע��
				UV_W_phase_inject();
				// MOS_Q24PWM();		//B+A-
				pos_check_stage = 1;
				break;
			
			case 1: //���
				charger_idx ++;
				Charger();
				pos_check_stage = 20;
				break;
			case  20://�ڶ�������ע��
				W_UV_phase_inject();
				// MOS_Q15PWM();		//A+B-
				pos_check_stage = 3;
				break; 
			
			case 3: //���
				charger_idx ++;	
				Charger();
				pos_check_stage =30;
				break;
			case 30://����������ע��
				WU_V_phase_inject();
				// MOS_Q35PWM();		//C+B-
				pos_check_stage =4;
				break; 
			case 4://���
				charger_idx ++;
				Charger();
				pos_check_stage =40;
				break;
			case 40://���ĸ�����ע��
				V_WU_phase_inject();
				// MOS_Q26PWM();		//B+C-
				pos_check_stage =5;
				break;
			case 5://���
				charger_idx ++;
				Charger();
				pos_check_stage =50;
				break;
			case 50://���������ע��
				VW_U_phase_inject();	 
				// MOS_Q16PWM();		//A+C- 
				pos_check_stage =6;
				break;
			case 6://���
				charger_idx ++;
				Charger();	
				pos_check_stage =60;
				break;
			case 60://����������ע��
				U_VW_phase_inject();
				// MOS_Q34PWM();		//C+A-
				pos_check_stage =7;
				break;
			case 7://����������ע��
				charger_idx ++;
				Charger();
				pos_check_stage =70;
				break;
			case 70://
				PhaseCnt=0;
			//�����Ƚϻ�ȡλ��
				if(ADC_check_buf[0]<=ADC_check_buf[1])PhaseCnt|= 0x04;  		//V4<V1		"1"����A+
				if(ADC_check_buf[2]<=ADC_check_buf[3])PhaseCnt|= 0x02;			//V2<V5		"1"����B+
				if(ADC_check_buf[4]<=ADC_check_buf[5])PhaseCnt|= 0x01;			//V6<V3		"1"����C+
				Initial_stage= PhaseCnt; //�����õ�  ��λ�ñ���
				Flag_OverCurr=1;         //��������֮��  ʹ��Ӳ����������
				All_Discharg();

				//Motor.PhaseCnt=PhaseCnt;
				Motor.Duty =ALIGNMENTDUTY ;

				switch(PhaseCnt)		//��ʱԤ��λ���
				{
					case  5:  
					{
						Motor.PhaseCnt=1;
						Startup_Turn(); //ǿ�ƻ���
						Delay_ms(ALIGNMENTNMS);//����ʱ��
						EnterDragInit();
					}
					break;
					
					case  1:   
					{
						Motor.PhaseCnt=2;
						Startup_Turn(); //ǿ�ƻ���
						Delay_ms(ALIGNMENTNMS);//����ʱ��
						EnterDragInit();

					}
					break;

					case  3:
					{
						Motor.PhaseCnt=3;
						Startup_Turn(); //ǿ�ƻ���
						Delay_ms(ALIGNMENTNMS);//����ʱ��
						EnterDragInit();

					}
					break;
				
					case  2:     
					{
						Motor.PhaseCnt=4;
						Startup_Turn(); //ǿ�ƻ���
						Delay_ms(ALIGNMENTNMS);//����ʱ��
						EnterDragInit();

					}
					break;
					
					case  6:    
					{
						Motor.PhaseCnt=5;
						Startup_Turn(); //ǿ�ƻ���
						Delay_ms(ALIGNMENTNMS);//����ʱ��
						EnterDragInit();

					}
					break;
					
					case  4:    
					{
						Motor.PhaseCnt=6;
						Startup_Turn(); //ǿ�ƻ���
						Delay_ms(ALIGNMENTNMS);//����ʱ��
						EnterDragInit();
					}
					break;	

					default:
					{
						Startup_Turn(); //ǿ�ƻ���
						Delay_ms(ALIGNMENTNMS);//����ʱ��
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

	// if((Flag_adc==1)||(Flag_Charger==1))//����־λ���ߵ�������־λ��1
	// {
	// 	Charger_Time++;  //ʱ�����++
	// }

	if(Flag_adc==1)//��������־λ��1
	{
		// if(pos_idx<=5) //ADC����������
		// {
		// 	// if(ADC_check_buf[pos_idx] < Motor.CheckCurrent)
		// 	// {
		// 		ADC_check_buf[pos_idx] = Motor.CheckCurrent;	//��ȡ��ֵ����	//�˴��д�����
		// 		// Motor.CheckCurrent = 0;
		// 	// }			
		// }
		// if((Charger_Time == ShortPulse_Time - 5) && (pos_idx<=5)) //ADC����������
		// CatchFlag = 1;
		// j = 0;
		// if(pos_idx<=5)
		// {
		ADC_check_buf[pos_idx] = Motor.CheckCurrent;	//��ȡ��ֵ����	//�˴��д�����
		// }
		curr_test[pos_idx][j] = Motor.CheckCurrent;
		j++;							
	} 
	
	if(Flag_Charger==1) //����־λ
	{
		if((charger_idx > 0) && (Charger_Time <= LongPulse_Time / 5))
		{
			ADC_check_buf[pos_idx] = Motor.CheckCurrent;
			curr_test[charger_idx - 1][j] = Motor.CheckCurrent;
			j++;				
		}

		if(Charger_Time>=LongPulse_Time)  //���ʱ�䵽
		{
			Charger_Time=0;  //������0
			All_Discharg();  //���ȫ���ر�
			Flag_Charger=0;  //����־λ��0
			j = 0;
			// test_idx ++;
			// if(test_idx>=6)
			// {
			// 	test_idx = 0;
			// }	
		}
	}
	else if(Flag_adc==1) //��������־λ��1
	{
		if(Charger_Time>=ShortPulse_Time) //ʱ�������
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
	Startup_Turn(); //ǿ�ƻ���
	Delay_ms(BeforDragTimes);//����ʱ��
	//Motor.PhaseCnt = 2;
	// Startup_Turn(); //ǿ�ƻ���
	// Delay_ms(BeforDragTimes);//����ʱ��
	Motor.PhaseCnt++;
	// All_Discharg();
	// Delay_ms(200);//����ʱ��
	EnterDragInit();
}
#endif

#endif
