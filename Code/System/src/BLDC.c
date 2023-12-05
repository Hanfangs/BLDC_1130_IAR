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
uint8_t  Flag_adc=0;
uint8_t  Charger_Time=0;
uint8_t  Flag_Charger=0;
uint8_t  Flag_OverCurr=0;
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
	
	UserRequireSpeed = 0;	//
	PID_Speed.Purpose = INIT_PURPOSE;
	Mask_TIME=Low_DutyMask_Time;
	Filter_Count=Delay_Filter;
	
	Flag_Charger=0;
	Flag_adc=0;
	Flag_OverCurr=0;
	pos_idx=0;
	pos_check_stage=0;
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable); //ʹ���Ϲ�
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
	 
	GPIO_SetBits(GPIOB, U_Mos_L_Pin);  //�¹ܽ�ֹ
	GPIO_SetBits(GPIOA, W_Mos_L_Pin);
	GPIO_SetBits(GPIOB, V_Mos_L_Pin); 
	 
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
	Sysvariable.ADCTimeCnt = 0;
	Sysvariable.DragTime = RAMP_TIM_STA;
	mcState =  mcDrag; //��ʼ�����   �������
	TIM_Cmd(TIM3, ENABLE);			
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
	sysflags.ChangePhase=0;
	sysflags.Angle_Mask=0;
	TuneDutyRatioCnt=0;
	// Motor.PhaseCnt++;
	//Startup_Turn(); //ǿ�ƻ���
	mcState = mcRun;
	
}
/*****************************************************************************
 �� �� ��  : StartupDrag
 ��������  : ���� ���� 
 �������  : ��
 �������  : void
*****************************************************************************/
 void StartupDrag(void)
{	
	Sysvariable.ADCTimeCnt ++;
	if(Sysvariable.ADCTimeCnt >= Sysvariable.DragTime)		//DragTime��ʼ����ʱ�� = 190,����ʱ����ڲ���ʱ��ʱ����ʼ�ݼ�
	{
		Sysvariable.ADCTimeCnt = 0;
		Motor.Duty += RAMP_DUTY_INC;
		Sysvariable.DragTime -= (Sysvariable.DragTime / RAMP_TIM_STEP) + 1;		//һ��ʼ��  190-[(190/9)+1]=168
		if(Sysvariable.DragTime < RAMP_TIM_END)				// 190��168��149��...20��ע��˴��ݼ�����Ӧ�ð��յ�����������
		{
			Sysvariable.DragTime = RAMP_TIM_END;		
			EnterRunInit();		//��ʱǿ������ˣ��л�Ϊ�����Ƽ��
			return;

	   //}
		}
		Motor.PhaseCnt++;		//����һ�Σ�����һ��
		Sysvariable.DelayTime30=TIM3->CNT; //�����ʱ��������0��һ�λ����ʱ�䣬����һ��������ʱ����
		Startup_Turn(); //����
		TIM3->CNT = 0;
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
				MOS_Q15PWM();//UV	�����Ϲ�ΪPWM
			//ADC1->CHSELR=ADC_CHSELR_CONFIG_W;		//ͨ��ѡ�񣬲����ǵ�ͨ�࣬��W��
			}
			break;
			
			case  2:   
			{
				MOS_Q16PWM();//UW
			//ADC1->CHSELR=ADC_CHSELR_CONFIG_V;
			}
			break;

			case  3:
			{
		  		MOS_Q26PWM();// VW
			//ADC1->CHSELR=ADC_CHSELR_CONFIG_U;
			}
			break;
		
			case  4:     
			{
				MOS_Q24PWM();//VU
			//ADC1->CHSELR=ADC_CHSELR_CONFIG_W;
			}
			break;
			
			case  5:    
			{
				MOS_Q34PWM();//WU
			//ADC1->CHSELR=ADC_CHSELR_CONFIG_V;
			}
			break;
			
			case  6:    
			{
				MOS_Q35PWM();//WV
			//ADC1->CHSELR=ADC_CHSELR_CONFIG_U;
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
		   	break;
			
		case mcDrag:	// ǿ��				
			  break;
			
		case mcRun:	//����
			 SpeedController();		//ռ�ձȿ��ƣ��ɿ������ɱջ���Դ�����ǿ�����������Ի�һ�±ջ�
			break;
									
		case mcReset:	// �����������
			break;
			
		case mcStop:	// ���ֹͣ�������ϵ�
		  	 MotorStop();
			break;
			
		default:
			MotorStop();
			break;			
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
	
 // TIM_Cmd(TIM14, DISABLE);    //ʧ�ܶ�ʱ��	 
	TIM_Cmd(TIM3, DISABLE);    //ʧ�ܶ�ʱ��	 
	TIM_SetCounter(TIM3,0);  //���¼���
	TIM_SetCounter(TIM14,0);  //���¼���
	
	//LIN �������������  ��ֹͣʱ Ӧ�����������ƽΪ�� �������Ϊ��--ɲ��
	if(STOPMODE==BRAKEDOWN)
	{
		GPIO_ResetBits(GPIOB, U_Mos_L_Pin);
		GPIO_ResetBits(GPIOA, W_Mos_L_Pin);	
		GPIO_ResetBits(GPIOB, V_Mos_L_Pin); 
		Delay_ms(300);
		sysflags.Motor_Stop=1;
				
	////	//LIN �������������  ��ֹͣʱ Ӧ�����������ƽΪ�� �������Ϊ��--����ͣ
 	}
	if(STOPMODE==FREEDOWN)
	{
		GPIO_SetBits(GPIOB, U_Mos_L_Pin);
		GPIO_SetBits(GPIOA, W_Mos_L_Pin);
		GPIO_SetBits(GPIOB, V_Mos_L_Pin); 
		//Delay_ms(300);
		Sysvariable.Stop_Time=POWER_DELAY;
		sysflags.Motor_Stop=1;
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
 �� �� ��  : UserSpeedControl
 ��������  : ���ٿ���
 �������  : ��
 �������  : void
*****************************************************************************/
void UserSpeedControl(void)
{
	static uint8_t RheostatCnt0=0;  //���پֲ�����
  static uint8_t RheostatCnt1=0;  //���پֲ�����
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
	else  //����
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
/*****************************************************************************
 �� �� ��  : Charger
 ��������  : ���
 �������  : ��
 �������  : void
*****************************************************************************/
void  Charger(void)
{
	TIM1->CCR1 = 0;   //������   
	TIM1->CCR2 = 0;   //   
	TIM1->CCR3 = 0;   //   
	
	GPIOB->BRR = U_Mos_L_Pin|V_Mos_L_Pin;  //UV���ſ�
	GPIOA->BRR =   W_Mos_L_Pin ;  //w�¹ܿ�
	Flag_Charger=1;  //����־λ

}
/*****************************************************************************
 �� �� ��  : All_Discharg
 ��������  : ȫ���ر�
 �������  : ��
 �������  : void
*****************************************************************************/
void  All_Discharg(void)
{
	TIM1->CCR1=0;
	TIM1->CCR2=0;
	TIM1->CCR3=0;
	GPIOB->BSRR = U_Mos_L_Pin|V_Mos_L_Pin;  //UV���Ź�
	GPIOA->BSRR =   W_Mos_L_Pin ;  //w�¹ܹ�
}
/*****************************************************************************
 �� �� ��  : UV_W_phase_inject
 ��������  : UV_Wע��
 �������  : ��
 �������  : void
*****************************************************************************/
void  UV_W_phase_inject(void)
{ 

	TIM1->CCR3 = Lock_Duty;	  //	 U��		
	TIM1->CCR2 = Lock_Duty;   //    V��
	GPIOA->BRR =   W_Mos_L_Pin ;  //w�¹ܿ�
	Flag_adc=1;  
	pos_idx=0;
}
/*****************************************************************************
 �� �� ��  : W_UV_phase_inject
 ��������  : W_UVע��
 �������  : ��
 �������  : void
*****************************************************************************/
void  W_UV_phase_inject(void)
{
	
	TIM1->CCR1 = Lock_Duty;   //W��
	GPIOB->BRR = U_Mos_L_Pin|V_Mos_L_Pin;  //���ſ�  uv 
	Flag_adc=1;  
	pos_idx=1;	
}
/*****************************************************************************
 �� �� ��  : WU_V_phase_inject
 ��������  : WU  Vע��
 �������  : ��
 �������  : void
*****************************************************************************/
void  WU_V_phase_inject(void)
{
	TIM1->CCR1 = Lock_Duty;   //W��
	TIM1->CCR3 = Lock_Duty;	 //	U��	
	GPIOB->BRR = V_Mos_L_Pin;  //���ſ�  v  
	Flag_adc=1;  
	pos_idx=2;
}
/*****************************************************************************
 �� �� ��  : V_WU_phase_inject
 ��������  : V_WU ע��
 �������  : ��
 �������  : void
*****************************************************************************/
void  V_WU_phase_inject(void)
{  
	TIM1->CCR2 = Lock_Duty;   //    V��
	GPIOA->BRR =   W_Mos_L_Pin ;  //w�¹ܿ�
	GPIOB->BRR =   U_Mos_L_Pin ;//U�¹ܿ�
	Flag_adc=1;  
	pos_idx=3;
}
/*****************************************************************************
 �� �� ��  : VW_U_phase_inject
 ��������  : VW_U ע��
 �������  : ��
 �������  : void
*****************************************************************************/
void  VW_U_phase_inject(void)
{

	 TIM1->CCR1 = Lock_Duty;   //W��
	 TIM1->CCR2 = Lock_Duty;	 //	V��	
	 GPIOB->BRR =   U_Mos_L_Pin ;//U�¹ܿ�
	 Flag_adc=1;  
	 pos_idx=4;
	
}
/*****************************************************************************
 �� �� ��  : U_VW_phase_inject
 ��������  : U_VW ע��
 �������  : ��
 �������  : void
*****************************************************************************/
void  U_VW_phase_inject(void)
{
	 
	TIM1->CCR3 = Lock_Duty;	 //	U��	
	GPIOA->BRR =   W_Mos_L_Pin ;  //w�¹ܿ�
	GPIOB->BRR =   V_Mos_L_Pin ;//V�¹ܿ�
	Flag_adc=1;  
	pos_idx=5;
}
/*****************************************************************************
 �� �� ��  : Align_pos_check_proc
 ��������  : λ�ü�����
 �������  : ��
 �������  : void
*****************************************************************************/
void Align_pos_check_proc(void)
{
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
				pos_check_stage = 1;
				break;
			
			case 1: //���
				Charger();
				pos_check_stage = 20;
				break;
			case  20://�ڶ�������ע��
				W_UV_phase_inject();
				pos_check_stage = 3;
				break; 
			
			case 3: //���	
				Charger();
				pos_check_stage =30;
				break;
			case 30://����������ע��
				WU_V_phase_inject();
				pos_check_stage =4;
				break; 
			case 4://���
				Charger();
				pos_check_stage =40;
				break;
			case 40://���ĸ�����ע��
				V_WU_phase_inject();
				pos_check_stage =5;
				break;
			case 5://���
				Charger();
				pos_check_stage =50;
				break;
			case 50://���������ע��
				VW_U_phase_inject();		   
				pos_check_stage =6;
				break;
			case 6://���
				Charger();
				pos_check_stage =60;
				break;
			case 60://����������ע��
				U_VW_phase_inject();
				pos_check_stage =7;
				break;
			case 7://
				PhaseCnt=0;
			//�����Ƚϻ�ȡλ��
				if(ADC_check_buf[0]<=ADC_check_buf[1])PhaseCnt|= 0x04;  
				if(ADC_check_buf[2]<=ADC_check_buf[3])PhaseCnt|= 0x02;
				if(ADC_check_buf[4]<=ADC_check_buf[5])PhaseCnt|= 0x01;	
				Initial_stage= PhaseCnt; //�����õ�  ��λ�ñ���
				Flag_OverCurr=1;         //��������֮��  ʹ��Ӳ����������
#if 1
				All_Discharg();
#endif
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
}
#endif
