#include "Timer_ISR.h"
#include "Analog.h"
#include "BLDC.h"
#include "PI_Cale.h"
#include "Timer.h"
#include "User_Config.h"


uint16_t ADCIntProtectCnt=0;
uint16_t TuneDutyRatioCnt = 0;

// static uint16_t curr_test0[5];
// static uint16_t curr_test1[5];
// static uint16_t curr_test2[5];
// static uint16_t curr_test3[5];
// static uint16_t curr_test4[5];
// static uint16_t curr_test5[5];
// static uint16_t curr_test[6][18];
// uint16_t i = 0;
// uint16_t j = 0;

/****************************************************************s*************
 �� �� ��  : TIM2_IRQHandler
 ��������  : TIM2�ж�   60ms
 �������  : ��
 �������  : void
*****************************************************************************/
void  TIM2_IRQHandler(void)
{
	
	if( TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET ) //�Ƿ����ж�
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
    
}
/*****************************************************************************
 �� �� ��  : TIM3_IRQHandler
 ��������  : TIM3�ж�   ����  100us
 �������  : ��
 �������  : void
*****************************************************************************/
void  TIM3_IRQHandler(void)
{
	if((Flag_adc==1)||(Flag_Charger==1))
	{
		Charger_Time++;
	}
	if(sysflags.Angle_Mask==1)
	{
		sysflags.Angle_Mask=0;
		TIM_Cmd(TIM3, DISABLE);   //ʧ�ܶ�ʱ��	 ��һ��30���ٿ���
	}		 
	if(sysflags.ChangePhase==1)
	{
		sysflags.ChangePhase=0;
		Startup_Turn();
		Motor.step_counter ++;	// ��¼50ms�ڵĻ������
		Sysvariable.ChangeCount ++;	//��¼�ܵĻ������
		TIM3->ARR=Mask_TIME;	//������ʱ
		TIM3->CNT = 0; 	
		sysflags.Angle_Mask=1;		 
	}
	// MotorTimerDecrease();
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);	   
}	
/*****************************************************************************
 �� �� ��  : TIM4_IRQHandler
 ��������  : TIM4�ж�   1ms		ת��PID
 �������  : ��
 �������  : void
*****************************************************************************/
void  TIM4_IRQHandler(void)
{
	if( TIM_GetITStatus(TIM4,TIM_IT_Update) != RESET ) //�Ƿ����ж�
	{
		Sysvariable.ChangeTime_Count++;		//����ʱ�� ms
		Sysvariable.ADCTimeCnt ++;
		Motor_Stop_Delay();
		MotorTimerDecrease();
		SpeedController();
		// TuneDutyRatioCnt ++;
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
}

#if 0
/*****************************************************************************
 �� �� ��  : TIM1_CC_IRQHandler
 ��������  : TIM1_CC�ж�   
 �������  : ��
 �������  : void
*****************************************************************************/
void TIM1_CC_IRQHandler(void)     
{
    if ( TIM_GetITStatus ( TIM1, TIM_IT_CC4 ) != RESET )              //CCR4�ж�
    {
        TIM_ClearFlag(TIM1, TIM_FLAG_CC4);
		switch(mcState)
		{
			case mcAlignment:
				Align_pos_check_proc();
#if PULSE_INJECTION
				Motor.CheckCurrent = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_4);
				if((Flag_adc==1)||(Flag_Charger==1))//����־λ���ߵ�������־λ��1
				{
					Charger_Time++;  //ʱ�����++
				}
				if(Flag_adc==1)//��������־λ��1
				{
					if(pos_idx<=5) //ADC����������
					{
						if(ADC_check_buf[pos_idx] < Motor.CheckCurrent)
						{
							ADC_check_buf[pos_idx] = Motor.CheckCurrent;	//��ȡ��ֵ����	//�˴��д�����
						}			
					}
					curr_test[pos_idx][j] = Motor.CheckCurrent;
					j++;								
				}
				
				if(Flag_Charger==1) //����־λ
				{
					curr_test[test_idx][j] = Motor.CheckCurrent;
					j++;

					if(Charger_Time>=LongPulse_Time)  //���ʱ�䵽
					{
						Charger_Time=0;  //������0
						All_Discharg();  //���ȫ���ر�
						Flag_Charger=0;  //����־λ��0
						test_idx ++;
						if(test_idx>=6)
						{
							test_idx = 0;
						}
					}
				}
				else if(Flag_adc==1) //��������־λ��1
				{
					if(Charger_Time>=ShortPulse_Time) //ʱ�������
					{
						All_Discharg();
						Charger_Time=0;
						Flag_adc=0;
						j = 0;
					}
				}
#endif
				break;
			case mcDrag:
				StartupDrag();//ǿ��
				break;		
			case mcRun:	
				BemfCheck();
				Instant_Current_Cale();
				break;			
			default:
				break;
		}
		ADCIntProtectCnt ++;
    }
}
#endif
/*****************************************************************************
 �� �� ��  : ADC1_2_IRQHandler
 ��������  : ADCע��ͨ��ת������ж�   
 �������  : ��
 �������  : void
*****************************************************************************/
void ADC1_2_IRQHandler(void)     
{
    if ( ADC_GetITStatus ( ADC1, ADC_IT_JEOC) != RESET )             
    {
		switch(mcState)
		{
			case mcAlignment:
#if 0
				Flag_Alignment = 1;
				Align_pos_check_proc();
#if PULSE_INJECTION
				Motor.CheckCurrent = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_4);
				if((Flag_adc==1)||(Flag_Charger==1))//����־λ���ߵ�������־λ��1
				{
					Charger_Time++;  //ʱ�����++
				}
				if(Flag_adc==1)//��������־λ��1
				{
					if(pos_idx<=5) //ADC����������
					{
						if(ADC_check_buf[pos_idx] < Motor.CheckCurrent)
						{
							ADC_check_buf[pos_idx] = Motor.CheckCurrent;	//��ȡ��ֵ����	//�˴��д�����
						}			
					}
					curr_test[pos_idx][j] = Motor.CheckCurrent;
					j++;								
				}
				
				if(Flag_Charger==1) //����־λ
				{
					curr_test[test_idx][j] = Motor.CheckCurrent;
					j++;

					if(Charger_Time>=LongPulse_Time)  //���ʱ�䵽
					{
						Charger_Time=0;  //������0
						All_Discharg();  //���ȫ���ر�
						Flag_Charger=0;  //����־λ��0
						test_idx ++;
						if(test_idx>=6)
						{
							test_idx = 0;
						}
					}
				}
				else if(Flag_adc==1) //��������־λ��1
				{
					if(Charger_Time>=ShortPulse_Time) //ʱ�������
					{
						All_Discharg();
						Charger_Time=0;
						Flag_adc=0;
						j = 0;
					}
				}
#endif
#endif
				break;
			case mcDrag:
				StartupDrag();//ǿ��
				break;		
			case mcRun:	
				BemfCheck();
				Instant_Current_Cale();
				break;		
			case mcDec:	
				BemfCheck();
			break;		
			default:
				break;
		}
		ADCIntProtectCnt ++;
		ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC);
    }
}

/*****************************************************************************
 �� �� ��  : DMA1_Channel1_IRQHandler
 ��������  : DMA�ж�   
 �������  : ��
 �������  : void
*****************************************************************************/
#if 1
void DMA1_Channel1_IRQHandler(void)
{
	if ( DMA_GetITStatus (DMA1_IT_TC1) != RESET )             
    {
		if(mcState == mcAlignment)
		{
			if((Flag_adc == 1) && (Motor.CheckCurrent < RegularConvData_Tab[0]))
			// if(Flag_adc==1)
			{
				Motor.CheckCurrent = RegularConvData_Tab[0];
			}
			if(Flag_Charger == 1 && charger_idx > 0)
			{
				if((Charger_Time <= LongPulse_Time / 5))
				{
					if(Motor.CheckCurrent < RegularConvData_Tab[0])
					{
						Motor.CheckCurrent = RegularConvData_Tab[0];
					}
					
				}
				else
				{
					// CatchFlag = 0;
					Motor.CheckCurrent = 0;
				}
			}			
			Flag_Alignment = 1;
#if 0
		Align_pos_check_proc();
#if PULSE_INJECTION
		Motor.CheckCurrent = RegularConvData_Tab[0];
		if((Flag_adc==1)||(Flag_Charger==1))//����־λ���ߵ�������־λ��1
		{
			Charger_Time++;  //ʱ�����++
		}
		if(Flag_adc==1)//��������־λ��1
		{
			if(pos_idx<=5) //ADC����������
			{
				if(ADC_check_buf[pos_idx] < Motor.CheckCurrent)
				{
					ADC_check_buf[pos_idx] = Motor.CheckCurrent;	//��ȡ��ֵ����	//�˴��д�����
				}			
			}
			curr_test[pos_idx][j] = Motor.CheckCurrent;
			j++;								
		}
		
		if(Flag_Charger==1) //����־λ
		{
			curr_test[test_idx][j] = Motor.CheckCurrent;
			j++;

			if(Charger_Time>=LongPulse_Time)  //���ʱ�䵽
			{
				Charger_Time=0;  //������0
				All_Discharg();  //���ȫ���ر�
				Flag_Charger=0;  //����־λ��0
				test_idx ++;
				if(test_idx>=6)
				{
					test_idx = 0;
				}
			}
		}
		else if(Flag_adc==1) //��������־λ��1
		{
			if(Charger_Time>=ShortPulse_Time) //ʱ�������
			{
				All_Discharg();
				Charger_Time=0;
				Flag_adc=0;
				j = 0;
			}
		}
#endif
#endif
		}
		DMA_ClearITPendingBit(DMA1_IT_TC1);
	}
	
}
#endif
