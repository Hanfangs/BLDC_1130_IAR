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
// static uint16_t curr_test[6][15];
// uint16_t i = 0;
// uint16_t j = 0;

/****************************************************************s*************
 �� �� ��  : TIM3_IRQHandler
 ��������  : TIM2�ж�   
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
 ��������  : TIM3�ж�   ����
 �������  : ��
 �������  : void
*****************************************************************************/
void  TIM3_IRQHandler(void)
{
	if(sysflags.Angle_Mask==1)
	{
		sysflags.Angle_Mask=0;
		TIM_Cmd(TIM3, DISABLE);   //ʧ�ܶ�ʱ��	 ��һ��30���ٿ���
	}		 
	if(sysflags.ChangePhase==1)
	{
		sysflags.ChangePhase=0;
		Startup_Turn();//
		TIM3->ARR=Mask_TIME;	//������ʱ
		TIM3->CNT = 0; 	
		sysflags.Angle_Mask=1;		 
	}
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);	   
}	
/*****************************************************************************
 �� �� ��  : TIM4_IRQHandler
 ��������  : TIM4�ж�   1ms
 �������  : ��
 �������  : void
*****************************************************************************/
void  TIM4_IRQHandler(void)
{
	if( TIM_GetITStatus(TIM4,TIM_IT_Update) != RESET ) //�Ƿ����ж�
	{
		Sysvariable.ChangeTime_Count++;		//����ʱ�� ms
		Sysvariable.ADCTimeCnt ++;
		// if(mcState == mcRun)
		// {
		// 	TuneDutyRatioCnt ++;	
		// }
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
}

/****************************************************************s*************
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
        if(mcState == mcRun)
		{
			BemfCheck();
			// TuneDutyRatioCnt ++;
		}
    }
}
/*****************************************************************************
 �� �� ��  : DMA1_Channel1_IRQHandler
 ��������  : DMA�ж�   �������
 �������  : ��
 �������  : void
*****************************************************************************/
void DMA1_Channel1_IRQHandler(void)
{

	switch(mcState)
	{
		case mcAlignment:
			Align_pos_check_proc();//��ʼλ�ü�����

#if PULSE_INJECTION
			if((Flag_adc==1)||(Flag_Charger==1))//����־λ���ߵ�������־λ��1
			{
				Charger_Time++;  //ʱ�����++

			}

			if(Flag_adc==1)//��������־λ��1
			{
				if(pos_idx<=5) //ADC����������
				{

					if(ADC_check_buf[pos_idx] < RegularConvData_Tab[4])
					{
						ADC_check_buf[pos_idx] = RegularConvData_Tab[4];//��ȡ��ֵ����	//�˴��д�����
					}			
					// curr_test[i][j] = RegularConvData_Tab[4];
					// j++;
				}
				if(pos_idx == 0)
				{
					curr_test[0][j] = RegularConvData_Tab[4];
					j++;
					// curr_test0[j] = RegularConvData_Tab[4];
					// j++;
				}
				if(pos_idx == 1)
				{
					curr_test[1][j] = RegularConvData_Tab[4];
					j++;					
					// curr_test1[j] = RegularConvData_Tab[4];
					// j++;
				}
				if(pos_idx == 2)
				{
					curr_test[2][j] = RegularConvData_Tab[4];
					j++;				
					// curr_test2[j] = RegularConvData_Tab[4];
					// j++;
				}
				if(pos_idx == 3)
				{
					curr_test[3][j] = RegularConvData_Tab[4];
					j++;
					// curr_test3[j] = RegularConvData_Tab[4];
					// j++;
				}
				if(pos_idx == 4)
				{
					curr_test[4][j] = RegularConvData_Tab[4];
					j++;
					// curr_test4[j] = RegularConvData_Tab[4];
					// j++;
				}
				if(pos_idx == 5)
				{
					curr_test[5][j] = RegularConvData_Tab[4];
					j++;					
					// curr_test5[j] = RegularConvData_Tab[4];
					// j++;
				}				
			}
			
			if(Flag_Charger==1) //����־λ
			{

				curr_test[test_idx][j] = RegularConvData_Tab[4];
				j++;

				if(Charger_Time>=LongPulse_Time)  //���ʱ�䵽
				{
					Charger_Time=0;  //������0
					All_Discharg();  //���ȫ���ر�
					Flag_Charger=0;  //����־λ��0
					test_idx ++;
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
			//BemfCheck();	//���綯�Ƽ�� 
		//JudgeErrorCommutation();
			break;			
		default:
			break;
	}
	
	if(mcState==mcRun)
	{	
		TuneDutyRatioCnt ++;
	}
	ADCIntProtectCnt ++;
 // GPIOB->ODR ^= GPIO_Pin_4;
	Instant_Current_Cale();
	DMA_ClearITPendingBit(DMA1_IT_TC1);
	
}
