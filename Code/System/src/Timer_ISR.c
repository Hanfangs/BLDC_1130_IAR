#include "Timer_ISR.h"
#include "Analog.h"
#include "BLDC.h"
#include "PI_Cale.h"
#include "Timer.h"
#include "User_Config.h"

uint16_t ADCIntProtectCnt=0;
uint16_t TuneDutyRatioCnt = 0;

/*****************************************************************************
 �� �� ��  : TIM3_IRQHandler
 ��������  : TIM3�ж�   
 �������  : ��
 �������  : void
*****************************************************************************/
void  TIM3_IRQHandler(void)
{
	
   if( TIM_GetITStatus(TIM3,TIM_IT_Update) != RESET ) //�Ƿ����ж�
	 {
		 	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	 }
    
}	
/*****************************************************************************
 �� �� ��  : TIM14_IRQHandler
 ��������  : TIM14�ж�   ����
 �������  : ��
 �������  : void
*****************************************************************************/
void  TIM14_IRQHandler(void)
{
	 if(sysflags.Angle_Mask==1)
	 {
	    sysflags.Angle_Mask=0;
      TIM_Cmd(TIM14, DISABLE);   //ʧ�ܶ�ʱ��	 
	 }		 
	 if(sysflags.ChangePhase==1)
	 {
		 	 sysflags.ChangePhase=0;
	  	 Startup_Turn();//
		 	 TIM14->ARR=Mask_TIME;
		 	 TIM14->CNT = 0;
		 	 sysflags.Angle_Mask=1;		 
	 }
	 TIM_ClearITPendingBit(TIM14, TIM_IT_Update);
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
			  if((Flag_adc==1)||(Flag_Charger==1))//����־λ���ߵ�������־λ��1
				{
			    Charger_Time++;  //ʱ�����++
				}
			  if(Flag_adc==1)//��������־λ��1
				{
					if(pos_idx<=5) //ADC����������
					{			
						ADC_check_buf[pos_idx]=RegularConvData_Tab[1];//��ȡ��ֵ����	//�˴��д�����
					}
				}
			 if(Flag_Charger==1) //����־λ
	     {
					if(Charger_Time>=LongPulse_Time)  //���ʱ�䵽
					{
						Charger_Time=0;  //������0
						All_Discharg();  //���ȫ���ر�
						Flag_Charger=0;  //����־λ��0
					}
				}
				else if(Flag_adc==1) //��������־λ��1
				{
					if(Charger_Time>=ShortPulse_Time) //ʱ�������
					{
							All_Discharg();
							Charger_Time=0;
							Flag_adc=0;
					}
	      }
			  break;
		case mcDrag:
			  StartupDrag();//ǿ��
			  break;		
		case mcRun:	
			  BemfCheck();	//���綯�Ƽ�� 
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
