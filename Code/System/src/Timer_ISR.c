#include "Timer_ISR.h"
#include "Analog.h"
#include "BLDC.h"
#include "PI_Cale.h"
#include "Timer.h"
#include "User_Config.h"

uint16_t ADCIntProtectCnt=0;
uint16_t TuneDutyRatioCnt = 0;

/*****************************************************************************
 函 数 名  : TIM3_IRQHandler
 功能描述  : TIM3中断   
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void  TIM3_IRQHandler(void)
{
	
   if( TIM_GetITStatus(TIM3,TIM_IT_Update) != RESET ) //是否发生中断
	 {
		 	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	 }
    
}	
/*****************************************************************************
 函 数 名  : TIM14_IRQHandler
 功能描述  : TIM14中断   换相
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void  TIM14_IRQHandler(void)
{
	 if(sysflags.Angle_Mask==1)
	 {
	    sysflags.Angle_Mask=0;
      TIM_Cmd(TIM14, DISABLE);   //失能定时器	 
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
 函 数 名  : DMA1_Channel1_IRQHandler
 功能描述  : DMA中断   过零点检测
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void DMA1_Channel1_IRQHandler(void)
{
	switch(mcState)
	{
		case mcAlignment:
				Align_pos_check_proc();//初始位置检测进程
			  if((Flag_adc==1)||(Flag_Charger==1))//充电标志位或者电流检测标志位置1
				{
			    Charger_Time++;  //时间计数++
				}
			  if(Flag_adc==1)//电流检测标志位置1
				{
					if(pos_idx<=5) //ADC电流检测序号
					{			
						ADC_check_buf[pos_idx]=RegularConvData_Tab[1];//获取峰值电流	//此处有待商酌
					}
				}
			 if(Flag_Charger==1) //充电标志位
	     {
					if(Charger_Time>=LongPulse_Time)  //充电时间到
					{
						Charger_Time=0;  //计数清0
						All_Discharg();  //输出全部关闭
						Flag_Charger=0;  //充电标志位清0
					}
				}
				else if(Flag_adc==1) //电流检测标志位置1
				{
					if(Charger_Time>=ShortPulse_Time) //时间计数到
					{
							All_Discharg();
							Charger_Time=0;
							Flag_adc=0;
					}
	      }
			  break;
		case mcDrag:
			  StartupDrag();//强拖
			  break;		
		case mcRun:	
			  BemfCheck();	//反电动势检测 
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
