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
 函 数 名  : TIM3_IRQHandler
 功能描述  : TIM2中断   
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void  TIM2_IRQHandler(void)
{
	
	if( TIM_GetITStatus(TIM2,TIM_IT_Update) != RESET ) //是否发生中断
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
    
}
/*****************************************************************************
 函 数 名  : TIM3_IRQHandler
 功能描述  : TIM3中断   换相
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void  TIM3_IRQHandler(void)
{
	if(sysflags.Angle_Mask==1)
	{
		sysflags.Angle_Mask=0;
		TIM_Cmd(TIM3, DISABLE);   //失能定时器	 下一个30°再开启
	}		 
	if(sysflags.ChangePhase==1)
	{
		sysflags.ChangePhase=0;
		Startup_Turn();//
		TIM3->ARR=Mask_TIME;	//续流定时
		TIM3->CNT = 0; 	
		sysflags.Angle_Mask=1;		 
	}
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);	   
}	
/*****************************************************************************
 函 数 名  : TIM4_IRQHandler
 功能描述  : TIM4中断   1ms
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void  TIM4_IRQHandler(void)
{
	if( TIM_GetITStatus(TIM4,TIM_IT_Update) != RESET ) //是否发生中断
	{
		Sysvariable.ChangeTime_Count++;		//计算时间 ms
		Sysvariable.ADCTimeCnt ++;
		// if(mcState == mcRun)
		// {
		// 	TuneDutyRatioCnt ++;	
		// }
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
}

/****************************************************************s*************
 函 数 名  : TIM1_CC_IRQHandler
 功能描述  : TIM1_CC中断   
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void TIM1_CC_IRQHandler(void)     
{
    if ( TIM_GetITStatus ( TIM1, TIM_IT_CC4 ) != RESET )              //CCR4中断
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

#if PULSE_INJECTION
			if((Flag_adc==1)||(Flag_Charger==1))//充电标志位或者电流检测标志位置1
			{
				Charger_Time++;  //时间计数++

			}

			if(Flag_adc==1)//电流检测标志位置1
			{
				if(pos_idx<=5) //ADC电流检测序号
				{

					if(ADC_check_buf[pos_idx] < RegularConvData_Tab[4])
					{
						ADC_check_buf[pos_idx] = RegularConvData_Tab[4];//获取峰值电流	//此处有待商酌
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
			
			if(Flag_Charger==1) //充电标志位
			{

				curr_test[test_idx][j] = RegularConvData_Tab[4];
				j++;

				if(Charger_Time>=LongPulse_Time)  //充电时间到
				{
					Charger_Time=0;  //计数清0
					All_Discharg();  //输出全部关闭
					Flag_Charger=0;  //充电标志位清0
					test_idx ++;
				}
			}
			else if(Flag_adc==1) //电流检测标志位置1
			{
				if(Charger_Time>=ShortPulse_Time) //时间计数到
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
			StartupDrag();//强拖
			break;		
		case mcRun:	
			//BemfCheck();	//反电动势检测 
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
