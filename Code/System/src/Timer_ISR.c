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
 函 数 名  : TIM2_IRQHandler
 功能描述  : TIM2中断   60ms
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
 功能描述  : TIM3中断   换相  100us
 输入参数  : 无
 输出参数  : void
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
		TIM_Cmd(TIM3, DISABLE);   //失能定时器	 下一个30°再开启
	}		 
	if(sysflags.ChangePhase==1)
	{
		sysflags.ChangePhase=0;
		Startup_Turn();
		Motor.step_counter ++;	// 记录50ms内的换相次数
		Sysvariable.ChangeCount ++;	//记录总的换相次数
		TIM3->ARR=Mask_TIME;	//续流定时
		TIM3->CNT = 0; 	
		sysflags.Angle_Mask=1;		 
	}
	// MotorTimerDecrease();
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);	   
}	
/*****************************************************************************
 函 数 名  : TIM4_IRQHandler
 功能描述  : TIM4中断   1ms		转速PID
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void  TIM4_IRQHandler(void)
{
	if( TIM_GetITStatus(TIM4,TIM_IT_Update) != RESET ) //是否发生中断
	{
		Sysvariable.ChangeTime_Count++;		//计算时间 ms
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
		switch(mcState)
		{
			case mcAlignment:
				Align_pos_check_proc();
#if PULSE_INJECTION
				Motor.CheckCurrent = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_4);
				if((Flag_adc==1)||(Flag_Charger==1))//充电标志位或者电流检测标志位置1
				{
					Charger_Time++;  //时间计数++
				}
				if(Flag_adc==1)//电流检测标志位置1
				{
					if(pos_idx<=5) //ADC电流检测序号
					{
						if(ADC_check_buf[pos_idx] < Motor.CheckCurrent)
						{
							ADC_check_buf[pos_idx] = Motor.CheckCurrent;	//获取峰值电流	//此处有待商酌
						}			
					}
					curr_test[pos_idx][j] = Motor.CheckCurrent;
					j++;								
				}
				
				if(Flag_Charger==1) //充电标志位
				{
					curr_test[test_idx][j] = Motor.CheckCurrent;
					j++;

					if(Charger_Time>=LongPulse_Time)  //充电时间到
					{
						Charger_Time=0;  //计数清0
						All_Discharg();  //输出全部关闭
						Flag_Charger=0;  //充电标志位清0
						test_idx ++;
						if(test_idx>=6)
						{
							test_idx = 0;
						}
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
 函 数 名  : ADC1_2_IRQHandler
 功能描述  : ADC注入通道转换完成中断   
 输入参数  : 无
 输出参数  : void
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
				if((Flag_adc==1)||(Flag_Charger==1))//充电标志位或者电流检测标志位置1
				{
					Charger_Time++;  //时间计数++
				}
				if(Flag_adc==1)//电流检测标志位置1
				{
					if(pos_idx<=5) //ADC电流检测序号
					{
						if(ADC_check_buf[pos_idx] < Motor.CheckCurrent)
						{
							ADC_check_buf[pos_idx] = Motor.CheckCurrent;	//获取峰值电流	//此处有待商酌
						}			
					}
					curr_test[pos_idx][j] = Motor.CheckCurrent;
					j++;								
				}
				
				if(Flag_Charger==1) //充电标志位
				{
					curr_test[test_idx][j] = Motor.CheckCurrent;
					j++;

					if(Charger_Time>=LongPulse_Time)  //充电时间到
					{
						Charger_Time=0;  //计数清0
						All_Discharg();  //输出全部关闭
						Flag_Charger=0;  //充电标志位清0
						test_idx ++;
						if(test_idx>=6)
						{
							test_idx = 0;
						}
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
#endif
				break;
			case mcDrag:
				StartupDrag();//强拖
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
 函 数 名  : DMA1_Channel1_IRQHandler
 功能描述  : DMA中断   
 输入参数  : 无
 输出参数  : void
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
		if((Flag_adc==1)||(Flag_Charger==1))//充电标志位或者电流检测标志位置1
		{
			Charger_Time++;  //时间计数++
		}
		if(Flag_adc==1)//电流检测标志位置1
		{
			if(pos_idx<=5) //ADC电流检测序号
			{
				if(ADC_check_buf[pos_idx] < Motor.CheckCurrent)
				{
					ADC_check_buf[pos_idx] = Motor.CheckCurrent;	//获取峰值电流	//此处有待商酌
				}			
			}
			curr_test[pos_idx][j] = Motor.CheckCurrent;
			j++;								
		}
		
		if(Flag_Charger==1) //充电标志位
		{
			curr_test[test_idx][j] = Motor.CheckCurrent;
			j++;

			if(Charger_Time>=LongPulse_Time)  //充电时间到
			{
				Charger_Time=0;  //计数清0
				All_Discharg();  //输出全部关闭
				Flag_Charger=0;  //充电标志位清0
				test_idx ++;
				if(test_idx>=6)
				{
					test_idx = 0;
				}
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
#endif
		}
		DMA_ClearITPendingBit(DMA1_IT_TC1);
	}
	
}
#endif
