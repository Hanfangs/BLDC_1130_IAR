/****************************************************
 * 文件名  ：main.c
**********************************************************************************/	

//头文件
#include "stm32f10x.h"
#include "User_Config.h"
#include "SysConfig.h"
#include "GPIO_Init.h"
#include "Analog.h"
#include "BLDC.h"
#include "Timer.h"
#include "Timer_ISR.h"
#include "PI_Cale.h"
#include "USART.h"

/****************************************************
  * @file   main
  * @brief  Main program.
  * @param  None
  * @retval None
  **************************************************/
int main(void)
{
 
	Motor.ControlMode =Control_Mode ;//开环运行
	/*******系统函数初始化******/
	Sys_Variable_Init();
	BLDC_GPIO_Config();         //BLDC GPIO初始化
	Exti_Config();
	NVIC_Configuration();		//NVIC初始化
	ADC_Configuration();        //ADC初始化
	TIM1_Config();              //定时器 PWM初始化
	TIM2_Config();              //定时器初始化  
	TIM4_Config();
	mcState=mcStop;

	while(1)
	{
		//Led_RunTask_Op(LED_FLASH);
		if((mcState==mcStop)&&(sysflags.Motor_Stop==1))
		{
		 	//非正常停机
		 
			if((mcFault == OverUnderVoltage)||(mcFault ==OverCurrent)||(mcFault ==OverTemperature)||(mcFault ==Motor_Stall_1)||
			 	(mcFault ==HardWare_OverCurrent)||(mcFault ==Overtime_PhaseChange)	)
			{
				if(ADJ_MODE==STEPLESS_SPEED)
				{
					UserSpeedSample =RegularConvData_Tab[4] ;   //获取电位器AD值0-4095	//Instance current
					if(UserSpeedSample<RHEOSTATMIN)							//200
					{
						MasterState=Startup;
						mcFault=RunNormal;
					}						
				}                                                                                                                                                      
				if(ADJ_MODE==DIRECT_GIVE)
				{
					if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_3) == 0)
					{
						//Led_RunTask_Op(LED_FLASH);
						GPIO_SetBits(LED_PORT, LED_PIN);
						MasterState=Startup;
						mcFault=RunNormal;
					}
				}
			}
			else
			{
				MasterState=Startup;
			}
			sysflags.Motor_Stop=0;
		}
		
		if(GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_4) == 0)
		{
			GPIO_SetBits(LED_PORT, LED_PIN);
		switch(MasterState)
		{
			case Startup:
		  		UserSpeedControlInit();
				if((sysflags.System_Start==1)&&(mcFault==RunNormal))
				{
					MasterState = Operation;      //工作流切换
					mcState = mcInit;             //电机状态切换到初始化
				}
				else
				{
					mcState=mcStop;
				}
				break;
				
			case Operation:
				MotorControl();
				if(ADCIntProtectCnt >= PWM_ADJ)	  //16*62.5=1ms
				{		
					ADCIntProtectCnt=0;
					Cal_AverCurrent();	
					//Fault_Detection();  
					Angle_Dealwith();
					UserSpeedControl();
				}
				break;
			default:
				MotorStop(); 
				break;
		} 
		}
		else 
		{
			GPIO_ResetBits(LED_PORT, LED_PIN);
		}

	}
}


/*********************************************************************************************************
      END FILE
*********************************************************************************************************/





