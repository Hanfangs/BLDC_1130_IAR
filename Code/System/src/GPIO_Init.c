#ifndef  _GPIO_FILE 
#define  _GPIO_FILE

#include "GPIO_Init.h"
#include "Timer.h"
#include "BLDC.h"
#include "Timer_ISR.h"
#include "User_Config.h"
 uint16_t  EE=0;
 void BLDC_GPIO_Config(void)
{	
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /*开启GPIOA时钟信号*/
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA,ENABLE );
    /*开启GPIOB时钟信号*/
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB,ENABLE );
    /*开启GPIOC时钟信号*/
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC,ENABLE );

    
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    /*地址拨码*/
    GPIO_InitStructure.GPIO_Pin = DEVICE_ADDR1_PIN | DEVICE_ADDR2_PIN | DEVICE_ADDR4_PIN\
                                  | DEVICE_ADDR8_PIN | DEVICE_ADDR16_PIN | DEVICE_ADDR32_PIN | DEVICE_ADDR64_PIN; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
    GPIO_Init(DEVICE_ADDR1_PORT, &GPIO_InitStructure); 

    /*LED  PB10*/
    GPIO_InitStructure.GPIO_Pin = LED_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(LED_PORT, &GPIO_InitStructure);
    GPIO_ResetBits(LED_PORT, LED_PIN);
}
void Exti_Config(void)
{	
	EXTI_InitTypeDef EXTI_InitStructure;						//定义结构体变量
	NVIC_InitTypeDef NVIC_InitStructure;						//定义结构体变量
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO ,ENABLE);
	
  	/*AFIO选择中断引脚*/
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12);//将外部中断的15号线映射到GPIOB，即选择PB12为外部中断引脚
	
	/*EXTI初始化*/
	EXTI_InitStructure.EXTI_Line = EXTI_Line12;					//选择配置外部中断的12号线
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;					//指定外部中断线使能
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;			//指定外部中断线为中断模式
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;		//指定外部中断线为下降沿触发
	EXTI_Init(&EXTI_InitStructure);								//将结构体变量交给EXTI_Init，配置EXTI外设

  	/*NVIC中断分组*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				//配置NVIC为分组2
																//即抢占优先级范围：0~3，响应优先级范围：0~3
																//此分组配置在整个工程中仅需调用一次
																//若有多个中断，可以把此代码放在main函数内，while循环之前
																//若调用多次配置分组的代码，则后执行的配置会覆盖先执行的配置
	
	/*NVIC配置*/

	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;		//选择配置NVIC的EXTI15_10线
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//指定NVIC线路使能
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	//指定NVIC线路的抢占优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			//指定NVIC线路的响应优先级为1
	NVIC_Init(&NVIC_InitStructure);								//将结构体变量交给NVIC_Init，配置NVIC外设

}
/*****************************************************************************
 函 数 名  : EXTI4_15_IRQHandler
 功能描述  : 硬件过流中断
 输入参数  : 无
 输出参数  : void
*****************************************************************************/
void EXTI15_10_IRQHandler(void)
{   
 /*检查指定的EXTI线路出发请求发生与否*/
  if (EXTI_GetITStatus(EXTI_Line12) != RESET)
  { 
    /*清除EXTI线路挂起位*/
    EXTI_ClearITPendingBit(EXTI_Line12); 
  }
	  if(Flag_OverCurr==1)
	 {
    mcFault=HardWare_OverCurrent;
	  Sysvariable.Stop_Time=Motor_DelayTime;			
    MotorStop();           //立马停止输出
	 }

}

#endif

