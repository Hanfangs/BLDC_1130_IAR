#ifndef _SYS_FILE
#define _SYS_FILE

#include "SysConfig.h"

static __IO uint32_t TimingDelay;
 void TimingDelay_Decrement(void);
 /**
  * @file   SYSTICK_Init
  * @brief  初始化SYSTICK，1Ms中断1次
  * @param  无
  * @retval 无
  */
void SYSTICK_Init(void)
{		/*SystemCoreClock/1000000：1us中断1次；SystemCoreClock/ 1000：1ms中断一次*/
	if(SysTick_Config(SystemCoreClock/1000))
	{
		while(1);
	}
}


/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay(__IO uint32_t nTime)
{ 
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (TimingDelay != 0x00)
  { 
    TimingDelay--;
  }
}

void Delay_ms( uint16_t time_ms )  
{

  u32 temp;
  SysTick->LOAD = 9000*time_ms;
  SysTick->VAL=0X00;//清空计数器
  SysTick->CTRL=0X01;//使能，减到零是无动作，采用外部时钟源

  do
  {
    temp=SysTick->CTRL;//读取当前倒计数值
  }while((temp&0x01)&&(!(temp&(1<<16))));//等待时间到达

  SysTick->CTRL=0x00; //关闭计数器
  SysTick->VAL =0X00; //清空计数器
}
void Delay_us( uint16_t time_us )  
{

 u32 temp;
 SysTick->LOAD = 9*time_us;
 SysTick->VAL=0X00;//清空计数器
 SysTick->CTRL=0X01;//使能，减到零是无动作，采用外部时钟源
 do
 {
  temp=SysTick->CTRL;//读取当前倒计数值
 }while((temp&0x01)&&(!(temp&(1<<16))));//等待时间到达
     SysTick->CTRL=0x00; //关闭计数器
    SysTick->VAL =0X00; //清空计数器

}

#endif
