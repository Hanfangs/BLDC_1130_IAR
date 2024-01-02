#ifndef _SYS_FILE
#define _SYS_FILE

#include "SysConfig.h"

static __IO uint32_t TimingDelay;
 void TimingDelay_Decrement(void);
 /**
  * @file   SYSTICK_Init
  * @brief  ��ʼ��SYSTICK��1Ms�ж�1��
  * @param  ��
  * @retval ��
  */
void SYSTICK_Init(void)
{		/*SystemCoreClock/1000000��1us�ж�1�Σ�SystemCoreClock/ 1000��1ms�ж�һ��*/
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
  SysTick->VAL=0X00;//��ռ�����
  SysTick->CTRL=0X01;//ʹ�ܣ����������޶����������ⲿʱ��Դ

  do
  {
    temp=SysTick->CTRL;//��ȡ��ǰ������ֵ
  }while((temp&0x01)&&(!(temp&(1<<16))));//�ȴ�ʱ�䵽��

  SysTick->CTRL=0x00; //�رռ�����
  SysTick->VAL =0X00; //��ռ�����
}
void Delay_us( uint16_t time_us )  
{

 u32 temp;
 SysTick->LOAD = 9*time_us;
 SysTick->VAL=0X00;//��ռ�����
 SysTick->CTRL=0X01;//ʹ�ܣ����������޶����������ⲿʱ��Դ
 do
 {
  temp=SysTick->CTRL;//��ȡ��ǰ������ֵ
 }while((temp&0x01)&&(!(temp&(1<<16))));//�ȴ�ʱ�䵽��
     SysTick->CTRL=0x00; //�رռ�����
    SysTick->VAL =0X00; //��ռ�����

}

#endif
