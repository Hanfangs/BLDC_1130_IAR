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
    
    /*����GPIOAʱ���ź�*/
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA,ENABLE );
    /*����GPIOBʱ���ź�*/
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB,ENABLE );
    /*����GPIOCʱ���ź�*/
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC,ENABLE );

    
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    /*��ַ����*/
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
	EXTI_InitTypeDef EXTI_InitStructure;						//����ṹ�����
	NVIC_InitTypeDef NVIC_InitStructure;						//����ṹ�����
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO ,ENABLE);
	
  	/*AFIOѡ���ж�����*/
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12);//���ⲿ�жϵ�15����ӳ�䵽GPIOB����ѡ��PB12Ϊ�ⲿ�ж�����
	
	/*EXTI��ʼ��*/
	EXTI_InitStructure.EXTI_Line = EXTI_Line12;					//ѡ�������ⲿ�жϵ�12����
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;					//ָ���ⲿ�ж���ʹ��
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;			//ָ���ⲿ�ж���Ϊ�ж�ģʽ
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;		//ָ���ⲿ�ж���Ϊ�½��ش���
	EXTI_Init(&EXTI_InitStructure);								//���ṹ���������EXTI_Init������EXTI����

  	/*NVIC�жϷ���*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);				//����NVICΪ����2
																//����ռ���ȼ���Χ��0~3����Ӧ���ȼ���Χ��0~3
																//�˷������������������н������һ��
																//���ж���жϣ����԰Ѵ˴������main�����ڣ�whileѭ��֮ǰ
																//�����ö�����÷���Ĵ��룬���ִ�е����ûḲ����ִ�е�����
	
	/*NVIC����*/

	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;		//ѡ������NVIC��EXTI15_10��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;				//ָ��NVIC��·ʹ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;	//ָ��NVIC��·����ռ���ȼ�Ϊ1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			//ָ��NVIC��·����Ӧ���ȼ�Ϊ1
	NVIC_Init(&NVIC_InitStructure);								//���ṹ���������NVIC_Init������NVIC����

}
/*****************************************************************************
 �� �� ��  : EXTI4_15_IRQHandler
 ��������  : Ӳ�������ж�
 �������  : ��
 �������  : void
*****************************************************************************/
void EXTI15_10_IRQHandler(void)
{   
 /*���ָ����EXTI��·�������������*/
  if (EXTI_GetITStatus(EXTI_Line12) != RESET)
  { 
    /*���EXTI��·����λ*/
    EXTI_ClearITPendingBit(EXTI_Line12); 
  }
	  if(Flag_OverCurr==1)
	 {
    mcFault=HardWare_OverCurrent;
	  Sysvariable.Stop_Time=Motor_DelayTime;			
    MotorStop();           //����ֹͣ���
	 }

}

#endif

