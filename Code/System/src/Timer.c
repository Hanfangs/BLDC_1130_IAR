#ifndef  _TIM_FILE
#define  _TIM_FILE

#include "SysConfig.h" 

#include "Timer.h"
#include "GPIO_Init.h"
#include "BLDC.h"
#include "Timer_ISR.h"
#include "User_Config.h"

const  uint16_t StopTime_Buffer[20]={0,100,200,300,400,500,600,700,800,900,1000,
	                                   1100 ,1200,1300,1400,1500,1600,1700,1800,1900
};
/*****************************************************************
  * @file     TIM2_Config
  * @brief    ��ʱ��2��ʼ��   ��ʱ100us
  * @param    ��
  * @retval   ��
  ***************************************************************/
 void TIM2_Config(void)
 {
	 TIM_TimeBaseInitTypeDef TIM_BaseInitStructure;
   NVIC_InitTypeDef   NVIC_InitStructure;  //

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

  /*****
	��������ʱ��=Fclk/(Prescaler+1)
	PWM����T= ( PWM_Arr+1  )*(Prescaler+1)/Fclk (HSI����  Fclk=48MHZ)--�õ�����   s
   ChannelxPulse = DutyCycle * (TIM1_Period - 1) / 100
	***/
  /* Time Base configuration ��ʼ����ʱ��*/
  TIM_DeInit(TIM2);  
  TIM_BaseInitStructure.TIM_Period = 100 - 1;           
  TIM_BaseInitStructure.TIM_Prescaler = 71;      
  TIM_BaseInitStructure.TIM_ClockDivision = 0;     
  TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Down; 
  TIM_TimeBaseInit(TIM2, &TIM_BaseInitStructure); 
    
  TIM_ClearFlag(TIM2, TIM_FLAG_Update);
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);          ////100us�ж�һ��
    
  TIM_Cmd(TIM2, ENABLE);

    /* Enable the TIM2 Interrupt    ����100us��ʱ�õ�  */
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;                 //�����ж�Դ
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;       //������ռ���ȼ�
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;              //���������ȼ�
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                 //ʹ���ж�ͨ��
  NVIC_Init(&NVIC_InitStructure);                                 //���ó�ʼ������
 
 }
 /*****************************************************************
  * @file     TIM4_Config
  * @brief    ��ʱ��4��ʼ��   ��ʱ1ms
  * @param    ��
  * @retval   ��
  ***************************************************************/
 void TIM4_Config(void)
 {
    TIM_TimeBaseInitTypeDef TIM_BaseInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
     
    TIM_BaseInitStructure.TIM_Period = 1000-1;           
    TIM_BaseInitStructure.TIM_Prescaler = 71;//
    TIM_BaseInitStructure.TIM_ClockDivision = 0;     
    TIM_BaseInitStructure.TIM_CounterMode = TIM_CounterMode_Down; 
    TIM_TimeBaseInit(TIM4, &TIM_BaseInitStructure); 
    
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);         ///�ж�����
    
    TIM_Cmd(TIM4, ENABLE);

    /* Enable the TIM4 Interrupt        ��ʱ��4������ʲô��?   1ms��ʱ�õ�      */
    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

 }
/*****************************************************************
  * @file     TIM1_Config
  * @brief    ��ʱ��1PWM��ʼ��  ���ڲ���PWM
  * @param    ��
  * @retval   ��
  ***************************************************************/
 void TIM1_Config(void)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  GPIO_InitTypeDef   GPIO_InitStructure;
 // NVIC_InitTypeDef   NVIC_InitStructure;  //
  /********************************
	U:H -PA10-CH3     L -PB1-CH3N
  V:H-PA9-CH2       L-PB0-CH2N
  W:H-PA8-CH1       L-PA7-CH1N
	*************************************/
	/* GPIO Configuration ---------------------------------------------------
    GPIOA, Clocks enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE); 

    /* ���ö�ʱ��ͨ��1�������ģʽ�������������ģʽ */
  GPIO_InitStructure.GPIO_Pin =  BLDC_TIM_CH1_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(BLDC_TIM_CH1_PORT, &GPIO_InitStructure);
  
    /* ���ö�ʱ��ͨ��2�������ģʽ */
  GPIO_InitStructure.GPIO_Pin =  BLDC_TIM_CH2_PIN;
  GPIO_Init(BLDC_TIM_CH2_PORT, &GPIO_InitStructure);
  
    /* ���ö�ʱ��ͨ��3�������ģʽ */
  GPIO_InitStructure.GPIO_Pin =  BLDC_TIM_CH3_PIN;
  GPIO_Init(BLDC_TIM_CH3_PORT, &GPIO_InitStructure);

    // ���ö�ʱ������ͨ��1�������ģʽ     
  GPIO_InitStructure.GPIO_Pin =  BLDC_TIM_CH1N_PIN;
  GPIO_Init(BLDC_TIM_CH1N_PORT, &GPIO_InitStructure); 
     
    // ���ö�ʱ������ͨ��2�������ģʽ 
  GPIO_InitStructure.GPIO_Pin =  BLDC_TIM_CH2N_PIN;
  GPIO_Init(BLDC_TIM_CH2N_PORT, &GPIO_InitStructure); 
     
    // ���ö�ʱ������ͨ��3�������ģʽ 
  GPIO_InitStructure.GPIO_Pin =  BLDC_TIM_CH3N_PIN;
  GPIO_Init(BLDC_TIM_CH3N_PORT, &GPIO_InitStructure);    
    
    /* Configuration: BKIN pin */
  GPIO_InitStructure.GPIO_Pin = BLDC_TIM_BKIN_PIN;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(BLDC_TIM_BKIN_PORT, &GPIO_InitStructure); 
  GPIO_SetBits(GPIOB, U_Mos_L_Pin);
  GPIO_SetBits(GPIOA, W_Mos_L_Pin);
  GPIO_SetBits(GPIOB, V_Mos_L_Pin); 
  
  /* TIM1 Configuration ---------------------------------------------------*/
  /* TIM1 clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 , ENABLE);
  /*****
	��������ʱ��=Fclk/(Prescaler+1)
	PWM����T=  (PWM_Arr+1 )*(Prescaler+1)/Fclk (HSI����  Fclk=48MHZ)---��λ��s
   ChannelxPulse = DutyCycle * (TIM1_Period - 1) / 100
	***/
  /* Time Base configuration ��ʼ����ʱ��*/
	TIM_DeInit(TIM1);
  TIM_TimeBaseStructure.TIM_Prescaler =0 ;      //����Ԥ��Ƶ   sys/48000
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1 ;  //  ���ϼ���ʱ������
  TIM_TimeBaseStructure.TIM_Period =(SystemCoreClock/Cent_PWMFRE_16K)-1;    //�����Զ���װ��ֵ 16k  1500  --16k ��Ӧ62.5us 
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;      //ʱ�ӷ�Ƶϵ��
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 1;   //����������ٴβŽ��ж�  0��ʾ���1�� ����1��  
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);   //TIM1����PWM����
	//TIM_PrescalerConfig(TIM1,0,TIM_PSCReloadMode_Immediate);  //Ԥ��Ƶֵ��ʱװ��
	//TIM_ARRPreloadConfig(TIM1,ENABLE);//ʹ��APRԤװ�ػ�����

#if  1
  /* Channel 1, 2,3  Configuration in PWM mode */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;   //PWMģʽ2:CNT>CCR �����Ч
  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //���ʹ��
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputState_Enable;//�������ʹ��
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //���ó�ʼ����  -�ߵ�ƽ 
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High;  //�����������--�ߵ�ƽ   ���ű�
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;
	
  TIM_OC1Init(TIM1, &TIM_OCInitStructure);  //ʹ��ͨ��1  ��ʼ������Ƚϲ���
	//TIM_OC1PreloadConfig(TIM1,TIM_OCPreload_Enable);//ʹ������Ԥת�ؼĴ���

  TIM_OCInitStructure.TIM_Pulse = 0;
  TIM_OC2Init(TIM1, &TIM_OCInitStructure);  //ʹ��ͨ��2

  TIM_OCInitStructure.TIM_Pulse = 0;  //ʹ��ͨ��3
  TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	
#endif
	#if  1
	TIM_ClearFlag(TIM1, TIM_IT_Update);  
	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);                             //���жϱ�־λ
  TIM_ITConfig(TIM1,TIM_IT_Update ,DISABLE);                              //��ֹ�ж� 


//	NVIC_InitStructure.NVIC_IRQChannel =TIM1_BRK_UP_TRG_COM_IRQn;// TIM1_BRK_UP_TRG_COM_IRQn
//  NVIC_InitStructure.NVIC_IRQChannelPriority = 1;
//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_Init(&NVIC_InitStructure);
	
	#endif  
	TIM_Cmd(TIM1, ENABLE);  //ʹ�ܶ�ʱ��
	TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);   
	TIM_CtrlPWMOutputs(TIM1,ENABLE);


}

/*****************************************************************
  * @file     Start_Motor
  * @brief    �������
  * @param    ��
  * @retval   ��
  ***************************************************************/
void Start_Motor(void)
{
	GPIO_SetBits(GPIOB, U_Mos_L_Pin);
  GPIO_SetBits(GPIOA, W_Mos_L_Pin);
  GPIO_SetBits(GPIOB, V_Mos_L_Pin);
	
	TIM_CCxCmd(TIM1, TIM_Channel_1, TIM_CCx_Enable);
	TIM_CCxCmd(TIM1, TIM_Channel_2, TIM_CCx_Enable);
	TIM_CCxCmd(TIM1, TIM_Channel_3, TIM_CCx_Enable);
}
/*****************************************************************
  * @file     MOS_Q15PWM
  * @brief    U���Ϲ�ͨ��V����¹�ͨ  �����ر�
  * @param    ��
  * @retval   ��
  ******************************0*********************************/
void MOS_Q15PWM(void)      
{   
	TIM1->CCR1 = 0; 
	TIM1->CCR2 = 0;
	TIM1->CCR3 = Motor.Duty;	  //PA10���pwm	 -U��			  

	GPIOB->BSRR = U_Mos_L_Pin;
	GPIOA->BSRR = W_Mos_L_Pin;
	GPIOB->BRR = V_Mos_L_Pin;

	
}
 /*****************************************************************
  * @file     MOS_Q16PWM
  * @brief    U���Ϲ�ͨ��W����¹�ͨ  �����ر�
  * @param    ��
  * @retval   ��
  ***************************************************************/
void  MOS_Q16PWM(void)
{    

  TIM1->CCR1= 0; 
	TIM1->CCR2 = 0;
	TIM1->CCR3 = Motor.Duty;   //PA10���pwm	 -U��							  	

	GPIOB->BSRR =  U_Mos_L_Pin | V_Mos_L_Pin;
	GPIOA->BRR = W_Mos_L_Pin;
	
}
 /*****************************************************************
  * @file     MOS_Q26PWM
  * @brief    V���Ϲ�ͨ��W����¹�ͨ  �����ر�
  * @param    ��
  * @retval   ��
  ***************************************************************/
void MOS_Q26PWM(void)
{    
	
	TIM1->CCR1=0; 
	TIM1->CCR3=0;
	TIM1->CCR2 = Motor.Duty;	//PA9���pwm -V��			  
 
	GPIOB->BSRR =  U_Mos_L_Pin | V_Mos_L_Pin;
	GPIOA->BRR = W_Mos_L_Pin;

}
 /*****************************************************************
  * @file     MOS_Q24PWM
  * @brief    V���Ϲ�ͨ��U����¹�ͨ  �����ر�
  * @param    ��
  * @retval   ��
  ***************************************************************/
void MOS_Q24PWM(void) 
{    
	
	
	TIM1->CCR1 = 0;  
	TIM1->CCR3 = 0;	
	TIM1->CCR2 = Motor.Duty;	 //PA9���pwm -V��				  

	GPIOB->BSRR =   V_Mos_L_Pin;
	GPIOA->BSRR =   W_Mos_L_Pin;
	GPIOB->BRR = U_Mos_L_Pin;

}

/*****************************************************************
  * @file     MOS_Q34PWM
  * @brief    W���Ϲ�ͨ��U����¹�ͨ  �����ر�
  * @param    ��
  * @retval   ��
  ***************************************************************/
void MOS_Q34PWM(void)
{

	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	TIM1->CCR1 = Motor.Duty;		    //PA8���pwm	-W��	  

	GPIOB->BSRR =   V_Mos_L_Pin;
	GPIOA->BSRR =   W_Mos_L_Pin;
  GPIOB->BRR = U_Mos_L_Pin;


	
}
 /*****************************************************************
  * @file     MOS_Q34PWM
  * @brief    W���Ϲ�ͨ��V����¹�ͨ  �����ر�
  * @param    ��
  * @retval   ��
  ***************************************************************/
void MOS_Q35PWM(void)
{  

  TIM1->CCR2 = 0; 
	TIM1->CCR3 = 0;
	TIM1->CCR1 = Motor.Duty;		        //W			  
  
	GPIOB->BSRR =   U_Mos_L_Pin;
	GPIOA->BSRR =   W_Mos_L_Pin;
	GPIOB->BRR = V_Mos_L_Pin;


}

#endif
